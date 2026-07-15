# Hybrid Control Mux ロジック

## 全体像

`hybrid_control_mux` のロジックは、MPC と Pure Pursuit の制御指令を監視し、どちらを最終制御指令として使うかを決める状態機械です。

基本方針は以下です。

- MPC が正常なら MPC を使う
- MPC が infeasible または timeout したら Pure Pursuit へ落とす
- Pure Pursuit も使えない場合は停止する
- MPC が十分に回復したら MPC へ戻す

中核の判定は `hybrid_control_mux/core.py` の `HybridMuxCore.update()` にあります。ROS 2 ノード側の `hybrid_control_mux_node.py` は、トピック購読、時刻管理、コマンド整形、debug 出力を担当します。

## 入力の受信

### MPC 制御指令

`input/mpc_control_cmd` で MPC の `AckermannControlCommand` を受け取ります。受信時刻を保存し、現在時刻との差が `mpc_cmd_timeout_sec` 以下なら fresh と判定します。

### Pure Pursuit 制御指令

`input/pure_pursuit_control_cmd` で Pure Pursuit の `AckermannControlCommand` を受け取ります。受信時刻を保存し、現在時刻との差が `pure_pursuit_cmd_timeout_sec` 以下なら fresh と判定します。

Pure Pursuit ノード側でも odometry と trajectory の受信時刻を見ます。鮮度判定は `/clock` 停止時にも進む steady time で行います。どちらかが `max_odom_age_sec` / `max_trajectory_age_sec` を超えて古い場合は、古い経路追従を続けず停止指令を出します。`/overtake/reference_override` のv1/v3横+速度overrideだけが古い場合は停止せず、その横overrideを破棄して元のtrajectoryを追います。v2速度のみoverrideが古い、またはv2受信後にpayloadが壊れた場合は、横方向を再利用せず通常trajectoryのまま最後に検証済みの速度capを保持します。

### MPC health

`input/mpc_health` で JSON 文字列を受け取ります。現在参照しているフィールドは以下です。

- `mpc_status`
- `mpc_infeasible_count`

JSON として読めない場合は `parse_error` になります。受信後に `mpc_health_timeout_sec` より長く更新されていない場合は `stale` になります。

## 内部状態

`HybridMuxCore` は以下の状態を保持します。

| 状態 | 内容 |
| --- | --- |
| `fallback_active` | Pure Pursuit フォールバック中か |
| `fallback_enter_time_sec` | フォールバックへ入った時刻 |
| `solved_cycles` | MPC が連続して正常だった回数 |
| `last_reason` | 最後に切り替え理由として記録した文字列 |

この状態により、MPC と Pure Pursuit の切り替えが毎周期振動しないようにしています。

## フォールバック開始判定

`_fallback_trigger()` は、次の順でフォールバック開始条件を調べます。

### 1. MPC 指令の timeout

MPC 指令が fresh でない場合、`use_pure_pursuit_on_mpc_cmd_timeout` が true なら `mpc_cmd_timeout` を理由にフォールバックします。

デフォルトは true です。MPC が重くて制御指令の publish が止まる場合に、Pure Pursuit へ逃がすためです。

### 2. MPC health の無効化

MPC health が missing、stale、parse_error の場合、`use_pure_pursuit_on_mpc_health_timeout` が true なら `mpc_health_timeout` を理由にフォールバックします。

デフォルトは false です。health トピックだけが一時的に遅れたケースで誤って Pure Pursuit に落とさないためです。

### 3. MPC infeasible

MPC health が有効で、`mpc_infeasible_count` が `fallback_trigger_infeasible_count` 以上なら `mpc_infeasible` を理由にフォールバックします。

デフォルトでは `fallback_trigger_infeasible_count` は 1 です。MPC が infeasible になったら早めに Pure Pursuit へ逃がします。切り替えが多すぎる場合は 2 以上へ上げます。

## 通常状態の判定

フォールバック中でない場合は、まず MPC が正常かを評価します。

MPC 正常の条件は以下です。

- MPC 指令が fresh
- MPC health が有効
- `mpc_status == "solved"`
- `mpc_infeasible_count <= 0`

フォールバック開始条件がなければ、MPC 指令を最終出力に使います。MPC 指令が fresh でない場合は、設定に応じて Pure Pursuit に入るか、停止します。

## フォールバック中の判定

フォールバック中は、Pure Pursuit の指令が fresh なら Pure Pursuit を使います。fresh でなければ停止します。

フォールバック中も毎周期 MPC の回復を見ています。ただし、1回 solved になっただけでは復帰しません。

復帰には以下が必要です。

- `fallback_min_hold_sec` 以上、フォールバック状態を保持している
- MPC 正常判定が `fallback_release_solved_cycles` 回連続している

この2条件を満たしたら `mpc_recovered` として MPC に戻します。

## 出力選択

`HybridMuxCore.update()` の結果は `MuxDecision` として返ります。

| `source` | 出力内容 |
| --- | --- |
| `mpc` | 最新の MPC 指令を stamp 更新して出力 |
| `pure_pursuit` | 最新の Pure Pursuit 指令を stamp 更新し、フォールバック用に速度と加速度を制限して出力 |
| `stop` | speed 0.0 の停止指令を出力 |

## Pure Pursuit 指令の整形

Pure Pursuit を使う場合でも、そのまま全てを通すわけではありません。フォールバック時の暴走を避けるため、以下の制限をかけます。

```text
0.0 <= speed <= fallback_speed_mps
fallback_decel_min_mps2 <= acceleration <= fallback_accel_max_mps2
```

さらに MPC / Pure Pursuit / stop のどの出力を選んだ場合でも、最終 publish 直前に操舵角を整形します。

```text
abs(steering_tire_angle) <= max_steering_angle_rad
abs(delta_steering) <= max_steering_rate_radps * dt
```

デフォルトでは `reset_steering_limiter_on_mode_change=false` なので、MPC から Pure Pursuit に切り替わった瞬間も前周期の最終操舵角から連続するように制限します。これにより、fallback の入り口で急に大きく切る挙動を抑えます。

`source=stop` の場合は例外です。停止意思を優先し、操舵 limiter の状態を `0.0 rad` にリセットして停止指令をそのまま出します。

`fallback_speed_mps` は `hybrid_control_mux` 側の速度上限であると同時に、`hybrid_delay_aware_mpc.launch.xml` から Pure Pursuit の `external_target_vel` にも渡されます。つまり、Pure Pursuit 自体の目標速度と mux 側の上限が同じ値になる構成です。

## Pure Pursuit と overtake planner

`hybrid_delay_aware_mpc.launch.xml` では、Pure Pursuit に `use_overtake_reference_override=true` を渡します。これにより、Pure Pursuit は `/overtake/reference_override` を購読します。

`/overtake/reference_override` は MPC と同じ、version付きの契約です（速度は `m/s`）。

```text
v3 lateral + speed: [1, mode_id>0, n>0, lateral_offsets[0..n), speed_caps[0..n), 3, generation, solver_horizon_intent]
v2 speed only:     [1, mode_id>0, 0, 2, generation, speed_cap_mps]
explicit inactive: [1, 0, 0, 1, generation]
```

Pure Pursuit は現在位置に最も近い trajectory index を基準にして、先の `n` 点へ以下を適用します。

- v1の `lateral_offsets[i]` が有限値なら、trajectory 点を yaw の法線方向に横移動する
- v1の `speed_caps[i]` が正の有限値なら、その点の速度を cap 以下にする
- v2はtrajectoryを横移動せず、単一の正の `speed_cap_mps` を全horizonの速度上限として使う
- `external_target_vel` を使う場合でも、現在点の `speed_caps[0]` を目標速度の上限として使う
- v1は `overtake_override_timeout_sec` より古くなったら無効化する。v2のtimeoutまたはv2受信後のmalformed payloadでは、明示解除を受けるまで最後の検証済みcapだけを保持する

Pure Pursuit の lookahead は基本的に `lookahead_gain * max(target_speed, current_speed) + lookahead_min_distance` で決まります。`curvature_adaptive_lookahead_enabled=true` の場合は、override 適用後の trajectory の実際の `x/y` 点列から前方曲率を推定し、曲率が大きい区間だけ `curvature_lookahead_min_distance` まで lookahead を短くします。曲率推定の距離窓は base lookahead から決め、trajectory 点密度だけで変わりにくくしています。直線や低曲率区間では従来の速度ベース lookahead を維持し、lookahead の急変は `curvature_lookahead_smoothing_alpha` で平滑化します。

これにより、Pure Pursuit fallback 中でも overtake planner の FOLLOW、YIELD、PASS 系の横オフセットと速度抑制が制御に乗ります。ただし、Pure Pursuit 自体が相手車両との制約を解くわけではありません。

現時点では `/mpc/prediction` を Pure Pursuit の制御入力には使いません。既存 topic は MarkerArray 可視化で、制御用の鮮度、frame、速度、index 契約が不足しているためです。将来使う場合は、MarkerArray の流用ではなく、型付きの制御用 horizon topic を追加します。

## 停止指令の生成

停止指令は以下の値で生成します。

```text
speed = 0.0
acceleration = stop_decel_mps2
steering_tire_angle = 0.0
```

これは「安全に停止できる軌道を計画する」処理ではなく、制御指令として止まる意思を出す最低限のフェイルセーフです。

## debug とログ

`source` が変わったときは warn ログを出します。

```text
hybrid control source=<source> reason=<reason> mpc_status=<status> mpc_infeasible=<count>
```

周期的な状態確認には `/hybrid_control_mux/debug` を使います。`source`、`reason`、`fallback_active`、`mpc_cmd_fresh`、`pure_pursuit_cmd_fresh`、`mpc_status`、`mpc_infeasible_count` を見ると、なぜ切り替わったかを追いやすいです。

操舵制限が効いた場合は、`raw_steer_rad`、`limited_steer_rad`、`steering_delta_rad`、`steering_angle_limited`、`steering_rate_limited` を確認します。

## よく見るべき状態

### `source=pure_pursuit`

MPC が infeasible または timeout しており、Pure Pursuit 指令は届いています。期待通りのフォールバック状態です。追い越し・追従が反映されているかは `/pure_pursuit/debug` の `overtake_override_applied` と `/debug/overtake/mode` を確認します。

### `source=stop`

MPC も Pure Pursuit も使える指令がない状態です。`pure_pursuit_cmd_fresh` が false なら、Pure Pursuit ノードが起動しているか、入力 trajectory や odometry が届いているかを確認します。

### `fallback_active=true` のまま戻らない

MPC の回復条件を満たしていません。`mpc_status`、`mpc_infeasible_count`、`mpc_cmd_fresh`、`solved_cycles` を見ます。

### フォールバックに入らない

`mpc_infeasible_count` が閾値まで増えているかを確認します。デフォルトでは 1 回の infeasible でフォールバックします。`fallback_trigger_infeasible_count` を 2 以上へ上げた場合は、count が閾値に届くまで切り替わりません。
