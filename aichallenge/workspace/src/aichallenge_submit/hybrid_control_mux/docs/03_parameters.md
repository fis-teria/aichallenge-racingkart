# Hybrid Control Mux パラメータ

## 設定ファイル

デフォルト設定は以下にあります。

```text
aichallenge/workspace/src/aichallenge_submit/hybrid_control_mux/config/hybrid_control_mux.param.yaml
```

`control_method:=hybrid_delay_aware_mpc` で起動した場合、この設定は `hybrid_control_mux.launch.xml` から読み込まれます。

Pure Pursuit fallback 側の launch パラメータは以下にあります。

```text
aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/control/pure_pursuit.launch.xml
```

## パラメータ一覧

| パラメータ | デフォルト | 内容 |
| --- | --- | --- |
| `enabled` | `true` | hybrid control mux の切り替え処理を有効にする。false の場合は MPC が fresh なら MPC、そうでなければ stop を出す |
| `control_rate_hz` | `50.0` | mux が出力判定を行う周期 |
| `mpc_cmd_timeout_sec` | `0.06` | MPC 指令を fresh とみなす最大時間 |
| `pure_pursuit_cmd_timeout_sec` | `0.20` | Pure Pursuit 指令を fresh とみなす最大時間 |
| `mpc_health_timeout_sec` | `0.75` | MPC health を有効とみなす最大時間 |
| `fallback_trigger_infeasible_count` | `1` | Pure Pursuit へ切り替えるために必要な連続 infeasible 回数 |
| `fallback_release_solved_cycles` | `3` | MPC へ復帰するために必要な連続 solved 回数 |
| `fallback_min_hold_sec` | `1.0` | Pure Pursuit フォールバックを最低限維持する時間 |
| `fallback_speed_mps` | `9.5` | フォールバック中の速度上限 |
| `fallback_accel_max_mps2` | `1.3` | フォールバック中の加速度上限 |
| `fallback_decel_min_mps2` | `-1.5` | フォールバック中の減速度下限 |
| `stop_decel_mps2` | `-1.5` | stop 指令を出すときの加速度 |
| `use_pure_pursuit_on_mpc_cmd_timeout` | `true` | MPC 指令が timeout したときに Pure Pursuit へ切り替える |
| `use_pure_pursuit_on_mpc_health_timeout` | `false` | MPC health が timeout したときに Pure Pursuit へ切り替える |
| `debug_publish_period_sec` | `0.25` | debug JSON を publish する周期 |
| `enable_steering_rate_limit` | `true` | 最終出力の操舵角レート制限を有効にする |
| `max_steering_angle_rad` | `0.5585053606381855` | 最終出力の操舵角上限 [rad] |
| `max_steering_rate_radps` | `8.0` | 最終出力の操舵角変化率上限 [rad/s] |
| `max_steering_delta_per_cycle` | `0.0` | 1周期あたりの追加操舵変化量上限 [rad]。0以下なら無効 |
| `steering_limiter_reset_dt_sec` | `0.50` | 前回出力からこの時間を超えたら操舵レート制限をリセットする |
| `reset_steering_limiter_on_mode_change` | `false` | true なら MPC/Pure Pursuit/stop の source 切替時に操舵レート制限をリセットする |
| `steering_log_throttle_sec` | `1.0` | 操舵制限ログの最短出力間隔 |

## 切り替え感度に関係するパラメータ

### `fallback_trigger_infeasible_count`

MPC が何回連続で infeasible になったら Pure Pursuit へ切り替えるかを決めます。

- 小さくすると、MPC が少しでも苦しくなった時点で早く Pure Pursuit に落ちる
- 大きくすると、MPC の一時的な失敗では切り替わりにくくなる

オンライン提出環境で MPC の失敗を早めに逃がしたい場合は `1` を検討します。ただし、短い infeasible でも切り替わるため、MPC と Pure Pursuit の切り替えが増えます。

### `fallback_release_solved_cycles`

Pure Pursuit から MPC に戻るために、MPC が何回連続で正常である必要があるかを決めます。

- 小さくすると、MPC に早く戻る
- 大きくすると、MPC が安定してから戻る

MPC と Pure Pursuit の切り替えが頻繁に起きる場合は、この値を少し大きくすると落ち着きやすいです。

### `fallback_min_hold_sec`

一度 Pure Pursuit へ入ったあと、最低何秒フォールバック状態を維持するかを決めます。

- 小さくすると、MPC に早く戻れる
- 大きくすると、切り替えの振動を抑えやすい

コーナー進入中に MPC が短時間だけ失敗する場合は、短すぎると即復帰してまた落ちる挙動になりやすいです。

## timeout に関係するパラメータ

### `mpc_cmd_timeout_sec`

MPC 指令がこの時間以上届かない場合、MPC 指令は古いとみなします。

- 小さくすると、MPC の遅れに敏感になる
- 大きくすると、多少遅れても MPC を使い続ける

制御周期に対して短すぎると、正常な publish 揺らぎでも timeout する可能性があります。

### `pure_pursuit_cmd_timeout_sec`

Pure Pursuit 指令がこの時間以上届かない場合、Pure Pursuit 指令は古いとみなします。

この値を大きくすると古い Pure Pursuit 指令を使い続けやすくなります。フォールバック中の安全性を考えると、基本は短めに保つ方が無難です。

### `mpc_health_timeout_sec`

MPC health がこの時間以上届かない場合、health は `stale` になります。

デフォルトでは `use_pure_pursuit_on_mpc_health_timeout` が false なので、health が stale になっただけでは Pure Pursuit に落ちません。ただし、MPC への復帰判定では health が有効である必要があります。

## フォールバック速度に関係するパラメータ

### `fallback_speed_mps`

Pure Pursuit フォールバック中の速度上限です。

この値は2箇所で効きます。

- `hybrid_control_mux` が Pure Pursuit 指令の speed をこの値以下に clamp する
- `hybrid_delay_aware_mpc.launch.xml` が Pure Pursuit の `external_target_vel` に同じ値を渡す

値を上げるとフォールバック中も速く走れますが、Pure Pursuit は障害物回避をしないため、サイドバイサイドやコーナーでは危険になりやすいです。

hybrid 起動では Pure Pursuit も `/overtake/reference_override` の速度 cap を読みます。実際の fallback 目標速度は、基本的に `fallback_speed_mps` と overtake planner の速度 cap の低い方になります。

## 操舵連続性に関係するパラメータ

### `enable_steering_rate_limit`

最終的に `/control/command/control_cmd` へ出す操舵角の急変を抑えます。MPC から Pure Pursuit へ切り替わる瞬間や、stop へ落ちる瞬間にも同じ limiter が効きます。

### `max_steering_angle_rad`

最終出力の絶対操舵角上限です。大きくすると大舵角を許しますが、Pure Pursuit fallback 中の急旋回リスクも上がります。

### `max_steering_rate_radps`

1秒あたりに許す操舵角変化量です。小さくすると切替時の急操作を抑えますが、低すぎると必要な旋回に追従できません。

デフォルトは `8.0 rad/s` です。50 Hz 出力では1周期あたり約 `0.16 rad` の変化を許し、fallback 切替時の段差を抑えつつ、コーナーでの追従遅れが大きくなりすぎない値にしています。

### `reset_steering_limiter_on_mode_change`

デフォルトは false です。source が MPC から Pure Pursuit に変わっても、直前の最終操舵角から滑らかにつなぎます。true にすると切替時に Pure Pursuit の操舵角へ即座に移るため、挙動確認用以外では慎重に使ってください。

## Pure Pursuit fallback と overtake planner に関係するパラメータ

以下は `pure_pursuit.launch.xml` 側の launch パラメータです。`hybrid_delay_aware_mpc.launch.xml` では、`use_overtake_reference_override` に `use_overtake_planner` と同じ値を渡します。

| パラメータ | デフォルト | 内容 |
| --- | --- | --- |
| `use_overtake_reference_override` | `true` | Pure Pursuit が `/overtake/reference_override` を読み、横オフセットと速度 cap を反映するか |
| `input_overtake_reference_override` | `/overtake/reference_override` | Pure Pursuit が読む overtake planner の override topic |
| `overtake_override_timeout_sec` | `0.50` | override を fresh とみなす最大時間 |
| `wheel_base` | `1.087` | Pure Pursuit の幾何計算に使う wheel base [m]。MPC / vehicle_info と合わせる |
| `steering_tire_angle_gain` | `1.639` | Pure Pursuit の操舵出力に掛けるゲイン。hybrid launch では simulation/非simulation とも同値 |
| `max_odom_age_sec` | `0.20` | odometry 受信を fresh とみなす最大時間 [s] |
| `max_trajectory_age_sec` | `0.50` | trajectory 受信を fresh とみなす最大時間 [s] |
| `max_override_age_sec` | `0.50` | overtake override 受信を fresh とみなす最大時間 [s] |
| `stop_on_stale_input` | `true` | odometry/trajectory が stale のとき停止指令を出す |
| `diagnostic_throttle_sec` | `1.0` | Pure Pursuit の stale/override 警告ログ間隔 [s] |
| `curvature_adaptive_lookahead_enabled` | `true` | 曲率が大きい区間で Pure Pursuit の lookahead を短くするか |
| `curvature_lookahead_min_distance` | `3.5` | 曲率適応後の lookahead 下限 [m]。`lookahead_min_distance` 未満にはならない |
| `curvature_lookahead_sensitivity` | `8.0` | 曲率に対して lookahead を短くする強さ |
| `curvature_lookahead_window_ratio` | `2.0` | 曲率推定に使う距離窓を base lookahead の何倍にするか |
| `curvature_lookahead_max_window_distance` | `10.0` | 曲率推定に使う距離窓の上限 [m] |
| `curvature_lookahead_min_arc_length` | `1.0` | 曲率推定に使う最小弧長 [m] |
| `curvature_lookahead_smoothing_alpha` | `0.35` | lookahead 更新の平滑化係数。`1.0` に近いほど即応する |

hybrid 起動時は、MPC fallback 用 Pure Pursuit に対して `use_overtake_reference_override=true` が渡されます。override が fresh な間、Pure Pursuit は現在位置から先の trajectory 点を横オフセット分だけずらし、速度 cap を目標速度の上限として使います。

これにより、MPC が infeasible で Pure Pursuit に落ちている間も、overtake planner の FOLLOW、YIELD、PASS、SAFE_STOP 系の速度抑制と横方向目標が反映されます。Pure Pursuit は曲率が大きい区間では lookahead を短くし、直線や低曲率区間では速度ベースの lookahead をそのまま使います。ただし、Pure Pursuit はMPCのように制約付き最適化を解くわけではないため、fallback速度は安全側に抑える前提です。

### `fallback_accel_max_mps2`

Pure Pursuit フォールバック中の加速度上限です。

大きくするとフォールバック中に速度を戻しやすくなります。小さくすると加速が穏やかになります。

### `fallback_decel_min_mps2`

Pure Pursuit フォールバック中の減速度下限です。負の値で指定します。

より小さい負の値にすると、フォールバック中でも強めの減速を許します。たとえば `-2.0` は `-1.5` より強い減速を許します。

### `stop_decel_mps2`

stop 指令を出すときの加速度です。MPC も Pure Pursuit も使えない場合に使います。

これは停止計画ではなく、制御指令として減速を要求する値です。路面や車両状態によって実際の停止距離は変わります。

## debug に関係するパラメータ

### `debug_publish_period_sec`

`/hybrid_control_mux/debug` の publish 周期です。

- 小さくするとログや rosbag で細かく追える
- 大きくすると debug 出力の量を減らせる
- 0 以下にすると debug publish を止める

## 調整例

### MPC が infeasible になったらすぐ Pure Pursuit に逃がしたい

```yaml
fallback_trigger_infeasible_count: 1
fallback_min_hold_sec: 1.0
fallback_release_solved_cycles: 3
```

早く逃がせますが、一時的な infeasible でも切り替わります。

### 切り替えが頻繁で不安定

```yaml
fallback_trigger_infeasible_count: 2
fallback_min_hold_sec: 1.5
fallback_release_solved_cycles: 5
```

Pure Pursuit に入ったあと、MPC が安定してから戻るようになります。

### フォールバック中の安全側を強めたい

```yaml
fallback_speed_mps: 1.5
fallback_accel_max_mps2: 0.5
fallback_decel_min_mps2: -1.5
```

速度と加速を抑えるので、MPC が苦しい場面で無理に走り続けにくくなります。

### health トピック欠落でも Pure Pursuit へ落としたい

```yaml
use_pure_pursuit_on_mpc_health_timeout: true
mpc_health_timeout_sec: 0.75
```

MPC health が止まった場合も Pure Pursuit へ切り替わります。ただし、health トピックだけが一時的に遅れた場合でも切り替わるため、まずは debug を見て必要性を確認してください。

## 調整時の確認ポイント

パラメータを変えたら、まず `/hybrid_control_mux/debug` を確認します。

```bash
ros2 topic echo /hybrid_control_mux/debug
```

特に見るべき値は以下です。

- `source`
- `reason`
- `fallback_active`
- `mpc_cmd_fresh`
- `pure_pursuit_cmd_fresh`
- `mpc_status`
- `mpc_infeasible_count`
- `output_speed_mps`

`source=stop` が出ている場合は、MPC だけでなく Pure Pursuit の指令も届いていない可能性があります。その場合は Pure Pursuit ノード、trajectory、odometry の入力を確認してください。
