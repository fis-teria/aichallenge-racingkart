# Hybrid Control Mux パラメータ

## 設定ファイル

デフォルト設定は以下にあります。

```text
aichallenge/workspace/src/aichallenge_submit/hybrid_control_mux/config/hybrid_control_mux.param.yaml
```

`control_method:=hybrid_delay_aware_mpc` で起動した場合、この設定は `hybrid_control_mux.launch.xml` から読み込まれます。

`control_method:=pure_pursuit_mpc_horizon` では、以下の専用設定を読みます。

```text
aichallenge/workspace/src/aichallenge_submit/hybrid_control_mux/config/pure_pursuit_mpc_horizon.param.yaml
```

この専用設定では `primary_source=pure_pursuit` の通常出力も
`fallback_accel_max_mps2` でclampされます。FREE_RUNでPure Pursuitが要求した
加速を1.3 m/s²で頭打ちにしないため、専用値は `2.0 m/s²` です。これはMPCの
`a_max=3.0 m/s²` 未満で、`fallback_decel_min_mps2` と `stop_decel_mps2` は
`-1.5 m/s²` のままです。

Pure Pursuit fallback 側の launch パラメータは以下にあります。

```text
aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/control/pure_pursuit.launch.xml
```

## パラメータ一覧

| パラメータ | デフォルト | 内容 |
| --- | --- | --- |
| `enabled` | `true` | hybrid control mux の切り替え処理を有効にする。false の場合は MPC が fresh なら MPC、そうでなければ stop を出す |
| `primary_source` | `mpc` | 最終制御の主ソース。`mpc` は従来Hybrid、`pure_pursuit` はPurePursuit主制御でMPCをhorizon生成役にする |
| `control_rate_hz` | `50.0` | mux が出力判定を行う周期 |
| `mpc_cmd_timeout_sec` | `0.12` | MPC 指令を fresh とみなす最大時間 |
| `pure_pursuit_cmd_timeout_sec` | `0.20` | Pure Pursuit 指令を fresh とみなす最大時間 |
| `lateral_stop_steering_hold_timeout_sec` | `0.12` | 認可済み横軌道のplan/status到着差中に、最後の検証済み操舵を保持できる上限時間 [s] |
| `planner_stop_release_bootstrap_max_speed_mps` | `0.20` | current-d STOPのPP処理証明を次のPASS warm-up発行へ使える最大速度上限 [m/s] |
| `mpc_health_timeout_sec` | `0.75` | MPC health を有効とみなす最大時間 |
| `fallback_trigger_infeasible_count` | `1` | Pure Pursuit へ切り替えるために必要な連続 infeasible 回数 |
| `fallback_release_solved_cycles` | `3` | MPC へ復帰するために必要な連続 solved 回数 |
| `fallback_min_hold_sec` | `1.0` | Pure Pursuit フォールバックを最低限維持する時間 |
| `fallback_speed_mps` | `10.0` | フォールバック中の速度上限 |
| `fallback_accel_max_mps2` | `1.3` | フォールバック中の加速度上限 |
| `fallback_decel_min_mps2` | `-1.5` | フォールバック中の減速度下限 |
| `stop_decel_mps2` | `-1.5` | stop 指令を出すときの加速度 |
| `use_pure_pursuit_on_mpc_cmd_timeout` | `true` | MPC 指令が timeout したときに Pure Pursuit へ切り替える |
| `use_pure_pursuit_on_mpc_health_timeout` | `false` | MPC health が timeout したときに Pure Pursuit へ切り替える |
| `use_mpc_on_pure_pursuit_cmd_timeout` | `false` | `primary_source=pure_pursuit` でPP指令がtimeoutした時、MPCがhealthyなら一時的にMPC指令へ退避する |
| `debug_publish_period_sec` | `0.25` | debug JSON を publish する周期 |
| `enable_steering_rate_limit` | `true` | 最終出力の操舵角レート制限を有効にする |
| `max_steering_angle_rad` | `0.64` | `vehicle_info`と共通化した最終出力のhard操舵角上限 [rad] |
| `tracking_usable_max_steering_angle_rad` | `0.64` | `ControllerTrackingStatus`をusableにできるraw PP操舵角上限 [rad] |
| `max_steering_rate_radps` | `128.0` | 最終出力の有限な操舵角変化率上限 [rad/s] |
| `max_steering_delta_per_cycle` | `0.0` | 1周期あたりの追加操舵変化量上限 [rad]。0以下なら無効 |
| `steering_limiter_reset_dt_sec` | `0.50` | 前回出力からこの時間を超えたら操舵レート制限をリセットする |
| `reset_steering_limiter_on_mode_change` | `false` | true なら MPC/Pure Pursuit/stop の source 切替時に操舵レート制限をリセットする |
| `steering_log_throttle_sec` | `1.0` | 操舵制限ログの最短出力間隔 |

## 切り替え感度に関係するパラメータ

### `primary_source`

`mpc` の場合は従来どおり、MPCを主制御にして infeasible / timeout 時だけPure Pursuitへ落とします。

`pure_pursuit` の場合は、Pure Pursuitを主制御にします。この時MPCの制御指令は通常は最終出力に使わず、Pure Pursuitが追従する予測ホライズンを作る役になります。MPCがinfeasibleになっても、Pure Pursuitは通常trajectoryへ戻って走行を継続できます。

`pure_pursuit` 主制御では、`fallback_active=false` のまま `source=pure_pursuit` になります。debugで「常にfallbackなのか」を誤読しないためです。

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

制御周期に対して短すぎると、正常な publish 揺らぎでも timeout する可能性があります。`0.12` は、MPC の通常 solve が 60ms を超える場面でも即座に Pure Pursuit へ落ちすぎないようにした値です。

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

`pure_pursuit_mpc_horizon` では同じ値をPurePursuit主制御の速度上限として使います。launch側の `pure_pursuit_speed_mps` とこの値を大きくずらすと、PurePursuitが出した速度をmux側でclampするため、意図より遅くなることがあります。

## 操舵連続性に関係するパラメータ

### `planner_stop_release_bootstrap_max_speed_mps`

停止中のPASS再認可は、current-dのSAFE_STOPまたは停止中ATTACK_FOLLOWを同一generationの
Pure Pursuitが処理できたことを確認してから、新tokenのPASS軌道を停止constraint下でwarm-up
する二段階契約です。この値を超えるSTOP constraintやPP指令、`release_authorized=true`、
契約外reason、targetなし、世代・stamp不一致は一段目proofへ使いません。
`maneuver_transaction_tracking_stop`をrelease gateが連続確認中の
`release_pending_safe_cycles`へ置換した場合も、停止要求・identityが厳密一致し、constraintと
PP指令がそれぞれこの上限以下の時だけ同じ一段目proofとして扱います。停止中のconstraint
上限は現在速度へ連続接続するため`0.20 m/s`未満へ縮むことがありますが、PP warm-up指令まで
その瞬間値以下である必要はありません。両者はこの専用上限で独立に拘束され、最終出力は
STOPのまま`0 m/s`に合成されます。既定値`0.20 m/s`はOvertake Plannerの
`safe_stop_v_mps`と一致します。このproof自体は停止を解除せず、新PASSの発行後に別途その
PASS自身のexact tracking proofが必要です。

### `lateral_stop_steering_hold_timeout_sec`

認可済み横軌道の停止constraintへ切り替わる際、planとPP command/statusの配送が1周期だけ
前後しても操舵を0へresetしないための上限時間です。保持できるのは最後に同一stamp・同一
generationで検証済みの操舵だけです。最新typed statusがusableで、現在または直前のplan
generationである配送差に限定します。同一generationの横STOP再送におけるplan/constraint
peer差は、stamp差がこの時間内で、直前の現在または直前generationをexact tracking済みの
場合だけ対象です。明示unusable、2世代以上の遅れ、外部停止、watchdog、control faultでは
適用しません。通常`FREE_RUN`のtracking reserve微小超過時は、同一generation・race epochで
直前に実publish済みのreserve内baseline操舵だけをこの時間内で保持できます。超過した現在値、
PASS、ATTACK_FOLLOW、ABORT_HOLD、横overrideには使いません。保持中も縦速度は常に0です。

### `enable_steering_rate_limit`

最終的に `/control/command/control_cmd` へ出す操舵角の急変を抑えます。MPC から Pure Pursuit へ切り替わる瞬間や、stop へ落ちる瞬間にも同じ limiter が効きます。

### `max_steering_angle_rad`

最終出力の絶対操舵角hard上限です。`vehicle_info`の
`max_steer_angle=0.64 rad`と共通化し、最後段のclampとして残します。

### `tracking_usable_max_steering_angle_rad`

raw Pure Pursuit指令を`trajectory_tracking_usable=true`にできる絶対操舵角上限です。hard上限以下の有限正値でなければfail-closedにします。境界値はusableですが、次の表現可能値を含め上限を超えた選択中PP指令はtracking usableへ昇格させず、通常motion、PASS/HOLD横追従、release-readyを認可しません。通常はゼロ速度・ゼロ操舵です。exactな`FREE_RUN` baselineだけは、現在値がhard上限内で、同一generation・plan stamp・race epochの直前にreserve内の操舵を実publish済みなら、その過去値を短時間保持して縦STOPへ合成し、reasonを`baseline_tracking_reserve_longitudinal_stop`にします。超過した現在値、stale/fault、PASS/HOLDには従来どおり`steering_command_exceeds_actuator_limit`のzero-steer STOPを適用します。NaN/Infまたはhard超過は保持値も即時失効させ、新しいreserve内commandの検証・publish前には再利用しません。非選択のPP入力は別sourceを停止させません。絶対角判定とrate limitは別契約です。

### `max_steering_rate_radps`

1秒あたりに許す操舵角変化量です。小さくすると切替時の急操作を抑えますが、低すぎると必要な旋回に追従できません。

デフォルトは `128.0 rad/s` です。100 HzのPPでは、許容範囲の端
`-0.64 rad`から反対端`+0.64 rad`までを1周期で許します。limiter構造は
維持しているため、実車・路面・天候に応じて設定値を下げられます。

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

### Pure Pursuit主制御でMPCをhorizon生成だけに使いたい

```yaml
primary_source: "pure_pursuit"
use_mpc_on_pure_pursuit_cmd_timeout: false
fallback_speed_mps: 10.0
```

この設定では、PP指令がfreshなら常にPPを最終出力にします。PP指令が止まった時は停止します。`use_mpc_on_pure_pursuit_cmd_timeout=true` にすると、PP指令が止まった時だけhealthyなMPC指令へ退避できますが、制御ソースが急に変わるため基本はfalse推奨です。

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
