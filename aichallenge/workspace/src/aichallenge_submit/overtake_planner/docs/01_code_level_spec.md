# overtake_planner コードレベル仕様書

この文書は、`overtake_planner` のコードを読むための地図です。
実装の目的は、MPC本体を大きく変えずに、V2Xで見える前走車や横並び車両に対して、短いhorizonの横オフセット列と速度上限列を `/overtake/reference_override` へ出すことです。

## 入口

パッケージはここです。

```text
aichallenge/workspace/src/aichallenge_submit/overtake_planner/
```

最初に読む順番は次がおすすめです。

1. `include/overtake_planner/types.hpp`
2. `src/overtake_planner_node.cpp`
3. `src/overtake_planner_core.cpp`
4. `src/behavior_state_machine.cpp`
5. `src/safety_evaluator.cpp`
6. `src/frenet_frame.cpp`

## 実行時の入出力

ROSノードは `overtake_planner_node` です。

購読:

- `/localization/kinematic_state`
  - `nav_msgs/msg/Odometry`
  - 自車の位置、yaw、速度を読む
- `/v2x/vehicle_positions`
  - `v2x_msgs/msg/V2XVehiclePositionArray`
  - 他車位置を読む

配信:

- `/overtake/reference_override`
  - `std_msgs/msg/Float32MultiArray`
  - MPCへ渡す軽量override
- `/debug/overtake/mode`
  - `std_msgs/msg/String`
  - 状態機械の現在モード
- `/debug/overtake/metrics`
  - `std_msgs/msg/String`
  - eval wrapperやレポート用のJSON

`/overtake/reference_override` の配列形式:

```text
[valid, mode_id, n, d[0], ..., d[n-1], v_ref[0], ..., v_ref[n-1]]
```

- `valid`: 現状は常に `1.0`
- `mode_id`: `BehaviorMode` の整数値
- `n`: overrideが有効ならhorizon点数、無効なら0
- `d`: Frenet横方向オフセット列
- `v_ref`: 各点の速度上限列

## 主要データ構造

定義は `include/overtake_planner/types.hpp` にあります。

### EgoState

自車状態です。

- `x`, `y`, `yaw`, `v`
- `frenet.s`, `frenet.d`
- `valid`

`valid=false` のとき、plannerは介入しません。

### OpponentState

V2Xから得た他車状態です。

- `id`
- `x`, `y`
- `vx`, `vy`, `v`
- `frenet.s`, `frenet.d`
- `valid`

速度はV2X位置の前回値との差分から推定します。

### BlockedInfo

前走車、横並び、追い越し可能幅、壁余裕をまとめた判断入力です。

重要フィールド:

- `blocked`
  - 同一コリドーの前方車両が近い、または詰まりつつある
- `side_by_side`
  - 縦方向差が小さく、横方向も近い
- `corner_side_by_side`
  - 横並びかつ前方短距離の曲率が大きい
- `front_delta_s`
  - 前走車とのFrenet縦方向距離
- `side_delta_s`
  - 横並び対象との差。正なら相手が前
- `ego_wall_clearance_m`
  - 自車の横位置が安全コリドーからどれだけ余裕を持つか
- `can_pass_left`, `can_pass_right`
  - 左右に追い越し可能な幅があるか
- `pass_gap_reason`
  - `ok`, `left_gap_narrow`, `right_gap_narrow`, `both_gap_narrow`, `no_target`

### CandidateTrajectory

MPCへ渡す候補軌道です。

- `type`
- `t`, `s`, `d`, `x`, `y`, `yaw`
- `v_ref`
- `feasible`
- `reject_reason`
- `min_safety_margin`
- `cbf_slack`

候補は作ったあとに `SafetyEvaluator` で壁と他車の安全性を評価します。

## 1周期の処理

中核は `OvertakePlannerCore::update()` です。

流れ:

1. 無効状態を早期return
   - `enabled=false`
   - `ego.valid=false`
   - 参照線なし
2. `detectBlocked()`
   - 前走車と横並び車両をFrenet座標で検出
3. `predictOpponents()`
   - 他車を短いhorizonで等速予測
4. `evaluatePassGap()`
   - 左右の追い越し可能幅を評価
5. 曲率と壁余裕を追加
   - `corner_abs_curvature`
   - `corner_side_by_side`
   - `ego_wall_clearance_m`
6. 候補軌道を生成
   - `FASTEST`
   - 必要に応じて `FOLLOW`, `PASS_LEFT`, `PASS_RIGHT`, `SIDE_BY_SIDE_KEEP`, `YIELD_BEHIND`, `RECOVERY`
   - 通常fallbackが成立しないときだけ `SAFE_STOP`
7. 各候補を安全評価
8. 候補スコアで1つ選ぶ
9. `BehaviorStateMachine` でモードを安定化
10. モードに合わせて候補を再生成
11. `PlannerOutput` に詰める

ポイントは、候補選択と状態遷移が分かれていることです。
候補は「今周期の良さそうな選択肢」、状態機械は「急に切り替えないための運転モード」です。

## BehaviorMode

`BehaviorMode` は状態機械のモードです。

- `FREE_RUN`
  - overrideなし。MPCの元参照を使う
- `FOLLOW_BLOCKED`
  - 前走車へ追従する。速度上限を下げる
- `PREPARE_OVERTAKE_LEFT`
  - 左追い越しへ入る準備
- `PREPARE_OVERTAKE_RIGHT`
  - 右追い越しへ入る準備
- `OVERTAKE_LEFT`
  - 左オフセットを維持
- `OVERTAKE_RIGHT`
  - 右オフセットを維持
- `MERGE_BACK`
  - 中心線へ戻る
- `ABORT_RECOVERY`
  - 危険候補や壁際から中心側へ戻る
- `SIDE_BY_SIDE_KEEP`
  - 横並び時に相手から距離を取る
- `YIELD_BEHIND`
  - 相手の後ろへ入るために減速する
- `SAFE_STOP`
  - 回避、追従、譲り、復帰が安全に成立しないとき、正の低速capで停止意図を出す

## CandidateType

`CandidateType` は毎周期評価する候補です。

- `FASTEST`
  - 横位置は現在dからそのまま。速度上限は実質通過
- `FOLLOW`
  - 中心方向へ寄せ、前走車より少し低い速度上限
- `PASS_LEFT`
  - `left_offset_m` へ横移動
- `PASS_RIGHT`
  - `right_offset_m` へ横移動
- `RECOVERY`
  - 中心線へ戻る
- `SIDE_BY_SIDE_KEEP`
  - 相手と横方向距離を保つ
- `YIELD_BEHIND`
  - 後ろに入るための速度上限と横目標を作る
- `SAFE_STOP`
  - 現在dを安全コリドー内にclampし、`safe_stop_v_mps` の速度上限を全点へ入れる

## 状態遷移の要点

実装は `src/behavior_state_machine.cpp` です。

主なルール:

- unsafeな候補なら基本は `ABORT_RECOVERY`
- 前方閉塞なら `FOLLOW_BLOCKED`
- PASS候補は `pass_safe_required_cycles` だけ連続安全になってから準備へ移る
- 横並びで `SIDE_BY_SIDE_KEEP` が選ばれたら横距離維持
- 相手が少し前へ出たら `YIELD_BEHIND`
- `YIELD_BEHIND` は前方ギャップと壁余裕が戻るまで解除しない
- `ABORT_RECOVERY` も壁余裕が戻るまで解除しない
- `SAFE_STOP` は通常fallback候補が全てunsafeな周期が `safe_stop_trigger_cycles` 続いたときだけ入る
- `SAFE_STOP` 中は停止候補がfeasibleな間だけoverrideを出し、解除条件が `safe_stop_release_cycles` 続くまで保持する
- `SAFE_STOP` 候補が途中でunsafeになった場合は、`SAFE_STOP` に居座らず通常fallback側へ戻す

最後のルールが重要です。
壁外または壁際で `RECOVERY -> FREE_RUN -> FASTEST wall_margin -> RECOVERY` と揺れると、速度指令やステアがチャタります。
そのため `ego_wall_clearance_m < yield_rejoin_wall_clearance_m` の間は `ABORT_RECOVERY` を保持します。

## 横並びとコーナーの扱い

通常の横並びでは `SIDE_BY_SIDE_KEEP` を使います。
ただしコーナー中はFrenetのd方向がカーブの法線方向になるため、横へ逃げるつもりが壁側へ膨らむことがあります。

そこで以下の判定を入れています。

```text
corner_side_by_side =
  side_by_side &&
  maxAbsCurvatureAhead(s, corner_side_yield_lookahead_m) >= corner_side_yield_curvature_m_inv
```

`corner_side_by_side=true` のときは、横並び維持より `YIELD_BEHIND` を優先します。
このとき `YIELD_BEHIND` の横目標は相手のdではなく、`corner_yield_target_d_m`、デフォルト中心線 `0.0` です。

## 壁余裕

安全コリドーは次で決まります。

```text
lower_d = d_min_m + min_wall_margin_m
upper_d = d_max_m - min_wall_margin_m
```

`ego_wall_clearance_m` は、自車dがこの範囲からどれだけ余裕を持つかです。

```text
min(ego_d - lower_d, upper_d - ego_d)
```

負なら自車は安全コリドー外です。

`RECOVERY` と `YIELD_BEHIND` の参照生成では、開始dを安全コリドー内にclampします。
これは壁外d列をMPCへ渡さないためです。

## 安全評価

実装は `src/safety_evaluator.cpp` です。

評価順:

1. 壁マージン
   - d列が安全コリドー外なら `reject_reason="wall_margin"`
2. 他車との楕円距離
   - 自車候補点と他車予測点を同じhorizon indexで比較
   - 閾値未満なら `reject_reason="opponent_collision"`

楕円距離は車体前後方向を広めに見ます。

```text
h = (x_body / safety_ellipse_a_m)^2
  + (y_body / safety_ellipse_b_m)^2
  - 1
```

`h <= min_ellipse_h` なら不可です。

## 速度上限

候補ごとに `v_ref` が変わります。

- `FASTEST`
  - `v_passthrough_mps`
- `FOLLOW`
  - `opponent.v - follow_speed_margin_mps`
- `SIDE_BY_SIDE_KEEP`
  - `side_by_side_speed_cap_mps`
  - 相手速度より `yield_speed_margin_mps` だけ低い値も見る
- `YIELD_BEHIND`
  - 通常は `opponent.v - yield_speed_margin_mps`
  - コーナー横並び中は `opponent.v - corner_follow_speed_margin_mps`
- `RECOVERY`
  - 通常は `recovery_v_max_mps`
  - 壁外なら `wall_margin_recovery_v_max_mps`

## 主要パラメータ

設定は `config/overtake_planner.param.yaml` です。

横並び:

- `side_by_side_s_m`
- `side_margin_m`
- `side_yield_s_m`
- `side_by_side_target_gap_m`
- `side_by_side_shift_distance_m`
- `side_by_side_speed_cap_mps`

コーナー横並び:

- `corner_side_yield_curvature_m_inv`
- `corner_side_yield_lookahead_m`
- `corner_side_yield_wall_clearance_m`
- `corner_yield_target_d_m`
- `corner_yield_rejoin_gap_m`
- `corner_follow_speed_margin_mps`

壁と復帰:

- `d_min_m`
- `d_max_m`
- `min_wall_margin_m`
- `yield_rejoin_wall_clearance_m`
- `recovery_v_max_mps`
- `wall_margin_recovery_v_max_mps`

追い越し:

- `min_pass_gap_m`
- `pass_gap_hysteresis_m`
- `pass_safe_required_cycles`
- `left_offset_m`
- `right_offset_m`
- `prepare_distance_m`
- `merge_distance_m`
- `merge_front_gap_m`
- `abort_timeout_sec`

## デバッグの見方

まず `report/overtake.html` と `processed/overtake_metrics.json` を見ます。

細かく見るときは `processed/overtake_timeseries.csv` の次の列が重要です。

- `overtake_state`
- `selected`
- `blocked`
- `side_by_side`
- `corner_side_by_side`
- `front_distance_m`
- `side_delta_s`
- `ego_lateral_offset`
- `target_lateral_offset_m`
- `active_override`
- `reason`
- `min_cbf_h`
- `cbf_slack`
- `mpc_infeasible_count`

`autoware.log` では次を探します。

```text
overtake decision:
```

このログには、モード、選択候補、前走車、横並び、曲率、自車d、目標d、CBF余裕、reject理由がまとまっています。

よく見るパターン:

- `side_by_side=true` なのに `active_override=false`
  - 横並びを状態機械が拾えていない可能性
- `corner_side_by_side=true` で `selected=SIDE_BY_SIDE_KEEP`
  - コーナー横並び抑制が効いていない可能性
- `mode=ABORT_RECOVERY` と `mode=FREE_RUN` が交互
  - 壁余裕が戻る前にFREE_RUNへ戻っている可能性
- `reason=wall_margin`
  - 候補d列が安全コリドー外
- `cbf_slack > 0`
  - 他車との楕円安全制約を割っている

## テスト

主なテストは以下です。

- `test/test_frenet_frame.cpp`
  - Frenet座標変換
- `test/test_safety_evaluator.cpp`
  - 壁と他車楕円の安全評価
- `test/test_state_machine.cpp`
  - 状態遷移
- `test/test_overtake_planner_core.cpp`
  - 候補生成から出力までの統合的な挙動

実行例:

```bash
source /opt/ros/humble/setup.bash
source /autoware/install/setup.bash
cd /aichallenge/workspace
colcon build --packages-select overtake_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select overtake_planner --event-handlers console_direct+
colcon test-result --verbose
```

Docker compose環境から実行する場合:

```bash
CMD='source /opt/ros/humble/setup.bash && source /autoware/install/setup.bash && cd /aichallenge/workspace && colcon build --packages-select overtake_planner --cmake-args -DCMAKE_BUILD_TYPE=Release && colcon test --packages-select overtake_planner --event-handlers console_direct+ && colcon test-result --verbose' docker compose run -T --rm --no-deps autoware-command
```
