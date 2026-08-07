# overtake_planner コードレベル仕様書

この文書は、`overtake_planner` のコードを読むための地図です。
実装の目的は、MPC本体を大きく変えずに、V2Xで見える前走車や横並び車両に対して、短いhorizonの横オフセット列と速度上限列を `/overtake/reference_override` へ出すことです。

より細かいソースファイル別の実装ロジックは `docs/06_source_logic_walkthrough.md` にまとめています。

## 入口

パッケージはここです。

```text
aichallenge/workspace/src/aichallenge_submit/overtake_planner/
```

最初に読む順番は次がおすすめです。

1. `include/overtake_planner/types.hpp`
2. `src/overtake_planner_node.cpp`
3. `src/overtake_planner_core.cpp`
4. `src/blocked_risk_analyzer.cpp`
5. `src/future_side_by_side_risk_analyzer.cpp`
6. `src/candidate_builder.cpp`
7. `src/planner_output_builder.cpp`
8. `src/behavior_state_machine.cpp`
9. `src/safety_evaluator.cpp`
10. `src/frenet_frame.cpp`

## 実行時の入出力

ROSノードは `overtake_planner_node` です。
`horizon_points` はMPC horizonと揃える必要があります。
基本YAMLとbaseline `mpc.launch.xml` は20点、`delay_aware_mpc.launch.xml` はdelay-aware MPCの `N: 50` に合わせてlaunch引数で50点へ上書きします。

購読:

- `/localization/kinematic_state`
  - `nav_msgs/msg/Odometry`
  - 自車の位置、yaw、速度を読む
- `/v2x/vehicle_positions`
  - `v2x_msgs/msg/V2XVehiclePositionArray`
  - 他車位置を読む
- `/mpc/speed_profile_debug`
  - `std_msgs/msg/String`
  - MPCのinfeasible回数とsolve timeを読む。古い値は `mpc_health_stale_time_sec` でstale guardとして扱い、速度を落とす

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

`/overtake/reference_override` は次の3形式を使います。

```text
v3 lateral: [1, mode_id!=0, n>0, d[0], ..., d[n-1], v_ref[0], ..., v_ref[n-1], 3, generation, solver_horizon_intent]
v2 speed-only: [1, mode_id!=0, 0, 2, generation, speed_cap_mps]
explicit inactive: [1, 0, 0, 1, generation]
```

- `valid`: 現状は常に `1.0`
- `mode_id`: `BehaviorMode` の整数値
- `n`: v3 lateralではhorizon点数、v2 speed-onlyとexplicit inactiveでは0
- `d`: Frenet横方向オフセット列
- `v_ref`: v3の各点の即時速度上限列。v2は単一の `speed_cap_mps` を全horizonへ適用する。応答遅れ・制動上限を含む安全予測は `s(t)` と `predicted_speed_mps` に分離する
- `contract_version`: explicit inactiveは `1`、横軌道を持たない速度guardは `2`、横軌道とsolver認可を持つpayloadは `3`
- `solver_horizon_intent`: `0` は通常trajectoryだけで使う横列、`1` はPASS/MERGEのsolver horizon認可、`2` は全車両・壁評価済みの必須回避。通常の`ABORT_RECOVERY`は `0` であり、mode番号だけではsolver horizonを認可しない
- `override_generation`: payloadが変わった時に更新する1以上の世代。MPC solver horizonがどのplanner requestで解かれたかをPP/eval解析が照合する

controllerはexplicit inactiveを受けた時だけoverrideを解除します。最後に受理したv2 speed-onlyの後でpayloadが不正になった、または0.50秒でtimeoutした場合は、横軌道を再利用せずbaselineのまま最後の有効speed capを保持します。v1/v3 lateral overrideは不正payload/timeoutでclearします。

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
- `parallel_side_candidate`
  - 通常の横並びより広い範囲で、未来コーナー予測に渡す並走相手がいる
- `front_delta_s`
  - 前走車とのFrenet縦方向距離
- `side_delta_s`
  - 横並び対象との差。正なら相手が前
- `ego_wall_clearance_m`
  - 自車の横位置が安全コリドーからどれだけ余裕を持つか
- `can_pass_left`, `can_pass_right`
  - 左右に追い越し可能な幅があるか
- `straight_overtake_start_allowed`
  - 直線限定追い越し開始ゲートが開いているか
- `overtake_start_abs_curvature`
  - 追い越し開始ゲートが見ている前方曲率の最大値
- `overtake_start_gate_reason`
  - `curve` なら、gapはあっても曲率により追い越し開始を止めている
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
2. `BlockedRiskAnalyzer::detectBlocked()`
   - 前走車と横並び車両をFrenet座標で検出
3. `predictOpponents()`
   - 他車を短いhorizonで等速予測
4. `promoteSlowObstacleChain()`
   - 低速/停止のparallel-side前方車を、停止車列の次の前方閉塞対象へ昇格
5. `BlockedRiskAnalyzer::evaluatePassGap()`
   - 左右の追い越し可能幅を評価
6. 曲率、壁余裕、未来横並びリスクを追加
   - `corner_abs_curvature`
   - `corner_side_by_side`
   - `straight_overtake_start_allowed`
   - `overtake_start_abs_curvature`
   - `overtake_permission_allowed`
   - `overtake_permission_section_name`
   - `overtake_permission_reason`
   - `slow_front_exception_active`
   - `slow_obstacle_chain_active`
   - `ego_wall_clearance_m`
   - `FutureSideBySideRiskAnalyzer::evaluate()`
   - `future_side_by_side`
   - `future_outer_wall_risk`
   - `future_yield_required`
7. 大きい横ずれ中の危険文脈では追い越し判断を凍結
   - `can_pass_left/right=false`
   - `pass_decision_frozen=true`
   - `RECOVERY` 候補を優先
8. 候補軌道を生成
   - `FASTEST`
   - 必要に応じて `FOLLOW`, `PASS_LEFT`, `PASS_RIGHT`, `SIDE_BY_SIDE_KEEP`, `YIELD_BEHIND`, `RECOVERY`
   - 通常fallbackが成立しないときだけ `SAFE_STOP`
9. 各候補を安全評価
10. 候補スコアで1つ選ぶ
10. `BehaviorStateMachine` でモードを安定化
   - `straight_overtake_start_allowed=false` のときは、PASS候補が安全でも `PREPARE_OVERTAKE_LEFT/RIGHT` へ入らず `FOLLOW_BLOCKED` を維持する
11. モードに合わせて候補を再生成
12. 危険候補を復活させない速度ガードを適用
13. `PlannerOutputBuilder` で `PlannerOutput` に詰める
14. `OvertakePlannerCore` でpublish直前の横オフセット列にレート制限をかける

ポイントは、候補選択と状態遷移が分かれていることです。
候補は「今周期の良さそうな選択肢」、状態機械は「急に切り替えないための運転モード」です。

## 責務分割

`OvertakePlannerCore` は処理順を制御するオーケストレータです。
前走車判定、未来予測、候補生成、状態遷移、出力整形を直接抱え込まず、次の小さなクラスへ分けています。

### BlockedRiskAnalyzer

`src/blocked_risk_analyzer.cpp` にあります。

入力:

- `EgoState`
- `std::vector<OpponentState>`
- `std::vector<PredictedOpponent>`
- `BehaviorMode`
- `PlannerConfig`
- `FrenetFrame`

出力:

- 更新済みの `BlockedInfo`

担当すること:

- staleな相手を除外する
- 逆走方向の相手を必要に応じて無視する
- 前方車、横並び、parallel side candidateを検出する
- `blocked` を前方距離と相対速度から決める
- 左右pass gap、`can_pass_left/right`、`pass_gap_reason` を決める
- `wallClearance()` を提供する

担当しないこと:

- 状態遷移
- 候補軌道生成
- section safety profile
- MPC overrideの出力整形

### FutureSideBySideRiskAnalyzer

`src/future_side_by_side_risk_analyzer.cpp` にあります。
`BlockedRiskAnalyzer` で見つけた `side_by_side` または `parallel_side_candidate` を入力にして、短い時間先のコーナー横並びリスクを評価します。

維持する契約:

- `future_side_by_side`
- `future_corner_side_by_side`
- `future_outer_wall_risk`
- `future_yield_required`
- `future_delta_s/d`
- `future_wall_clearance_m`
- `yield_reason`

`yield_reason` は既存互換のため、外壁余裕が閾値未満なら主に `future_outer_wall_risk` を入れます。
この判定は `BlockedInfo` を壊さずに追記する層で、状態機械を直接動かしません。

### CandidateBuilder

`src/candidate_builder.cpp` にあります。
`FASTEST`, `FOLLOW`, `PASS_LEFT/RIGHT`, `RECOVERY`, `SIDE_BY_SIDE_KEEP`, `YIELD_BEHIND`, `SAFE_STOP` のd列と速度上限列を作ります。
候補が安全かどうかはここでは確定せず、後段の `SafetyEvaluator` が判定します。

### PlannerOutputBuilder

`src/planner_output_builder.cpp` にあります。
選ばれた候補、`BlockedInfo`、safe stop文脈、section profile、MPC healthを `PlannerOutput` へ詰めます。
速度ガードやMPC override契約はここで集約します。
そのため、risk判定側は `SPEED_GUARD` の出力形式を知らなくてよい構造です。
`pass_horizon_publish_mode=overtake_only` では、`BehaviorStateMachine` へ渡す内部候補は `PASS_LEFT/RIGHT` のまま維持しますが、`PREPARE_OVERTAKE_*` 中に `PlannerOutputBuilder` へ渡すpublish用候補だけ `FOLLOW` へ差し替えます。
これにより `pass_safe_required_cycles` は従来どおり貯まり、MPCへ追い越し横オフセットを出すのは `OVERTAKE_LEFT/RIGHT` に入った周期からになります。
横軌道を安全評価できない `SPEED_GUARD` では、`active_override=false` と空の `lateral_offsets` を使い、v2 speed-onlyで減速だけを下流へ渡します。これにより横列を推測・0埋めしてMPCへ渡しません。
安全評価済みの横列がある場合だけ、`OvertakePlannerCore` が前回publishした横オフセット列との差分を `lateral_target_max_step_m` で制限します。
高速カーブ中の `YIELD_BEHIND`, `ABORT_RECOVERY`, `SAFE_STOP`, `SPEED_GUARD` で横軌道が有効な場合は、rate limit後の横オフセット列を `high_speed_curve_lateral_hold_*` 条件でholdし、低速化またはカーブ脱出まで短周期の再選択を抑えます。
PASSでは、この最終 `d[]` を再度SafetyEvaluatorへ通します。unsafeならoverrideを無効化して通常速度へ戻さず、同じ周期にRECOVERY、さらに不可ならspeed-only fallbackを出します。

`BlockedRiskAnalyzer` と `FutureSideBySideRiskAnalyzer` への責務分割そのものでは、新しいYAMLパラメータは追加していません。
この文書では、分割前から入っているfuture side prediction、parallel side、straight-only gate、speed guard、MPC health guard系のパラメータもあわせて説明しています。
既存の `BlockedInfo` フィールドとYAMLパラメータの意味を保ったまま、今回の分割では責務だけを分けています。

## BehaviorMode

`BehaviorMode` は状態機械のモードです。

- `FREE_RUN`
  - overrideなし。MPCの元参照を使う
- `FOLLOW_BLOCKED`
  - 前走車へ追従する。速度上限を下げる
- `PREPARE_OVERTAKE_LEFT`
  - 左追い越しへ入る準備。直線限定ゲートが閉じていると入らない
- `PREPARE_OVERTAKE_RIGHT`
  - 右追い越しへ入る準備。直線限定ゲートが閉じていると入らない
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
- `SPEED_GUARD`
  - 横方向候補は出さず、現在横位置を保持するd列と速度capだけをMPCへ渡す。MPCがoverrideを消さないよう `mode_id != 0` にするための出力用モード

## CandidateType

`CandidateType` は毎周期評価する候補です。

- `FASTEST`
  - 横位置は現在dからそのまま。速度上限は実質通過
- `FOLLOW`
  - 中心方向へ寄せ、前走車より少し低い速度上限
- `PASS_LEFT`
  - `minimum_clearance` では相手の必要楕円間隔を満たす最小dへ横移動し、raw profile全区間が回廊内の場合だけ採用
- `PASS_RIGHT`
  - `minimum_clearance` では相手の必要楕円間隔を満たす最小dへ横移動し、raw profile全区間が回廊内の場合だけ採用
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
- `YIELD_BEHIND` は前方ギャップ、壁余裕、横ずれが戻るまで解除しない
- `ABORT_RECOVERY` も壁余裕と横ずれが戻るまで解除しない
- `SAFE_STOP` は通常fallback候補が全てunsafeな周期が `safe_stop_trigger_cycles` 続いたときだけ入る
- `SAFE_STOP` 中は停止候補がfeasibleな間だけoverrideを出し、解除条件が `safe_stop_release_cycles` 続くまで保持する
- `SAFE_STOP` 候補が途中でunsafeになった場合は、`SAFE_STOP` に居座らず通常fallback側へ戻す

最後のルールが重要です。
壁外または壁際で `RECOVERY -> FREE_RUN -> FASTEST wall_margin -> RECOVERY` と揺れると、速度指令やステアがチャタります。
そのため `ego_wall_clearance_m < yield_rejoin_wall_clearance_m`、または `abs(ego_lateral_offset_m) > recovery_release_lateral_error_m` の間は `ABORT_RECOVERY` を保持します。
`YIELD_BEHIND` と `future_yield_hold` は `yield_release_lateral_error_m` も解除条件に含め、横回復が終わる前に `FREE_RUN` へ戻りにくくします。

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

スタートから第1コーナーまでのように、見た目は並走でも `side_margin_m` より横に離れている場面は
`parallel_side_candidate` として記録します。
この候補は通常の `SIDE_BY_SIDE_KEEP` には使わず、未来コーナー予測と `YIELD_BEHIND` の対象選択にだけ使います。
そのため、狭義の `side_by_side` を過剰に広げずに、d2視点で相手が斜め後方にいるケースも先読みできます。

## 壁余裕

回廊CSVの `d_min/d_max` はlanelet路面端です。中心点の互換安全帯は次で決まります。

```text
lower_d = d_min_m + min_wall_margin_m
upper_d = d_max_m - min_wall_margin_m
```

`ego_wall_clearance_m` は、自車中心dがこの範囲からどれだけ余裕を持つかを示す
診断値です。

```text
min(ego_d - lower_d, upper_d - ego_d)
```

負なら自車中心は安全コリドー外です。正でも車体接触なしを意味しません。
最終候補認可では `SafetyEvaluator` が `base_link`基準の前後端・半幅から四隅を
展開し、走行中自己位置余裕を残して全cornerが路面端内かを別途検査します。

`RECOVERY` と `YIELD_BEHIND` の参照生成では、開始dを安全コリドー内にclampします。
これは壁外d列をMPCへ渡さないためです。
`RECOVERY` の横profileは物理前進距離だけで進めます。停止近傍で時間比率を併用すると、極小の前進距離へ大きな横移動が圧縮されて曲率が過大になるためです。`outside_corridor_recovery_centering_time_sec` は設定互換用として残しますが、横profile生成には使用しません。

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

FOLLOW、RECOVERY、SIDE_BY_SIDE_KEEP、YIELD_BEHIND、SAFE_STOPで目標速度が現在速度より低い場合も、公開する `v_ref[0]` は目標capです。下流MPC/PPが各周期に `v_ref[0]` を直接読むため、ここに応答遅れを載せると減速要求が毎周期リセットされるからです。

安全評価用の `s(t)` と `predicted_speed_mps(t)` には、`longitudinal_response_delay_sec` の等速区間と、その後 `max_brake_decel_mps2` を超えない制動を残します。従って予測上の最遠到達距離を甘くせず、実際の速度capだけを即時に要求します。

- `FASTEST`
  - `v_passthrough_mps`
- `FOLLOW`
  - 通常は `opponent.v - follow_speed_margin_mps`
  - `follow_gap_closing_*` が有効で、同一コリドーの通常前走車とのgapが
    `follow_gap_closing_engage_gap_m` 以上、入力fresh、MPC健全、横並び/譲り/
    停止低速障害物でない時だけ、最大 `follow_gap_closing_max_speed_bonus_mps` を加える
  - 実運用では `follow_gap_closing_engage_gap_m` を目標車間と同じ `4.5 m` にし、
    目標車間を超えたら即座にbonusを計算する。固定距離で加速を許可するのではなく、
    bonusを含む予測軌道がSafetyEvaluatorを通ることを必須とする
  - bonusを使う候補の `s(t)` は
    `follow_gap_closing_assumed_accel_mps2` で即時加速する最遠到達距離として
    SafetyEvaluatorへ渡し、通らなければ候補を採用しない
- `SIDE_BY_SIDE_KEEP`
  - `side_by_side_speed_cap_mps`
  - 相手速度より `yield_speed_margin_mps` だけ低い値も見る
- `YIELD_BEHIND`
  - 通常は `opponent.v - yield_speed_margin_mps`
  - コーナー横並び中は `opponent.v - corner_follow_speed_margin_mps`
  - 未来横並びの外壁リスクがある場合も、中心寄りの `corner_yield_target_d_m` を使う
- `RECOVERY`
  - 通常は `recovery_v_max_mps`
  - 壁外なら `wall_margin_recovery_v_max_mps`
  - 大きい横ずれ中に壁リスク、閉塞、横並び、未来横並びがある場合は、追い越し/横並び候補より優先する

## 速度ガード層

速度ガードは候補生成の後段で動きます。
目的は、`SafetyEvaluator` がrejectした横方向候補を復活させず、MPCへ「横へ逃げる参照」ではなく「速度だけ落とす参照」を渡すことです。

発動条件:

- `speed_only_fallback_enabled`
  - `SIDE_BY_SIDE_KEEP` や `YIELD_BEHIND` が `opponent_collision` などでrejectされた場合、または `SAFE_STOP` 候補がunsafeな場合、現在横位置を保持するd列と低い `v_ref` を出す
  - reject理由が `opponent_collision` の場合は、通常の `speed_only_fallback_v_max_mps` より `opponent_collision_fallback_v_max_mps` を優先し、安全ゲートで接触へ進み続けないようにする
- `wall_risk_speed_guard_enabled`
  - 自車の壁余裕が `wall_soft_margin_m` 未満なら `wall_risk_v_max_mps` へ絞る
- `mpc_health_speed_guard_enabled`
  - `/mpc/speed_profile_debug` の `mpc_infeasible_count`、`mpc_solve_time_ms`、またはstale状態で `mpc_health_v_max_mps` へ絞る
- `recovery_speed_guard_enabled`
  - `RECOVERY` / `ABORT_RECOVERY` 中に壁リスク、MPC health悪化、大きい横ずれのいずれかがある場合、横方向overrideは維持したまま `recovery_speed_guard_v_max_mps` を追加で重ねる
- section safety profile
  - `side_by_side_corner_strict` などの区間プロファイルが有効なら、壁余裕や速度capをさらに厳しくする

複数の速度ガードが同時に成立した場合は、最も低い速度capを使います。
すでにfeasibleな横方向overrideがある場合は、その `v_ref` だけをさらに低くします。
overrideがない、または選ばれた候補がunsafeな場合だけ `SPEED_GUARD` として現在横位置を保持する速度only overrideを出します。

## 主要パラメータ

設定は `config/overtake_planner.param.yaml` です。

横並び:

- `side_by_side_s_m`
- `side_margin_m`
- `parallel_side_detection_enabled`
- `parallel_side_s_m`
- `parallel_side_margin_m`
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

速度ガード:

- `speed_only_fallback_enabled`
- `speed_only_fallback_v_max_mps`
- `opponent_collision_fallback_v_max_mps`
- `wall_risk_speed_guard_enabled`
- `wall_soft_margin_m`
- `wall_risk_v_max_mps`
- `mpc_health_speed_guard_enabled`
- `mpc_health_infeasible_count_threshold`
- `mpc_health_solve_time_warn_ms`
- `mpc_health_v_max_mps`
- `mpc_health_stale_time_sec`

区間安全プロファイル:

- `section_safety_profile_enabled`
- `section_safety_names`
- `section_safety_profiles`
- `section_safety_role_policies`
- `section_safety_start_s_m`
- `section_safety_end_s_m`
- `section_safety_start_wp`
- `section_safety_end_wp`

追い越し許可区間:

- `overtake_permission_profile_enabled`
- `overtake_permission_package`
- `overtake_permission_csv`
- `default_overtake_allowed`
- `overtake_permission_lookahead_m`
- `slow_front_exception_enabled`
- `slow_front_permission_exception_enabled`
- `slow_front_exception_speed_mps`
- `slow_front_exception_distance_m`
- `slow_front_exception_required_cycles`
- `slow_obstacle_chain_enabled`
- `slow_obstacle_chain_distance_m`

壁と復帰:

- `d_min_m`
- `d_max_m`
- `min_wall_margin_m`
- `yield_rejoin_wall_clearance_m`
- `recovery_v_max_mps`
- `wall_margin_recovery_v_max_mps`
- `outside_corridor_recovery_centering_time_sec`

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
- `parallel_side_candidate`
- `front_distance_m`
- `side_delta_s`
- `parallel_side_delta_s`
- `parallel_side_delta_d`
- `ego_lateral_offset`
- `target_lateral_offset_m`
- `active_override`
- `reason`
- `speed_only_fallback_active`
- `wall_risk_speed_guard_active`
- `mpc_health_speed_guard_active`
- `lateral_target_hold_active`
- `lateral_target_hold_reason`
- `speed_cap_reason`
- `applied_speed_cap_mps`
- `active_section_name`
- `active_section_profile`
- `mpc_infeasible_count`
- `mpc_solve_time_ms`
- `min_cbf_h`
- `cbf_slack`

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
- `mode=SPEED_GUARD`
  - 横方向回避ではなく速度only fallbackが出ている
- `speed_cap_reason=mpc_health_infeasible_guard`
  - MPC側のinfeasible増加を受けてplanner側が速度を絞っている

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

今回の分割で守っている契約:

- 前方車と横並びが同じ相手でも `blocked` と `side_by_side` を同時に立てる
- 前方車と横並びが別の相手でも、前方閉塞対象と横並び対象を取り違えない
- `SIDE_BY_SIDE_KEEP` 中に横並びが消えても前方閉塞が残るなら `FOLLOW_BLOCKED` へ戻す
- `follow_trigger_s_m` より遠くても、相対速度で詰まる前方車は `blocked` にする
- 左右どちらのpass gapが失われても、追い越し方向を即反転せず `YIELD_BEHIND` を選ぶ
- 直線限定ゲートが閉じている区間では、gapがあっても追い越し開始へ入らない
- 追い越し許可CSVで `allow_overtake=false` の区間では、gapがあっても追い越し開始へ入らず `FOLLOW_BLOCKED` を維持する
- `allow_overtake=false` でも、`slow_front_permission_exception_enabled=true` で、前方車が停止/低速条件を連続で満たし、現在地点が禁止、SafetyEvaluator通過済みPASS、freshな全入力、横並び/未来譲りなしをすべて満たす場合だけPASS開始を許可する。lookahead先だけの禁止、高曲率、車列昇格は例外化しない
- 1台目通過後に2台目が `parallel_side_candidate` として見える停止車列では、条件を満たす場合だけ `slow_obstacle_chain_active=true` として前方閉塞へ昇格する
- `overtake_permission_lookahead_m` 内に不可区間がある場合は、現在位置が許可区間でも追い越し開始を抑制する
- 大きい横ずれ中は、parallel side candidateや壁リスクがあれば `RECOVERY` を優先する
- `pass_gap_reason` は `no_target`, `ok`, `left_gap_narrow`, `right_gap_narrow`, `both_gap_narrow`, `large_lateral_error` の意味を崩さない
- 複数の速度ガードが同時に成立した場合は、最も低い速度capとその理由を出力する
- V2 shadowの `/overtake/v2/shadow/plan` は、未認可理由を
  `authorization_failure_mask` と `authorization_failure_reasons` に出す。候補欠落、
  SafetyEvaluator未評価/reject、入力stale、tracking不成立、trajectory形状不正、
  constraint停止中を区別する
- V2 shadowの `trajectory_authorized=true` は、SafetyEvaluator通過済みで配信可能な
  trajectoryと、同一plan generationの非停止constraintが揃った周期だけとする。
  release確認中はtrajectory候補が有効でもfalseのままになる
- PASS中にControllerTrackingStatusが一時的なplan/command stamp mismatchだけを示す周期は、
  未認可の現在d保持へ閉じつつ同一attempt、target、side、committed profileを有界回数だけ
  保持する。回復時は同じPASSを再評価し、連続不成立またはMPC stale/hard failureでは
  `ABORT_HOLD`へ移る。target欠測とtracking不成立のhold回数は共有し、交互発生で無期限保持しない
- start-gridの1 generation continuityは、世代差だけでは許可しない。ControllerTrackingStatusの
  header stampとcommand ageがfreshで、PP commandがfresh、mux理由が`ready`または次plan先着だけを
  示す`plan_generation_mismatch`であり、Nodeが保持するcurrent/previous generationの型付き履歴が
  「同じtarget ID・同じPASS sideのcommitted PASS」である場合だけ継続根拠にする。current-d
  stop/FOLLOW世代、別target、2世代以上前、stale、watchdog/stop理由のproofは流用しない。
  final PPが同じ条件で`mpc_horizon_usable=false`を証明した場合、非稼働MPCのhealth stale/solve-timeを
  PASS停止理由にはしないが、候補のSafetyEvaluatorとcontroller trackabilityは毎周期再評価する
- start-grid transactionでtrackingを失い、現在dのSAFE_STOP trajectory自体も他車予測との
  重なりで不成立な場合は、未評価の横軌道を選ばない。`maneuver_transaction_tracking_stop_active`
  により縦停止を要求し、横trackingは無認可のまま、FSMは同じtarget/sideを保持する
- start-gridの`ATTACK_FOLLOW` STOP解除確認では、Muxが同じtarget・attempt・direction=0、
  freshなplan/constraint/PP command/status、連続trajectoryを検証した正確なN-1世代だけを
  `attack_follow_stop_transport_release_ready`として受理する。これはSTOP解除の連続確認専用で、
  PASS warm-up ACK待ち、PASSING、target/attempt変更、E-stop/watchdog/faultでは使用しない。
  `ControllerTrackingStatus.reason`は診断専用とし、Plannerの認可分岐には使用しない
  `FOLLOW_BLOCKED`へ留まる。中心向きYIELD/RECOVERYを理由にABORTへ遷移しない
- Gate 2認可済みPASSは原則としてtarget ID、side、committed target dを通過完了まで保持する。
  例外は、選択側への実横進捗が設定閾値以下の「未開始PASS」で、freshな同一targetが
  安全相互作用包絡より先へ設定相対速度以上で離れ続けたことを連続確認できた場合だけとする。
  このhandoff周期はPASS候補を凍結し、SafetyEvaluator通過済みFOLLOW/RECOVERYだけを選べる。
  次周期から近い対象を別transactionとして再分類する。解放した同一targetが相互作用包絡外へ
  離れ続ける間は再ラッチを抑止し、別IDが近傍対象になった時、または同一IDが再びcatch可能な
  相対距離・相対速度になった時だけ新規Gate 2評価を許す。横へコミット済みのPASS、一時欠測、
  stale観測、単発の距離・速度変動はこの例外で解放しない
- commit済みPASSは、実publish済みwireがtarget dへ到達し、終端0.5 m以上が平坦に
  なった時点でtransactionの空間形状を固定する。以後はraw localized profileへ毎周期
  張り直さず、unwrapped ego sで既通過部分をcropする。有限publish wireより先に
  認可済みslow chainが続く場合は、初回に固定したchain ID/side/各waypoint dとの
  identity一致を確認し、fresh観測で前進更新した同じwaypoint sから空間形状を補完する。
  新しいID、side、waypoint dへ過去の認可を移さない。固定wireと実測dの差は
  診断値として残し、候補可否は実測d connectorを含む実publish形状のtrackability、
  corridor、fresh SafetyEvaluatorで判定する。staged chain中の横移動は先頭車のscalar
  target dではなく、固定した全waypointの最外側dまでを認可包絡とし、その包絡端を
  `safe_stop_lateral_error_threshold_m`より越えた逸脱だけは明示拒否する。時刻列、速度予測、
  全相手予測、壁、controller trackability、SafetyEvaluatorは毎周期freshに再評価し、
  どれかが不成立ならraw PASSへ暗黙に戻さずcurrent-d hold/STOPへ閉じる。前周期の
  ControllerTrackingStatusを候補生成の前提へ戻して循環させず、実行可否は既存の
  tracking/constraint authorityで独立にfail-closedする
- slow chainの現在targetを幾何的に抜いた場合は、次周期にtarget IDだけを次waypointへ
  進める。PASS side、初回avoid/full-offset marker、全chain waypoint、実行commitを保持し、
  現在egoから短い新規profileを作り直さない。これにより、先頭車の`merge_front_gap_m`を
  確保した時点で次車までの距離が短くても、すでに進めていた横分離を失わない。
  handoff後の候補も現周期の全相手SafetyEvaluator、corridor、controller trackability、
  下流generation authorityを通過した場合だけ実行する
- 通常`SAFE_STOP`でも、最終候補が同周期のSafetyEvaluator、controller幾何制約、longitudinal
  modelを通り、ego/V2X/reference/全観測相手と予測列が完全で、publish直前再評価後の全d点が
  現在ego dと一致する定数holdの場合だけ`lateral_tracking_authorized_during_stop=true`とする。
  Muxは同一generation/stampのplan/constraintとfreshなPP proofが揃った場合だけ、その操舵と縦停止を
  合成する。中心復帰、非定数d、stale/欠損入力、publish reject、E-stop/watchdogは対象外とする
- shadow解析では未認可候補の形状も比較できるよう、配信可能なpoints自体は
  `trajectory_authorized=false` でも格納する。下流authorityは必ず認可flagと同一generationの
  constraintを同時確認し、未認可pointsを制御入力に使わない
- `/debug/overtake/metrics` には `v2_effective_trajectory_authorized`、
  `v2_trajectory_publishable`、`v2_authorization_failure_reasons`、
  `v2_constraint_prefilter_reason`、`v2_constraint_filtered_reason` を出す

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
