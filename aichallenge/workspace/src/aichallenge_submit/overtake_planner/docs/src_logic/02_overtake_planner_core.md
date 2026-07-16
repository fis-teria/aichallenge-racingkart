# overtake_planner_core.cpp

対象:

- `src/overtake_planner_core.cpp`
- `include/overtake_planner/overtake_planner_core.hpp`

## 役割

plannerの中核です。
各サブロジックを呼ぶ順番を管理し、最終的に `PlannerOutput` を返します。

このファイルは「判断の司令塔」で、個別の幾何計算や安全評価は別クラスへ委譲します。

## ローカルhelper

| 関数 | 処理 |
|---|---|
| `isLeftPassMode()` | 現在modeが左追い越し系かを見る。 |
| `isRightPassMode()` | 現在modeが右追い越し系かを見る。 |
| `currentPassGapLost()` | 現在の追い越し方向のpass gapが失われたかを見る。 |
| `isPassCandidate()` | 候補が `PASS_LEFT` / `PASS_RIGHT` かを見る。 |
| `isFallbackCandidate()` | `FOLLOW`, `YIELD_BEHIND`, `RECOVERY`, `SIDE_BY_SIDE_KEEP` など回避系候補かを見る。 |
| `hasFeasibleCandidate()` | 条件に合うfeasible候補があるかを見る。 |
| `isFeasibleReleaseCandidate()` | SAFE_STOP解除に使える候補かを見る。 |
| `hasFeasibleReleaseCandidate()` | SAFE_STOP解除候補が候補集合内にあるかを見る。 |

## `OvertakePlannerCore()`

`FrenetFrame` と `PlannerConfig` を受け取り、内部コンポーネントを構築します。

- `BlockedRiskAnalyzer`
- `FutureSideBySideRiskAnalyzer`
- `SafetyEvaluator`
- `BehaviorStateMachine`

## `update()`

1制御周期のメイン処理です。

処理順:

1. 無効状態なら `FREE_RUN` で早期return。
2. section safetyを取得する。
3. `detectBlocked()` で前方車/横並びを判定する。
4. `predictOpponents()` で他車予測を作る。
5. `promoteSlowObstacleChain()` で低速/停止のparallel-side前方車を必要なら前方閉塞へ昇格する。
6. `evaluatePassGap()` で左右追い越し余裕を判定する。
7. 曲率からcorner判定、straight-only gateを更新する。
8. `FutureSideBySideRiskAnalyzer::evaluate()` で未来リスクを足す。
9. 候補経路を複数作る。
10. `SafetyEvaluator::evaluate()` で候補を安全評価する。
11. `candidateScore()` で候補に点数を付ける。
12. SAFE_STOP条件を判定する。
13. `selectCandidate()` で暫定候補を選ぶ。
14. `BehaviorStateMachine::update()` でmodeを安定化する。
15. modeに合わせてpublish用候補を再生成する。
    - `pass_horizon_publish_mode=overtake_only` では、`PREPARE_OVERTAKE_*` 中だけpublish用候補を `FOLLOW` に差し替える。
    - 内部候補の `PASS_LEFT/RIGHT` は状態機械へ渡しているため、PASS安全周期の蓄積は止まらない。
16. `PlannerOutputBuilder::build()` でpublish用出力に整形する。
17. `applyLateralTargetRateLimit()` で横オフセットの急変を抑える。
18. `rememberPublishedLateralTarget()` で次周期用に記憶する。

## 候補生成の流れ

常に `FASTEST` を作ります。

追加候補:

- 大きな横ずれで判断凍結: `RECOVERY`
- 横並びでコーナー譲り不要: `SIDE_BY_SIDE_KEEP`
- 横並びで譲るべき: `YIELD_BEHIND`
- 前方閉塞: `FOLLOW`
- 左にgapあり: `PASS_LEFT`
- 右にgapあり: `PASS_RIGHT`
- pass gapなし: `YIELD_BEHIND`
- 追い越し中/復帰中: `RECOVERY`
- 回避不能候補あり: `SAFE_STOP`

ここで作った候補の実体は [candidate_builder](05_candidate_builder.md) が作ります。

## `predictOpponents()`

他車を短いhorizonで等速予測します。

入力:

- `OpponentState` の `x/y/vx/vy`
- `horizon_points`
- `horizon_dt_sec`

出力:

- `PredictedOpponent` の `t/x/y/s/d`

安全評価とpass gapの未来確認に使います。

## `makeCandidate()`

`CandidateBuilder::makeCandidate()` への薄い委譲です。
Coreから見ると「候補種別を指定して、候補軌道を1本作る」関数です。

## `candidateScore()`

候補を数値スコアにします。
小さいほど優先されます。

基本方針:

- infeasible候補は基本的に大罰点。
- `SAFE_STOP` は必要時に強く優先。
- future yieldやcorner side-by-sideでは `YIELD_BEHIND` を優先。
- 通常の前方閉塞では `PASS_LEFT/RIGHT` を狙う。
- 現在modeに沿う候補は `keep_mode_bonus` で少し優遇する。
- `FOLLOW_BLOCKED` 中に追い越し開始ゲートが開いた場合は、安全な `PASS_LEFT/RIGHT` を少しだけ優先し、禁止区間やカーブ後に追従へ張り付かないようにする。

## `maxAbsCurvatureAhead()`

現在 `s` から指定距離先までの最大曲率絶対値をサンプリングします。

用途:

- コーナー横並び判定。
- straight-only overtake gate。
- future yield holdの継続判定。

## `activeSectionSafety()`

現在の `s` がsection safety ruleに入っているかを見ます。
一致したruleのprofileから、壁余裕scale、速度cap scale、outer yield強制などを作ります。

## `activeOvertakePermission()`

現在の `s` と `overtake_permission_lookahead_m` 先までを見て、新規追い越し開始を許可する区間かを判定します。
`config/overtake_permission.csv` の `allow_overtake=false` に入ると、PASS候補が安全でも状態機械は `FOLLOW_BLOCKED` を維持します。

低速前方車例外はこの後段で評価されます。`slow_front_permission_exception_enabled=true` の場合だけ、停止/低速の直接前走車を近距離で連続検出し、現在地点が不可区間、PASS候補がSafetyEvaluator通過済み、入力fresh、横並び/未来譲り/reentry holdなしをすべて満たす時に限り、不可区間でもPASS開始を許します。lookahead先だけの禁止、高曲率、停止車列への昇格は例外化しません。

## `sectionContainsS()`

閉ループコース上で、`s` がsection範囲内かを判定します。
startがendをまたぐ区間にも対応します。

## `permissionRuleContainsS()`

`OvertakePermissionRule` 用の区間判定です。
`sectionContainsS()` と同じく、最終区間のようにstartがendをまたぐwrap-around範囲にも対応します。

## `effectiveWallSoftMargin()`

基本の `wall_soft_margin_m` にsection profileのscaleを掛けた実効値を返します。

## `promoteSlowObstacleChain()`

`parallel_side_candidate` のうち、自車より前方にいて低速/停止している相手を、停止車列の次の回避対象として前方閉塞へ昇格します。

主な条件:

- `slow_obstacle_chain_enabled=true`
- まだ通常の `nearest_index` がない
- `parallel_side_delta_s > 0`
- `parallel_side_delta_s <= slow_obstacle_chain_distance_m`
- 相手速度が `slow_front_exception_speed_mps` 以下

昇格すると `slow_obstacle_chain_active=true` になり、`nearest_id` / `front_delta_s` / `front_vehicle_speed_mps` もその相手で埋めます。
これにより、1台目を抜いた後に2台目がfrontではなくparallel-sideとして見えても、既存のPASS候補生成と安全評価に載ります。

## `shouldYieldBehindSideBySide()`

横並び中に後ろへ譲るべきかを判定します。

譲る条件:

- future side-by-side riskが譲り要求を出している。
- 相手が自車より前に出ている。
- コーナー横並びで、相手が明確に後ろではない。
- コーナー横並びで自車が壁に近い。

## `applyLateralTargetRateLimit()`

MPCへpublishする横オフセット列の1周期あたり変化量を制限します。
`lateral_target_max_step_m` が0以下なら無効です。

SAFE_STOPでは止める意図を優先し、このrate limitはかけません。

## `revalidatePublishedLateral()`

PASS候補だけは、rate limit/hold後に実際にpublishする `d[]` をCartesianへ戻し、元候補と同じ時刻列・壁余裕・相手楕円で再評価します。

ここでunsafeならPASS overrideを消して通常走行へ戻すことはしません。同じ周期で `ABORT_RECOVERY` の候補を安全評価してpublishし、RECOVERYもunsafeなら既存のspeed-only fallbackを使います。

## `rememberPublishedLateralTarget()`

前回publishした横オフセット列を記憶します。
一定時間 `active_override` がなければ記憶を消します。

## `selectCandidate()`

候補集合から採用候補を選びます。

優先順位:

1. feasible候補があるならfeasible候補だけでscore最小を選ぶ。
2. 全部infeasibleなら、診断用にscore最小のinfeasible候補を返す。
