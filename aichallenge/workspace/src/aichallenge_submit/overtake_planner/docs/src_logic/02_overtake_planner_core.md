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
5. `evaluatePassGap()` で左右追い越し余裕を判定する。
6. 曲率からcorner判定、straight-only gateを更新する。
7. `FutureSideBySideRiskAnalyzer::evaluate()` で未来リスクを足す。
8. 候補経路を複数作る。
9. `SafetyEvaluator::evaluate()` で候補を安全評価する。
10. `candidateScore()` で候補に点数を付ける。
11. SAFE_STOP条件を判定する。
12. `selectCandidate()` で暫定候補を選ぶ。
13. `BehaviorStateMachine::update()` でmodeを安定化する。
14. modeに合わせて候補を再生成する。
15. `PlannerOutputBuilder::build()` でpublish用出力に整形する。
16. `applyLateralTargetRateLimit()` で横オフセットの急変を抑える。
17. `rememberPublishedLateralTarget()` で次周期用に記憶する。

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

## `maxAbsCurvatureAhead()`

現在 `s` から指定距離先までの最大曲率絶対値をサンプリングします。

用途:

- コーナー横並び判定。
- straight-only overtake gate。
- future yield holdの継続判定。

## `activeSectionSafety()`

現在の `s` がsection safety ruleに入っているかを見ます。
一致したruleのprofileから、壁余裕scale、速度cap scale、outer yield強制などを作ります。

## `sectionContainsS()`

閉ループコース上で、`s` がsection範囲内かを判定します。
startがendをまたぐ区間にも対応します。

## `effectiveWallSoftMargin()`

基本の `wall_soft_margin_m` にsection profileのscaleを掛けた実効値を返します。

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

## `rememberPublishedLateralTarget()`

前回publishした横オフセット列を記憶します。
一定時間 `active_override` がなければ記憶を消します。

## `selectCandidate()`

候補集合から採用候補を選びます。

優先順位:

1. feasible候補があるならfeasible候補だけでscore最小を選ぶ。
2. 全部infeasibleなら、診断用にscore最小のinfeasible候補を返す。

