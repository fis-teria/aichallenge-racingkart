# overtake_planner src logic index

このディレクトリは、`overtake_planner` のソースコードをファイル単位で読むためのロジック解説です。
大きな流れを掴む資料は `../06_source_logic_walkthrough.md`、パラメータ一覧は `../02_parameter_guide.md` を参照してください。

## 読む順番

1. [types](00_types.md): planner全体で共有するデータ型。
2. [overtake_planner_node](01_overtake_planner_node.md): ROS入出力、parameter、debug publish。
3. [overtake_planner_core](02_overtake_planner_core.md): 1周期の判断順序。
4. [blocked_risk_analyzer](03_blocked_risk_analyzer.md): 前方車、横並び、pass gap判定。
5. [future_side_by_side_risk_analyzer](04_future_side_by_side_risk_analyzer.md): 未来横並びと外壁リスク。
6. [candidate_builder](05_candidate_builder.md): 追い越し、譲り、復帰、停止の候補経路生成。
7. [safety_evaluator](06_safety_evaluator.md): 壁マージンと他車安全楕円。
8. [behavior_state_machine](07_behavior_state_machine.md): モード遷移とチャタリング防止。
9. [planner_output_builder](08_planner_output_builder.md): MPC override向け出力整形。
10. [frenet_frame](09_frenet_frame.md): 参照CSVとFrenet変換。

## 全体の処理順

```text
OvertakePlannerNode
  -> OvertakePlannerCore::update()
     -> BlockedRiskAnalyzer
     -> FutureSideBySideRiskAnalyzer
     -> CandidateBuilder
     -> SafetyEvaluator
     -> BehaviorStateMachine
     -> PlannerOutputBuilder
  -> /overtake/reference_override
  -> /debug/overtake/mode
  -> /debug/overtake/metrics
```

## 追い越し経路生成を見る入口

追い越し用の経路は [candidate_builder](05_candidate_builder.md) が作ります。
ただし「どちらに追い越せるか」は [blocked_risk_analyzer](03_blocked_risk_analyzer.md) の `evaluatePassGap()` が決め、「その候補を使ってよいか」は [overtake_planner_core](02_overtake_planner_core.md) と [behavior_state_machine](07_behavior_state_machine.md) が決めます。

