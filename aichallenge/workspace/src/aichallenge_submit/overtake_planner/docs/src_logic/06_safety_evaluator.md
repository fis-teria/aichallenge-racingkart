# safety_evaluator.cpp

対象:

- `src/safety_evaluator.cpp`
- `include/overtake_planner/safety_evaluator.hpp`

## 役割

`CandidateBuilder` が作った候補経路を、安全に使えるか評価します。
評価結果は `CandidateTrajectory` に直接書き戻します。

## `SafetyEvaluator()`

`PlannerConfig` を保持します。

## `ellipseMargin()`

自車候補点と他車予測点の距離を、自車姿勢基準の楕円で評価します。

処理:

1. 候補点から見た他車位置差 `dx/dy` を計算する。
2. 候補yawを使ってbody座標へ変換する。
3. 前後方向 `safety_ellipse_a_m`、横方向 `safety_ellipse_b_m` の楕円値を計算する。
4. `h = ellipse_value - 1.0` を返す。

`h` が小さいほど近接しています。

## `evaluate()`

候補が安全かどうかを判定します。

入力:

- `CandidateTrajectory& candidate`
- `PredictedOpponent[]`

処理:

1. 候補状態を初期化する。
2. 各中心 `d` が従来の安全コリドー内か確認する。
3. 中心点の壁マージン外なら `reject_reason="wall_margin"` で即reject。
4. Cartesian候補列の接線yawから、`base_link`基準の車体四隅を展開する。
5. 各cornerをFrenetへ再投影し、自己位置余裕を残してlanelet路面端内か確認する。
6. 車体cornerが外なら `reject_reason="wall_footprint_margin"` で即reject。
7. 各他車予測と同じhorizon indexで楕円marginを計算する。
8. `margin <= min_ellipse_h + 0.10` なら active constraintとして数える。
9. `margin <= min_ellipse_h` なら `reject_reason="opponent_collision"` でreject。
10. 全て通れば `feasible=true` のまま返る。

出力として更新される値:

- `candidate.feasible`
- `candidate.min_safety_margin`
- `candidate.cbf_slack`
- `candidate.active_safety_constraint_count`
- `candidate.reject_reason`

## 判定の意味

回廊CSVは車体中心境界ではなくlanelet路面端です。壁判定は候補中心d列と、
候補Cartesian列から求めた車体四隅の両方を見ます。
他車判定は候補の `x/y/yaw` と他車予測の `x/y` を同じindexで突き合わせます。

安全評価は「候補を作るかどうか」ではなく、「作った候補を採用してよいか」を決める層です。
