# future_side_by_side_risk_analyzer.cpp

対象:

- `src/future_side_by_side_risk_analyzer.cpp`
- `include/overtake_planner/future_side_by_side_risk_analyzer.hpp`

## 役割

現在はまだ安全に見えても、少し先で横並びのままコーナーや外壁リスクに入るかを予測します。
ここで危険と判断した場合、`future_yield_required=true` になり、後段で `YIELD_BEHIND` が優先されます。

## ローカルhelper

| 関数 | 処理 |
|---|---|
| `smoothstep()` | 横移動の進行率を滑らかにする。 |
| `signedDeltaS()` | 周回を考慮した符号付き前後距離を返す。 |

## `FutureSideBySideRiskAnalyzer()`

`FrenetFrame`, `PlannerConfig`, `BlockedRiskAnalyzer` を参照として持ちます。
`BlockedRiskAnalyzer` は壁余裕計算に使います。

## `evaluate()`

未来横並びリスクを `BlockedInfo` に追記します。

入力:

- 自車 `EgoState`
- 現在の `BlockedInfo`
- 他車一覧

処理:

1. `future_side_prediction_enabled=false` なら何もしない。
2. 狭義の横並び、またはparallel side candidateから対象車を選ぶ。
3. 逆方向の対象なら無視する。
4. 自車が相手から離れる方向 `away_sign` を決める。
5. `SIDE_BY_SIDE_KEEP` 相当の将来目標dを仮に作る。
6. `future_side_prediction_dt_sec` 刻みで将来を走査する。
7. 各時刻で自車s/d、相手s/d、前後横差、曲率、壁余裕を計算する。
8. 将来も横並びで、コーナーまたは外壁リスクがあれば `future_yield_required` を立てる。

重要な出力:

- `future_side_by_side`
- `future_corner_side_by_side`
- `future_outer_wall_risk`
- `future_yield_required`
- `future_delta_s`, `future_delta_d`
- `future_wall_clearance_m`
- `future_prediction_time_sec`
- `yield_reason`

## 外壁リスク判定

将来自車が横並び維持のために外側へ逃げた場合、壁余裕が `future_side_yield_wall_clearance_m` を下回ると危険です。
このとき `yield_reason="future_outer_wall_risk"` になり、Core側で `YIELD_BEHIND` が選ばれやすくなります。

## `sideRiskIndex()`

横並び対象のindexを返します。
狭義の横並びがあればそれを優先し、なければparallel side candidateを使います。

## `maxAbsCurvatureAhead()`

指定 `s` から前方 `lookahead_m` までを8分割でサンプリングし、最大曲率絶対値を返します。
未来の横並びがコーナーに入るかを見るために使います。

