# blocked_risk_analyzer.cpp

対象:

- `src/blocked_risk_analyzer.cpp`
- `include/overtake_planner/blocked_risk_analyzer.hpp`

## 役割

前方閉塞、横並び、parallel side candidate、左右pass gapを判定します。
ここでは候補経路は作らず、「今どういう相手がいて、左右どちらに余裕があるか」を `BlockedInfo` に書きます。

## ローカルhelper

| 関数 | 処理 |
|---|---|
| `isLeftPassMode()` | 現在modeが左追い越し系かを見る。 |
| `isRightPassMode()` | 現在modeが右追い越し系かを見る。 |
| `signedDeltaS()` | 閉ループコース上で、相手が前か後ろかを符号付き距離で返す。 |

## `BlockedRiskAnalyzer()`

`FrenetFrame` と `PlannerConfig` を参照として保持します。

## `detectBlocked()`

前方車と横並びを検出します。

入力:

- 自車 `EgoState`
- 他車 `OpponentState[]`
- 現在時刻

処理:

1. 無効または古い他車を無視する。
2. `delta_s`, `signed_delta_s`, `delta_d` を計算する。
3. 他車の参照線方向速度 `s_dot` を計算する。
4. 逆走方向に見える相手を必要に応じて除外する。
5. `side_by_side` を判定する。
6. `parallel_side_candidate` を判定する。
7. 前方かつ同一コリドーの最近車両を探す。
8. 最近前方車が近い、または自車が詰めている場合に `blocked=true` にする。

主な出力:

- `nearest_id`, `front_delta_s`, `front_delta_d`, `front_rel_v`
- `side_id`, `side_delta_s`, `side_delta_d`
- `parallel_side_id`, `parallel_side_delta_s`, `parallel_side_delta_d`
- `blocked`, `side_by_side`, `parallel_side_candidate`

## `evaluatePassGap()`

左右に追い越し可能な幅があるかを判定します。

入力:

- `BlockedInfo`
- 他車一覧
- 他車予測
- 現在mode

処理:

1. 安全コリドー `lower_d/upper_d` を計算する。
2. `min_pass_gap_m` と安全楕円幅から必要gapを決める。
3. 対象車両を選ぶ。前方車がいれば前方車、いなければ横並び/parallel side。
4. 現在位置と予測d列から左側/右側の最小gapを計算する。
5. 現在追い越し中の方向だけ `pass_gap_hysteresis_m` で少し粘らせる。
6. `can_pass_left/right` と `pass_gap_reason` を設定する。

`pass_gap_reason`:

- `ok`
- `left_gap_narrow`
- `right_gap_narrow`
- `both_gap_narrow`
- `no_target`

## `wallClearance()`

自車または候補dが安全コリドーからどれだけ余裕を持つかを返します。

```text
lower_d = d_min_m + min_wall_margin_m
upper_d = d_max_m - min_wall_margin_m
clearance = min(d - lower_d, upper_d - d)
```

負なら安全コリドー外です。

## `opponentSDot()`

他車速度 `vx/vy` を参照線方向へ射影します。
同方向/逆方向判定に使います。

## `sideRiskIndex()`

狭義の横並びがあれば `side_index`、なければ `parallel_side_index` を返します。

## `yieldTargetIndex()`

譲り対象を返します。
前方車がいれば前方車、いなければ横並び/parallel sideを使います。

