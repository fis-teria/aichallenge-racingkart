# frenet_frame.cpp

対象:

- `src/frenet_frame.cpp`
- `include/overtake_planner/frenet_frame.hpp`

## 役割

参照CSVを読み、Cartesian座標とFrenet座標を相互変換します。
追い越し判断は主に `s/d` で行うため、このファイルが座標系の土台です。

## ローカルhelper

| 関数 | 処理 |
|---|---|
| `splitCsvLine()` | CSV行を`,`で分割する。 |
| `readCell()` | header名からCSVセルを読み、なければfallbackを返す。 |

## `toString(BehaviorMode)`

`BehaviorMode` をdebug用文字列へ変換します。
`/debug/overtake/mode` やmetrics JSONで使われます。

## `toString(CandidateType)`

`CandidateType` をdebug用文字列へ変換します。

## `isPassMode()`

modeが追い越し系かを判定します。

trueになるmode:

- `PREPARE_OVERTAKE_LEFT`
- `PREPARE_OVERTAKE_RIGHT`
- `OVERTAKE_LEFT`
- `OVERTAKE_RIGHT`

## `normalizeAngle()`

角度を `(-pi, pi]` に正規化します。
yaw差や補間時に使います。

## `FrenetFrame::loadCsv()`

参照CSVを読みます。

読む列:

- `s_m`
- `x_m`
- `y_m`
- `psi_rad`
- `kappa_radpm`
- `vx_mps`

読み込み後は `setReference()` を呼んで参照線を確定します。

## `FrenetFrame::setReference()`

参照点列を保存し、必要なら `s` を補完します。

処理:

1. 参照点を保存する。
2. `s` がNaNの点は隣接点距離の累積で補う。
3. 最後の点から先頭点への距離も足し、閉ループの `track_length_` を作る。

## `FrenetFrame::wrapS()`

`s` をコース長で折り返します。
周回コース上の位置として扱うための関数です。

## `FrenetFrame::deltaS()`

`from_s` から `to_s` まで前向きに進んだ距離を返します。
相手が次周側にいる場合でも正の前方距離として扱えます。

## `FrenetFrame::cartesianToFrenet()`

Cartesian座標をFrenetへ変換します。

処理:

1. 最近傍の参照点を探す。
2. その参照点のyawを使って横ずれ `d` を計算する。
3. `yaw_error` を計算する。

この実装は簡易変換で、最近傍点ベースです。

## `FrenetFrame::interpolate()`

指定 `s` を囲む2点を線形補間します。

補間する値:

- `x`
- `y`
- `yaw`
- `kappa`
- `v_ref`

候補経路生成や曲率先読みで使います。

## `FrenetFrame::frenetToCartesian()`

Frenetの `s/d` をCartesianへ戻します。

処理:

1. `interpolate(s)` で中心線上の点を得る。
2. yawの法線方向へ `d` だけずらす。

式:

```text
x = center_x - d * sin(yaw)
y = center_y + d * cos(yaw)
```

`CandidateBuilder` はこの関数で候補の `x/y/yaw` を作ります。

