# candidate_builder.cpp

対象:

- `src/candidate_builder.cpp`
- `include/overtake_planner/candidate_builder.hpp`

## 役割

追い越し向けの候補経路を実際に生成するファイルです。
`OvertakePlannerCore` が「`PASS_LEFT` を作って」と依頼し、このクラスがFrenet上の `s[]/d[]`、Cartesianの `x[]/y[]/yaw[]`、速度上限 `v_ref[]` を作ります。

ここで作るのは「MPCに渡す短いhorizonの参照」です。
MPC本体の最適化は別で、このplannerは横方向オフセット列と速度cap列を渡します。

## ローカルhelper

| 関数 | 処理 |
|---|---|
| `smoothstep()` | 0から1へ滑らかに変化する補間率を作る。急な横移動を避ける。 |
| `finitePositiveOr()` | 正の有限値ならその値、そうでなければfallbackを返す。 |
| `sideRiskIndex()` | 横並び対象のindexを返す。 |
| `yieldTargetIndex()` | 譲り対象を返す。前方車を優先し、なければ横並び/parallel side。 |

## `CandidateBuilder()`

`FrenetFrame` と `PlannerConfig` を参照として持ちます。

## `makeCandidate()`

候補経路生成の本体です。

入力:

- `CandidateType type`
- 自車 `EgoState`
- 状況認識 `BlockedInfo`
- 他車一覧

出力:

- `CandidateTrajectory`

生成手順は大きく4段です。

1. 候補種別から `target_d` と `shift_distance` を決める。
2. 候補種別から `speed_cap` を決める。
3. horizon各点の `s/d` を作る。
4. `FrenetFrame::frenetToCartesian()` で `x/y/yaw` に戻す。

## 追い越し候補の横目標

### `PASS_LEFT`

左側へ追い越す候補です。

- `target_d = left_offset_m`
- `shift_distance = prepare_distance_m`

意味:

`ego.frenet.d` から `left_offset_m` へ、`prepare_distance_m` かけて滑らかに寄ります。
左に十分なpass gapがあるときだけCoreがこの候補を作ります。

### `PASS_RIGHT`

右側へ追い越す候補です。

- `target_d = right_offset_m`
- `shift_distance = prepare_distance_m`

`PASS_LEFT` と対称で、右に十分なpass gapがあるときに作られます。

### `FOLLOW`

横方向は中心線に戻す追従候補です。

- `target_d = 0.0`
- `shift_distance = merge_distance_m`

前方車がいるが、まだ追い越しに入らない、または追従すべきときに使います。

### `RECOVERY`

中心線へ復帰する候補です。

- `target_d = 0.0`
- 通常は `merge_distance_m`
- 安全コリドー外かつ横並びでない場合は、`prepare_distance_m` と `merge_distance_m` の小さい方をもとに短めの復帰距離にする。

低速/停止中の注意:

通常の横補間は `ds = ego.v * t` に依存します。
そのため停止中は横目標がほぼ進みません。
現在の実装では、次の場合に `outside_corridor_recovery_centering_time_sec` を使って時間ベースでも中央寄せを進めます。

- 安全コリドー外にいる。
- または `recovery_release_lateral_error_m` を超える横誤差が残っている。
- かつ `side_by_side` ではない。

これにより、停止中でも `ABORT_RECOVERY` の参照が境界付近に残り続けるのを避けます。

### `SIDE_BY_SIDE_KEEP`

横並び中に相手から離れて並走する候補です。

処理:

1. 相手が左にいるなら右へ、右にいるなら左へ逃げる `away_sign` を決める。
2. `opp.d + away_sign * side_by_side_target_gap_m` を目標にする。
3. 自車の現在dより相手側へ戻らないようにする。
4. 最後に安全コリドーへclampする。

コーナーで外側に膨らむリスクがあるため、Core側では `corner_side_by_side` やfuture risk時に `YIELD_BEHIND` を優先します。

### `YIELD_BEHIND`

相手の後ろへ入る候補です。

基本:

- `target_d = corner_yield_target_d_m` を安全コリドーへclamp。

ただし、コーナー譲りではなく、壁余裕が十分で、対象車が存在する場合は、相手のdへ寄せます。
これは「後ろについて同じラインへ戻る」意図です。

### `SAFE_STOP`

停止用候補です。

- `target_d = ego.d` を安全コリドーへclamp。
- ただし壁外、または `safe_stop_lateral_error_threshold_m` を超えて中心からずれている場合は、`target_d = 0.0` へ寄せる。
- `shift_distance = max(merge_distance_m, prepare_distance_m)`
- 速度は `safe_stop_v_mps` の小さい正値。

完全な0ではなく小さい正値にするのは、下流側で0速度が無効扱いになりにくくするためです。

## 速度上限の決定

候補ごとの基本速度:

| 候補 | 速度上限 |
|---|---|
| `FASTEST` | `v_passthrough_mps` |
| `FOLLOW` | 前方車速度 `opp.v - follow_speed_margin_mps` |
| `RECOVERY` | `recovery_v_max_mps`、壁外なら `wall_margin_recovery_v_max_mps` |
| `SIDE_BY_SIDE_KEEP` | `side_by_side_speed_cap_mps` と相手速度基準の小さい方。相手速度基準は `yield_min_speed_cap_mps` を下限にする |
| `YIELD_BEHIND` | 対象車速度から `yield_speed_margin_mps` または `corner_follow_speed_margin_mps` を引く。対象が取れない/低速なら `yield_min_speed_cap_mps` を使う |
| `SAFE_STOP` | `safe_stop_v_mps` |

追加の速度抑制:

- `RECOVERY`, `YIELD_BEHIND`, `SIDE_BY_SIDE_KEEP` で壁外なら `wall_margin_recovery_v_max_mps` を上限にする。
- 横誤差が `large_lateral_error_threshold_m` を超える場合は `large_lateral_error_v_max_mps` を上限にする。

## horizon列生成

各点で次を計算します。

```text
t = i * horizon_dt_sec
longitudinal_speed = SAFE_STOPならsafe_stop_v_mps、それ以外はmax(0.5, ego.v)
ds = longitudinal_speed * t
s = wrapS(ego.s + ds)
ratio = smoothstep(ds / shift_distance)
d = start_d + (target_d - start_d) * ratio
```

`RECOVERY`, `YIELD_BEHIND`, `SAFE_STOP` では `start_d` を安全コリドー内へclampします。
MPCへ壁外d列を渡さないためです。

`RECOVERY` と中心寄せが必要な `SAFE_STOP` では、必要に応じて次も使います。

```text
ratio = max(ratio, smoothstep(t / outside_corridor_recovery_centering_time_sec))
```

つまり低速でも時間経過で中心方向へ進ませます。

## Cartesianへの変換

Frenet上で作った `s/d` は、各点で `frame_.frenetToCartesian(s, d)` により `x/y/yaw` へ戻します。
この `x/y/yaw` は `SafetyEvaluator` が他車予測と衝突判定するために使います。

## `wallClearance()`

安全コリドーからの余裕を返します。
`RECOVERY` の速度上限や、中心復帰の強さを決める補助に使います。

## 追い越し経路生成の要点

追い越し経路は「中心線を書き換える」のではなく、Frenetの横オフセット `d` を短いhorizonだけ変化させることで作っています。

- 左追い越し: `d` を `left_offset_m` へ寄せる。
- 右追い越し: `d` を `right_offset_m` へ寄せる。
- 戻る: `d` を `0.0` へ寄せる。
- 横並び維持: 相手から `side_by_side_target_gap_m` 離れたdへ寄せる。
- 譲り: 中央または相手のdへ寄せながら速度を落とす。

この設計のおかげで、MPCは通常の経路追従構造を保ったまま、plannerから短期的な横位置/速度の意図だけを受け取れます。
