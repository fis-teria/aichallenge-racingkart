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
| `LongitudinalProfile` | FOLLOW/RECOVERY/YIELD/SAFE_STOPなどの減速候補で、遅れ・制動上限から安全評価用の `s(t)` と `predicted_speed_mps(t)` を作る。公開する `v_ref` は即時cap。 |

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
静的pass gapは診断値です。Coreは壁・相手楕円の時系列安全評価が必要な場合にも候補を作りますが、走行中に反対側PASSへ切り替えるためには使いません。

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

通常の横補間は、制動モデルを含む物理前進距離 `ds` に依存します。
停止近傍では横目標を無理に進めず、現在位置付近を低速で保持します。前進距離が得られた周期から、評価済みの空間profileに沿って中央寄せを進めます。`outside_corridor_recovery_centering_time_sec` は設定互換用です。

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

通常/加速候補は既存の速度cap契約を保ちます。減速候補では、`LongitudinalProfile` が次を同時に作ります。

```text
t = i * horizon_dt_sec
delay中: predicted_v(t) = ego.v
delay後: predicted_v(t) = max(target_speed, ego.v - max_brake_decel_mps2 * (t - delay))
s(t) = delay中の等速距離 + delay後の制動距離
predicted_speed_mps[i] = predicted_v(t)
v_ref[i] = target_speed
s = wrapS(ego.s + s(t))
ratio = smoothstep(s(t) / shift_distance)
d = start_d + (target_d - start_d) * ratio
```

`predicted_speed_mps` と `s(t)` は同じ保守的な遅れ・制動モデルです。対して `v_ref[0]` は下流MPC/PPが毎周期すぐ読む速度capなので、予測上の遅れを入れず即時に目標を要求します。これにより減速要求を毎周期リセットせず、安全評価だけは保守的に保ちます。

PASSとラッチ済み攻めFOLLOWでは、固定時間horizonの空間長がPure Pursuitの実行条件より短い場合があります。その時は次の最大値を必要arcとして、同じ `LongitudinalProfile::distanceAt(t)` が届く時刻まで `evaluation_dt_sec` を広げます。

- `0.5 m` の固定下限
- `ego.v * 0.75 sec`
- `lateral_override_lookahead_gain * max(ego.v, speed_cap) + lateral_override_lookahead_min_distance_m`
- `0.25 sec`以上の応答遅れ後、`1.0 m/s^2`以下で`0.2 m/s`まで落とす制動距離

時刻軸を広げた後も点数は固定し、相手予測と同じ各時刻でSafetyEvaluatorへ渡します。PASSとPASS側を保持する攻めFOLLOWでは、`lateral_override_execution_speed_reserve_sec` 中の最大残留加速分をtarget capにかかわらず初期速度へ加え、停止可能距離だけでなく `s(t)` と予測速度にも反映します。全観測相手が静止閾値以下の時だけ`lateral_override_max_evaluation_horizon_sec`を使い、moving/欠損相手を含む時は`moving_lateral_override_max_evaluation_horizon_sec`へ閉じます。各上限でも必要arcへ届かない場合は `controller_tracking_profile_valid=false` とし、横profileを認可しません。RECOVERY・YIELD・SAFE_STOPは停止可能距離と応答遅れの既存契約が異なるため、同じ変更を未検証で広げません。

`localized_latched` の固定targetをrear-clearanceまで抜き切っていない間は、延長したhorizonの末尾にも中心方向のmerge点を生成しません。対象IDとPASS側を維持し、同側保持またはslow chainに必要な外向き遷移だけをSafetyEvaluatorへ渡します。抜き切り確認後の中心復帰は、過去のmarkerを流用せず、Coreが現在のego状態からRECOVERYを再生成して同周期に再評価します。

`max_brake_decel_mps2` はplannerの保守上限 `1.5 m/s^2` 以下だけを許可します。active launchで使う実制御器がこの想定減速を出せることは、別途ログと設定で確認します。設定が不正なら候補を安全扱いにせず、node起動時にoverride自体を無効化します。

`RECOVERY`, `YIELD_BEHIND`, `SAFE_STOP` では `start_d` を安全コリドー内へclampします。
MPCへ壁外d列を渡さないためです。

`RECOVERY` と中心寄せが必要な `SAFE_STOP` も同じ距離比率を使います。

```text
ratio = smoothstep(ds / shift_distance)
```

つまり、時間経過だけでは横参照を進めず、実際に評価できた前進距離の範囲で中心方向へ進ませます。

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
