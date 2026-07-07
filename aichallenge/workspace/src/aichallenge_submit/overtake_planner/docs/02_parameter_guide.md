# overtake_planner パラメータ調整ガイド

この文書は、`overtake_planner` のパラメータを変更したときに「どの責務のロジックが変わるか」を確認するための資料です。
対象ファイルは次です。

```text
aichallenge/workspace/src/aichallenge_submit/overtake_planner/config/overtake_planner.param.yaml
```

## 読み方

表の「現在値」は上記YAMLの値です。
「fallback」は `overtake_planner_node.cpp` の `declare_parameter()` と `PlannerConfig` の未指定時デフォルトです。
YAMLに書かれている値が優先されるため、現在の実走値は「現在値」を見てください。

重要な注意:

- baseline MPCは `horizon_points=20` で使います。
- delay-aware MPCでは launch から `horizon_points=50` へ上書きし、MPC側の `N: 50` と合わせます。
- `pass_distance_m` と `max_overtake_v_bonus_mps` は `PlannerConfig` に残っていますが、現状のnodeではROS parameterとして宣言していません。YAMLに追加しても効かないため、通常の調整対象から外します。

## 責務別の全体像

| 責務 | 主な実装 | 主なパラメータ |
|---|---|---|
| 起動/入出力 | `overtake_planner_node.cpp` | reference, own vehicle ID, stale判定, V2X速度推定 |
| 前方/横並び/pass gap判定 | `BlockedRiskAnalyzer` | lookahead, same direction, side-by-side, parallel side, pass gap |
| 未来横並びリスク | `FutureSideBySideRiskAnalyzer` | future side prediction, future wall clearance |
| 候補生成 | `CandidateBuilder` | offsets, shift distance, follow/yield/recovery/safe stop speed |
| 安全評価 | `SafetyEvaluator` | wall corridor, safety ellipse |
| 状態遷移 | `BehaviorStateMachine` | safe cycles, mode hold, merge/rejoin gap, abort timeout |
| 出力整形/速度ガード | `PlannerOutputBuilder` | speed-only fallback, wall guard, MPC health guard, section profile |

## 最初に見る調整項目

サイドバイサイドでコーナーに入ったときに外側が壁へ膨らむ場合:

| パラメータ | 現在値 | fallback | 見る理由 |
|---|---:|---:|---|
| `future_side_yield_wall_clearance_m` | `0.35` | `0.35` | 未来の横並び維持目標が壁に近いと早めに譲る。上げると保守的。 |
| `corner_side_yield_wall_clearance_m` | `0.55` | `0.55` | コーナー横並び中の壁余裕判定。上げると壁から遠いうちに譲る。 |
| `corner_yield_v_max_mps` | `8.0` | `3.0` | コーナー譲り時の速度上限。下げると曲がりやすいが遅くなる。 |
| `side_by_side_target_gap_m` | `0.75` | `0.75` | 相手から離れる横距離。上げると接触余裕は増えるが壁側へ逃げやすい。 |
| `side_by_side_shift_distance_m` | `7.0` | `7.0` | 横方向へ移る距離。上げると操舵が穏やか。 |
| `wall_risk_v_max_mps` | `8.0` | `5.0` | 壁リスク時の速度上限。下げると壁際の破綻を抑えやすい。 |
| `mpc_health_v_max_mps` | `8.0` | `3.0` | MPC不調時の速度上限。下げると計算破綻時に保守的。 |
| `recovery_speed_guard_v_max_mps` | `3.0` | `3.0` | `RECOVERY` / `ABORT_RECOVERY` 中だけ、壁リスク・MPC不調・大横ずれ時に追加で速度を絞る。通常の壁/MPC capを攻めた値にしている場合の保険。 |

スタート直後から第1コーナーまで横並びを認識しない場合:

| パラメータ | 現在値 | fallback | 見る理由 |
|---|---:|---:|---|
| `side_by_side_s_m` | `4.0` | `4.0` | 狭義の横並び前後範囲。 |
| `side_margin_m` | `1.20` | `1.20` | 狭義の横並び横幅。広げすぎると通常の横並びが過敏になる。 |
| `parallel_side_detection_enabled` | `true` | `true` | 広めの並走候補を未来予測へ渡す。 |
| `parallel_side_s_m` | `12.0` | `12.0` | 広めの並走候補の前後範囲。 |
| `parallel_side_margin_m` | `4.0` | `4.0` | 広めの並走候補の横幅。 |

カーブで追い越し開始してほしくない場合:

| パラメータ | 現在値 | fallback | 見る理由 |
|---|---:|---:|---|
| `straight_only_overtake_enabled` | `true` | `true` | 直線以外で新規PASS開始を止める。 |
| `straight_overtake_max_curvature_m_inv` | `0.025` | `0.025` | 追い越し開始を許可する最大曲率。下げるほど保守的。 |
| `straight_overtake_lookahead_m` | `12.0` | `12.0` | 追い越し開始ゲートが先読みする距離。 |
| `straight_overtake_release_hysteresis_m_inv` | `0.005` | `0.005` | 一度閉じたゲートを開き直すためのヒステリシス。 |

ref velocity区間を元に追い越し開始を許可/禁止したい場合:

| パラメータ | 現在値 | fallback | 見る理由 |
|---|---:|---:|---|
| `overtake_permission_profile_enabled` | `true` | `true` | `config/overtake_permission.csv` の区間許可gateを使う。 |
| `overtake_permission_package` | `overtake_planner` | `overtake_planner` | 追い越し許可CSVを探すROS package。 |
| `overtake_permission_csv` | `config/overtake_permission.csv` | 同左 | `name,start_wp,end_wp,allow_overtake` 形式のCSV。 |
| `default_overtake_allowed` | `true` | `true` | CSVに該当しない区間で追い越し開始を許すか。 |
| `overtake_permission_lookahead_m` | `8.0` | `8.0` | 近い将来の不可区間も見て追い越し開始を止める距離。 |
| `slow_front_exception_enabled` | `true` | `true` | 不可区間でも前方車が停止/低速なら例外的にPASS開始を許す。 |
| `slow_front_exception_speed_mps` | `1.0` | `1.0` | 停止/低速とみなす前方車速度。 |
| `slow_front_exception_distance_m` | `8.0` | `8.0` | 低速例外を許す前方距離。 |
| `slow_front_exception_required_cycles` | `3` | `3` | 低速例外に必要な連続判定周期数。 |

CSV例:

```csv
name,start_wp,end_wp,allow_overtake
s4,155,190,false
s5,190,265,true
s9,335,1,true
```

## 起動/入出力

主に `overtake_planner_node.cpp` が読みます。

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `enabled` | `true` | `true` | `false` ならplannerは介入せず、MPCの元参照を使う。 |
| `reference_package` | `multi_purpose_mpc_ros` | `multi_purpose_mpc_ros` | Frenet参照CSVを探すROS package。 |
| `reference_csv` | `env/final_ver3/traj_mincurv_manual.csv` | 同左 | Frenetの基準線。変えると `s/d`、壁余裕、左右offsetの意味が変わる。 |
| `own_vehicle_id` | `auto` | `auto` | `auto` は `ROS_DOMAIN_ID=N` から `dN` を推定する。 |
| `control_rate_hz` | `20.0` | `20.0` | planner更新周期。上げると反応は細かいが負荷が増える。 |
| `ego_stale_time_sec` | `0.50` | `0.50` | 自車odometryが古いとplannerを無効化する。 |
| `ignore_near_ego_m` | YAML未記載 | `1.0` | 自車近傍のV2X点を無視する。自車誤認識対策。 |
| `position_jump_threshold_m` | YAML未記載 | `5.0` | V2X位置が急に飛んだとき速度推定をリセットする。 |

## Horizon

`horizon_points` と `horizon_dt_sec` は、MPCへ出すoverride列の長さと時間刻みです。
未来横並びリスクの先読み時間は別の `future_side_prediction_*` が担当します。

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `horizon_points` | `50` | `20` | overrideの点数。baseline MPCでは20、delay-aware/hybridでは50へ広げる。 |
| `horizon_dt_sec` | `0.025` | `0.025` | override各点の時間刻み。 |

## BlockedRiskAnalyzer

`BlockedRiskAnalyzer` は、前方閉塞、横並び、parallel side candidate、左右pass gapを判定します。
状態遷移や候補生成は担当しません。

### 前方閉塞

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `lookahead_s_m` | `10.0` | `10.0` | 前方何mまで他車を見るか。 |
| `follow_trigger_s_m` | `12.0` | `12.0` | 前方車がこの距離より近いと `blocked=true` になりやすい。 |
| `same_corridor_width_m` | `0.90` | `0.90` | 自車と他車の横方向差がこの範囲なら同一コリドー。 |
| `dv_block_threshold_mps` | `0.20` | `0.20` | 自車が相手よりこの速度差以上速いと、遠めでも閉塞扱いしやすい。 |
| `opponent_stale_time_sec` | `0.50` | `0.50` | V2X他車情報が古いと無視する。 |

### 同方向フィルタ

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `same_direction_filter_enabled` | `true` | `true` | 十分な速度があり逆方向に見える相手を対象から外す。 |
| `same_direction_min_speed_mps` | `0.30` | `0.30` | 進行方向を信頼する最低速度。これ未満は方向不明として残す。 |
| `same_direction_min_s_dot_mps` | `0.05` | `0.05` | 参照線方向速度 `s_dot` がこの値以上なら同方向。 |

### 横並び/parallel side

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `side_by_side_s_m` | `4.0` | `4.0` | 狭義の横並び前後範囲。 |
| `side_margin_m` | `1.20` | `1.20` | 狭義の横並び横幅。 |
| `parallel_side_detection_enabled` | `true` | `true` | 広めの並走候補を未来リスクへ渡す。 |
| `parallel_side_s_m` | `12.0` | `12.0` | parallel sideの前後範囲。 |
| `parallel_side_margin_m` | `4.0` | `4.0` | parallel sideの横幅。 |
| `side_yield_s_m` | `0.30` | `0.30` | 横並び相手がこの前後差より前なら、後ろへ譲りやすい。 |

### Pass gap

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `min_pass_gap_m` | `1.8` | `1.80` | 左右に必要な追い越し空間。上げるとPASS候補が減る。 |
| `pass_gap_hysteresis_m` | `0.15` | `0.15` | 既に追い越し中の方向だけ、必要gapを少し緩める。 |
| `safety_ellipse_b_m` | `1.8` | `1.8` | pass gap必要量にも効く横方向安全幅。 |
| `min_ellipse_h` | `0.20` | `0.20` | pass gap必要量と他車安全評価に効く余裕。 |

`pass_gap_reason` は `ok`, `left_gap_narrow`, `right_gap_narrow`, `both_gap_narrow`, `no_target`, `large_lateral_error` を見ます。

## FutureSideBySideRiskAnalyzer

`FutureSideBySideRiskAnalyzer` は、現在の `side_by_side` または `parallel_side_candidate` が短時間後のコーナーで壁余裕を失うかを先読みします。
危険なら `future_yield_required=true` とし、`YIELD_BEHIND` を優先しやすくします。

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `future_side_prediction_enabled` | `true` | `true` | 未来横並び予測を使う。 |
| `future_side_prediction_horizon_sec` | `1.2` | `1.2` | 何秒先まで見るか。上げると早めに譲るが保守的。 |
| `future_side_prediction_dt_sec` | `0.3` | `0.3` | 予測の時間刻み。下げると細かいが少し重い。 |
| `future_side_yield_wall_clearance_m` | `0.35` | `0.35` | 未来の横並び維持目標がこの壁余裕を下回ると譲る。 |

## Core側ゲート

`OvertakePlannerCore` が、risk判定後に追い越し開始ゲートや大きい横ずれ時の凍結を加えます。

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `straight_only_overtake_enabled` | `true` | `true` | 直線ゲートが開いている時だけ新規PASS開始を許可する。 |
| `straight_overtake_max_curvature_m_inv` | `0.025` | `0.025` | 追い越し開始を許可する最大曲率。 |
| `straight_overtake_lookahead_m` | `12.0` | `12.0` | 開始ゲートが曲率を見る前方距離。 |
| `straight_overtake_release_hysteresis_m_inv` | `0.005` | `0.005` | 閉じた開始ゲートを開き直すヒステリシス。 |
| `overtake_permission_profile_enabled` | `true` | `true` | 区間許可CSVで新規PASS開始を制御する。 |
| `overtake_permission_lookahead_m` | `8.0` | `8.0` | 前方の不可区間を見て開始を早めに抑制する。 |
| `slow_front_exception_enabled` | `true` | `true` | 不可区間でも停止/低速前方車だけ例外的に追い越しを許す。 |
| `slow_front_exception_speed_mps` | `1.0` | `1.0` | 低速例外の相手速度しきい値。 |
| `slow_front_exception_distance_m` | `8.0` | `8.0` | 低速例外を使う前方距離。 |
| `slow_front_exception_required_cycles` | `3` | `3` | 低速例外を確定する連続周期数。 |
| `corner_side_yield_curvature_m_inv` | `0.05` | `0.05` | 横並び中に譲りを強めるコーナー曲率。下げるほど緩いカーブでも譲る。 |
| `corner_side_yield_lookahead_m` | `10.0` | `10.0` | 横並びコーナー判定で曲率を見る前方距離。 |
| `large_lateral_error_threshold_m` | `0.60` | `0.60` | 横ずれがこの値を超え、危険文脈があるとPASSを凍結して復帰寄りにする。 |
| `large_lateral_error_v_max_mps` | `8.5` | `2.5` | 大きい横ずれ時の速度上限。 |

## CandidateBuilder

`CandidateBuilder` は、候補ごとの横オフセット列と速度上限列を作ります。
ここでは候補を作るだけで、安全かどうかは `SafetyEvaluator` が評価します。

### 横目標/軌道形状

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `left_offset_m` | `0.80` | `0.80` | 左追い越し時の目標d。 |
| `right_offset_m` | `-0.80` | `-0.80` | 右追い越し時の目標d。 |
| `prepare_distance_m` | `8.0` | `8.0` | PASS目標dへ移る距離。上げると横移動が穏やか。 |
| `merge_distance_m` | `12.0` | `12.0` | 中心線へ戻る距離。 |
| `side_by_side_target_gap_m` | `0.75` | `0.75` | 横並び時に相手から確保したい横距離。 |
| `side_by_side_shift_distance_m` | `7.0` | `7.0` | 横並び維持目標へ移る距離。 |
| `corner_yield_target_d_m` | `0.0` | `0.0` | コーナー譲り時の横目標。通常は中心線。 |
| `outside_corridor_recovery_centering_time_sec` | `1.0` | `1.0` | 安全コリドー外、または `recovery_release_lateral_error_m` を超えるRECOVERYで、低速/停止中でも中心方向へ参照を寄せる時間目安。`0` 以下で距離ベースのみ。 |

### 候補速度

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `v_passthrough_mps` | `50.0` | `50.0` | plannerが速度を制限しない時の実質上限。 |
| `follow_speed_margin_mps` | `0.20` | `0.20` | FOLLOW時に前走車よりどれだけ遅くするか。 |
| `yield_speed_margin_mps` | `0.60` | `0.60` | YIELD/SIDE_BY_SIDEで相手よりどれだけ遅くするか。 |
| `yield_min_speed_cap_mps` | `0.50` | `0.50` | YIELD/SIDE_BY_SIDEで相手速度基準capが低くなりすぎる時の下限。PurePursuit fallback時の約2km/h低速化を上げたい時に調整する。 |
| `corner_follow_speed_margin_mps` | `0.20` | `0.20` | コーナー譲り時に相手よりどれだけ遅くするか。 |
| `side_by_side_speed_cap_mps` | `7.5` | `7.5` | SIDE_BY_SIDE_KEEPの通常速度上限。 |
| `corner_yield_v_max_mps` | `8.0` | `3.0` | コーナー譲り時の最大速度。 |
| `recovery_v_max_mps` | `8.5` | `8.5` | RECOVERY中の速度上限。 |
| `wall_margin_recovery_v_max_mps` | `8.5` | `8.5` | 安全コリドー外から復帰する時の速度上限。 |
| `safe_stop_v_mps` | `0.20` | `0.20` | SAFE_STOP候補の速度上限。0ではなく小さい正値を使う。 |

## SafetyEvaluator

`SafetyEvaluator` は、候補のd列と他車予測を見て `feasible` / `reject_reason` を決めます。

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `d_min_m` | `-1.35` | `-1.35` | 右側境界。 |
| `d_max_m` | `1.35` | `1.35` | 左側境界。 |
| `min_wall_margin_m` | `0.50` | `0.50` | 壁から残す余裕。 |
| `safety_ellipse_a_m` | `3.0` | `3.0` | 他車との前後方向安全楕円。 |
| `safety_ellipse_b_m` | `1.8` | `1.8` | 他車との横方向安全楕円。 |
| `min_ellipse_h` | `0.20` | `0.20` | 安全楕円の余裕しきい値。 |

安全コリドー:

```text
lower_d = d_min_m + min_wall_margin_m
upper_d = d_max_m - min_wall_margin_m
```

現在YAMLとfallbackでは `-0.85 <= d <= 0.85` です。

## BehaviorStateMachine

`BehaviorStateMachine` は、候補選択結果をそのままモードにせず、切替を安定化します。

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `pass_safe_required_cycles` | `5.0` | `5.0` | PASS候補が連続して安全と判定される必要回数。 |
| `merge_front_gap_m` | `6.0` | `6.0` | 追い越し後、前方ギャップがこの値を超えると戻り始める。 |
| `yield_rejoin_gap_m` | `3.0` | `3.0` | YIELD後に通常状態へ戻るための前方ギャップ。 |
| `corner_yield_rejoin_gap_m` | `5.5` | `5.5` | コーナー譲り後に戻るための前方ギャップ。 |
| `yield_rejoin_wall_clearance_m` | `0.25` | `0.25` | YIELD/RECOVERY解除に必要な壁余裕。 |
| `recovery_release_lateral_error_m` | `0.60` | `0.60` | ABORT_RECOVERYを抜けるために必要な中心線からの横ずれ上限。負値で無効。 |
| `yield_release_lateral_error_m` | `0.60` | `0.60` | YIELD_BEHIND/future_yield_holdを抜けるために必要な中心線からの横ずれ上限。負値で無効。 |
| `abort_timeout_sec` | `5.0` | `5.0` | 追い越し状態が長引いた時に復帰へ倒す時間。 |
| `min_mode_hold_time_sec` | `0.60` | `0.60` | モード切替直後の保持時間。 |
| `keep_mode_bonus` | `25.0` | `25.0` | 現在モードに沿う候補をスコア上優遇する強さ。 |
| `lateral_target_max_step_m` | `0.25` | `0.25` | MPCへpublishする横オフセット列の1周期あたり最大変化量。SAFE_STOPも対象。`0` 以下で無効。 |
| `high_speed_curve_lateral_hold_enabled` | `true` | `true` | 高速カーブ中の横目標holdを有効化する。 |
| `high_speed_curve_lateral_hold_min_speed_mps` | `4.0` | `4.0` | この速度以上で、カーブ中のYIELD/RECOVERY/SAFE_STOP/SPEED_GUARD横目標をhold対象にする。 |
| `high_speed_curve_lateral_hold_release_speed_mps` | `2.5` | `2.5` | この速度以下まで落ちたら、カーブ中でもholdを解除できる。 |
| `high_speed_curve_lateral_hold_release_curvature_m_inv` | `0.025` | `0.025` | この曲率以下ならカーブ脱出とみなし、holdを解除できる。 |

## PlannerOutputBuilder / 速度ガード

`PlannerOutputBuilder` は、選ばれた候補を `PlannerOutput` に詰めます。
また、unsafe候補や壁リスク、MPC health悪化時に、横方向候補を復活させず速度だけを落とす `SPEED_GUARD` を作ります。
`SPEED_GUARD` が速度だけを目的とする場合、横オフセットは中心線へ0埋めせず、現在の横位置を保持します。
複数の速度capが同時に成立した場合は、最も低いcapとその理由を採用します。

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `speed_only_fallback_enabled` | `true` | `true` | unsafeな横方向候補やSAFE_STOP infeasible時に速度only fallbackを出す。 |
| `speed_only_fallback_v_max_mps` | `8.0` | `3.0` | 速度only fallbackの上限。 |
| `wall_risk_speed_guard_enabled` | `true` | `true` | 壁余裕不足時に速度だけ落とす。 |
| `wall_soft_margin_m` | `0.25` | `0.25` | 壁リスク速度ガードを始めるソフト余裕。 |
| `wall_risk_v_max_mps` | `8.0` | `5.0` | 壁リスク時の速度上限。 |
| `mpc_health_speed_guard_enabled` | `true` | `true` | MPC health debugを見て速度を落とす。 |
| `mpc_health_infeasible_count_threshold` | `1` | `1` | infeasible countがこの値以上なら速度ガード。 |
| `mpc_health_solve_time_warn_ms` | `80.0` | `80.0` | solve timeがこの値以上なら速度ガード。 |
| `mpc_health_v_max_mps` | `8.0` | `3.0` | MPC health悪化時の速度上限。 |
| `mpc_health_stale_time_sec` | `0.60` | `0.60` | MPC health debugが古い場合のstale判定。 |
| `recovery_speed_guard_enabled` | `true` | `true` | `RECOVERY` / `ABORT_RECOVERY` 中に復帰専用の低速capを重ねる。 |
| `recovery_speed_guard_v_max_mps` | `3.0` | `3.0` | 復帰専用速度cap。`wall_risk_v_max_mps` や `mpc_health_v_max_mps` を高めにしていても、壁際復帰とPP fallback中はこの値で抑えやすくする。 |

## Section safety profile

区間安全プロファイルは、MPCのref velocity YAMLのように区間ごとに壁余裕や速度capの厳しさを変えるplanner側ポリシーです。
YAMLには空配列を常設せず、必要なときだけ同じ長さの配列を追加してください。

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `section_safety_profile_enabled` | `true` | `true` | 区間安全プロファイルを有効化する。 |
| `section_safety_names` | YAML未記載 | `[]` | 区間名。 |
| `section_safety_profiles` | YAML未記載 | `[]` | `wall_risk_moderate` / `side_by_side_corner_strict` など。 |
| `section_safety_role_policies` | YAML未記載 | `[]` | `outer_yields` など。 |
| `section_safety_start_s_m` | YAML未記載 | `[]` | s座標で区間開始を指定。 |
| `section_safety_end_s_m` | YAML未記載 | `[]` | s座標で区間終了を指定。 |
| `section_safety_start_wp` | YAML未記載 | `[]` | waypoint indexで区間開始を指定。 |
| `section_safety_end_wp` | YAML未記載 | `[]` | waypoint indexで区間終了を指定。 |

例:

```yaml
section_safety_names: ["first_corner"]
section_safety_profiles: ["side_by_side_corner_strict"]
section_safety_role_policies: ["outer_yields"]
section_safety_start_wp: [120]
section_safety_end_wp: [180]
```

| profile / role | 効果 |
|---|---|
| `wall_risk_moderate` | `wall_soft_margin_m` を1.15倍、速度capを0.85倍にする。 |
| `side_by_side_corner_strict` | `wall_soft_margin_m` を1.35倍、速度capを0.65倍にし、外側譲りを強める。 |
| `outer_yields` | 外側車両が壁リスクを持つサイドバイサイドでは `YIELD_BEHIND` を優先する。 |

## SAFE_STOP

`SAFE_STOP` は、追い越し、追従、譲り、復帰の通常候補が安全に成立しないときだけ使う最後のplanner内fallbackです。
MPC側で `0.0 m/s` が無効扱いにならないよう、停止意図は小さい正の速度上限で表現します。

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `safe_stop_enabled` | `true` | `true` | SAFE_STOP候補を使う。 |
| `safe_stop_v_mps` | `0.20` | `0.20` | SAFE_STOP中に出す速度上限。 |
| `safe_stop_trigger_cycles` | `1` | `1` | 通常fallbackがunsafeな状態が何周期続いたらSAFE_STOPへ入るか。 |
| `start_grace_safe_stop_enabled` | `true` | `true` | 発進直後の横並び/並走リスクで、前方閉塞が無い時だけSAFE_STOP突入を猶予する。 |
| `start_grace_duration_sec` | `8.0` | `8.0` | 最初の有効な自車状態を受けてから一度だけ、何秒間start graceを有効にするか。 |
| `start_grace_max_speed_mps` | `1.5` | `1.5` | start graceを適用する自車速度上限。負値で速度条件を無効化。 |
| `safe_stop_release_cycles` | `5` | `5` | 解除条件が何周期続いたらSAFE_STOPを抜けるか。 |
| `safe_stop_release_front_gap_m` | `5.0` | `5.0` | 再発進に必要な前方ギャップ。 |
| `safe_stop_release_wall_clearance_m` | `0.20` | `0.20` | 再発進に必要な壁余裕。 |
| `safe_stop_lateral_error_threshold_m` | `0.40` | `0.40` | SAFE_STOP目標dからの許容横ずれ。 |
| `safe_stop_release_speed_mps` | `0.50` | `0.50` | 再発進判定に入る自車速度上限。 |

SAFE_STOP中に停止要求が消えても、壁余裕または中心からの横ずれが解除条件を満たさない場合は `FREE_RUN` へ直帰せず、`ABORT_RECOVERY` に渡します。
このとき `CandidateBuilder` は SAFE_STOP / RECOVERY の参照を中央方向へ寄せ、`recovery_speed_guard_*` が有効なら復帰中の速度を追加で抑えます。

## 症状別の調整例

### 横並びコーナーで外側へ膨らむ

まずは速度と壁余裕の判定を保守的にします。

```yaml
future_side_yield_wall_clearance_m: 0.35
corner_side_yield_wall_clearance_m: 0.55
corner_yield_v_max_mps: 8.0
wall_risk_v_max_mps: 8.0
mpc_health_v_max_mps: 8.0
recovery_speed_guard_enabled: true
recovery_speed_guard_v_max_mps: 3.0
outside_corridor_recovery_centering_time_sec: 1.0
recovery_release_lateral_error_m: 0.60
yield_release_lateral_error_m: 0.60
```

それでも壁側へ逃げる場合は、相手から離れる横距離を少し弱めます。

```yaml
side_by_side_target_gap_m: 0.65
```

### 横並びなのに譲らず並び続ける

```yaml
side_yield_s_m: 0.20
yield_speed_margin_mps: 0.80
corner_side_yield_curvature_m_inv: 0.04
```

相手が少し前に出た時点で `YIELD_BEHIND` へ入りやすくなります。

### 追い越しに入りにくい

```yaml
min_pass_gap_m: 1.30
pass_safe_required_cycles: 3.0
follow_trigger_s_m: 14.0
```

追い越し判断が早く、緩くなります。
ただし接近リスクも増えるので、`/debug/overtake/metrics` の `reason` と `pass_gap_reason` を確認してください。

### 判断が左右・追従で揺れる

```yaml
min_mode_hold_time_sec: 0.80
keep_mode_bonus: 35.0
pass_gap_hysteresis_m: 0.25
lateral_target_max_step_m: 0.20
```

現在モードを維持しやすくなり、細かい切替が減ります。
反応は少し遅くなります。

## 調整時に見るdebug

`/debug/overtake/metrics` では、まず以下を見ます。

- `blocked`
- `side_by_side`
- `parallel_side_candidate`
- `corner_side_by_side`
- `future_side_by_side`
- `future_corner_side_by_side`
- `future_yield_required`
- `future_outer_wall_risk`
- `future_wall_clearance_m`
- `yield_reason`
- `straight_overtake_start_allowed`
- `overtake_start_abs_curvature`
- `overtake_start_gate_reason`
- `pass_gap_reason`
- `left_pass_gap_m`
- `right_pass_gap_m`
- `ego_wall_clearance_m`
- `active_override`
- `selected`
- `reason`
- `speed_only_fallback_active`
- `wall_risk_speed_guard_active`
- `mpc_health_speed_guard_active`
- `speed_cap_reason`
- `applied_speed_cap_mps`
- `safe_stop_triggered`
- `safe_stop_reason`
- `safe_stop_trigger_count`
- `safe_stop_release_count`
- `min_cbf_h`
- `cbf_slack`

パラメータを変えたら、まず期待したmodeとreasonに入っているかを見てください。
modeが変わっていない場合、速度や横オフセットの値だけを変えても期待した挙動にはなりにくいです。
