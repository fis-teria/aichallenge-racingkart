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
| `wall_risk_v_max_mps` | `10.0` | `5.0` | 壁リスク時の速度上限。壁外・衝突時は別のhard安全経路が優先する。 |
| `mpc_health_v_max_mps` | `10.0` | `3.0` | MPC不調時の速度上限。stale/infeasibleのhard failureは別のfail-closed経路が優先する。 |
| `recovery_speed_guard_v_max_mps` | `3.0` | `3.0` | `RECOVERY` / `ABORT_RECOVERY` 中だけ、壁リスク・MPC不調・大横ずれ時に追加で速度を絞る。通常の壁/MPC capを攻めた値にしている場合の保険。 |

スタート直後から第1コーナーまで横並びを認識しない場合:

| パラメータ | 現在値 | fallback | 見る理由 |
|---|---:|---:|---|
| `side_by_side_s_m` | `1.0` | `4.0` | 狭義の横並び前後範囲。 |
| `side_margin_m` | `1.20` | `1.20` | 狭義の横並び横幅。広げすぎると通常の横並びが過敏になる。 |
| `parallel_side_detection_enabled` | `true` | `true` | 広めの並走候補をdebug/予測の入力として記録する。単独では譲り根拠にしない。 |
| `parallel_side_s_m` | `12.0` | `12.0` | 広めの並走候補の前後範囲。行動判定には `side_by_side_s_m` を使う。 |
| `parallel_side_margin_m` | `4.0` | `4.0` | 広めの並走候補の横幅。行動判定には `side_margin_m` を使う。 |

カーブで追い越し開始してほしくない場合:

| パラメータ | 現在値 | fallback | 見る理由 |
|---|---:|---:|---|
| `straight_only_overtake_enabled` | `true` | `true` | 直線以外で新規PASS開始を止める。 |
| `straight_overtake_max_curvature_m_inv` | `0.025` | `0.025` | 追い越し開始を許可する最大曲率。下げるほど保守的。 |
| `straight_overtake_lookahead_m` | `12.0` | `12.0` | 追い越し開始ゲートが先読みする距離。 |
| `straight_overtake_release_hysteresis_m_inv` | `0.005` | `0.005` | 一度閉じたゲートを開き直すためのヒステリシス。 |
| `gentle_curve_safe_pass_enabled` | `true` | `false` | 通常curve gateが閉じた緩い曲線で、制限済みPASSをSafetyEvaluatorへ評価する明示opt-in。 |
| `gentle_curve_safe_pass_max_curvature_m_inv` | `0.090` | `0.0` | この値以下だけを例外候補にする上限。通常gateの代替にはしない。 |
| `gentle_curve_safe_pass_v_max_mps` | `10.0` | `0.0` | 例外PASS中もラッチする絶対速度上限。実際は横加速度上限との小さい方を使う。 |
| `gentle_curve_safe_pass_max_lateral_displacement_m` | `2.20` | `0.0` | 例外PASSで開始時anchor dから動ける最大横移動量。PASS中に現在d基準で累積拡大しない。 |
| `gentle_curve_safe_pass_max_lateral_accel_mps2` | `4.0` | `0.0` | 必須の横加速度上限。`v <= sqrt(a_lat_max / abs(kappa))` を候補と実行中の両方に適用する。0以下・非有限なら例外PASSを閉じる。 |
| `gentle_curve_safe_pass_max_cbf_slack` | `0.0` | `0.0` | 許可するPASS候補の最大CBF slack。`0` はslackなしだけを許可する。 |
| `gentle_curve_safe_pass_bypass_mode_hold_enabled` | `true` | `false` | 同周期の制限PASSがSafetyEvaluatorを通り続け、必要連続回数に達した時だけFOLLOWの最低holdを通過する。時間だけでgateは延長しない。 |

ref velocity区間を元に追い越し開始を許可/禁止したい場合:

| パラメータ | 現在値 | fallback | 見る理由 |
|---|---:|---:|---|
| `overtake_permission_profile_enabled` | `true` | `true` | `config/overtake_permission.csv` の区間許可gateを使う。 |
| `overtake_permission_package` | `overtake_planner` | `overtake_planner` | 追い越し許可CSVを探すROS package。 |
| `overtake_permission_csv` | `config/overtake_permission.csv` | 同左 | `name,start_wp,end_wp,allow_overtake` 形式のCSV。 |
| `default_overtake_allowed` | `true` | `true` | CSVに該当しない区間で追い越し開始を許すか。 |
| `overtake_permission_lookahead_m` | `8.0` | `8.0` | 近い将来の不可区間も見て追い越し開始を止める距離。 |
| `slow_front_exception_enabled` | `true` | `true` | 不可区間でも前方車が停止/低速なら例外的にPASS開始を許す。 |
| `slow_front_permission_exception_enabled` | `true` | `false` | `slow_front_exception_enabled` に加え、停止/低速の連続判定、現在地点の禁止区間、SafetyEvaluator通過済みPASS、freshな入力をすべて満たす時だけpermission開始gateを例外許可する。 |
| `slow_front_exception_speed_mps` | `1.0` | `1.0` | 低速前走車の診断用しきい値。高曲率とlookahead先だけの禁止は迂回しない。 |
| `slow_front_exception_distance_m` | `8.0` | `8.0` | 低速前走車の診断対象距離。 |
| `slow_front_exception_required_cycles` | `3` | `3` | 低速前走車を連続判定する周期数。 |
| `slow_obstacle_chain_enabled` | `true` | `true` | 同一コリドー内かつ接近中の低速parallel車だけを前方閉塞へ昇格する。 |
| `slow_obstacle_chain_distance_m` | `12.0` | `12.0` | 停止車列として昇格を検討する最大前方距離。 |
| `early_stationary_parallel_pass_enabled` | `true` | `false` | 全V2X snapshot fresh・全観測車両を包含・MPC healthyで、停止した前方parallel車だけを同一コリドーへ入る前からPASS候補としてGate 2へ載せる。 |
| `early_stationary_parallel_permission_exception_enabled` | `true` | `false` | `slow_front_permission_exception_enabled` と `early_stationary_parallel_pass_enabled` に加える明示opt-in。確認済みの停止parallel車について、現在地点の禁止区間でもGate 2通過済みPASSだけを開始可能にする。CSVのpermission診断、lookahead禁止、曲率、future-yield、reentry、壁・CBF・鮮度の拒否は維持する。 |
| `early_stationary_parallel_pass_distance_m` | `8.0` | `8.0` | early PASS probeを許す最大前方距離。 |
| `early_stationary_parallel_pass_lateral_width_m` | `1.50` | `1.50` | `same_corridor_width_m` より外側で、early PASS probeを許す最大横差。`parallel_side_margin_m` 以下でなければならない。 |

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
| `drivable_corridor_enabled` | `true` | `true` | `s` ごとの実走行可能幅を使う。ロードまたは参照照合に失敗するとplanner overrideを無効化する。 |
| `drivable_corridor_package` / `drivable_corridor_csv` | `overtake_planner` / `config/final_ver3_drivable_corridor.csv` | 同左 | final_ver3のlanelet2境界から作った物理回廊。参照CSVを変える時は必ず同じサンプル列で再生成する。 |
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
| `lookahead_s_m` | `15.0` | `10.0` | 前方何mまで他車を見るか。PASS候補を早めから毎周期評価する。 |
| `follow_trigger_s_m` | `12.0` | `12.0` | 前方車がこの距離より近いと `blocked=true` になりやすい。gap-closingを有効にする場合は `follow_gap_closing_engage_gap_m` 以上にする。 |
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
| `parallel_follow_enabled` | `true` | `false` | 同一コリドー外でも、SafetyEvaluatorを通った前方近接parallel車を現d保持FOLLOWの対象へ追加する。 |
| `parallel_follow_s_m` | `12.0` | `12.0` | parallel FOLLOW対象として見る前方距離。短くすると近い車だけを車間形成対象にする。 |
| `parallel_follow_lateral_width_m` | `1.20` | `1.20` | parallel FOLLOW対象にする最大横差。`same_corridor_width_m` より広い値にした範囲だけが追加対象になる。 |
| `side_yield_s_m` | `0.30` | `0.30` | 横並び相手がこの前後差より前なら、後ろへ譲りやすい。 |

### Pass gap

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `min_pass_gap_m` | `1.8` | `1.80` | 左右に必要な追い越し空間。上げるとPASS候補が減る。 |
| `pass_gap_hysteresis_m` | `0.25` | `0.25` | 既に追い越し中の方向だけ、必要gapを少し緩める。 |
| `dynamic_pass_candidate_enabled` | `true` | `true` | 静的gapが狭くても左右PASS候補を生成し、楕円・壁の時系列安全評価で最終判定する。 |
| `safety_ellipse_b_m` | `1.8` | `1.8` | pass gap必要量にも効く横方向安全幅。 |
| `min_ellipse_h` | `0.20` | `0.20` | pass gap必要量と他車安全評価に効く余裕。 |

`can_pass_left/right` は静的gapの診断値です。`dynamic_pass_candidate_enabled=true` の実運用では、PASS開始は候補の時系列安全評価、開始から目標d到達まで0.25 m刻みで確認するs依存回廊、直線gate、追い越し許可区間をすべて満たす時だけです。`localized_latched` でも実際のavoid/full-offset markerを同じ刻みで確認します。horizon内だけ安全でも目標dへ届く前に回廊が狭くなる候補は `pass_target_unreachable` で拒否します。`early_stationary_parallel_pass_*` は、fresh・同方向・接近中・停止・前方8 m以内かつ指定横幅内のparallel車だけをこのPASS評価へ早期に載せます。Gate 2不合格ならPASSは開始せず、同周期のYIELD/RECOVERY/SAFE_STOP評価へ戻ります。`slow_front_permission_exception_enabled=true` と `early_stationary_parallel_permission_exception_enabled=true` の場合だけ、同一IDを必要周期連続確認した停止parallel車も、現在地点の禁止区間でGate 2通過済みPASSを開始できます。CSVのpermission値はfalseのまま記録し、lookahead先だけの禁止、高曲率、future-yield、reentry、壁・CBF、stale判定は迂回しません。別経路の `gentle_curve_safe_pass_*` は停止/低速車ではない直接前走車に対しても、緩い曲線・通常permission・入力fresh・future yieldなし・制限済みPASSのSafetyEvaluator通過（CBF slack上限内）の全条件を満たす時だけ開始します。すでにPASS中は反対側へ横切って切り替えず、現在側の候補がunsafeならYIELD/RECOVERYへ戻ります。

early probeは、V2X snapshotを含む全入力がfreshで全観測車両を評価対象に含み、MPCがhealthyな時だけ有効です。Gate 2がPASSを拒否した周期はFASTESTへ戻さず、YIELD/RECOVERY/SAFE_STOPだけを評価します。

### 停止障害物と制動予測

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `max_brake_decel_mps2` | `1.0` | `1.0` | `s(t)` 評価に使う想定減速度。plannerの保守上限 `1.5 m/s^2` を超えてはいけない。 |
| `longitudinal_response_delay_sec` | `0.25` | `0.25` | 減速指令から実減速までの保守的な遅れ。上げると必要停止距離が増える。 |
| `stationary_obstacle_speed_threshold_mps` | `0.30` | `0.30` | 前方停止障害物として分類する相手速度上限。 |

停止障害物では、遅れ後に最大制動した最遠到達距離で `s(t)` と `predicted_speed_mps(t)` を作ります。一方、MPC/PPへ出す `v_ref[0]` は目標capです。下流は先頭値を各周期に直接消費するため、予測用の遅れを `v_ref` に入れると制動要求が毎周期先送りになるからです。安全評価は保守的な遅れを保持したまま、速度capだけを即時に要求します。FOLLOW/YIELDの必要制動距離が安全楕円を除いた前方距離を超える場合は通常候補から外し、SAFE_STOPまたは既存speed-only fallbackへ倒します。

起動時に `horizon_points >= 2`、有限かつ正の `horizon_dt_sec`、`0 < max_brake_decel_mps2 <= 1.5`、有限かつ非負の遅れ・停止車閾値を検証します。不正ならerrorを出してplanner overrideを無効化し、baseline controller/watchdogに委ねます。`max_brake_decel_mps2` を変える前には、実際に選ばれるcontrol mode（Pure Pursuit / mux / MPC）の制動可能値がこの想定以上であることをログと設定で確認します。

`pass_gap_reason` は `ok`, `left_gap_narrow`, `right_gap_narrow`, `both_gap_narrow`, `no_target`, `large_lateral_error` を見ます。

## FutureSideBySideRiskAnalyzer

`FutureSideBySideRiskAnalyzer` は、現在の `side_by_side`、または実際に接近して狭義の横並びへ入る `parallel_side_candidate` が短時間後のコーナーで壁余裕を失うかを先読みします。
広いparallel観測だけでは `future_yield_required=true` にしません。

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
| `slow_front_exception_enabled` | `true` | `true` | 停止/低速前方車の連続検出を有効にする。 |
| `slow_front_permission_exception_enabled` | `true` | `false` | 禁止permissionの例外を明示的に有効にする。PASS候補がSafetyEvaluatorを通り、現在地点が禁止、入力fresh、横並び/未来譲りなしの場合に限る。 |
| `slow_front_exception_speed_mps` | `1.0` | `1.0` | 低速例外の相手速度しきい値。 |
| `slow_front_exception_distance_m` | `8.0` | `8.0` | 低速例外を使う前方距離。 |
| `slow_front_exception_required_cycles` | `3` | `3` | 低速例外を確定する連続周期数。 |
| `slow_obstacle_chain_enabled` | `true` | `true` | frontではなくparallel-sideで見えた停止/低速前方車もPASS対象に載せる。 |
| `slow_obstacle_chain_distance_m` | `12.0` | `12.0` | slow obstacle chain昇格を許す前方距離。 |
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
| `left_offset_m` | `2.10` | `0.70` | `legacy_fixed_offset` 時だけ使う左PASS固定目標d。 |
| `right_offset_m` | `-2.10` | `-0.70` | `legacy_fixed_offset` 時だけ使う右PASS固定目標d。 |
| `pass_target_policy` | `minimum_clearance` | `legacy_fixed_offset` | `minimum_clearance` は、相手楕円間隔と `pass_target_lateral_margin_m` を満たす最小横移動を目標にする。余計な壁側への横移動はせず、raw profile・壁・SafetyEvaluatorを通る場合だけ採用する。 |
| `overtake_lateral_profile_mode` | `localized_latched` | `legacy` | `localized_latched` では停止車列の対象sと回避区間を保持し、通過直後に中心へ戻りすぎるのを抑える。 |
| `pass_horizon_publish_mode` | `overtake_only` | `prepare_and_overtake` | `overtake_only` では `PREPARE_OVERTAKE_*` 中に内部PASS判定だけ進め、MPCへはFOLLOW horizonを出す。 |
| `prepare_distance_m` | `8.0` | `8.0` | PASS目標dへ移る距離。上げると横移動が穏やか。 |
| `merge_distance_m` | `12.0` | `12.0` | 中心線へ戻る距離。 |
| `side_by_side_target_gap_m` | `0.75` | `0.75` | 横並び時に相手から確保したい横距離。 |
| `side_by_side_shift_distance_m` | `7.0` | `7.0` | 横並び維持目標へ移る距離。 |
| `corner_yield_target_d_m` | `0.0` | `0.0` | コーナー譲り時の横目標。通常は中心線。 |
| `parallel_follow_enabled` | `true` | `false` | 有効時のparallel FOLLOWは通常FOLLOWと違い、`target d = 0` に戻さず現在の `ego.frenet.d` を保持して減速する。 |
| `outside_corridor_recovery_centering_time_sec` | `1.0` | `1.0` | 安全コリドー外、または `recovery_release_lateral_error_m` を超えるRECOVERYで、低速/停止中でも中心方向へ参照を寄せる時間目安。`0` 以下で距離ベースのみ。 |

### 候補速度

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `v_passthrough_mps` | `50.0` | `50.0` | plannerが速度を制限しない時の実質上限。 |
| `follow_speed_margin_mps` | `0.00` | `0.20` | 通常FOLLOW時に前走車よりどれだけ遅くするか。PASS不能時も同等速度を維持する。 |
| `follow_gap_closing_enabled` | `true` | `false` | 十分離れた同一レーンの通常前走車だけ、速度bonusを許可する明示opt-in。 |
| `follow_gap_closing_target_gap_m` | `4.5` | `5.0` | gapを詰める目標距離。安全楕円の長手半径以上が必須。 |
| `follow_gap_closing_engage_gap_m` | `10.0` | `6.0` | これ以上離れた時だけbonusを有効化する距離。target以上が必須。 |
| `follow_gap_closing_speed_gain_per_m` | `0.15` | `0.10` | targetを超えたgap 1 m当たりの速度bonus。 |
| `follow_gap_closing_max_speed_bonus_mps` | `0.80` | `0.30` | 前走車基準capに足す最大速度bonus。 |
| `follow_gap_closing_assumed_accel_mps2` | `3.0` | `3.0` | SafetyEvaluatorのs(t)で仮定する最大加速。下流上限以上かつ3.0以下にする。 |
| `pass_speed_cap_mps` | `10.0` | `10.0` | PASS候補の速度上限。fresh入力かつMPC健全時にだけ加速予測とセットで使う。 |
| `pass_assumed_accel_mps2` | `3.0` | `3.0` | PASS候補のSafetyEvaluator用最遠到達距離に使う加速。実際の`v_ref`と一致させる。 |
| `pass_target_lateral_margin_m` | `0.10` | `0.10` | 楕円制約を満たす最小横間隔に足す余裕。小さくしてもSafetyEvaluator本体の楕円は緩まない。 |
| `yield_speed_margin_mps` | `0.60` | `0.60` | YIELD/SIDE_BY_SIDEで相手よりどれだけ遅くするか。 |
| `yield_min_speed_cap_mps` | `0.50` | `0.50` | YIELD/SIDE_BY_SIDEで相手速度基準capが低くなりすぎる時の下限。通常走行の最高速度ではない。これを高くすると接近並走で後方へ譲れず、CBF safe-stopへ入りやすくなる。 |
| `corner_follow_speed_margin_mps` | `0.20` | `0.20` | コーナー譲り時に相手よりどれだけ遅くするか。 |
| `side_by_side_speed_cap_mps` | `10.0` | `7.5` | SIDE_BY_SIDE_KEEPの通常速度上限。横並び・譲りの安全判定は別途維持する。 |
| `corner_yield_v_max_mps` | `8.0` | `3.0` | コーナー譲り時の最大速度。 |
| `recovery_v_max_mps` | `10.0` | `8.5` | RECOVERY中の速度上限。壁外・stale・CBF失敗の保護経路は別の低速guardを維持する。 |
| `wall_margin_recovery_v_max_mps` | `8.5` | `8.5` | 安全コリドー外から復帰する時の速度上限。 |
| `safe_stop_v_mps` | `0.20` | `0.20` | SAFE_STOP候補の速度上限。0ではなく小さい正値を使う。 |

### 復帰ゲートとMPC health

| パラメータ | 現在値 | fallback | 変更すると何が変わるか |
|---|---:|---:|---|
| `reentry_hold_v_max_mps` | `0.50` | `0.50` | CBF衝突、MPC infeasible、入力stale時のhard hold上限。上げない。 |
| `reentry_mpc_degraded_hold_v_max_mps` | `10.0` | `3.0` | solve timeだけの一過性遅延で、全相手へ安全評価済みの現在d保持を出す時だけの上限。中心線へは復帰しない。 |
| `post_abort_curve_hold_v_max_mps` | `10.0` | `8.5` | ABORT後の再合流が認可・中心収束した高速カーブで、PASSを凍結してSafetyEvaluator済みRECOVERYを出す上限。 |
| `reentry_mpc_latency_degraded_enter_samples` | `2` | `1` | 新しいMPC debug sampleでlatency warningがこの回数続いた時、SafetyEvaluator済みの現d holdへ入る。単発は速度capだけ。 |
| `reentry_mpc_unhealthy_enter_samples` | `3` | `2` | latency warningがさらに連続した時、hard holdへ落とす回数。planner周期では数えない。 |
| `reentry_mpc_healthy_release_samples` | `3` | `3` | hard/transient holdから復帰判定を再開するために必要な連続healthy MPC sample数。既存`reentry_safe_cycles`も別途必要。 |

`safety_ellipse_*`、`min_ellipse_h`、V2X/ego/MPC debug stale、MPC infeasibleはこのdegraded holdの対象外です。常にhard holdまたは既存watchdogへfail-closedします。

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
`pass_horizon_publish_mode=overtake_only` の場合でも、PASS候補は内部評価と安全周期の蓄積に使います。
変わるのはMPCへpublishする候補だけで、`PREPARE_OVERTAKE_*` 中は `FOLLOW` horizon を出し、`OVERTAKE_*` に入ってから `PASS_LEFT/RIGHT` horizon を出します。

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
| `speed_only_fallback_v_max_mps` | `1.0` | `1.0` | 速度only fallbackの上限。 |
| `normal_recovery_speed_only_v_max_mps` | `10.0` | `10.0` | 追越禁止区間でreentry gateが許可した通常復帰に重ねる速度上限。SafetyEvaluator済みの横復帰列は中心収束まで維持する。SAFE_STOP、接触、壁余裕、MPC healthのfail-safe capは変更しない。 |
| `opponent_collision_fallback_v_max_mps` | `0.5` | `0.5` | `opponent_collision` で横候補がunsafeな場合だけ使う低速上限。後続車や優先権なしの膠着ではこちらを使う。 |
| `side_by_side_leader_priority_enabled` | `true` | `true` | 横並び/並走で自車が明確に先行している場合だけ、SAFE_STOP要求と `opponent_collision` fallbackの低速固定を緩める。安全評価自体は無効化しない。 |
| `side_by_side_leader_priority_enter_s_m` | `1.0` | `1.0` | 先行車扱いへ入るために必要な、相手が後方にいる距離。`side_delta_s <= -enter` または `parallel_side_delta_s <= -enter` で入る。 |
| `side_by_side_leader_priority_release_s_m` | `0.3` | `0.3` | 先行車扱いを解除しにくくする距離。`enter` より小さくしてチャタリングを抑える。 |
| `side_by_side_leader_priority_hold_sec` | `1.0` | `1.0` | 先行車扱いを最低保持する時間。短すぎるとd1/d2間で優先権が揺れやすい。 |
| `side_by_side_leader_priority_v_max_mps` | `3.0` | `3.0` | 先行車扱い中に `opponent_collision` fallbackへ使う速度上限。高くすると先行車が逃げやすいが、壁/制御遅延リスクは増える。 |
| `wall_risk_speed_guard_enabled` | `true` | `true` | 壁余裕不足時に速度だけ落とす。 |
| `wall_soft_margin_m` | `0.25` | `0.25` | 壁リスク速度ガードを始めるソフト余裕。 |
| `wall_risk_v_max_mps` | `10.0` | `5.0` | 壁リスク時の速度上限。 |
| `mpc_health_speed_guard_enabled` | `true` | `true` | MPC health debugを見て速度を落とす。 |
| `mpc_health_infeasible_count_threshold` | `3` | `1` | infeasible countがこの値以上なら速度ガード。 |
| `mpc_health_solve_time_warn_ms` | `450.0` | `80.0` | solve timeがこの値以上なら速度ガード。 |
| `mpc_health_v_max_mps` | `10.0` | `3.0` | MPC health悪化時の速度上限。 |
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
| `start_grace_duration_sec` | `8.0` | `8.0` | 最初に自車速度が動き出し判定を超えてから一度だけ、何秒間start graceを有効にするか。WAIT_START中の有効ego受信時間では消費しない。 |
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
mpc_health_v_max_mps: 3.0
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
- `leader_priority_active`
- `leader_priority_latched`
- `leader_priority_id`
- `leader_priority_delta_s`
- `leader_priority_reason`
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
