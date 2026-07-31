# Simple Pure Pursuit Parameter Guide

この資料は `simple_pure_pursuit` のパラメータを、MPC fallback と overtake planner 連携を含めて整理したものです。単位は SI 単位で、距離は m、角度は rad、速度は m/s、時間は sec です。

## Launchごとの有効値

| 項目 | C++デフォルト | `pure_pursuit.launch.xml` | `hybrid_delay_aware_mpc.launch.xml` | `pure_pursuit_mpc_horizon.launch.xml` |
| --- | ---: | ---: | ---: | ---: |
| `wheel_base` | `1.087` | `1.087` | `1.087` | `1.087` |
| `use_external_target_vel` | `false` | `false` | `true` | `true` |
| `external_target_vel` | `0.0` | `9.5` | `fallback_speed_mps` default `10.0` | `pure_pursuit_speed_mps` default `10.0` |
| `lookahead_gain` | `1.0` | `0.5` | `0.5` | `0.5` |
| `lookahead_min_distance` | `1.0` | `2.5` | `3.5` | `3.5` |
| `speed_proportional_gain` | `1.0` | `1.0` | `0.8` | `0.8` |
| `steering_tire_angle_gain` | `1.0` | `1.639` | `1.639` | `1.639` |
| `use_overtake_reference_override` | `false` | `true` | `use_overtake_planner` | `use_overtake_planner` |
| `use_mpc_predicted_horizon` | `false` | `false` | `true` | `true` |
| `max_mpc_horizon_age_sec` | `0.15` | `0.50` | `0.15` | `0.35` |
| `require_solved_mpc_health_for_horizon` | `false` | `false` | `false` | `true` |
| `max_mpc_health_age_sec` | `0.30` | `0.30` | `0.30` | `0.35` |
| `pp_control_delay_sec` | `0.0` | `0.0` | `0.0` | `0.0` |
| `steering_time_constant_sec` | `0.30` | `0.30` | `0.30` | `0.30` |
| `horizon_curvature_feedforward_gain` | `0.0` | `0.0` | `0.0` | `0.0` |

注意: `pure_pursuit.launch.xml` 単体のデフォルトと、hybrid launch から渡す値は別です。MPC から PurePursuit へ fallback した時に予測ホライズンを追わせる設定は、通常 `hybrid_delay_aware_mpc.launch.xml` 側を確認します。

hybrid launch では `simple_pure_pursuit` の入力 odom がすでに `/delay_aware_mpc/localization/kinematic_state` です。そのため PP 内部の delayed-pose compensation は二重補償を避けるため `pp_control_delay_sec=0.0` を安全デフォルトにしています。PPだけを raw odom で動かす実験をする場合に限り、`pp_control_delay_sec` を `0.05`-`0.20` へ上げて比較します。

## 基本制御

| パラメータ | 役割 | 調整の目安 |
| --- | --- | --- |
| `wheel_base` | PurePursuit の操舵角計算に使うホイールベース。 | 車両モデル値と合わせます。誤ると全体的に曲がり方がズレます。 |
| `lookahead_gain` | 速度に比例して lookahead 距離を伸ばす係数。 | 大きいほど直線で安定しますが、カーブで膨らみやすくなります。 |
| `lookahead_min_distance` | lookahead 距離の下限。 | 小さいほど鋭く曲がります。大きいほど操舵が穏やかになります。 |
| `actual_lookahead_distance_blend` | 要求先読み距離と、離散軌道上で実際に選ばれた点までの距離を操舵計算で混ぜる割合。 | `0.0` は従来、`1.0` は実距離。点間隔が粗い軌道の過大操舵を抑えます。 |
| `dual_preview_near_ratio` | 遠方プレビュー距離に対する近方プレビュー距離の割合。 | デフォルトは `0.5`。 |
| `dual_preview_blend` | 遠方点の操舵へ近方点の操舵を混ぜる割合。 | `0.0` は従来と完全互換、`1.0` は近方点のみ。 |
| `speed_proportional_gain` | 目標速度と現在速度の差から加速度指令を作る比例ゲイン。 | 大きいほど速度追従が強くなりますが、切替時の縦加速度が急になります。 |
| `steering_tire_angle_gain` | 算出した操舵角へのゲイン。 | 大きいほどよく曲がりますが、rate limit/angle limit に当たりやすくなります。 |

## 速度指令

| パラメータ | 役割 | 調整の目安 |
| --- | --- | --- |
| `use_external_target_vel` | trajectory の速度ではなく `external_target_vel` を使うか。 | hybrid fallback では `true` にして、PP の基本速度を明示します。 |
| `external_target_vel` | 外部指定の目標速度。 | 高いほど勝ちに行けますが、MPC timeout 後の回避余裕は減ります。 |
| `corner_speed_retention` | 曲率速度制限後の速度を最高速度側へ戻す割合。`0.0` は従来どおり、`1.0` は曲率による減速なし。 | A/B比較では `0.0` と `1.0` を使います。中間値ならコーナー減速だけを連続的に弱められます。 |
| `corner_speed_retention_lateral_error_soft` | 速度保持を弱め始める横偏差[m]。 | デフォルトは `0.5`。 |
| `corner_speed_retention_lateral_error_hard` | 速度保持を完全に解除し、従来の曲率速度へ戻す横偏差[m]。 | デフォルトは `1.0`。soft より大きく設定します。 |

MPC horizon を使う場合、horizon 上の速度が `external_target_vel` より低ければ、PP の速度は horizon 側で cap されます。`neutral_reference` / `fixed_neutral_reference` では、現在poseを表す先頭アンカー点の速度だけでcapすると停止付近で加速できなくなるため、速度capは最近傍点から少し先の点を参照します。`solver_prediction` ではMPCが出した即時減速を尊重するため、最近傍点速度をそのまま使います。さらに overtake override の speed cap が有効なら、その cap も適用されます。

## 入力stale guard

| パラメータ | 役割 | 調整の目安 |
| --- | --- | --- |
| `max_odom_age_sec` | odometry を fresh とみなす最大 age。 | 小さいほど安全寄りですが、通信/処理遅延で停止しやすくなります。 |
| `max_trajectory_age_sec` | 通常 trajectory を fresh とみなす最大 age。 | `/planning/scenario_planning/trajectory` が低頻度なら広げます。 |
| `max_override_age_sec` | overtake override を fresh とみなす最大 age。 | planner の publish 周期より少し広くします。 |
| `stop_on_stale_input` | 入力が stale/missing の時に停止指令を publish するか。 | 安全寄りなら `true`。レース中の一瞬の欠落を許したい場合のみ慎重に変更します。 |
| `diagnostic_throttle_sec` | stale 警告ログの throttle 周期。 | 調査時は短く、通常運用では長めにします。 |

通常 trajectory が stale でも、MPC horizon が usable なら `fresh_mpc_horizon` として制御継続できます。

## MPC predicted horizon

| パラメータ | 役割 | 調整の目安 |
| --- | --- | --- |
| `use_mpc_predicted_horizon` | PP が MPC の予測ホライズンを通常 trajectory より優先して使うか。 | hybrid fallback では `true` 推奨です。 |
| `max_mpc_horizon_age_sec` | horizon を fresh とみなす最大 age。 | MPC solve が 200-400ms かかる場合、`0.30`-`0.50` が候補です。古すぎる horizon を追うリスクとセットで調整します。 |
| `min_mpc_horizon_points` | horizon として使う最小点数。 | 短すぎる予測を避けます。通常は `5` 以上。 |
| `max_mpc_horizon_start_distance_m` | horizon の先頭点と ego の最大距離。 | 大きくすると古い/遠い horizon を拾いやすくなります。 |
| `min_mpc_horizon_arc_length_m` | horizon 全体の最小弧長。 | 短すぎる horizon で lookahead が末端に張り付くのを防ぎます。 |
| `require_solved_mpc_health_for_horizon` | MPC health が solved の時だけ horizon を使うか。 | PurePursuit主制御では `true` 推奨です。MPC infeasible時は通常trajectoryへ戻せます。 |
| `max_mpc_health_age_sec` | MPC health を fresh とみなす最大 age。 | MPC debug周期より少し広くします。`0.30`-`0.50` が調整候補です。 |
| `require_matching_overtake_horizon_contract` | activeなplanner overrideと、MPC horizonのmode/generationが一致する時だけhorizonを採用するか。 | 追い越しを使う`hybrid_delay_aware_mpc`と`pure_pursuit_mpc_horizon`では`true`です。不一致時は通常trajectory+overrideへ戻ります。 |

horizon は以下を満たす時だけ使われます。

- `use_mpc_predicted_horizon=true`
- 受信済みで `max_mpc_horizon_age_sec` 以内
- 点数が `min_mpc_horizon_points` 以上
- frame が `map`
- NaN/Inf を含まない
- 先頭点が `max_mpc_horizon_start_distance_m` 以内
- 弧長が `min_mpc_horizon_arc_length_m` 以上
- ego 最近傍 index が先頭から大きくズレていない
- `require_solved_mpc_health_for_horizon=true` の場合、`/mpc/speed_profile_debug` が fresh で `mpc_status=solved` かつ `mpc_infeasible_count=0`
- `require_matching_overtake_horizon_contract=true` かつoverride有効時は、`/mpc/predicted_horizon_contract` のstamp、`source=solver_prediction`、mode、generationがplanner requestと一致

MPC側の `/mpc/predicted_horizon` は、`OVERTAKE_LEFT`, `OVERTAKE_RIGHT`, `MERGE_BACK`, `ABORT_RECOVERY` 中だけ solver の予測結果をpublishします。
それ以外の通常走行、追従、追い越し準備中は、現在poseとMPC参照pathから作る neutral horizon をpublishします。
`neutral_horizon_publish_period_sec > 0` の場合、neutral horizon はMPC solve loopから分離され、固定周期のtimerで `fixed_neutral_reference` としてpublishされます。
neutral horizon の先頭アンカー点は現在poseを表しますが、速度は現在速度ではなく参照path速度を優先して入れます。
これにより、MPC solverが一時的に重くなっても PP fallback が horizon を stale 扱いしにくくなります。
`neutral_reference` を生成できない場合は、追い越し込み solver horizon へ戻さず empty horizon をpublishします。
これにより、PP fallbackが通常走行中に古い追い越し横オフセットを追い続けることを避けます。

`pure_pursuit_mpc_horizon` では、MPC側の `predicted_horizon_publish_mode` を `overtake_or_neutral` にします。
このモードでは、planner v3契約で認可された `OVERTAKE_LEFT`, `OVERTAKE_RIGHT`, `MERGE_BACK`、または必須回避として明示された `ABORT_RECOVERY` だけsolver予測horizonを出し、それ以外は固定周期のneutral horizonを出します。
Pure Pursuit側はhealth gateでempty/unsolved horizonを採用せず、通常trajectoryとovertake overrideによる走行へ戻ります。

## 曲率適応lookahead

| パラメータ | 役割 | 調整の目安 |
| --- | --- | --- |
| `curvature_adaptive_lookahead_enabled` | 曲率に応じて lookahead を短くするか。 | カーブでのインカット/膨らみ対策として `true` 推奨です。 |
| `curvature_lookahead_min_distance` | 曲率適応後の lookahead 下限。 | 小さいほど曲がりますが、操舵が急になります。 |
| `curvature_lookahead_sensitivity` | 曲率に対する短縮感度。 | 大きいほどカーブで手前を見るようになります。 |
| `curvature_lookahead_window_ratio` | 曲率を見る窓を base lookahead の何倍にするか。 | 大きいほど先のカーブを拾います。 |
| `curvature_lookahead_max_window_distance` | 曲率評価窓の上限距離。 | 直線中に遠すぎるカーブへ過反応する場合は下げます。 |
| `curvature_lookahead_min_arc_length` | 曲率計算に使う最小弧長。 | ノイズや近すぎる点列の影響を抑えます。 |
| `curvature_lookahead_smoothing_alpha` | lookahead 変化の平滑化係数。 | `1.0` に近いほど即応、`0.0` に近いほど滑らかです。 |

## Delayed pose compensation / DPP補助

| パラメータ | 役割 | 調整の目安 |
| --- | --- | --- |
| `input_steering_status` | 実舵角 `/vehicle/status/steering_status` の入力トピック。 | PP内部補償を使う時に必要です。 |
| `pp_control_delay_sec` | PP内でlookahead探索に使う ego pose を何秒先読みするか。 | hybridでは二重補償回避のため `0.0`。raw odom入力実験時のみ小さく上げます。 |
| `pp_prediction_dt_sec` | pose先読みの積分刻み。 | 通常 `0.02`。小さいほど精密ですが負荷が増えます。 |
| `steering_time_constant_sec` | 実舵角が直前のPP指令へ一次遅れで近づく時定数。 | delay-aware MPC と合わせるなら `0.30` が基準です。 |
| `steering_status_timeout_sec` | steering status を fresh とみなす最大 age。 | 古い実舵角で予測しないため `0.20` 程度にします。 |
| `min_velocity_for_delay_compensation_mps` | PP内部補償を有効にする最低速度。 | ゼロ速近傍の yaw drift を避けるため `0.20` 程度にします。 |

PP内部補償は、`pp_control_delay_sec > 0`、steering status が fresh、速度が `min_velocity_for_delay_compensation_mps` 以上の時だけ働きます。条件を満たさない時は従来poseでlookaheadを探します。MPC horizon の stale / start distance / nearest index 判定は従来どおり現在の入力 odom 基準で行い、古い horizon を誤って採用しない契約を維持します。

## Horizon curvature feed-forward

| パラメータ | 役割 | 調整の目安 |
| --- | --- | --- |
| `horizon_curvature_feedforward_gain` | MPC horizon / trajectory の符号付き曲率からステア補助量を足すゲイン。 | 安全デフォルトは `0.0`。使う場合は `0.1` など小さく始めます。 |
| `horizon_curvature_feedforward_max_rad` | feed-forward の絶対値上限。 | 過大操舵を避けるため `0.08` rad 程度に制限します。 |

feed-forward は raw tire-angle rad に加算してから `steering_tire_angle_gain` を適用します。入れすぎると後段の steering rate limiter に当たりやすくなるため、まずは debug の `curvature_feedforward_steering_rad` を確認します。

## Overtake override

| パラメータ | 役割 | 調整の目安 |
| --- | --- | --- |
| `use_overtake_reference_override` | `/overtake/reference_override` を PP 側にも適用するか。 | overtake planner と fallback PP の経路意図を揃えるため、hybrid では有効にします。 |
| `overtake_override_timeout_sec` | override の有効期限。 | planner publish 周期より少し長くします。v1/v3 lateralはtimeoutでclearする一方、最後に受理したv2 speed-onlyは明示inactiveまで速度capだけを保持します。 |
| `overtake_short_spatial_horizon_v_max_mps` | 空間profileが実行不能な場合の速度上限。 | `0.2 m/s`を上限とし、単独で引き上げません。 |
| `overtake_spatial_horizon_min_arc_m` | 停止近傍でも横profileに要求する最小検証済みarc。 | D1の50点・0.025秒RECOVERY終端約0.571 mに対して`0.5 m`。基準trajectory点間なら終端点を補間します。 |
| `overtake_spatial_horizon_min_time_sec` | 現在速度に対して要求する前方時間。 | `0.75 sec`。検証済みarcが`current_speed * time`未満なら横profileを使いません。 |
| `overtake_spatial_horizon_response_delay_sec` | 短profile許可時の保守的な制動応答遅れ。 | Plannerの制動証明と同等以上にします。既定は`0.25 sec`。 |
| `overtake_spatial_horizon_brake_decel_mps2` | 短profile許可時に証明済みとみなす減速度。 | 実ControllerとPlanner設定の共通上限`1.0 m/s^2`以下にします。既定は`1.0 m/s^2`。 |

MPC horizon が usable な時は、horizon を優先するため通常 trajectory への overtake lateral override は適用しません。ただし overtake speed cap は PP の目標速度 cap として使われます。MPC horizonが使えずV4空間profileを直接適用する場合、通常lookaheadより短くても、検証済み終端までに0.2 m/sへ制動可能で最低arc/前方時間を満たす低速profileだけを許可します。終端は基準trajectory上へ補間し、trajectory自体をそこで切るため、曲率・lookahead・操舵は未評価suffixを参照しません。

短profileの許可条件は設定で緩和できません。最小arc `0.5 m`、前方時間 `0.75 sec`、応答遅れ `0.25 sec` は下限として固定し、fail-safe速度は最大 `0.2 m/s` に制限します。検証減速度が `0 m/s^2` 以下または実行中Planner設定の `1.0 m/s^2` を超える設定は不正として横profileを拒否し、speed-only fail-safeへ移ります。

`[1, mode_id!=0, 0, 2, generation, speed_cap_mps]` のv2 speed-onlyは横offsetを含みません。受理後にpayloadが不正になった、または `overtake_override_timeout_sec` を超えた場合も、PPはbaseline trajectoryを使い続けて最後の有効speed capだけを適用します。`[1, 0, 0, 1, generation]` のexplicit inactiveはこの保持を解除します。v1/v3 lateral overrideは不正payload/timeoutでclearします。

## デバッグ確認

`/pure_pursuit/debug` には以下が出ます。

- `trajectory_source`: `trajectory`, `trajectory_overtake_override`, `mpc_horizon`
- `mpc_horizon_applied`: horizon を実際に使ったか
- `mpc_horizon_reject_reason`: 上記に加え、契約不一致時は `mpc_horizon_contract_missing`, `mpc_horizon_contract_stale`, `mpc_horizon_contract_stamp_mismatch`, `mpc_horizon_contract_source`, `mpc_horizon_contract_mode_mismatch`, `mpc_horizon_contract_generation_mismatch`
- `mpc_horizon_age_sec`, `mpc_horizon_points`, `mpc_horizon_start_distance_m`, `mpc_horizon_arc_length_m`
- `mpc_health_status`, `mpc_health_age_sec`, `mpc_infeasible_count`, `mpc_predicted_horizon_source`
- `mpc_horizon_contract_*`, `overtake_override_generation`: horizonの出所とplanner requestが同じsolver解かを確認するための契約情報
- `overtake_override_apply_reason`: `applied`, `insufficient_verified_spatial_horizon`, `spatial_endpoint_unavailable` など、横overrideを実際に採用/拒否した理由
- `overtake_spatial_horizon_arc_m`, `overtake_spatial_horizon_required_arc_m`: 基準trajectory上で実際に覆えた検証済みarcと、その周期の速度・lookahead・制動契約から要求したarc。該当しない時は`-1.0`
- `lookahead_distance_m`, `path_curvature_1pm`, `target_speed_mps`
- `control_pose_shifted`, `control_pose_x/y/yaw_rad`, `steering_source`, `steering_age_sec`
- `pure_pursuit_steering_tire_angle_rad`, `curvature_feedforward_steering_rad`, `signed_path_curvature_1pm`

`/mpc/speed_profile_debug` の `mpc_predicted_horizon_source` では、MPCがpublishしたhorizonの種類を確認できます。
通常時は `fixed_neutral_reference` または `neutral_reference`、実追い越し/merge中は `solver_prediction`、MPC未解決時またはneutral生成不可時は `empty` / `fixed_neutral_empty` になります。
`neutral_horizon_cache_age_sec` は固定周期publishで再利用しているneutral horizon形状の経過時間です。

MPC が遅い場面で horizon を使わせたい時は、まず `mpc_horizon_reject_reason` が `stale` かどうかを確認します。
`stale` が多い場合はMPC側の `neutral_horizon_publish_period_sec` を有効にし、それでも残る場合だけ `max_mpc_horizon_age_sec` を広げます。
