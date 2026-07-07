# types.hpp

対象:

- `include/overtake_planner/types.hpp`

## 役割

`overtake_planner` 全体で共有する型定義です。
このファイルを読むと、plannerが何を入力として見て、何を判断材料として保持し、最後に何をpublishするかが分かります。

## enum

| 型 | 値 | 役割 |
|---|---|---|
| `BehaviorMode` | `FREE_RUN`, `FOLLOW_BLOCKED`, `PREPARE_OVERTAKE_LEFT`, `OVERTAKE_LEFT`, `MERGE_BACK`, `ABORT_RECOVERY`, `SIDE_BY_SIDE_KEEP`, `YIELD_BEHIND`, `SAFE_STOP`, `SPEED_GUARD` | 状態機械が保持する運転モード。 |
| `CandidateType` | `FASTEST`, `FOLLOW`, `PASS_LEFT`, `PASS_RIGHT`, `RECOVERY`, `SIDE_BY_SIDE_KEEP`, `YIELD_BEHIND`, `SAFE_STOP` | 1周期内で評価する候補経路の種類。 |

`BehaviorMode` は「今どういう運転状態か」、`CandidateType` は「今回評価する案」です。
同じものに見えますが、候補は一時的な案、modeは時間方向に保持される状態です。

## 状態表現

| 型 | 処理内容 |
|---|---|
| `ReferencePoint` | 参照CSVの1点。`s`, `x`, `y`, `yaw`, `kappa`, `v_ref` を持つ。 |
| `FrenetPose` | 参照線に対する `s`, `d`, `yaw_error`。追い越し判断の主座標。 |
| `EgoState` | 自車の現在状態。odomから作られる。 |
| `OpponentState` | V2Xから作る他車状態。位置、推定速度、Frenet座標を持つ。 |
| `PredictedOpponent` | 他車を短時間だけ等速予測した列。安全評価で使う。 |

## 判断材料

`BlockedInfo` は、前方車、横並び、pass gap、未来リスク、壁余裕をまとめる中心的な構造体です。

主なフィールド:

- `blocked`: 前方車で詰まっているか。
- `side_by_side`: 狭義の横並びか。
- `parallel_side_candidate`: 広めの並走候補か。
- `future_yield_required`: 未来予測上、譲りが必要か。
- `can_pass_left` / `can_pass_right`: 左右に追い越し余裕があるか。
- `pass_gap_reason`: 左右gap判定の理由。
- `ego_wall_clearance_m`: 自車の安全コリドー余裕。

## 候補と出力

`CandidateTrajectory` は、MPCへ渡す可能性がある短いhorizonの候補です。

- `s[]`, `d[]`: Frenet上の候補列。
- `x[]`, `y[]`, `yaw[]`: Cartesianへ戻した候補列。
- `v_ref[]`: 速度上限列。
- `feasible`: 安全評価で使える候補か。
- `reject_reason`: `wall_margin` や `opponent_collision` など。

`PlannerOutput` はROSノードへ返す最終結果です。

- `mode`: 最終モード。
- `selected`: 採用した候補種別。
- `lateral_offsets`: `/overtake/reference_override` の `d[]`。
- `speed_caps`: `/overtake/reference_override` の `v_ref[]`。
- `active_override`: MPC overrideを有効にするか。
- debug用に `safe_stop_*`, `speed_cap_reason`, `mpc_health` も持つ。

## PlannerConfig

ROS parameterを集約した設定です。
ソース内では各クラスが `PlannerConfig` を参照して、しきい値、速度上限、壁余裕、状態保持時間を決めます。

特に経路生成に効く値:

- `left_offset_m`, `right_offset_m`
- `prepare_distance_m`, `merge_distance_m`
- `side_by_side_target_gap_m`, `side_by_side_shift_distance_m`
- `recovery_v_max_mps`, `wall_margin_recovery_v_max_mps`
- `outside_corridor_recovery_centering_time_sec`

