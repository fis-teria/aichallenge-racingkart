# planner_output_builder.cpp

対象:

- `src/planner_output_builder.cpp`
- `include/overtake_planner/planner_output_builder.hpp`

## 役割

Coreが選んだ候補、状態機械のmode、安全停止情報、速度guard情報を `PlannerOutput` に整形します。
MPCへ渡す `/overtake/reference_override` の実体は、この出力の `lateral_offsets` と `speed_caps` です。

## ローカルhelper

| 関数 | 処理 |
|---|---|
| `finitePositiveOr()` | 正の有限値ならその値、そうでなければfallback。 |
| `candidateSpeedCapOr()` | 候補の `v_ref[]` から最小の有効速度capを取り出す。 |
| `applyUniformSpeedCap()` | 全horizon点に同じ速度capを適用する。既存capより低い値だけ採用する。 |
| `uniformVector()` | 指定長の同一値vectorを作る。 |
| `clampedCurrentLateralOffset()` | 現在の自車dを安全コリドー内へclampする。 |

## `PlannerOutputBuildInput`

`build()` に渡す入力をまとめた構造体です。

含むもの:

- mode
- ego
- selected candidate
- blocked info
- safe stop context
- safe stop candidate
- section safety
- MPC health

## `PlannerOutputBuilder()`

`PlannerConfig` を保持します。

## `build()`

出力整形の本体です。

処理:

1. `PlannerOutput` にmode、selected、blocked infoをコピーする。
2. SAFE_STOP状態や理由を整理する。
3. 選択候補がoverrideを有効にできるかを判定する。
4. speed-only fallback、wall risk、section profile、MPC healthから速度cap要求を集める。
5. 複数capがあれば最も低いcapとその理由を採用する。
6. 選択候補がfeasibleなら、その候補の `speed_caps` にcapを重ねる。
7. 横候補が使えない場合は、現在dを保持する `SPEED_GUARD` overrideを作る。
8. debug用のflagと理由を詰める。

## override有効化

`selected.type == FASTEST` の場合は基本的にoverrideなしです。
それ以外で候補がfeasibleなら、候補の `d[]/v_ref[]` を使ってoverrideします。

SAFE_STOPは停止候補がfeasibleなときだけoverrideします。

## SPEED_GUARD

横方向の候補を出さず、速度だけ落としたい場合に使います。

特徴:

- `active_override=true`
- `lateral_offsets` は現在dを安全コリドー内にclampした値で一様。
- `speed_caps` は要求capで一様。
- modeが `FREE_RUN` なら `SPEED_GUARD` に変更する。

現在dの保持もSAFE_STOP候補も安全に作れない場合は、横列を捏造しません。このとき `active_override=false` と `longitudinal_speed_cap_active=true` を出し、nodeはv2速度のみpayload `[1, mode_id>0, 0, 2, generation, speed_cap_mps]` をpublishします。下流は通常参照の横方向を使い、`speed_cap_mps`（`m/s`）だけを適用します。v2のmalformed/timeoutでは最後に検証済みのcapを保持し、明示解除 `[1, 0, 0, 1, generation]` またはv1横+速度overrideで解除します。

中心線へ0埋めしないのがポイントです。
壁際や横ずれ中に急に中央へ引っ張ると、MPCが破綻しやすいためです。

## speed cap理由

主な理由:

- `speed_only_fallback_safe_stop_infeasible`
- `speed_only_fallback_<reject_reason>`
- `wall_risk_speed_guard`
- `section_profile_speed_guard`
- `mpc_health_infeasible_guard`
- `mpc_health_solve_time_guard`
- `mpc_health_stale_guard`
- `recovery_wall_risk_speed_guard`
- `recovery_mpc_health_speed_guard`
- `recovery_lateral_error_speed_guard`

## `scaledSpeedCap()`

section safety profileが有効な場合、速度capに `speed_cap_scale` を掛けます。
scaleは `0.05` から `1.0` にclampされます。

`speed_only_fallback_<reject_reason>` のうち、`reject_reason` が
`opponent_collision` の場合は `opponent_collision_fallback_v_max_mps` を使います。
これは安全ゲート向けの低速crawlで、横回避候補が不安全なのに速度だけ高く残る状態を避けるためです。
