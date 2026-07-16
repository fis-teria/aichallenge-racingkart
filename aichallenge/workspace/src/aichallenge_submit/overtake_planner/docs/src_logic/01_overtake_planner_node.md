# overtake_planner_node.cpp

対象:

- `src/overtake_planner_node.cpp`

## 役割

ROS 2との境界です。
plannerの判断そのものは `OvertakePlannerCore` に任せ、このファイルではparameter読み込み、入力変換、publish、debug logを担当します。

## helper関数

| 関数 | 処理 |
|---|---|
| `yawFromQuaternion()` | odometry姿勢のQuaternionからyawを計算する。 |
| `resolveReferencePath()` | 相対CSVパスをROS package share配下の絶対パスへ変換する。 |
| `jsonNumber()` | finiteな数値をJSON用文字列へ変換し、NaN/Infは `null` にする。 |
| `jsonNumberField()` | `/mpc/speed_profile_debug` のJSON文字列から数値fieldを読む簡易parser。 |
| `vehicleIdFromRosDomainId()` | `ROS_DOMAIN_ID=N` を `dN` に変換する。 |
| `resolveOwnVehicleId()` | `own_vehicle_id=auto` のとき自車IDをROS_DOMAIN_IDから推定する。 |

## `OvertakePlannerNode()`

起動時の初期化です。

1. `reference_package`, `reference_csv`, `own_vehicle_id` を読む。
2. `PlannerConfig` に対応する大量のparameterを宣言して読む。
3. `FrenetFrame::loadCsv()` で参照線をロードする。
4. 有効時は `FrenetFrame::loadCorridorCsv()` で参照照合済みの物理回廊をロードする。失敗時はoverrideを無効化する。
5. section safety ruleを読む。
6. `OvertakePlannerCore` を生成する。
7. publisher/subscriber/timerを作る。

重要なpublisher:

- `/overtake/reference_override`
- `/debug/overtake/mode`
- `/debug/overtake/metrics`

重要なsubscriber:

- `/localization/kinematic_state`
- `/v2x/vehicle_positions`
- `/mpc/speed_profile_debug`

## `stampToSec()`

ROS timestampを `double` 秒へ変換します。
odom鮮度、V2X鮮度、MPC health鮮度の計算で使います。

## `sectionSFromWp()`

section safety ruleをwaypoint indexで指定したとき、参照線の `s` に変換します。
indexが範囲外なら `std::nullopt` を返し、そのruleは後段でskipされます。

## `readSectionSafetyRules()`

区間ごとの安全設定を読みます。

対応するparameter:

- `section_safety_names`
- `section_safety_profiles`
- `section_safety_role_policies`
- `section_safety_start_s_m`, `section_safety_end_s_m`
- `section_safety_start_wp`, `section_safety_end_wp`

`s` 指定があればそれを使い、なければwp indexから `s` へ変換します。

## `readOvertakePermissionRules()`

`config/overtake_permission.csv` を読み、`OvertakePermissionRule` の配列へ変換します。
CSVは次の4列です。

```csv
name,start_wp,end_wp,allow_overtake
```

`start_wp/end_wp` は参照線の `s` に変換されます。
`allow_overtake=false` の区間では、通常は追い越し開始せず `FOLLOW_BLOCKED` を維持します。
停止/低速の直接前走車だけは、`slow_front_permission_exception_enabled=true` とCore側の `slow_front_exception_*` 条件を満たし、SafetyEvaluator通過済みPASSとfreshな入力がそろう場合に限り例外的にPASS開始できます。

## `updateMpcHealth()`

MPC debug JSONから `mpc_infeasible_count` と `mpc_solve_time_ms` を読み、`MpcHealthStatus` として保持します。
この値は `PlannerOutputBuilder` で `mpc_health_*` の速度guardに使われます。

## `currentMpcHealth()`

保持しているMPC healthに現在時刻からの `age_sec` を付けます。
未受信なら `valid=false` を返します。

## `updateOpponents()`

V2X位置から他車速度を推定します。

```text
dt = current_stamp - previous_stamp
jump = distance(current, previous)

dt > 0 && jump <= position_jump_threshold_m
  -> vx/vy = dx/dt, dy/dt
else
  -> vx/vy = 0
```

大きな位置ジャンプは速度推定に使わず、等速予測が暴れないようにします。

## `collectOpponents()`

保持しているV2X sampleを `OpponentState` に変換します。

処理:

1. 自車IDと一致するsampleを除外する。
2. 自車に近すぎる点を `ignore_near_ego_m` で除外する。
3. `FrenetFrame::cartesianToFrenet()` で `s/d` を付ける。
4. `now_sec - stamp_sec <= 2.0` なら `valid=true` にする。

## `publishOverride()`

`PlannerOutput` を `Float32MultiArray` に変換します。

```text
v1 lateral: [1, mode_id!=0, n>0, d[0], ..., d[n-1], v_ref[0], ..., v_ref[n-1], 1, generation]
v2 speed-only: [1, mode_id!=0, 0, 2, generation, speed_cap_mps]
explicit inactive: [1, 0, 0, 1, generation]
```

`active_override=false` でも速度guardが有効なら `n=0` のv2を出し、下流は横方向baselineを維持したまま全horizonへ `speed_cap_mps` を適用します。明示inactiveだけがclearです。v2受理後のmalformed payloadまたはtimeoutでは、下流MPC/PPは横軌道を保持せず最後の有効speed capだけをfail-closedで維持します。v1 lateral overrideのmalformed/timeout clear動作は変えません。

## `updateAttemptId()`

追い越しattemptを解析しやすくするため、追い越し開始から終了まで同じIDを出します。

開始:

- `PREPARE_OVERTAKE_LEFT`
- `PREPARE_OVERTAKE_RIGHT`
- `OVERTAKE_LEFT`
- `OVERTAKE_RIGHT`

終了:

- `MERGE_BACK`, `ABORT_RECOVERY`, `YIELD_BEHIND`, `SAFE_STOP` を経て `FREE_RUN` / `FOLLOW_BLOCKED` に戻る場合。

## `publishDebug()`

軽量な `/debug/overtake/mode` と詳細な `/debug/overtake/metrics` をpublishします。

`metrics` はJSON文字列で、前方車、横並び、future risk、pass gap、safe stop、MPC health、speed guardなどを含みます。
ログ解析やHTML reportは基本的にこのtopicを読みます。

## `makeDecisionLogSnapshot()`

`PlannerOutput` からログ比較用のスナップショットを作ります。
毎周期logを出すと読めなくなるので、次の `shouldLogDecisionEvent()` で差分判定するための圧縮表現です。

## `isInterestingDecisionEvent()`

ログに残す価値がある状態かを判定します。
通常の `FREE_RUN + FASTEST + no override` は基本的に出さず、追い越しや安全判定に関係する変化だけ残します。

## `shouldLogDecisionEvent()`

前回snapshotと比較し、mode、selected、blocked、pass gap、safe stop、speed guardなどが変わったときだけlog対象にします。

## `logDecisionEvent()`

重要な判断イベントを `autoware.log` に1行で出します。
実走ログを見るときはこの行が「なぜ今の判断になったか」の入口になります。

## `onTimer()`

1周期の入口です。

処理:

1. odomが新鮮なら `EgoState` を作る。
2. `collectOpponents()` で他車一覧を作る。
3. `core_->update()` を呼ぶ。
4. attempt idを更新する。
5. override/debug/logをpublishする。

## `main()`

ROSを初期化し、`OvertakePlannerNode` をspinします。
