# overtake_planner 安全停止処理 仕様案

## 目的

この仕様は、`overtake_planner` に「回避・追従・譲り・復帰のどれも安全に成立しないと判断した場合、MPCへ低速停止意図を出し続ける」処理を追加するためのものです。

既存の `YIELD_BEHIND` や `ABORT_RECOVERY` は「譲る」「中心へ戻る」ための状態であり、完全な停止判断ではありません。安全停止は、それらの通常回避が成立しないときの最後の planner 内 fallback として扱います。

## 非目的

- MPC本体の停止実装を変更しない。
- `/control/mpc/stop_request` を通常の回避不能判定で直接publishしない。
- `0.0 m/s` の速度capや override解除で停止を表現しない。
- 壁マージン、他車楕円、安全評価、stale odom / stale opponent 判定をバイパスしない。

`/control/mpc/stop_request` は controller を無効化するラッチ動作に近く、自律復帰契約を別途持たないと通常の一時停止には重すぎます。これは controller異常、override契約破損、衝突直前でMPC追従継続が危険な場合の terminal failsafe として残します。

注意: `overtake_planner` が `enabled=false`、`ego.valid=false`、参照線なし、または publish timeout になった場合、車両停止ではなく planner override の解除に近い挙動になります。これは安全停止の代替ではありません。

## 現行契約の制約

`/overtake/reference_override` は次の形式です。

```text
[valid, mode_id, n, d[0], ..., d[n-1], v_ref[0], ..., v_ref[n-1]]
```

MPC側は `mode_id == 0` または `n <= 0` を override解除として扱います。また、`speed_cap <= 0.0` は無効値として扱われます。

そのため、安全停止は次のように表現します。

- `BehaviorMode::SAFE_STOP` を追加する。
- `CandidateType::SAFE_STOP` を追加する。
- `active_override=true` として horizon点数ぶんの `d` と `v_ref` をpublishする。
- `v_ref` は `0.0` ではなく、小さい正の値 `safe_stop_v_mps` を使う。

## 追加パラメータ案

```yaml
safe_stop_enabled: true
safe_stop_v_mps: 0.20
safe_stop_trigger_cycles: 3
safe_stop_release_cycles: 5
safe_stop_release_front_gap_m: 5.0
safe_stop_release_wall_clearance_m: 0.20
safe_stop_lateral_error_threshold_m: 0.40
safe_stop_release_speed_mps: 0.50
```

意味:

- `safe_stop_enabled`
  - 安全停止処理全体の有効/無効。
- `safe_stop_v_mps`
  - STOP中にMPCへ渡す速度上限。`0.0` はMPC側で無効化されるため禁止。
- `safe_stop_trigger_cycles`
  - 回避不能判定が何周期続いたら STOP に入るか。
- `safe_stop_release_cycles`
  - STOP解除条件が何周期続いたら解除するか。
- `safe_stop_release_front_gap_m`
  - 前方車両との再発進に必要なFrenet縦方向gap。
- `safe_stop_release_wall_clearance_m`
  - 再発進に必要な壁余裕。
- `safe_stop_lateral_error_threshold_m`
  - STOP中の目標dと現在dの差を許す範囲。
- `safe_stop_release_speed_mps`
  - STOP解除を検討できる自車速度上限。停止または低速安定を確認するために使う。

## STOP候補の生成

`makeCandidate(CandidateType::SAFE_STOP, ...)` は次を満たします。

- 目標横位置 `target_d` は現在の `ego.frenet.d` を安全コリドー内にclampした値。
- 候補の全 `d` は `d_min_m + min_wall_margin_m` から `d_max_m - min_wall_margin_m` の範囲内。
- `v_ref` は全点 `safe_stop_v_mps`。
- `shift_distance` は急な横移動を避けるため、`merge_distance_m` 以上を使う。
- safety evaluator で `wall_margin` / `opponent_collision` を評価する。

STOP候補は、停止しながら横へ大きく逃げる軌道ではありません。目的は「安全コリドー内の現在位置を保ちながら、前方へ進む意図を最小化する」ことです。

## STOPトリガ条件

STOPは `!can_pass_left && !can_pass_right` だけでは発動しません。それは単に「追い越せない」状態であり、通常は `FOLLOW` または `YIELD_BEHIND` で扱います。

STOP候補を検討する条件:

1. `safe_stop_enabled == true`
2. `blocked_info.blocked || blocked_info.side_by_side`
3. `!blocked_info.can_pass_left && !blocked_info.can_pass_right`
4. 次の通常候補が成立しない、または危険側へ押し込む:
   - `FOLLOW`
   - `YIELD_BEHIND`
   - `RECOVERY`
   - `SIDE_BY_SIDE_KEEP`
5. 上記状態が `safe_stop_trigger_cycles` 以上連続する。

成立しない判定の例:

- `SafetyEvaluator` が `opponent_collision` でrejectした。
- `SafetyEvaluator` が `wall_margin` でrejectした。
- `YIELD_BEHIND` が best-effort override になりそうだが、相手車楕円内へ進む。
- 横並びや壁際で、通常候補の `target_d` へ戻るには大きな横移動が必要で、かつ速度capを下げても安全余裕が戻らない。

## 状態遷移

`BehaviorStateMachine` に `SAFE_STOP` を追加します。

基本遷移:

```text
FOLLOW_BLOCKED / YIELD_BEHIND / SIDE_BY_SIDE_KEEP / ABORT_RECOVERY
  -> SAFE_STOP
```

遷移条件:

- `CandidateType::SAFE_STOP` が選ばれている。
- `SAFE_STOP` 候補が feasible。
- 回避不能判定が `safe_stop_trigger_cycles` 続いている。

保持条件:

- STOP中は `safe_stop_release_cycles` ぶん解除条件が続くまで `SAFE_STOP` を保持する。
- STOP中に planner publish が途切れないよう、毎周期 `active_override=true` を出し続ける。

解除条件:

- `ego.valid == true`
- `blocked_info.ego_wall_clearance_m >= safe_stop_release_wall_clearance_m`
- `!blocked_info.side_by_side`
- 前方対象がいない、または `front_delta_s >= safe_stop_release_front_gap_m`
- PASS候補またはFOLLOW/YIELD候補が feasible
- `ego.v <= safe_stop_release_speed_mps`
- 上記が `safe_stop_release_cycles` 続く

解除先:

- 前方がまだ詰まっているなら `FOLLOW_BLOCKED`
- 横並びが残るなら `YIELD_BEHIND` または `SIDE_BY_SIDE_KEEP`
- 何もなければ `FREE_RUN`

## active_override ルール

`SAFE_STOP` は best-effort override を許可しません。

```text
SAFE_STOP active_override = selected.feasible のときだけ true
```

理由:

- `opponent_collision` でrejectされた停止候補をpublishすると、停止意図でも衝突判定済み軌道をMPCへ渡すことになる。
- `wall_margin` でrejectされた停止候補をpublishすると、壁側に残る軌道を固定する可能性がある。

`SAFE_STOP` が infeasible の場合は、planner内だけで解決できない状態として扱い、debug reason に `safe_stop_infeasible` を出します。terminal failsafe へ接続するかは別仕様で決めます。

## debug / report 契約

`/debug/overtake/mode`:

- `SAFE_STOP`

`/debug/overtake/metrics` に追加または明示する項目:

```json
{
  "mode": "SAFE_STOP",
  "selected": "SAFE_STOP",
  "active_override": true,
  "reason": "no_safe_avoidance",
  "safe_stop_triggered": true,
  "safe_stop_reason": "no_pass_and_no_safe_fallback",
  "safe_stop_v_mps": 0.2,
  "min_cbf_h": 0.0,
  "cbf_slack": 0.0
}
```

`reason` の候補:

- `no_safe_avoidance`
- `no_pass_and_no_safe_fallback`
- `safe_stop_holding`
- `safe_stop_release_pending`
- `safe_stop_infeasible`

ログ出力では、少なくとも次を残します。

- `mode`
- `selected`
- `front_id`, `side_id`
- `can_pass_left`, `can_pass_right`
- `pass_gap_reason`
- `safe_stop_reason`
- `min_cbf_h`, `cbf_slack`
- `ego_s`, `ego_d`, `target_d`

## 実装対象ファイル

主な変更対象:

- `include/overtake_planner/types.hpp`
  - `BehaviorMode::SAFE_STOP`
  - `CandidateType::SAFE_STOP`
  - `PlannerConfig` に safe stop パラメータ
  - 必要なら `BlockedInfo` または `PlannerOutput` に stop理由
- `src/frenet_frame.cpp`
  - `toString()` に `SAFE_STOP`
- `src/overtake_planner_core.cpp`
  - STOP候補生成
  - 回避不能判定
  - STOP候補のscore
  - best-effort override対象から `SAFE_STOP` を除外
- `src/behavior_state_machine.cpp`
  - `SAFE_STOP` への遷移、保持、解除
- `src/overtake_planner_node.cpp`
  - パラメータ宣言
  - debug JSON / decision log の拡張
- `config/overtake_planner.param.yaml`
  - safe stop パラメータ追加

必要に応じて更新:

- `docs/01_code_level_spec.md`
- `docs/02_parameter_guide.md`
- `tools/eval_wrapper` の overtake解析

## テスト仕様

### Core tests

`test_overtake_planner_core.cpp` に追加します。

1. no-pass だが FOLLOW が feasible なら STOPしない。
2. no-pass だが YIELD_BEHIND が feasible なら STOPしない。
3. no-pass かつ FOLLOW/YIELD/RECOVERY が unsafe なら `SAFE_STOP`。
4. `SAFE_STOP` の `speed_caps` は全点 `safe_stop_v_mps`。
5. `SAFE_STOP` の `lateral_offsets` は安全コリドー内。
6. `SAFE_STOP` が `opponent_collision` で infeasible のとき、best-effort publish しない。

### State machine tests

`test_state_machine.cpp` に追加します。

1. 回避不能が `safe_stop_trigger_cycles` 未満なら `FOLLOW_BLOCKED` / `YIELD_BEHIND` を維持。
2. 回避不能が連続したら `SAFE_STOP` へ遷移。
3. `SAFE_STOP` 中に解除条件が不足していれば保持。
4. 解除条件が `safe_stop_release_cycles` 続けば `FOLLOW_BLOCKED` または `FREE_RUN` へ戻る。

### Safety evaluator tests

必要なら `test_safety_evaluator.cpp` に追加します。

- STOP候補も壁マージン違反ならreject。
- STOP候補も相手楕円内ならreject。

### Report tests

`tools/eval_wrapper/tests/test_overtake_analysis.py` で、`SAFE_STOP` 行を含むdebug metricsを読めることを確認します。

- `SAFE_STOP` を aborted / safety_stop として分類できる。
- `safe_stop_reason` が timeseries / report に残る。
- 既存の `YIELD_BEHIND` / `ABORT_RECOVERY` 分類を壊さない。

## 受け入れ条件

- `SAFE_STOP` は `0.0 m/s` や override解除で表現しない。
- `SAFE_STOP` は feasible なときだけ publish する。
- `safe_stop_v_mps` は正の値で、MPCの入力制約下限と矛盾しない。
- 既存の `FREE_RUN`, `FOLLOW_BLOCKED`, `YIELD_BEHIND`, `ABORT_RECOVERY` の基本挙動を壊さない。
- `ego.valid=false` や参照線なしでは従来通り非介入。
- STOP中はdebug topicとdecision logで理由が追える。
- planner無効、stale odom、stale opponent、override timeout が安全停止として扱われないことをテストまたはログで確認する。
- C++ unit test と eval wrapper test が通る。

## 残論点

- `safe_stop_v_mps` の初期値を `0.20` とするか、MPCの最小速度制約に合わせて別値にするか。
- STOPが長時間続いたとき、terminal failsafe `/control/mpc/stop_request` へ昇格する条件を別途設けるか。
- STOP解除後の再加速を planner側で段階的にするか、MPC側の通常速度計画に任せるか。
- eval report 上で `SAFE_STOP` を `ABORTED` に分類するか、独立した `SAFETY_STOP` として分類するか。
