# behavior_state_machine.cpp

対象:

- `src/behavior_state_machine.cpp`
- `include/overtake_planner/behavior_state_machine.hpp`

## 役割

候補選択結果をそのまま運転状態にせず、時間方向に安定化します。
一瞬だけ安全に見えた候補で追い越し開始したり、横並び/譲り/停止が細かく揺れたりするのを防ぎます。

## `BehaviorStateMachine()`

`PlannerConfig` を保持します。

内部状態:

- `mode_enter_time_sec_`
- `pass_left_safe_cycles_`
- `pass_right_safe_cycles_`
- `safe_stop_hold_count_`
- `safe_stop_release_count_`
- `future_yield_hold_active_`

## `canSwitch()`

現在modeに入ってから `min_mode_hold_time_sec` 以上経ったかを返します。
追い越し開始などのモード切替を短時間で振動させないために使います。

## `markIfChanged()`

modeが変わったときに `mode_enter_time_sec_` を更新します。

## `lateralReleaseReady()`

中心線からの横ずれが解除閾値内かを判定します。

- 閾値が負なら無効扱いで常にtrue。
- それ以外は `abs(ego_lateral_offset_m) <= threshold_m`。

`ABORT_RECOVERY` や `YIELD_BEHIND` を抜ける条件に使います。

## `shouldHoldFutureYield()`

future yield holdを継続すべきか判定します。

継続理由:

- 最小保持時間がまだ終わっていない。
- 横ずれや壁余裕が戻っていない。
- まだコーナーが関係している。
- 前方ギャップが足りない。

## `update()`

状態遷移の本体です。

入力:

- 現在時刻
- 現在mode
- Coreが選んだ候補種別
- `BlockedInfo`
- 選択候補がfeasibleか
- `SafeStopContext`

出力:

- 次の `BehaviorMode`

## SAFE_STOP中

`current == SAFE_STOP` のときは専用処理です。

- 停止候補が安全でなくなったら、`YIELD_BEHIND`, `SIDE_BY_SIDE_KEEP`, `FOLLOW_BLOCKED`, `ABORT_RECOVERY` などへ退避する。
- 停止候補が安全なら保持countを増やす。
- 停止要求が消えていても、壁余裕または中心からの横ずれが解除条件を満たさない場合は `ABORT_RECOVERY` へ渡す。
- `release_ready` が連続で成立したら `FOLLOW_BLOCKED` または `FREE_RUN` へ戻る。

## infeasible候補時

選択候補が危険なら、通常の追い越し開始はしません。

- `YIELD_BEHIND` 候補なら `YIELD_BEHIND`
- 横並び維持なら `SIDE_BY_SIDE_KEEP`
- 前方閉塞なら `FOLLOW_BLOCKED`
- それ以外は `ABORT_RECOVERY`

## PASS開始

`PASS_LEFT` / `PASS_RIGHT` が選ばれていて、straight-only gateが開いている場合だけsafe cycleを貯めます。

`pass_safe_required_cycles` 以上連続して安全なら、`PREPARE_OVERTAKE_LEFT/RIGHT` に入ります。

## mode別の主な遷移

| 現在mode | 主な遷移 |
|---|---|
| `FREE_RUN` | blockedなら `FOLLOW_BLOCKED`、pass準備が整えば `PREPARE_OVERTAKE_*`。 |
| `FOLLOW_BLOCKED` | 閉塞解除で `FREE_RUN`、pass準備が整えば `PREPARE_OVERTAKE_*`。 |
| `PREPARE_OVERTAKE_*` | 次周期で `OVERTAKE_*` へ。 |
| `OVERTAKE_*` | 横並び中は維持、前方gapが戻れば `MERGE_BACK`、長引けば `ABORT_RECOVERY`。 |
| `MERGE_BACK` | blockedなら `FOLLOW_BLOCKED`、空けば `FREE_RUN`。 |
| `ABORT_RECOVERY` | 壁余裕と横ずれが戻るまで維持。 |
| `SIDE_BY_SIDE_KEEP` | 横並び解消で `FOLLOW_BLOCKED` または `FREE_RUN`。 |
| `YIELD_BEHIND` | future hold、横ずれ、前方gapを見て解除。 |
| `SPEED_GUARD` | 次周期でblocked状況に応じて `FOLLOW_BLOCKED` または `FREE_RUN`。 |

## future yield hold

未来横並びリスクで `YIELD_BEHIND` に入った場合、一瞬リスクが消えてもすぐ解除しません。
コーナーや横ずれが落ち着くまで `future_yield_hold_active_` で保持します。
