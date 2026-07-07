# 横目標安定化メモ

最新ログでは、MPCのhorizon長ではなく、`overtake_planner` がMPCへ渡す横オフセット列と速度guardの切替が短周期で揺れていました。
このメモは、その対策案と実装状況を残すための資料です。

## 対策案

1. 横目標 `target_d` のレート制限
   - `lateral_target_max_step_m` で、publishする横オフセット列の周期ごとの変化量を制限する。
   - `SAFE_STOP` も制限対象にし、停止指令へ入る瞬間に横目標が中心へワープしないようにする。
2. mode切替ヒステリシス
   - `FREE_RUN`, `ABORT_RECOVERY`, `SPEED_GUARD`, `YIELD_BEHIND` の短周期切替を抑える。
   - 既存の `min_mode_hold_time_sec` と `keep_mode_bonus` を中心に調整する。
3. `SPEED_GUARD` は横目標を変えない
   - 速度guardだけが必要なとき、横オフセットを0埋めして中心線へ戻さない。
   - 現在の横位置を保持し、速度上限だけをMPCへ渡す。
4. コーナ中の相手・譲り方向ロック
   - future side-by-side中は対象車両ID、譲り方向、target dの符号を短時間固定する。
   - 現実装では、まず高速カーブ中の出力済み横オフセット列をholdする。
5. wall recoveryの滑らか化
   - 壁リスクからの中央寄せを、急な横ジャンプではなく段階的な復帰にする。

## 今回実装した内容

- 案1: `lateral_target_max_step_m` を追加し、前回publishした横オフセット列からの変化量を制限。
- 案3: speed-only guard時の横オフセットを中心線0埋めから現在横位置保持へ変更。
- 案4の一部: `high_speed_curve_lateral_hold_*` を追加し、`YIELD_BEHIND`, `ABORT_RECOVERY`, `SAFE_STOP`, `SPEED_GUARD` の高速カーブ中は、低速化またはカーブ脱出まで横オフセット列をhold。
- 案5の一部: `SAFE_STOP` と `ABORT_RECOVERY` の中心寄せもrate limit対象にし、停止/復帰の境界で横目標が飛ばないように変更。

## 未実装

- 案2の追加ヒステリシス強化。
- 案4の相手ID/譲り方向そのもののロック。
- 案5のwall recovery専用プロファイル化。

これらはログでまだ横目標の揺れが残る場合に順番に追加します。
