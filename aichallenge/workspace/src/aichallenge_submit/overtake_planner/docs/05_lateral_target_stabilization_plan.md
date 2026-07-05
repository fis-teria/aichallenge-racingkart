# 横目標安定化メモ

最新ログでは、MPCのhorizon長ではなく、`overtake_planner` がMPCへ渡す横オフセット列と速度guardの切替が短周期で揺れていました。
このメモは、その対策案と実装状況を残すための資料です。

## 対策案

1. 横目標 `target_d` のレート制限
   - `lateral_target_max_step_m` で、publishする横オフセット列の周期ごとの変化量を制限する。
   - `SAFE_STOP` は停止優先のため制限対象外。
2. mode切替ヒステリシス
   - `FREE_RUN`, `ABORT_RECOVERY`, `SPEED_GUARD`, `YIELD_BEHIND` の短周期切替を抑える。
   - 既存の `min_mode_hold_time_sec` と `keep_mode_bonus` を中心に調整する。
3. `SPEED_GUARD` は横目標を変えない
   - 速度guardだけが必要なとき、横オフセットを0埋めして中心線へ戻さない。
   - 現在の横位置を保持し、速度上限だけをMPCへ渡す。
4. コーナ中の相手・譲り方向ロック
   - future side-by-side中は対象車両ID、譲り方向、target dの符号を短時間固定する。
5. wall recoveryの滑らか化
   - 壁リスクからの中央寄せを、急な横ジャンプではなく段階的な復帰にする。

## 今回実装した内容

- 案1: `lateral_target_max_step_m` を追加し、前回publishした横オフセット列からの変化量を制限。
- 案3: speed-only guard時の横オフセットを中心線0埋めから現在横位置保持へ変更。

## 未実装

- 案2の追加ヒステリシス強化。
- 案4の相手ID/譲り方向ロック。
- 案5のwall recovery専用の滑らか化。

これらはログでまだ横目標の揺れが残る場合に順番に追加します。
