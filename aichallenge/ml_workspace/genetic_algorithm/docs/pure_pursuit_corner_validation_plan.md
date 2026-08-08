# Pure Pursuit コーナー改善の検証メモ

## 目的

最速ラップだけでなく、複数走行で再現するコーナー区間・2周目ラップの短縮を目指す。
対象は進入、回頭、クリップ、操舵戻し、出口から次の加速区間までとする。

## 現在のベースライン

- 対象個体: constrained CMA-ES run `20260804T175800Z` の `g0083-i0025`
- 2周目: `43.1350 / 43.0600 / 43.0850 s`
- 横偏差 p95: `1.27783 / 1.30199 / 1.29938 m`（制約 `<= 1.35 m`）
- 全3回で完走、壁接触・衝突なし。

この個体は、前の安定最良 `g0065-i0010` とPure Pursuit設定が同一であり、主な改善は経路オフセット最適化によるものだった。

## 現行実装から確認した事実

1. Pure Pursuitは100 Hzのwall timerで実行する。
2. 35 km/hを目標とし、加速度指令は `0 <= a_cmd <= limit` に制限される。負加速度はこのノードから出さない。
3. 最速周の2周目では、ログ上の加速度指令は常に `0.8 m/s^2` だった。
4. 実加速度は操舵中に低下する。`|steer| >= 0.30 rad` の記録サンプルでは平均 `-0.444 m/s^2`、最大減速は約 `-0.997 m/s^2`。実加速度が負の記録サンプルは約42.6%だった。
5. AWSIMアダプタは操舵指令を `0.2 s` 遅延させる。8.5 m/sでは約1.7 mの位置差に相当する。
6. 最適経路CSVは131点で、点間隔の中央値は約2.97 m、最大約6.63 m。現在は指定距離を超えた最初の離散点を先読み点としており、補間していない。
7. `Odometry.twist.angular.z` の実ヨーレートは取得できる。`Odometry.twist.linear.y` はライブTopicで0だったため、横滑り角 `atan2(vy, vx)` はそのままでは使えない。

## 仮説

主因候補は、旋回中の大きな操舵と0.2秒の操舵遅延により、操舵を切り過ぎ／戻し遅れし、駆動力が旋回抵抗に相殺されることである。

「常時の正加速度」が主因とは未確定。回頭不足時の短時間コースト（rotation gate）は、タイヤの縦横力競合を減らして回頭を助ける可能性がある一方、現在は加速余力が小さいため再加速損失となる可能性もある。よって最初の変更にはしない。

## 改修候補と優先順位

### 1. 先読み点の連続補間

軌道の累積弧長上で、要求Lookahead距離に対応する位置・姿勢を線形補間する。

- 期待: 離散点切替による操舵振動、切り足し、切り戻しを低減する。
- リスク: 低い。
- 最初にA/Bする。

### 2. 0.2秒の操舵遅延補償

実操舵角・実速度・運動学的自転車モデルで遅延後の予測姿勢を求め、その姿勢からPure Pursuitを計算する。

- 期待: 回頭開始の遅れと出口の操舵残りを減らす。
- リスク: 横滑りはモデル化しないため、大きな遅延設定や過大な補償は避ける。
- 連続補間が有効と確認できた後に追加する。

### 3. 符号付き曲率フィードフォワードと出口アンワインド

符号付き経路曲率から `delta_ff = atan(L * kappa)` を作り、Pure Pursuitを誤差補正に限定する。曲率が減少する出口では遠方Previewの比率を上げ、操舵を早く戻す。

- 期待: 操舵のピーク時間と出口での抵抗を減らす。
- リスク: フィードフォワードの単純加算は過大操舵になり得る。重み付き混合と角度・レート制限を併用する。

### 4. Rotation gate（最後に検証）

以下が一定時間継続するときだけ、正加速度を一時的に0へ制限する。

```text
r_ref = v * abs(kappa)
e_r = sign(kappa) * (r_ref - r_measured)

rotation_gate =
  corner_active
  && e_r > threshold
  && abs(actual_steering) > threshold
  && predicted_or_observed_margin_is_safe
```

- 初期動作はブレーキなしのコーストのみ（`a_out = min(a_base, 0)`）。
- 最小・最大継続時間、ヒステリシス、再発火防止時間、ジャーク制限を必須とする。
- 速度ベクトル／横滑り角は、安定した推定方法を得るまで判定条件に入れない。

## 推奨状態遷移

```text
APPROACH
  遅延補償と曲率FFで回頭開始を前倒し
      ↓
ROTATION
  近方Previewとヨーレート追従を重視
  加速度は原則維持
      ↓
EXIT
  遠方Previewを強めて操舵を早く戻す
  加速度を維持
      ↓
STRAIGHT
```

最初は状態を横制御のPreview/フィードフォワード切替にのみ使う。縦制御へrotation gateを加えるのは、ヨーレートログで必要性が確認されてからとする。

## A/B実験順

同一の経路・遺伝子 `g0083-i0025` を固定し、各条件を最低5回ずつ評価する。

| 条件 | 変更 |
|---|---|
| A | 現行ベースライン |
| B | 連続先読み点補間のみ |
| C | B + 0.2秒遅延補償 |
| D | C + 曲率フィードフォワード／動的Dual Preview |
| E | D + rotation gate（コーストのみ） |

各段階でA-B-A形式の再評価を入れ、同時に経路やGA評価式を変更しない。最良の横制御構造が確定してから、経路最適化を再開する。

## 実装状態（2026-08-08）

条件B〜Eを切り替えられる実装を追加した。全flagは既定値`false`のため、既存の凍結個体と進行中GAの挙動は変わらない。

- `continuous_preview_interpolation_enabled`: 条件B。nearest index以降の累積弧長で、far/near両方のPreview点の位置を補間する。
- `delay_compensation_enabled`: 条件C。`/vehicle/status/steering_status`が新鮮なときだけ、AWSIMアダプタと同じ既定`0.2 s`を運動学モデルで予測する。Topic欠落・時刻切れ・低速時は自動で現行姿勢へフォールバックする。
- `curvature_feedforward_enabled`: 条件D。符号付き曲率の`atan(wheel_base * curvature)`を、指定gainでPure Pursuit出力へ加える。
- `exit_unwind_enabled`: 条件D。前方で曲率が減る出口ではnear Previewの比率を下げ、遠方Previewへ寄せて操舵を早く戻す。
- `rotation_gate_enabled`: 条件E。新鮮な実操舵・横偏差・符号付きヨーレート誤差がすべて安全条件を満たす場合だけ、最小`0.05 s`から最大`0.20 s`のコーストを行う。ブレーキは要求しない。ヒステリシスと`0.50 s`の再発火クールダウンを持つ。
- `/pure_pursuit/debug`に各flag、実際に補償を使用したか、実操舵情報が新鮮か、補間元/先のtrajectory index、制御用予測姿勢を追加する。

GAの評価関数・遺伝子空間は変更しない。まず固定個体で各flagのA/Bを確認してから、効果が再現した項目だけをGA遺伝子に加える。

## 記録すべき値

- ラップ・セクタータイム、出口5m／10m地点速度
- 横偏差p95・最大値、完走・接触・スタック
- 目標／実速度、目標／実加速度
- 指令／実操舵角、操舵角速度、`|steer| > 0.2 rad` 継続時間
- signed curvature、`r_ref`、実ヨーレート、`e_r`
- Lookahead要求値・実際の補間先、遅延予測姿勢
- rotation gateの状態と継続時間

## 成功判定

- 5回の2周目ラップ中央値がベースラインから `>= 0.10 s` 改善する。
- 全走行で横偏差p95が `<= 1.35 m`、接触・スタックなし。
- 標準偏差がベースラインから大きく悪化しない。
- 対象コーナー出口5mまたは10mの速度、または当該セクター中央値が改善する。
- 操舵角RMS、過大操舵の継続時間、実加速度が負の時間のいずれかが悪化していない。

## 関連実装箇所

- `aichallenge/workspace/src/aichallenge_submit/ga_simple_pure_pursuit/src/simple_pure_pursuit.cpp`
- `aichallenge/workspace/src/aichallenge_submit/ga_simple_pure_pursuit/include/simple_pure_pursuit/delay_compensation.hpp`
- `aichallenge/workspace/src/aichallenge_submit/ga_simple_pure_pursuit/include/simple_pure_pursuit/lookahead.hpp`
- `aichallenge/workspace/src/aichallenge_system/aichallenge_awsim_adapter/src/actuation_cmd_converter.cpp`
