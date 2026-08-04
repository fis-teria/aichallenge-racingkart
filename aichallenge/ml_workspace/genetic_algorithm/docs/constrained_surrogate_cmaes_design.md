# 制約付きLap最適化・ブロック別代理モデル・CMA-ES移行設計

## 1. 目的

現在のGA資産、AWSIM 3環境×4台の並列評価、既存SQLite履歴、ダッシュボードを維持しながら、安定して速いPure Pursuit制御と走行経路を探索する。

主目的は、handicap相当の加速度上限`0.8`、2周目評価、衝突なしという現行条件のまま、再現性のある43.5秒台以下の個体を得ることである。

## 2. 背景と現状の問題

精密探索ラン`20260802T221610Z`では、次の結果が得られた。

| 個体 | 特徴 | Fitness | 2周目 |
|---|---|---:|---:|
| `g0099-i0022` | Fitness最良 | 53.0929 | 中央値45.130秒 |
| `g0086-i0028` | 安定Lap上位 | 53.4695 | 43.385 / 43.740 / 43.755秒 |

Fitnessは改善したが、Fitness最良個体がLap最速個体より約1.4秒遅い。現行の加算型ペナルティでは、Lap短縮より追従誤差などの改善が強く選択されている。

また、第99世代以降36世代以上Fitnessが更新されず、代理モデルは実エリート`53.0929`に対して候補最良を約`53.793`と予測していた。単一KNNがControllerとPathの全遺伝子を同時に距離計算するため、高次元化によって局所的な優良領域を識別できていない可能性が高い。

上位個体は探索範囲の上下限へ集中していないため、単純な範囲拡張は主対策としない。

## 3. スコープ

### 対象

- 制約付きLap評価
- 既存履歴のオフライン再評価
- Controller用・Path用代理モデルの分離
- Extra Treesによる回帰・実行可能性判定
- ブロック別CMA-ES
- 既存SQLiteへの追加スキーマ
- checkpoint/resume
- ダッシュボードへの比較情報追加

### 対象外

- AWSIM車両物理やhandicapの変更
- 加速度上限`0.8`の回避
- Pure Pursuit以外の制御方式への全面置換
- 4台ゴースト並列評価方式の変更
- 現行GAの削除

現行GAはフォールバックとして残し、設定で切り替えられるようにする。

## 4. 新しい評価方式

### 4.1 実行可能性の必須条件

各repeatが次をすべて満たす場合のみ、Lap最適化対象とする。

- 2周を完走
- タイマー検証成功
- 衝突なし
- 壁接触なし
- 横偏差p95が`1.35 m`以下
- 最大横偏差が`2.0 m`以下

閾値は設定ファイルから変更可能にする。

```json
"constrained_objective": {
  "required_complete_repeat_ratio": 1.0,
  "maximum_collision_count": 0,
  "maximum_wall_count": 0,
  "lateral_error_p95_limit_m": 1.35,
  "lateral_error_max_limit_m": 2.0
}
```

### 4.2 実行不能個体の順位

実行不能個体は固定の巨大ペナルティだけで潰さず、制約違反量で順位付けする。

```text
violation =
  unfinished_repeat_ratio * W_unfinished
  + collision_count * W_collision
  + wall_count * W_wall
  + max(0, lateral_p95 - 1.35) * W_lateral_p95
  + max(0, lateral_max - 2.0) * W_lateral_max
```

これにより、完全に失敗した個体より「わずかに横偏差超過した高速個体」を境界探索へ利用できる。

### 4.3 実行可能個体の目的値

実行可能個体は2周目Lapを主目的にする。

```text
lap_spread = percentile90(lap2) - percentile10(lap2)

robust_lap =
  median(lap2)
  + 0.25 * lap_spread
  + 0.02 * high_steering_exposure
  + 0.02 * steering_unwind_delay
```

操舵項目はLap差を覆さない程度の小さなタイブレークとして扱う。既存の追従誤差・速度損失指標はダッシュボードと診断に残すが、実行可能範囲内では主目的へ大きく加算しない。

### 4.4 順位比較

単一の巨大スカラーではなく、次の辞書順で比較する。

```text
(
  feasibleではないか,
  violation,
  robust_lap,
  lateral_error_p95,
  parameter_hash
)
```

これにより、実行可能個体は常に実行不能個体より上位となり、実行可能個体同士ではLap短縮が最優先される。

## 5. 既存履歴の再評価

新しい探索を開始する前に、既存runの`metrics_json`から新評価値を計算する読み取り専用ツールを追加する。

```text
tools/analyze_constrained_objective.py
```

出力内容：

- 対象個体数
- 実行可能率
- 制約ごとの失敗数
- robust Lap上位30個体
- 現行Fitness順位との順位相関
- `g0086-i0028`と`g0099-i0022`の新順位
- 各閾値を変更した場合の感度分析

元DBのFitnessは上書きしない。分析結果をJSONとMarkdownで別途保存する。

## 6. ブロック別代理モデル

### 6.1 遺伝子ブロック

Controllerブロック：

- `lookahead_gain`
- `lookahead_min_distance`
- `steering_tire_angle_gain`
- `curvature_lookahead_min_distance`
- `curvature_lookahead_sensitivity`
- `curvature_lookahead_smoothing_alpha`
- `curvature_speed_preview_distance`
- `dual_preview_near_ratio`
- `dual_preview_blend`

Pathブロック：

- `path_offset_00`～`path_offset_15`

Jointブロックは両方を含むが、最終微調整時だけ使用する。

### 6.2 モデル

各ブロックに次の2モデルを持つ。

1. `ExtraTreesClassifier`: 実行可能性を予測
2. `ExtraTreesRegressor`: 実行可能個体のrobust Lapを予測

木ごとの予測分散を不確実性として利用する。モデル入力は現在のブロック遺伝子に加え、固定側ブロックと中心個体との差を要約した値を使用する。

既存KNNは、学習サンプル不足時とscikit-learn利用不能時のフォールバックとして残す。

### 6.3 候補選択

2048候補から32評価個体を選ぶ標準配分：

| 枠 | 個体数 | 内容 |
|---|---:|---|
| Exploit | 16 | 実行可能確率が高く予測Lapが短い |
| Uncertainty | 6 | 予測分散が大きい |
| Novelty | 6 | 既評価個体から遠い |
| Safety/Random | 4 | エリート再評価とランダム探索 |

同一parameter hashは評価前に除外する。

代理モデルの品質として、世代ごとにMAE、順位相関、実行可能性precision/recallを記録する。予測MAEが閾値を超えた場合は、代理モデルによる除外を弱める。

## 7. ブロック別CMA-ES

### 7.1 採用理由

遺伝子はすべて連続値であり、現在は有望領域が得られている。交叉GAを長期間継続するより、平均・分散・共分散を更新するCMA-ESの方が局所的な変数間相関を利用しやすい。

Python依存として`numpy`と`cma`を明示的に追加する。`scikit-learn`も現在のイメージには存在するが、再現性のため依存宣言へ追加する。

### 7.2 フェーズ

#### Phase A: Controller最適化

- 中心: `g0086-i0028`と`g0099-i0022`のうち、新評価で上位の個体
- Path: 中心個体の値へ固定
- Controller 9遺伝子のみ更新
- 初期sigma: 正規化空間で`0.04`

#### Phase B: Path最適化

- Controller: Phase A最良へ固定
- Path 16遺伝子のみ更新
- 初期sigma: 正規化空間で`0.03`
- 経路平滑性修復を候補生成直後に適用

#### Phase C: Joint微調整

- Phase A/Bの最良を中心に全25遺伝子を更新
- 初期sigma: `0.015`
- 最大5世代、または改善が止まるまで

### 7.3 フェーズ遷移

- 6世代連続で`0.01秒`以上改善しなければ次フェーズへ移る
- Phase C停滞後は、上位3個体を中心にPhase Aへ戻る
- sigmaは改善時に縮小し、停滞時に最大1.5倍まで再拡張
- ユーザー停止までA→B→Cを循環可能

### 7.4 並列評価

- CMA-ESの評価母集団は32個体
- 12台へ3バッチで投入
- 既存`SharedAwsimBatchEvaluator`をそのまま利用
- 高速候補は3 repeats、通常候補は1 repeat
- 世代上位4個体は追加repeatし、最低3回の中央値を確定する

## 8. データモデル

既存テーブルは変更せず、次を追加する。

```sql
CREATE TABLE IF NOT EXISTS candidate_metadata (
  candidate_id TEXT PRIMARY KEY,
  optimizer_type TEXT NOT NULL,
  phase TEXT NOT NULL,
  emitter TEXT,
  reference_id TEXT,
  predicted_feasibility REAL,
  predicted_lap REAL,
  predicted_uncertainty REAL,
  feasible INTEGER,
  violation REAL,
  robust_lap REAL,
  FOREIGN KEY(candidate_id) REFERENCES candidates(candidate_id)
);
```

実親または生成中心を`reference_id`へ保存する。これによりダッシュボードで近似親ではなく実際の生成元と比較できる。

## 9. Checkpointと復旧

checkpointには次を保存する。

- optimizer type
- 現在フェーズ
- 世代番号
- CMA平均ベクトル
- sigma
- 共分散行列
- evolution paths
- RNG状態
- ブロック固定値
- エリート一覧
- 代理モデル学習run一覧と設定hash

SQLiteは現行のWAL・busy timeout・書き込みリトライを継続する。resume時はDBの`quick_check`とcheckpoint/config hashを検証する。

## 10. ダッシュボード

追加表示：

- 現在のoptimizer/phase
- feasible率
- robust Lap最良・中央値
- 制約違反理由の内訳
- 代理モデルMAE・順位相関
- CMA sigma推移
- 実際の`reference_id`とのΔLap・遺伝子差
- Fitnessではなくrobust Lapを主グラフへ表示

既存のFitness、1周目、2周目、速度追従、経路、遺伝子表は残す。

## 11. 設定例

```json
"optimizer": {
  "type": "block_cmaes",
  "population_size": 32,
  "candidate_pool_size": 2048,
  "phase_stagnation_generations": 6,
  "minimum_improvement_seconds": 0.01,
  "controller_sigma": 0.04,
  "path_sigma": 0.03,
  "joint_sigma": 0.015,
  "joint_max_generations": 5
},
"surrogate": {
  "type": "block_extra_trees",
  "trees": 256,
  "minimum_training_samples": 100,
  "exploit_count": 16,
  "uncertainty_count": 6,
  "novelty_count": 6,
  "safety_count": 4
}
```

## 12. 実装順序

### Step 1: 評価式のオフライン検証

- 制約付き評価関数を独立モジュールとして実装
- 既存1,437個体以上を再評価
- 閾値感度分析
- 新上位個体を人間が走行ログで確認

この段階では稼働中探索を変更しない。

### Step 2: 評価式とDBメタデータ

- 新評価をOptimizerから利用可能にする
- additive schema migration
- dashboardへfeasible/robust Lap追加
- 現行GAでsmoke test

### Step 3: ブロック別代理モデル

- Extra Trees classifier/regressor
- オフライン交差検証
- KNNとの候補選出比較
- 代理モデル不調時の自動フォールバック

### Step 4: CMA-ES

- Controllerフェーズのみ実装
- deterministic evaluatorでcheckpoint/resume試験
- AWSIM 1世代smoke test
- Path、Jointの順に追加

### Step 5: 本探索

- `g0086-i0028`と`g0099-i0022`を明示シード
- 3環境×4台で開始
- 旧GAランは停止せずに保存し、いつでもresume可能にする

## 13. テスト計画

### Unit

- 制約境界値
- robust Lap計算
- 辞書順ランキング
- ブロック分割・正規化・修復
- Extra Trees予測と不確実性
- CMA ask/tell、境界修復
- checkpoint round trip
- additive DB migration

### Integration

- deterministic evaluatorで20世代再現
- 同じseedで同じ候補列になること
- 12並列DB書き込み
- ダッシュボード同時読み取り時のWAL動作
- 第N世代途中停止からのresume

### AWSIM acceptance

- 12台へ別個体が割り当てられる
- 全個体のLapと遺伝子がDBへ保存される
- `g0086-i0028`の再評価が既存結果の許容範囲内
- 5世代以内に現行エリートを初期集団へ保持
- 衝突なし・横偏差制約内で43.7秒以下を再現

## 14. 成功基準

最低成功：

- 3 repeats中央値`43.70秒`以下
- 全repeat完走・衝突なし
- 横偏差p95`1.35m`以下
- 最大横偏差`2.0m`以下

目標成功：

- 3 repeats中央値`43.50秒`以下
- p90-p10が`0.30秒`以下

ストレッチ目標：

- 3 repeats中央値`43.30秒`以下
- オンライン相当の1台・handicap ONでも再現

## 15. ロールバック

- `optimizer.type: genetic_algorithm`で現行方式へ戻せる
- 新DB変更はテーブル追加のみで既存readerを壊さない
- 旧checkpointとrun DBは変更しない
- 新評価値は別カラム・別テーブルへ保存し、既存Fitnessを上書きしない

この構成により、評価式、代理モデル、探索器を個別に有効化し、どの変更が改善へ寄与したかを切り分けながら移行できる。
