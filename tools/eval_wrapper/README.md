# evalwrap

Automotive AI Challenge レーシングカート実行向けの、ローカル専用評価
ラッパーです。

このラッパーは、評価成果物を提出用ワークスペースの外側に保存します。

```text
tools/eval_wrapper/      # このツール
analysis/runs/<run_id>/  # 生成されるローカル結果
```

`aichallenge/workspace/src/aichallenge_submit` には書き込みません。

## クイックスタート

`aichallenge-racingkart/tools/eval_wrapper` から実行する場合:

```bash
python -m evalwrap doctor
python -m evalwrap ingest --label baseline --path ../../output/latest
```

リポジトリルートから実行する場合:

```bash
tools/evalwrap run --label baseline
tools/evalwrap list
tools/evalwrap leaderboard --metric total_time_sec
```

`run` は次の処理を実行します。

```text
./create_submit_file.bash
./docker_build.sh eval
make eval
```

評価出力が既にあり、収集とレポート生成だけを行いたい場合は `ingest` を使います。

## 出力

```text
analysis/runs/<run_id>/manifest.yaml
analysis/runs/<run_id>/raw/d1/
analysis/runs/<run_id>/processed/metrics.json
analysis/runs/<run_id>/processed/corner_summary.csv
analysis/runs/<run_id>/processed/trajectory_reference.csv
analysis/runs/<run_id>/processed/motion_log.csv
analysis/runs/<run_id>/report/index.html
analysis/experiments.sqlite
```

公式 JSON ファイルが見つからない場合でもクラッシュせず、`partial` 実行として
記録します。

## 参照軌道フォールバック

rosbag に `/planning/scenario_planning/trajectory` が含まれていない場合、
evalwrap は `multi_purpose_mpc_ros/config/config.yaml` で指定された MPC
参照 CSV をフォールバック軌道として使えます。

これにより、計画軌道が記録されていない場合でも、`corner_summary.csv`、
経路誤差メトリクス、HTML レポート内の Corner Splits マップを生成できます。

コーナー番号は `configs/thresholds.yaml` の `corner_id_rotation` で回転できます。
AI Challenge のデフォルト設定では、検出された2番目のコーナーから番号付けを
開始するため、最初に検出されたコーナーは `corner_08` になります。

## コーナー別タイムを出すための設定

通常は追加設定なしで、`run` または `ingest` 後の HTML レポートに
`Corner Splits` が表示されます。表示元は次のファイルです。

```text
analysis/runs/<run_id>/processed/corner_summary.csv
```

この CSV がヘッダだけの場合、HTML レポートには
`No corner split rows were generated.` と表示されます。

コーナー別タイムの生成には、評価出力内の `rosbag2_autoware` と、走行位置を
コース上のコーナーへ対応付けるための参照軌道が必要です。別環境で表示されない
場合は、まず次を確認してください。

```bash
wc -l analysis/runs/<run_id>/processed/corner_summary.csv
grep -nE "rosbag|reference|trajectory|MPC|warning" analysis/runs/<run_id>/manifest.yaml
```

### 参照軌道

`configs/default.yaml` では、rosbag に
`/planning/scenario_planning/trajectory` が含まれない場合でも、MPC 設定から
参照 CSV を読むフォールバックが有効です。

```yaml
reference_trajectory:
  enabled: true
  source: mpc_config
  mpc_config_path: aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/config.yaml
  use_when_rosbag_trajectory_missing: true
```

MPC の `config.yaml` とは別の CSV を明示したい場合は、任意の evalwrap 設定
YAML を作り、`--config` で渡します。`csv_path` は
`mpc_package_path` からの相対パス、またはリポジトリルートからの相対パスで
指定できます。

```yaml
reference_trajectory:
  enabled: true
  source: mpc_config
  mpc_package_path: aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros
  mpc_config_path: aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/config.yaml
  csv_path: env/final_ver3/traj_mincurv_manual.csv
  use_when_rosbag_trajectory_missing: true
```

```bash
tools/evalwrap --config path/to/evalwrap.yaml ingest --label baseline --path output/latest
```

### コーナー検出しきい値

コーナーの検出や番号付けは `configs/thresholds.yaml` で調整します。

```yaml
corner_curvature_min_1pm: 0.035
corner_min_length_m: 2.0
corner_merge_gap_m: 2.0
corner_padding_m: 1.0
corner_id_rotation: 1
```

まずは `corner_id_rotation` だけを使って、HTML 上の番号と実コース上の
呼び方を合わせるのがおすすめです。コーナー自体が検出されない場合だけ、
`corner_curvature_min_1pm` や `corner_min_length_m` を調整してください。

### よくある原因

- `raw/d*/rosbag2_autoware` が収集されていない。
- 実行環境に `rosbag2_py`、`rclpy`、対象メッセージ型などの ROS Python 依存が
  足りず、rosbag を読めていない。
- rosbag に `/localization/kinematic_state` などの位置付き時系列が入っていない。
- rosbag に計画軌道がなく、MPC 参照 CSV フォールバックも読めていない。
- 古い `evalwrap` を使っていて、コーナー分割や参照軌道フォールバックの実装が
  入っていない。
