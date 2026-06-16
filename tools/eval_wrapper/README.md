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
