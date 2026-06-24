# AI Challenge ローカルツール

AI Challenge レーシングカートリポジトリ向けのローカル補助ツール群です。

このディレクトリは、上流の AI Challenge ソースツリーとは独立して管理できる
ように作られています。配置先は次のパスです。

```text
aichallenge-racingkart/tools/
```

各ツールは親ディレクトリを AI Challenge リポジトリのルートとして扱います。
そのため、このディレクトリを Git サブモジュールとして配置した場合でも、
ZIP アーカイブから展開した場合でも同じレイアウトで動作します。

## コマンド

```bash
tools/evalwrap doctor
tools/evalwrap run --label baseline
tools/evalwrap ingest --label manual --path output/latest
tools/run_tuning_gui.bash --background
```

## 初心者向け資料

- [Tuning GUI 変更点ガイド ROS 2 初心者向け](docs/tuning_gui_changes_for_ros2_beginners.md)
- [evalwrap レポート出力ガイド ROS 2 初心者向け](docs/evalwrap_report_outputs_for_ros2_beginners.md)

## 2026 更新メモ

AI Challenge 2026 では、上流側の `Makefile`、`docker-compose.yml`、
`aichallenge/run_simulator.bash`、`aichallenge/run_evaluation.bash`、
autostart 周辺、評価用 launch 構成が更新されています。そのため、2025/旧環境向けに
作った headless/GUI 連携差分を 2026 の上流へそのまま `cherry-pick` すると
衝突します。

2026 環境へ入れる場合は、次のどちらかで導入してください。

- すでに 2026 上流とローカル差分を解決済みのブランチを使う。
- 既存の AI Challenge 2026 チェックアウトへ、この `tools/` の
  `tools/scripts/setup.sh` から上書きパッチを適用する。

このツール群で 2026 向けに共有している主な内容:

- tuning GUI から `CONTROL_METHOD`、AWSIM ヘッドレス、NPC 台数、周回数、
  timeout を渡すための起動系上書きファイル。
- `delay_aware_mpc` などの control method を dev/eval の launch 経路へ通す設定。
- 2026 full patch profile による autostart の initial pose timeout、
  capture 停止 timeout、`output/latest` の `result-summary.json` リンク作成。
- GUI eval 後に `evalwrap` で `result-summary.json`、`result-details.json`、
  rosbag、motion log、HTML レポートを回収・比較するためのローカル評価補助。
- 2026 上流の compose/setup 変更と共存しやすいようにした
  headless override の適用・復元手順。

### 2026 ブランチを使う場合

2026 上流とローカル差分を解決済みの作業ブランチがある場合は、そのブランチを使うのが
一番安全です。

```bash
cd aichallenge-racingkart
git fetch origin
git checkout codex/investigate-2026-merge
git pull --ff-only
tools/evalwrap doctor
tools/run_tuning_gui.bash --background
```

GUI から build/eval した後は、最新 run を取り込んで比較できます。

```bash
tools/evalwrap ingest --label gui-eval --path output/latest
tools/evalwrap leaderboard --metric best_lap_sec
```

### 既存の 2026 チェックアウトを更新する場合

`aichallenge_support_tools` から `tools/` を直接コピーして使う場合の例です。
生成物やバックアップはコピー対象から外します。

```bash
cd aichallenge-racingkart
rsync -a --delete \
  --exclude '.git/' \
  --exclude '.pytest_cache/' \
  --exclude '__pycache__/' \
  --exclude 'scripts/backups/' \
  --exclude 'tuning_gui/.state/' \
  --exclude 'tuning_gui/backups/' \
  --exclude 'tuning_gui/history/' \
  --exclude 'tuning_gui/runtime/' \
  ../aichallenge_support_tools/tools/ tools/
tools/scripts/setup.sh --dry-run --profile 2026-full
tools/scripts/setup.sh --apply --profile 2026-full --yes
```

`tools/` を Git サブモジュールとして管理している場合は、サブモジュールを更新してから
同じ setup を実行します。

```bash
cd aichallenge-racingkart
git submodule update --init --recursive
git -C tools pull --ff-only
tools/scripts/setup.sh --dry-run --profile 2026-full
tools/scripts/setup.sh --apply --profile 2026-full --yes
```

## tuning GUI ヘッドレス連携パッチ

`tuning_gui` から control method、AWSIM ヘッドレス、NPC 台数を切り替えるには、
`tools/` だけでなく AI Challenge 本体側の起動系も同じ前提にしておく必要があります。
元の AI Challenge 環境へ恒久的に変更を入れないように、上書き用ファイルは
`tools/scripts/headless_overrides/` に格納しています。

まずは dry-run で上書き対象を確認します。

```bash
tools/scripts/setup.sh --dry-run
```

問題なければ適用します。適用時は `tools/scripts/backups/` に元ファイルの
バックアップを作ってからコピーします。

```bash
tools/scripts/setup.sh --apply
```

2026 full patch として autostart の終了処理安定化まで含める場合:

```bash
tools/scripts/setup.sh --dry-run --profile 2026-full
tools/scripts/setup.sh --apply --profile 2026-full
```

バックアップから戻す場合:

```bash
tools/scripts/apply_headless_overrides.sh --restore --backup-dir tools/scripts/backups/<backup-dir>
```

このパッチで上書きする主な内容:

- `Makefile` は GUI の `gate` ボタンから呼ぶ `make gate1`〜`make gate3` を追加し、
  Safety Gate シナリオを `make dev` と同じAWSIM/Autoware起動経路で実行します。
- `docker-compose.yml` は `CONTROL_METHOD`、`LAUNCH_AWSIM`、`RUN_RVIZ`、
  `AWSIM_START_MODE`、`AWSIM_START_COUNT_SECONDS`、`AWSIM_VEHICLES`、
  `AWSIM_LAPS`、`AWSIM_TIMEOUT`、`AWSIM_EXTRA_ARGS` を Autoware/AWSIM
  コンテナへ渡します。
- `aichallenge/run_evaluation.bash` は上記の環境変数を
  `evaluation.launch.xml` の launch 引数へ変換します。ヘッドレス時は
  `AWSIM_EXTRA_ARGS='-batchmode -nographics --camera false --lidar false'`
  を渡し、AWSIM を起動したまま画面描画と重いセンサ描画を抑えます。
- `aichallenge/run_simulator.bash` は dev 側のAWSIM起動でも
  `AWSIM_START_MODE`、`AWSIM_VEHICLES`、`AWSIM_LAPS`、`AWSIM_TIMEOUT`
  を受け取り、Tuning GUIのSafety GateやSimulator設定と同じ起動経路を使います。
- `aichallenge/run_autoware.bash` は `CONTROL_METHOD` を通常 dev 起動にも
  渡します。`aichallenge/build_autoware.bash` は
  `COLCON_PARALLEL_WORKERS` を見て、重い環境では並列数を絞れるようにしています。
- `aichallenge_system.launch.xml`、`evaluation.launch.xml`、
  `aichallenge_submit.launch.xml` は `control_method` を launch チェーンへ通します。
  `evaluation.launch.xml` は `launch_awsim`、`awsim_vehicles`、`awsim_laps`、
  `awsim_timeout`、`awsim_extra_args` も受け取ります。
- `--profile 2026-full` は上記に加えて
  `autostart_orchestrator_node.py` と `autostart_orchestrator.param.yaml` を上書きし、
  initial pose service の timeout、capture 停止 timeout、`result-summary.json`
  の latest リンク作成を有効にします。

MPC の参照CSVや controller package 本体などの走行チューニングデータは、
このヘッドレス連携パッチには含めません。`--profile 2026-full` で入る
rosbag topic 設定は、評価成果物回収用の autostart 設定として扱います。
走行チューニングは必要な環境ごとに tuning GUI や別管理の設定差分として扱います。

## 任意のトップレベルショートカット

ワークスペースが次の構成になっている場合:

```text
workspace-root/
  aichallenge-racingkart/
    tools/
```

`workspace-root/evalwrap` と `workspace-root/run_tuning_gui.bash` に
ショートカットスクリプトを置けます。これにより、ワークスペースルートから
次のようにツールを実行できます。

```bash
./evalwrap doctor
./evalwrap run --label baseline
./run_tuning_gui.bash --background
```

`workspace-root/evalwrap` を作成します。

```bash
#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AIC_REPO="${ROOT_DIR}/aichallenge-racingkart"
EVALWRAP_DIR="${AIC_REPO}/tools/eval_wrapper"

usage() {
    cat <<'EOF'
Usage:
  ./evalwrap <evalwrap-command> [options]

Examples:
  ./evalwrap doctor
  ./evalwrap ingest --label baseline --path aichallenge-racingkart/output/latest
  ./evalwrap list
  ./evalwrap leaderboard --metric total_time_sec
  ./evalwrap run --label baseline --skip-build

This wrapper runs the evalwrap Python package inside:
  aichallenge-racingkart/tools/eval_wrapper
EOF
}

if [ ! -d "${AIC_REPO}" ]; then
    echo "[evalwrap][ERROR] Missing submodule directory: ${AIC_REPO}" >&2
    echo "Run: git submodule update --init --recursive" >&2
    exit 1
fi

if [ ! -f "${EVALWRAP_DIR}/evalwrap/cli.py" ]; then
    echo "[evalwrap][ERROR] evalwrap package is not found under: ${EVALWRAP_DIR}" >&2
    exit 1
fi

export PYTHONPATH="${EVALWRAP_DIR}${PYTHONPATH:+:${PYTHONPATH}}"
cd "${AIC_REPO}"

case "${1-}" in
    "" | -h | --help)
        usage
        echo
        exec python3 -m evalwrap --help
        ;;
esac

exec python3 -m evalwrap --repo-root "${AIC_REPO}" "$@"
```

`workspace-root/run_tuning_gui.bash` を作成します。

```bash
#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GUI_SCRIPT="${WORKSPACE_DIR}/aichallenge-racingkart/tools/tuning_gui/run_tuning_gui.bash"

if [[ ! -x "${GUI_SCRIPT}" ]]; then
    echo "error: tuning GUI launcher not found or not executable: ${GUI_SCRIPT}" >&2
    exit 1
fi

exec "${GUI_SCRIPT}" "$@"
```

両方のスクリプトに実行権限を付けます。

```bash
chmod +x evalwrap run_tuning_gui.bash
```

## Git サブモジュール構成

推奨する親リポジトリ構成:

```text
aichallenge-racingkart/
  tools/  # submodule
```

クローン時:

```bash
git clone --recurse-submodules <aichallenge-racingkart-url>
```

既にクローン済みのリポジトリでは:

```bash
git submodule update --init --recursive
```

## ZIP 配置

ZIP で導入する場合は、この README が次の場所に来るように展開してください。

```text
aichallenge-racingkart/tools/README.md
```

GUI の状態、実行ログ、バックアップ、Python キャッシュなどの生成物は、この
ツール用リポジトリでは無視されます。
