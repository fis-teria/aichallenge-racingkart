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

バックアップから戻す場合:

```bash
tools/scripts/apply_headless_overrides.sh --restore --backup-dir tools/scripts/backups/<backup-dir>
```

このパッチで上書きする主な内容:

- `docker-compose.yml` は `CONTROL_METHOD`、`LAUNCH_AWSIM`、`RUN_RVIZ`、
  `AWSIM_VEHICLES`、`AWSIM_LAPS`、`AWSIM_TIMEOUT`、`AWSIM_EXTRA_ARGS`
  を Autoware コンテナへ渡します。
- `aichallenge/run_evaluation.bash` は上記の環境変数を
  `evaluation.launch.xml` の launch 引数へ変換します。ヘッドレス時は
  `AWSIM_EXTRA_ARGS='-batchmode -nographics --camera false --lidar false'`
  を渡し、AWSIM を起動したまま画面描画と重いセンサ描画を抑えます。
- `aichallenge/run_autoware.bash` は `CONTROL_METHOD` を通常 dev 起動にも
  渡します。`aichallenge/build_autoware.bash` は
  `COLCON_PARALLEL_WORKERS` を見て、重い環境では並列数を絞れるようにしています。
- `aichallenge_system.launch.xml`、`evaluation.launch.xml`、
  `aichallenge_submit.launch.xml` は `control_method` を launch チェーンへ通します。
  `evaluation.launch.xml` は `launch_awsim`、`awsim_vehicles`、`awsim_laps`、
  `awsim_timeout`、`awsim_extra_args` も受け取ります。

MPC の参照CSVや rosbag 収録トピックなどの走行チューニングデータは、
このヘッドレス連携パッチには含めません。必要な環境ごとに tuning GUI や
別管理の設定差分として扱います。

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
