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
