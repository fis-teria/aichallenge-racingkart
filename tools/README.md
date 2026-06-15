# AI Challenge Local Tools

Local tooling for the AI Challenge racing kart repository.

This directory is designed to be managed independently from the upstream AI
Challenge source tree. Place it at:

```text
aichallenge-racingkart/tools/
```

The tools use the parent directory as the AI Challenge repository root, so the
same layout works whether this directory is installed as a Git submodule or
extracted from a ZIP archive.

## Commands

```bash
tools/evalwrap doctor
tools/evalwrap run --label baseline
tools/evalwrap ingest --label manual --path output/latest
tools/run_tuning_gui.bash --background
```

## Optional Top-Level Shortcuts

If your workspace has this layout:

```text
workspace-root/
  aichallenge-racingkart/
    tools/
```

you can place two shortcut scripts at `workspace-root/evalwrap` and
`workspace-root/run_tuning_gui.bash`. They let you run the tools from the
workspace root exactly like:

```bash
./evalwrap doctor
./evalwrap run --label baseline
./run_tuning_gui.bash --background
```

Create `workspace-root/evalwrap`:

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

Create `workspace-root/run_tuning_gui.bash`:

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

Make both scripts executable:

```bash
chmod +x evalwrap run_tuning_gui.bash
```

## Git Submodule Layout

Recommended parent-repository layout:

```text
aichallenge-racingkart/
  tools/  # submodule
```

Clone with:

```bash
git clone --recurse-submodules <aichallenge-racingkart-url>
```

For an already cloned repository:

```bash
git submodule update --init --recursive
```

## ZIP Layout

For a ZIP install, extract the archive so that this README ends up at:

```text
aichallenge-racingkart/tools/README.md
```

Generated local files such as GUI state, runtime logs, backups, and Python
caches are ignored by this tools repository.
