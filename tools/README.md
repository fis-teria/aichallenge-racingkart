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

## tuning GUI 連携で AI Challenge 本体側に入れている変更

`tuning_gui` から control method、AWSIM ヘッドレス、NPC 台数を切り替えるには、
`tools/` だけでなく AI Challenge 本体側の起動系も同じ前提にしておく必要があります。
この作業ブランチでは次の連携変更を入れています。

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

また、ヘッドレス評価や tuning GUI の Path Editor で使うローカル調整データとして、
AI Challenge 本体側では次の差分を使っています。

- `multi_purpose_mpc_ros/config/config.yaml` の参照経路を
  `env/final_ver3/traj_mincurv_manual.csv` に切り替えています。
- 同じ MPC 設定で `a_min=-3.0`、`a_max=3.0`、`v_max=35.0km/h`、
  `ay_max=10`、および対応する `Q` / `QN` を有効にしています。
- `autostart_orchestrator.param.yaml` と `bag_manager.param.yaml` の rosbag 対象に、
  actuation command、planning trajectory、velocity / steering status、
  camera image / camera info、LiDAR scan、acceleration などを追加しています。
- Path Editor が生成した手動経路 CSV は
  `multi_purpose_mpc_ros/env/final_ver3/traj_mincurv_manual*.csv` に置かれます。

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
