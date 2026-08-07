#!/bin/bash
# shellcheck disable=SC1091
# delay-aware MPC本体は既存のmulti_purpose_mpc_rosを再利用する。
# launch側でodomを補償済みトピックへremapし、このスクリプトは同じMPC実行体を起動するだけ。
source "$(ros2 pkg prefix multi_purpose_mpc_ros)/.venv/bin/activate"
exec python3 "$(ros2 pkg prefix multi_purpose_mpc_ros)/lib/multi_purpose_mpc_ros/mpc_controller" "$@"
