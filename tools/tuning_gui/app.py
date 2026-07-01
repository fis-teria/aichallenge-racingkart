#!/usr/bin/env python3
"""Local tuning GUI for AI Challenge launch parameters."""

from __future__ import annotations

import csv
import difflib
import hashlib
import json
import math
import os
import re
import shutil
import signal
import struct
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as ET
import zlib
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, unquote, urlparse

import yaml


APP_DIR = Path(__file__).resolve().parent
REPO_ROOT = APP_DIR.parents[1]
STATIC_DIR = APP_DIR / "static"
STATIC_CACHE_CONTROL = "no-store, max-age=0"
STATE_DIR = APP_DIR / ".state"
BACKUP_DIR = APP_DIR / "backups"
PRESET_DIR = APP_DIR / "presets"
HISTORY_DIR = APP_DIR / "history"
RUNTIME_DIR = APP_DIR / "runtime"
COMMAND_DIR = RUNTIME_DIR / "commands"
STATE_FILE = STATE_DIR / "state.json"
DESCRIPTION_FILE = STATE_DIR / "parameter_descriptions.json"
HISTORY_FILE = HISTORY_DIR / "runs.jsonl"

LAUNCH_ROOT = Path("aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch")
SYSTEM_LAUNCH_ROOT = Path("aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch")
MPC_ROOT = Path("aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros")
OVERTAKE_ROOT = Path("aichallenge/workspace/src/aichallenge_submit/overtake_planner")
MPC_CONFIG_PATH = MPC_ROOT / "config/config.yaml"
DELAY_AWARE_MPC_CONFIG_PATH = Path(
    "aichallenge/workspace/src/aichallenge_submit/delay_aware_mpc_ros/config/delay_aware_config.yaml"
)
REFERENCE_CONFIG_PATHS = {
    "mpc": MPC_CONFIG_PATH,
    "delay_aware_mpc": DELAY_AWARE_MPC_CONFIG_PATH,
}

DESCRIPTION_DEFAULTS: dict[str, str] = {
    "common.save_config": "実行時の設定保存を有効にするフラグです。通常のチューニングではfalseのままでOKです。",
    "sim_logger.animation_enabled": "MPCシミュレーションログのアニメーション出力を有効にする設定です。",
    "map.yaml_path": "MPCが参照する占有格子地図YAMLのパッケージ内パスです。",
    "waypoints.csv_path": "参照パスを自動生成する場合に使うwaypoint CSVです。reference_path.csv_pathが空でない場合はそちらが優先されます。",
    "obstacles.csv_path": "静的障害物CSVのパスです。空の場合は障害物情報をtopicから受け取ります。",
    "obstacles.radius": "障害物を円として扱うときの半径[m]です。大きいほど保守的に避けます。",
    "reference_path.update_by_topic": "trueにするとAutoware Trajectory topicから参照パスを更新します。CSV固定で走る場合はfalseです。",
    "reference_path.csv_path": "MPCが追従する参照パスCSVです。Path Editorの保存先切り替え対象です。",
    "reference_path.resolution": "参照パスを内部waypointへ補間するときのおおよその間隔[m]です。",
    "reference_path.smoothing_distance": "参照パス生成時の移動平均幅です。大きいほど滑らかになりますが、ラインが丸まりやすくなります。",
    "reference_path.max_width": "参照パス左右に確保する走行可能幅[m]です。経路制約や可視化に使われます。",
    "reference_path.circular": "参照パスを周回コースとして扱うかどうかです。AI Challengeの周回では通常trueです。",
    "reference_path.use_path_constraints_topic": "PathConstraints topicから走行可能境界を受け取るかどうかです。",
    "reference_path.use_border_cells_topic": "BorderCells topicから境界セル情報を受け取るかどうかです。",
    "bicycle_model.length": "MPC内の車両ホイールベース相当の長さ[m]です。ステア応答に影響します。",
    "bicycle_model.width": "MPCが安全余裕込みで扱う車幅[m]です。大きいほど壁や障害物に対して保守的になります。",
    "mpc.N": "MPCの予測ホライズン数です。大きいほど先を見ますが計算負荷が増えます。",
    "mpc.Q": "[横ずれ, 姿勢ずれ, 時間]の状態コストです。横ずれを大きくすると参照線へ強く戻ろうとします。",
    "mpc.R": "[速度, ステア]の入力コストです。速度コストを大きくすると速度指令の変化を抑えやすくなります。",
    "mpc.QN": "予測終端での[横ずれ, 姿勢ずれ, 時間]コストです。終端の姿勢や位置の収束性に効きます。",
    "mpc.a_min": "最小加速度[m/s^2]です。負の値を大きくすると強く減速できます。",
    "mpc.a_max": "最大加速度[m/s^2]です。大きいほど立ち上がりが速くなります。",
    "mpc.delta_max_deg": "許容する最大タイヤ切れ角[deg]です。大きいほど曲がれますが不安定になりやすいです。",
    "mpc.steer_rate_max": "ステア角変化速度の上限[rad/s]です。小さいほど滑らか、大きいほど素早く切れます。",
    "mpc.control_rate": "MPC制御周期[Hz]です。高いほど細かく制御しますが計算負荷が増えます。",
    "mpc.steering_tire_angle_gain_var": "MPC出力ステアと実車/AWSIM入力のゲイン補正です。曲がり量のずれを補正します。",
    "mpc.accel_low_pass_gain": "加速度指令のローパス係数です。1.0でフィルタなし、低いほど滑らかになります。",
    "mpc.steer_low_pass_gain": "ステア指令のローパス係数です。1.0でフィルタなし、低いほど滑らかになります。",
    "mpc.wp_id_offset": "MPC予測開始時に参照waypointを何点先へ進めるかです。制御遅れや戻りすぎ対策に使います。",
    "mpc.use_max_kappa_pred": "予測ホライズン内の最大曲率から速度上限を決めるかどうかです。trueの方がコーナー手前で保守的になりやすいです。",
    "mpc.use_curvature_speed_profile": "参照パス曲率からwaypointごとの速度プロファイルを作るかどうかです。trueでコーナー速度が曲率に応じて下がります。",
    "mpc.use_ref_vel_as_speed_cap": "ref_vel.yamlの区間速度を目標速度の上限として使うかどうかです。trueでも曲率速度は保持され、低い方が採用されます。",
    "mpc.speed_profile_debug_publish_period_sec": "/mpc/speed_profile_debugをpublishする周期[s]です。0以下で停止します。",
    "mpc.v_max": "MPCの最高速度[km/h]です。区間速度や曲率制限より上には出ません。",
    "mpc.ay_max": "許容横加速度[m/s^2]です。小さいほどコーナー速度が下がり安全寄りになります。",
    "v2x_obstacle_avoidance.vehicle_radius": "V2X車両を障害物として扱うときの半径[m]です。",
    "v2x_obstacle_avoidance.v_max_safety": "V2X速度推定の安全上限[m/s]です。異常に速い推定値を破棄します。",
    "v2x_obstacle_avoidance.position_jump_threshold": "V2X位置が瞬間的に飛んだと判定する距離[m]です。",
    "overtake_planner_node.ros__parameters.enabled": "overtake_planner全体を有効にするフラグです。falseにするとMPCへの追い抜きoverrideを止めます。",
    "overtake_planner_node.ros__parameters.reference_csv": "追い抜き判断のFrenet基準に使う参照CSVです。MPCの走行ラインと合わせるのが基本です。",
    "overtake_planner_node.ros__parameters.own_vehicle_id": "V2X上で自車として除外するIDです。autoならROS_DOMAIN_IDからd1などを推定します。",
    "overtake_planner_node.ros__parameters.lookahead_s_m": "前方車両を追い抜き判断に入れる縦方向距離[m]です。大きいほど早めに反応します。",
    "overtake_planner_node.ros__parameters.follow_trigger_s_m": "前走車へ追従を始める距離[m]です。大きいほど詰める前に減速します。",
    "overtake_planner_node.ros__parameters.same_corridor_width_m": "前方閉塞判定で同じ走行コリドーとみなす横幅[m]です。",
    "overtake_planner_node.ros__parameters.dv_block_threshold_mps": "相対速度で詰まり中と判定するしきい値[m/s]です。小さいほど早くblockedになります。",
    "overtake_planner_node.ros__parameters.side_by_side_s_m": "横並びとみなす縦方向距離[m]です。",
    "overtake_planner_node.ros__parameters.side_margin_m": "横並びとみなす横方向距離[m]です。大きいほど横並び検出が広くなります。",
    "overtake_planner_node.ros__parameters.side_yield_s_m": "横並び中に相手がこの距離[m]以上前なら、自車が後ろへ譲る判定にします。",
    "overtake_planner_node.ros__parameters.side_by_side_target_gap_m": "横並び維持時に相手から横へ確保したい距離[m]です。",
    "overtake_planner_node.ros__parameters.side_by_side_shift_distance_m": "横並び維持の横移動をならす距離[m]です。大きいほどゆっくり横へ逃げます。",
    "overtake_planner_node.ros__parameters.side_by_side_speed_cap_mps": "横並び維持時の速度上限[m/s]です。相手速度からyield marginを引いた値との低い方を使います。",
    "overtake_planner_node.ros__parameters.min_pass_gap_m": "左右追い抜き候補に必要な最小横ギャップ[m]です。",
    "overtake_planner_node.ros__parameters.yield_speed_margin_mps": "譲り・横並び時に相手速度から引く速度余裕[m/s]です。大きいほど後ろに下がりやすくなります。",
    "overtake_planner_node.ros__parameters.yield_rejoin_gap_m": "譲り状態から追従へ戻る前方距離[m]です。",
    "overtake_planner_node.ros__parameters.left_offset_m": "左追い抜き時にMPCへ渡す横オフセット[m]です。",
    "overtake_planner_node.ros__parameters.right_offset_m": "右追い抜き時にMPCへ渡す横オフセット[m]です。",
    "overtake_planner_node.ros__parameters.follow_speed_margin_mps": "追従時に前走車速度から引く速度余裕[m/s]です。",
    "overtake_planner_node.ros__parameters.safety_ellipse_a_m": "他車との安全楕円の前後方向半径[m]です。",
    "overtake_planner_node.ros__parameters.safety_ellipse_b_m": "他車との安全楕円の左右方向半径[m]です。",
    "overtake_planner_node.ros__parameters.min_ellipse_h": "安全楕円の最小余裕です。大きいほど他車へ保守的になります。",
    "overtake_planner_node.ros__parameters.pass_safe_required_cycles": "追い抜き候補が安全と連続判定される必要周期数です。",
    "overtake_planner_node.ros__parameters.merge_front_gap_m": "追い抜き後に中心へ戻るための前方ギャップ[m]です。",
    "overtake_planner_node.ros__parameters.abort_timeout_sec": "追い抜き状態を続けすぎた場合に中止復帰へ入る時間[s]です。",
    "overtake_planner_node.ros__parameters.keep_mode_bonus": "現在モードを少し優先してチャタリングを抑えるスコア補正です。",
}

DELAY_AWARE_XML_DEFAULTS: dict[str, str] = {
    "delay_enabled": "delay-aware odometry補償を有効にするフラグです。baseline比較ではfalseまたはmode=baselineを使います。",
    "delay_mode": "遅延補償モードです。baseline, state_shift, state_shift_with_steer_lag, delay_augmentedを選べます。",
    "steering_delay_sec": "ステアリング遅延として前方予測する時間[s]です。AWSIM想定値は0.20sです。",
    "prediction_dt": "遅延中の車両運動を積分する刻み幅[s]です。小さいほど精細ですが計算量が増えます。",
    "steering_time_constant_sec": "state_shift_with_steer_lagで使うステア一次遅れの時定数[s]です。",
    "wheelbase": "遅延中のyaw予測に使うホイールベース[m]です。",
    "use_reference_time_shift": "trueにすると遅延後poseをMPCへ渡し、参照path投影も遅延後基準になります。",
    "use_steering_status": "/vehicle/status/steering_statusが新鮮な場合に実測ステアを優先して使います。",
    "steering_status_timeout_sec": "steering_statusを新鮮とみなす最大経過時間[s]です。",
    "use_yaw_rate_fallback": "steering_statusが使えない場合にyaw rateからステアを推定します。",
    "use_command_history_fallback": "steering_status/yaw rateが使えない場合に過去の制御指令からステアを推定します。",
    "min_velocity_for_yaw_prediction": "yaw rateからステア推定する最低速度[m/s]です。低速時の発散を避けます。",
    "debug_publish_period_sec": "/delay_aware_mpc/debugと/delayed_poseをpublishする周期[s]です。",
}

CONTROL_DEFAULT_XMLS = [
    SYSTEM_LAUNCH_ROOT / "launch/evaluation.launch.xml",
    SYSTEM_LAUNCH_ROOT / "launch/aichallenge_system.launch.xml",
    LAUNCH_ROOT / "launch/aichallenge_submit.launch.xml",
    LAUNCH_ROOT / "launch/reference.launch.xml",
]

CATALOG: dict[str, list[dict[str, str]]] = {
    "common": [
        {
            "label": "System launch",
            "path": str(SYSTEM_LAUNCH_ROOT / "launch/aichallenge_system.launch.xml"),
            "kind": "xml",
        },
        {
            "label": "Evaluation launch",
            "path": str(SYSTEM_LAUNCH_ROOT / "launch/evaluation.launch.xml"),
            "kind": "xml",
        },
        {
            "label": "Submit launch",
            "path": str(LAUNCH_ROOT / "launch/aichallenge_submit.launch.xml"),
            "kind": "xml",
        },
        {
            "label": "Reference launch",
            "path": str(LAUNCH_ROOT / "launch/reference.launch.xml"),
            "kind": "xml",
        },
        {
            "label": "Autostart / rosbag topics",
            "path": "aichallenge/workspace/src/aichallenge_system/autostart_orchestrator_py/config/autostart_orchestrator.param.yaml",
            "kind": "yaml",
        },
        {
            "label": "AWSIM state manager",
            "path": "aichallenge/workspace/src/aichallenge_system/autostart_orchestrator_py/config/awsim_state_manager.param.yaml",
            "kind": "yaml",
        },
    ],
    "mpc": [
        {
            "label": "MPC core config",
            "path": "aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/config.yaml",
            "kind": "yaml",
        },
        {
            "label": "MPC reference velocity",
            "path": "aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/ref_vel.yaml",
            "kind": "yaml",
        },
        {
            "label": "MPC launch",
            "path": str(LAUNCH_ROOT / "launch/control/mpc.launch.xml"),
            "kind": "xml",
        },
        {
            "label": "Overtake planner params",
            "path": str(OVERTAKE_ROOT / "config/overtake_planner.param.yaml"),
            "kind": "yaml",
        },
        {
            "label": "Overtake planner launch",
            "path": str(OVERTAKE_ROOT / "launch/overtake_planner.launch.xml"),
            "kind": "xml",
        },
        {
            "label": "MPC reference path",
            "path": "aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/env/final_ver3/traj_mincurv.csv",
            "kind": "csv",
        },
    ],
    "delay_aware_mpc": [
        {
            "label": "Delay-aware MPC launch params",
            "path": str(LAUNCH_ROOT / "launch/control/delay_aware_mpc.launch.xml"),
            "kind": "xml",
        },
        {
            "label": "Overtake planner params",
            "path": str(OVERTAKE_ROOT / "config/overtake_planner.param.yaml"),
            "kind": "yaml",
        },
        {
            "label": "Overtake planner launch",
            "path": str(OVERTAKE_ROOT / "launch/overtake_planner.launch.xml"),
            "kind": "xml",
        },
        {
            "label": "Delay-aware MPC base YAML",
            "path": "aichallenge/workspace/src/aichallenge_submit/delay_aware_mpc_ros/config/delay_aware_config.yaml",
            "kind": "yaml",
        },
        {
            "label": "Delay compensator YAML fallback",
            "path": "aichallenge/workspace/src/aichallenge_submit/delay_aware_mpc_ros/config/delay_compensator.param.yaml",
            "kind": "yaml",
        },
        {
            "label": "Delay-aware MPC ref velocity YAML",
            "path": "aichallenge/workspace/src/aichallenge_submit/delay_aware_mpc_ros/config/ref_vel.yaml",
            "kind": "yaml",
        },
        {
            "label": "Delay-aware MPC C++ node",
            "path": "aichallenge/workspace/src/aichallenge_submit/delay_aware_mpc_ros/src/delay_compensated_odometry_node.cpp",
            "kind": "text",
        },
    ],
    "pure_pursuit": [
        {
            "label": "Pure Pursuit launch params",
            "path": str(LAUNCH_ROOT / "launch/control/pure_pursuit.launch.xml"),
            "kind": "xml",
        },
        {
            "label": "Trajectory generator raceline",
            "path": "aichallenge/workspace/src/aichallenge_submit/simple_trajectory_generator/data/raceline_awsim_30km_from_garage.csv",
            "kind": "csv",
        },
    ],
    "simple_delay_aware_control": [
        {
            "label": "Simple delay-aware control launch",
            "path": str(LAUNCH_ROOT / "launch/control/simple_delay_aware_control.launch.xml"),
            "kind": "xml",
        },
        {
            "label": "Simple delay-aware control params",
            "path": "aichallenge/workspace/src/aichallenge_submit/simple_delay_aware_control/config/simple_delay_aware_control.param.yaml",
            "kind": "yaml",
        },
        {
            "label": "Simple delay-aware control core",
            "path": "aichallenge/workspace/src/aichallenge_submit/simple_delay_aware_control/src/control_core.cpp",
            "kind": "text",
        },
        {
            "label": "Simple delay-aware control node",
            "path": "aichallenge/workspace/src/aichallenge_submit/simple_delay_aware_control/src/simple_delay_aware_control_node.cpp",
            "kind": "text",
        },
        {
            "label": "Exercise holes",
            "path": "aichallenge/workspace/src/aichallenge_submit/simple_delay_aware_control/src/control_core_exercise.cpp",
            "kind": "text",
        },
        {
            "label": "Teaching spec",
            "path": "aichallenge/workspace/src/aichallenge_submit/simple_delay_aware_control/docs/01_implementation_spec.md",
            "kind": "text",
        },
        {
            "label": "Logic explanation",
            "path": "aichallenge/workspace/src/aichallenge_submit/simple_delay_aware_control/docs/02_logic_explanation.md",
            "kind": "text",
        },
        {
            "label": "Launch integration guide",
            "path": "aichallenge/workspace/src/aichallenge_submit/simple_delay_aware_control/docs/03_launch_integration_guide.md",
            "kind": "text",
        },
        {
            "label": "Trajectory generator raceline",
            "path": "aichallenge/workspace/src/aichallenge_submit/simple_trajectory_generator/data/raceline_awsim_30km_from_garage.csv",
            "kind": "csv",
        },
    ],
    "simple_dealay_aware_contorol": [
        {
            "label": "Simple delay-aware control launch",
            "path": str(LAUNCH_ROOT / "launch/control/simple_delay_aware_control.launch.xml"),
            "kind": "xml",
        },
        {
            "label": "Simple delay-aware control params",
            "path": "aichallenge/workspace/src/aichallenge_submit/simple_delay_aware_control/config/simple_delay_aware_control.param.yaml",
            "kind": "yaml",
        },
        {
            "label": "Exercise holes",
            "path": "aichallenge/workspace/src/aichallenge_submit/simple_delay_aware_control/src/control_core_exercise.cpp",
            "kind": "text",
        },
        {
            "label": "Teaching spec",
            "path": "aichallenge/workspace/src/aichallenge_submit/simple_delay_aware_control/docs/01_implementation_spec.md",
            "kind": "text",
        },
        {
            "label": "Logic explanation",
            "path": "aichallenge/workspace/src/aichallenge_submit/simple_delay_aware_control/docs/02_logic_explanation.md",
            "kind": "text",
        },
        {
            "label": "Launch integration guide",
            "path": "aichallenge/workspace/src/aichallenge_submit/simple_delay_aware_control/docs/03_launch_integration_guide.md",
            "kind": "text",
        },
    ],
    "tiny_lidar_net": [
        {
            "label": "Tiny LiDAR Net wrapper launch",
            "path": str(LAUNCH_ROOT / "launch/control/tiny_lidar_net.launch.xml"),
            "kind": "xml",
        },
        {
            "label": "Tiny LiDAR Net node launch",
            "path": "aichallenge/workspace/src/aichallenge_submit/tiny_lidar_net_controller/launch/tiny_lidar_net.launch.xml",
            "kind": "xml",
        },
        {
            "label": "Tiny LiDAR Net params",
            "path": "aichallenge/workspace/src/aichallenge_submit/tiny_lidar_net_controller/config/tiny_lidar_net_node.param.yaml",
            "kind": "yaml",
        },
        {
            "label": "Virtual LaserScan generator launch",
            "path": "aichallenge/workspace/src/aichallenge_submit/laserscan_generator/launch/laserscan_generator.launch.xml",
            "kind": "xml",
        },
        {
            "label": "Virtual LaserScan generator params",
            "path": "aichallenge/workspace/src/aichallenge_submit/laserscan_generator/config/laserscan_generator_node.param.yaml",
            "kind": "yaml",
        },
        {
            "label": "Virtual lane CSV",
            "path": "aichallenge/workspace/src/aichallenge_submit/laserscan_generator/map/lane.csv",
            "kind": "csv",
        },
    ],
    "pilot_net": [
        {
            "label": "PilotNet wrapper launch",
            "path": str(LAUNCH_ROOT / "launch/control/pilot_net.launch.xml"),
            "kind": "xml",
        },
        {
            "label": "PilotNet node launch",
            "path": "aichallenge/workspace/src/aichallenge_submit/pilot_net_controller/launch/pilot_net.launch.xml",
            "kind": "xml",
        },
        {
            "label": "PilotNet params",
            "path": "aichallenge/workspace/src/aichallenge_submit/pilot_net_controller/config/pilot_net_node.param.yaml",
            "kind": "yaml",
        },
    ],
    "joycon": [
        {
            "label": "JoyCon launch",
            "path": str(LAUNCH_ROOT / "launch/control/joycon.launch.xml"),
            "kind": "xml",
        },
        {
            "label": "Teleop params",
            "path": "aichallenge/workspace/src/aichallenge_tools/teleop_manager/config/teleop.param.yaml",
            "kind": "yaml",
        },
    ],
}


@dataclass
class CommandState:
    id: str
    action: str
    command: str
    control_method: str
    note: str
    headless: bool
    npc_count: int
    simulator_options: dict[str, Any]
    safety_gate: str | None
    status: str
    started_at: str
    finished_at: str | None
    returncode: int | None
    log_path: str
    pid: int | None
    snapshot_dir: str | None


command_lock = threading.Lock()
active_process: subprocess.Popen[str] | None = None
active_state: CommandState | None = None

SAFETY_GATES: dict[str, dict[str, Any]] = {
    "gate1": {
        "label": "gate1 障害物停止",
        "scenario": "SafetyGate/scenario1.yaml",
        "vehicles": 4,
    },
    "gate2": {
        "label": "gate2 追い越し",
        "scenario": "SafetyGate/scenario2.yaml",
        "vehicles": 4,
    },
    "gate3": {
        "label": "gate3 車線維持",
        "scenario": "SafetyGate/scenario3.yaml",
        "vehicles": 1,
    },
}

SIMULATOR_BOOL_OPTIONS = {
    "camera": "--camera",
    "lidar": "--lidar",
    "sound": "--sound",
    "collisions": "--collisions",
    "wall_recovery": "--wall-recovery",
    "ranking": "--ranking",
    "manual_mode": "--manual-mode",
}

AWSIM_FIXED_OPTIONS = {
    "-force-vulkan",
    "--start-mode",
    "--start-count-seconds",
    "--vehicles",
    "--laps",
    "--timeout",
}


def now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def ensure_dirs() -> None:
    for path in (STATE_DIR, BACKUP_DIR, PRESET_DIR, HISTORY_DIR, COMMAND_DIR):
        path.mkdir(parents=True, exist_ok=True)


def rel_path(path: str | Path) -> Path:
    value = Path(str(path))
    if value.is_absolute():
        value = value.relative_to(REPO_ROOT)
    value = Path(os.path.normpath(str(value)))
    if value.parts and value.parts[0] == "..":
        raise ValueError("path escapes repository")
    return value


def abs_path(path: str | Path) -> Path:
    candidate = (REPO_ROOT / rel_path(path)).resolve()
    candidate.relative_to(REPO_ROOT.resolve())
    return candidate


def read_state() -> dict[str, Any]:
    if not STATE_FILE.exists():
        return {}
    try:
        return json.loads(STATE_FILE.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {}


def write_state(state: dict[str, Any]) -> None:
    ensure_dirs()
    STATE_FILE.write_text(json.dumps(state, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def read_descriptions() -> dict[str, dict[str, str]]:
    if not DESCRIPTION_FILE.exists():
        return {}
    try:
        data = json.loads(DESCRIPTION_FILE.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {}
    if not isinstance(data, dict):
        return {}
    result: dict[str, dict[str, str]] = {}
    for path, rows in data.items():
        if isinstance(rows, dict):
            result[str(path)] = {str(key): str(value) for key, value in rows.items() if str(value).strip()}
    return result


def write_descriptions(descriptions: dict[str, dict[str, str]]) -> None:
    ensure_dirs()
    DESCRIPTION_FILE.write_text(json.dumps(descriptions, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def catalog_files(control_method: str) -> list[dict[str, str]]:
    files: list[dict[str, str]] = []
    seen: set[str] = set()
    method_items = CATALOG.get(control_method, [])
    for item in [*method_items, *CATALOG["common"]]:
        path = str(rel_path(item["path"]))
        if path not in seen:
            entry = dict(item)
            entry["path"] = path
            entry["exists"] = str(abs_path(path).exists()).lower()
            files.append(entry)
            seen.add(path)
    if control_method in REFERENCE_CONFIG_PATHS:
        try:
            active_reference = current_mpc_reference_path(reference_config_path(control_method))
        except Exception:
            active_reference = None
        if active_reference and active_reference not in seen:
            files.append(
                {
                    "label": f"{control_method} active reference path",
                    "path": active_reference,
                    "kind": "csv",
                    "exists": str(abs_path(active_reference).exists()).lower(),
                }
            )
    return files


def allowed_paths() -> set[str]:
    paths: set[str] = set()
    for items in CATALOG.values():
        for item in items:
            paths.add(str(rel_path(item["path"])))
    for config_path in REFERENCE_CONFIG_PATHS.values():
        try:
            paths.add(current_mpc_reference_path(config_path))
        except Exception:
            pass
    return paths


def parse_control_methods() -> list[str]:
    reference = abs_path(LAUNCH_ROOT / "launch/reference.launch.xml")
    text = reference.read_text(encoding="utf-8")
    methods = re.findall(r"control_method\)'\s*==\s*'([^']+)'", text)
    if not methods:
        methods = re.findall(r"'([^']+)'\s*==\s*'\$\(var control_method\)'", text)
    return list(dict.fromkeys(methods))


def parse_control_default() -> str:
    for path in CONTROL_DEFAULT_XMLS:
        text = abs_path(path).read_text(encoding="utf-8")
        match = re.search(r'<arg\s+name="control_method"\s+default="([^"]+)"', text)
        if match:
            return match.group(1)
    return "mpc"


def validate_content(path: str, content: str) -> None:
    suffix = rel_path(path).suffix.lower()
    if suffix in {".yaml", ".yml"}:
        yaml.safe_load(content)
    elif suffix == ".xml":
        ET.fromstring(content)
    elif suffix == ".json":
        json.loads(content)
    elif suffix == ".csv":
        rows = list(csv.reader(content.splitlines()))
        if content.strip() and not rows:
            raise ValueError("CSV parse produced no rows")


def structured_rows(path: str, content: str) -> dict[str, Any]:
    relative = str(rel_path(path))
    suffix = Path(relative).suffix.lower()
    if suffix in {".yaml", ".yml"}:
        rows = _yaml_rows(content)
        enrich_row_descriptions(relative, "yaml", rows)
        return {"kind": "yaml", "rows": rows}
    if suffix == ".xml":
        rows = _xml_rows(content)
        enrich_row_descriptions(relative, "xml", rows)
        return {"kind": "xml", "rows": rows}
    return {"kind": "text", "rows": []}


def apply_structured_rows(path: str, content: str, rows: list[dict[str, Any]]) -> dict[str, Any]:
    relative = str(rel_path(path))
    suffix = Path(relative).suffix.lower()
    if suffix in {".yaml", ".yml"}:
        updated = _apply_yaml_rows(content, rows)
    elif suffix == ".xml":
        updated = _apply_xml_rows(content, rows)
    else:
        raise ValueError("structured edit is only available for YAML and XML")
    validate_content(path, updated)
    descriptions_changed = save_row_descriptions(relative, rows)
    return {"content": updated, "descriptions_changed": descriptions_changed}


def enrich_row_descriptions(path: str, kind: str, rows: list[dict[str, Any]]) -> None:
    stored = read_descriptions().get(path, {})
    for row in rows:
        key = description_key(kind, row)
        row["description_key"] = key
        row["description"] = stored.get(key, default_description(kind, row))


def save_row_descriptions(path: str, rows: list[dict[str, Any]]) -> bool:
    descriptions = read_descriptions()
    current = dict(descriptions.get(path, {}))
    updated = dict(current)
    for row in rows:
        if not isinstance(row, dict):
            continue
        key = str(row.get("description_key") or description_key_from_row(row))
        if not key:
            continue
        description = str(row.get("description", "")).strip()
        if description:
            updated[key] = description
        else:
            updated.pop(key, None)
    if updated == current:
        return False
    if updated:
        descriptions[path] = updated
    else:
        descriptions.pop(path, None)
    write_descriptions(descriptions)
    return True


def description_key_from_row(row: dict[str, Any]) -> str:
    row_id = str(row.get("id", ""))
    if row_id.startswith("yaml:"):
        return description_key("yaml", row)
    if row_id.startswith("xml:"):
        return description_key("xml", row)
    return row_id


def description_key(kind: str, row: dict[str, Any]) -> str:
    if kind == "yaml":
        return f"yaml:{row.get('path') or row.get('id') or ''}"
    if kind == "xml":
        attrs = row.get("attrs") if isinstance(row.get("attrs"), dict) else {}
        tag = str(row.get("tag") or "element")
        for attr in ("name", "pkg", "exec", "file", "to", "from"):
            value = str(attrs.get(attr, "")).strip()
            if value:
                return f"xml:{tag}:{attr}={value}"
        return f"xml:{row.get('path') or row.get('id') or ''}"
    return str(row.get("id", ""))


def default_description(kind: str, row: dict[str, Any]) -> str:
    if kind == "yaml":
        path = str(row.get("path") or "")
        name = str(row.get("name") or path.rsplit(".", maxsplit=1)[-1] or "parameter")
        value_type = str(row.get("type") or "value")
        if path in DESCRIPTION_DEFAULTS:
            return DESCRIPTION_DEFAULTS[path]
        if re.match(r"ref_vel_configulator\.[^.]+\.ref_vel$", path):
            return "この区間の目標速度[km/h]です。コーナー前で下げると手前から減速しやすくなります。"
        if re.match(r"ref_vel_configulator\.[^.]+\.wp_id$", path):
            return "速度区間の開始waypoint IDです。この点以降、次の区間まで対応するref_velが使われます。"
        if path.endswith(".topic") or name.endswith("_topic"):
            return "ROS topic名です。接続するpublish/subscribe先を変えたいときに編集します。"
        if path.endswith(".frame_id") or name.endswith("_frame"):
            return "TF frame名です。センサ、車体、地図座標系の接続先を指定します。"
        if name in {"use_sim_time", "simulation"}:
            return "シミュレーション時刻を使うかどうかのフラグです。AWSIM評価では通常trueです。"
        if value_type == "bool":
            return f"{name} を有効/無効にするフラグです。"
        if value_type in {"int", "float"}:
            return f"{name} の数値パラメータです。単位や効果が未整理なら、このdescription欄にメモして保存できます。"
        return f"{name} の設定値です。意味や調整メモはこのdescription欄でGUIに保持できます。"
    if kind == "xml":
        attrs = row.get("attrs") if isinstance(row.get("attrs"), dict) else {}
        tag = str(row.get("tag") or "")
        name = str(attrs.get("name") or "")
        if name in DELAY_AWARE_XML_DEFAULTS:
            return DELAY_AWARE_XML_DEFAULTS[name]
        if tag == "arg":
            if name == "control_method":
                return "使用する制御方式です。mpc、pure_pursuit、tiny_lidar_netなどを切り替えます。"
            return "launch引数です。defaultを変更すると、このlaunch内で使われる既定値が変わります。"
        if tag == "include":
            return "別のlaunchファイルを読み込む設定です。fileやargで起動内容を切り替えます。"
        if tag == "node":
            return "ROSノードの起動設定です。pkg、exec、name、argsなどで実行対象を指定します。"
        if tag == "remap":
            return "topic名の置換設定です。fromからtoへ接続先を差し替えます。"
        if tag == "param":
            return "ROSパラメータ設定です。nameとvalueでノードへ渡す値を指定します。"
        return f"{tag or 'XML要素'} の設定です。調整意図や注意点はこのdescription欄でGUIに保持できます。"
    return ""


def _yaml_rows(content: str) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    stack: list[tuple[int, str]] = []
    list_counts: dict[str, int] = {}
    for line_no, line in enumerate(content.splitlines(), start=1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        indent = len(line) - len(line.lstrip(" "))
        while stack and indent <= stack[-1][0]:
            stack.pop()
        parent_path = ".".join(item[1] for item in stack)

        list_match = re.match(r"^(\s*)-\s+(.*?)(\s+#.*)?$", line)
        if list_match and parent_path:
            value = list_match.group(2).strip()
            if not value or re.match(r"^[^:]+:\s*", value):
                continue
            index = list_counts.get(parent_path, 0)
            list_counts[parent_path] = index + 1
            rows.append(
                {
                    "id": f"yaml:{line_no}",
                    "line": line_no,
                    "path": f"{parent_path}[{index}]",
                    "name": f"[{index}]",
                    "value": value,
                    "type": _yaml_value_type(value),
                    "editable": True,
                }
            )
            continue

        key_match = re.match(r"^(\s*)([^:#][^:]*?):(\s*)(.*?)(\s+#.*)?$", line)
        if not key_match:
            continue
        key = key_match.group(2).strip().strip("'\"")
        value = key_match.group(4).strip()
        path_items = [*(item[1] for item in stack), key]
        path = ".".join(path_items)
        if not value:
            stack.append((indent, key))
            list_counts.setdefault(path, 0)
            continue
        rows.append(
            {
                "id": f"yaml:{line_no}",
                "line": line_no,
                "path": path,
                "name": key,
                "value": value,
                "type": _yaml_value_type(value),
                "editable": True,
            }
        )
    return rows


def _yaml_value_type(value: str) -> str:
    try:
        parsed = yaml.safe_load(value)
    except yaml.YAMLError:
        return "raw"
    if parsed is None:
        return "null"
    if isinstance(parsed, bool):
        return "bool"
    if isinstance(parsed, int):
        return "int"
    if isinstance(parsed, float):
        return "float"
    if isinstance(parsed, list):
        return "list"
    if isinstance(parsed, dict):
        return "object"
    return "str"


def _apply_yaml_rows(content: str, rows: list[dict[str, Any]]) -> str:
    values = {str(row.get("id")): str(row.get("value", "")) for row in rows}
    lines = content.splitlines(keepends=True)
    for index, line in enumerate(lines, start=1):
        row_id = f"yaml:{index}"
        if row_id not in values:
            continue
        new_value = values[row_id]
        newline = ""
        body = line
        if body.endswith("\r\n"):
            body, newline = body[:-2], "\r\n"
        elif body.endswith("\n"):
            body, newline = body[:-1], "\n"
        if re.match(r"^\s*-\s+", body):
            updated = re.sub(r"^(\s*-\s+)(.*?)(\s+#.*)?$", lambda m: f"{m.group(1)}{new_value}{m.group(3) or ''}", body)
        else:
            updated = re.sub(r"^(\s*[^:#][^:]*?:\s*)(.*?)(\s+#.*)?$", lambda m: f"{m.group(1)}{new_value}{m.group(3) or ''}", body)
        lines[index - 1] = updated + newline
    return "".join(lines)


def _xml_rows(content: str) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    tag_index = 0
    for tag_match in re.finditer(r"<(?!!|/|\?)([A-Za-z_][\w:.-]*)([^<>]*?)(/?)>", content, flags=re.S):
        tag_index += 1
        tag_name = tag_match.group(1)
        attrs_text = tag_match.group(2)
        attrs = list(re.finditer(r"([A-Za-z_:][\w:.-]*)\s*=\s*\"([^\"]*)\"", attrs_text))
        attr_map = {match.group(1): match.group(2) for match in attrs}
        attr_order = [match.group(1) for match in attrs]
        label = attr_map.get("name") or attr_map.get("pkg") or attr_map.get("file") or tag_name
        line_no = content.count("\n", 0, tag_match.start()) + 1
        if not attr_map:
            continue
        rows.append(
            {
                "id": f"xml:{tag_index}",
                "line": line_no,
                "path": f"{tag_name}[{tag_index}]",
                "tag": tag_name,
                "label": label,
                "attrs": attr_map,
                "attr_order": attr_order,
                "editable": True,
            }
        )
    return rows


def _apply_xml_rows(content: str, rows: list[dict[str, Any]]) -> str:
    row_attrs: dict[str, dict[str, str]] = {}
    attr_row_values: dict[str, str] = {}
    for row in rows:
        row_id = str(row.get("id"))
        attrs = row.get("attrs")
        if isinstance(attrs, dict):
            row_attrs[row_id] = {str(key): str(value) for key, value in attrs.items()}
        elif row_id:
            attr_row_values[row_id] = str(row.get("value", ""))
    replacements: list[tuple[int, int, str]] = []
    tag_index = 0
    for tag_match in re.finditer(r"<(?!!|/|\?)([A-Za-z_][\w:.-]*)([^<>]*?)(/?)>", content, flags=re.S):
        tag_index += 1
        attrs_text = tag_match.group(2)
        attrs_offset = tag_match.start(2)
        element_values = row_attrs.get(f"xml:{tag_index}", {})
        for attr in re.finditer(r"([A-Za-z_:][\w:.-]*)\s*=\s*\"([^\"]*)\"", attrs_text):
            attr_name = attr.group(1)
            legacy_row_id = f"xml:{tag_index}:{attr_name}"
            if attr_name in element_values:
                value = element_values[attr_name]
            elif legacy_row_id in attr_row_values:
                value = attr_row_values[legacy_row_id]
            else:
                continue
            value_start = attrs_offset + attr.start(2)
            value_end = attrs_offset + attr.end(2)
            replacements.append((value_start, value_end, _xml_escape_attr(value)))
    updated = content
    for start, end, value in sorted(replacements, reverse=True):
        updated = updated[:start] + value + updated[end:]
    return updated


def _xml_escape_attr(value: str) -> str:
    return (
        value.replace("&", "&amp;")
        .replace('"', "&quot;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )


def backup_file(path: str) -> Path | None:
    source = abs_path(path)
    if not source.exists():
        return None
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    target = BACKUP_DIR / timestamp / rel_path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
    return target


def write_file(path: str, content: str, make_backup: bool = True) -> dict[str, Any]:
    relative = str(rel_path(path))
    if relative not in allowed_paths():
        raise PermissionError(f"not editable by tuning GUI: {relative}")
    validate_content(relative, content)
    target = abs_path(relative)
    old = target.read_text(encoding="utf-8") if target.exists() else ""
    if old == content:
        return {"changed": False, "backup": None}
    backup = backup_file(relative) if make_backup else None
    target.write_text(content, encoding="utf-8")
    state = read_state()
    state["dirty_since"] = now_iso()
    state["last_edited_path"] = relative
    write_state(state)
    return {"changed": True, "backup": str(backup.relative_to(REPO_ROOT)) if backup else None}


def set_control_method_default(method: str) -> dict[str, Any]:
    methods = parse_control_methods()
    if method not in methods:
        raise ValueError(f"unknown control_method: {method}")
    changed: list[str] = []
    for path in CONTROL_DEFAULT_XMLS:
        relative = str(rel_path(path))
        absolute = abs_path(relative)
        text = absolute.read_text(encoding="utf-8")
        new_text, count = re.subn(
            r'(<arg\s+name="control_method"\s+default=")([^"]*)(")',
            rf"\g<1>{method}\3",
            text,
            count=1,
        )
        if count == 0:
            continue
        validate_content(relative, new_text)
        if new_text != text:
            backup_file(relative)
            absolute.write_text(new_text, encoding="utf-8")
            changed.append(relative)
    state = read_state()
    state["selected_control_method"] = method
    if changed:
        state["dirty_since"] = now_iso()
        state["last_edited_path"] = ",".join(changed)
    write_state(state)
    return {"method": method, "changed": changed}


def file_diff(path: str, content: str | None = None) -> str:
    relative = str(rel_path(path))
    old = abs_path(relative).read_text(encoding="utf-8") if abs_path(relative).exists() else ""
    new = old if content is None else content
    return "".join(
        difflib.unified_diff(
            old.splitlines(keepends=True),
            new.splitlines(keepends=True),
            fromfile=f"a/{relative}",
            tofile=f"b/{relative}",
        )
    )


def reference_config_path(control_method: str | Path | None = None) -> Path:
    if isinstance(control_method, Path):
        relative = rel_path(control_method)
        if relative not in {rel_path(path) for path in REFERENCE_CONFIG_PATHS.values()}:
            raise ValueError(f"unsupported reference config: {relative}")
        return relative
    if isinstance(control_method, str) and control_method.endswith((".yaml", ".yml")):
        relative = rel_path(control_method)
        if relative not in {rel_path(path) for path in REFERENCE_CONFIG_PATHS.values()}:
            raise ValueError(f"unsupported reference config: {relative}")
        return relative
    method = control_method if isinstance(control_method, str) and control_method else selected_method()
    return REFERENCE_CONFIG_PATHS.get(method, MPC_CONFIG_PATH)


def current_mpc_config(config_path: str | Path | None = None) -> dict[str, Any]:
    config_relative = reference_config_path(config_path)
    config_path = abs_path(config_relative)
    if not config_path.exists():
        raise FileNotFoundError(f"MPC config not found: {config_relative}")
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError("MPC config is not a YAML mapping")
    return data


def current_mpc_reference_path(config_path: str | Path | None = None) -> str:
    cfg = current_mpc_config(config_path)
    reference_path = cfg.get("reference_path") or {}
    csv_path = reference_path.get("csv_path")
    if not isinstance(csv_path, str) or not csv_path:
        raise ValueError("MPC reference_path.csv_path is empty")
    return str(rel_path(MPC_ROOT / csv_path))


def current_mpc_map_yaml_path(config_path: str | Path | None = None) -> str:
    cfg = current_mpc_config(config_path)
    map_cfg = cfg.get("map") or {}
    yaml_path = map_cfg.get("yaml_path")
    if not isinstance(yaml_path, str) or not yaml_path:
        raise ValueError("MPC map.yaml_path is empty")
    return str(rel_path(MPC_ROOT / yaml_path))


def available_reference_paths(config_path: str | Path | None = None) -> list[dict[str, Any]]:
    roots = [abs_path(MPC_ROOT / "env"), abs_path(MPC_ROOT / "maps")]
    paths: list[Path] = []
    for root in roots:
        if root.exists():
            paths.extend(root.glob("**/*.csv"))
    active = current_mpc_reference_path(config_path)
    result = []
    for path in sorted(set(paths), key=lambda p: str(p.relative_to(REPO_ROOT))):
        relative = str(path.relative_to(REPO_ROOT))
        result.append(
            {
                "path": relative,
                "name": str(path.relative_to(abs_path(MPC_ROOT))),
                "active": relative == active,
            }
        )
    return result


def read_map_yaml(path: str | None = None, config_path: str | Path | None = None) -> dict[str, Any]:
    relative = str(rel_path(path or current_mpc_map_yaml_path(config_path)))
    map_yaml = abs_path(relative)
    data = yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}
    image_name = str(data.get("image", ""))
    image_path = (map_yaml.parent / image_name).resolve()
    image_path.relative_to(REPO_ROOT.resolve())
    width, height, _max_value, _offset = read_pgm_header(image_path)
    origin = data.get("origin") or [0.0, 0.0, 0.0]
    if len(origin) < 3:
        origin = [*origin, *([0.0] * (3 - len(origin)))]
    return {
        "yaml_path": relative,
        "image_path": str(image_path.relative_to(REPO_ROOT)),
        "image_url": f"/api/path-editor/map.png?path={image_path.relative_to(REPO_ROOT)}",
        "resolution": float(data.get("resolution", 1.0)),
        "origin": [float(origin[0]), float(origin[1]), float(origin[2])],
        "negate": int(data.get("negate", 0)),
        "occupied_thresh": float(data.get("occupied_thresh", 0.65)),
        "free_thresh": float(data.get("free_thresh", 0.196)),
        "width": width,
        "height": height,
    }


def read_pgm_header(path: Path) -> tuple[int, int, int, int]:
    with path.open("rb") as f:
        magic = _read_pgm_token(f)
        if magic != b"P5":
            raise ValueError(f"unsupported map image format: {path}")
        width = int(_read_pgm_token(f))
        height = int(_read_pgm_token(f))
        max_value = int(_read_pgm_token(f))
        if max_value > 255:
            raise ValueError("16-bit PGM maps are not supported")
        return width, height, max_value, f.tell()


def _read_pgm_token(file_obj: Any) -> bytes:
    token = bytearray()
    while True:
        char = file_obj.read(1)
        if not char:
            return bytes(token)
        if char == b"#":
            file_obj.readline()
            if token:
                return bytes(token)
            continue
        if char.isspace():
            if token:
                return bytes(token)
            continue
        token.extend(char)


def pgm_to_png(path: Path) -> bytes:
    width, height, max_value, offset = read_pgm_header(path)
    data = path.read_bytes()[offset:]
    expected = width * height
    if len(data) < expected:
        raise ValueError(f"PGM data is shorter than expected: {path}")
    pixels = data[:expected]
    if max_value != 255:
        pixels = bytes(round(value * 255 / max_value) for value in pixels)
    raw_rows = b"".join(b"\x00" + pixels[row * width : (row + 1) * width] for row in range(height))
    return make_png(width, height, raw_rows, color_type=0)


def make_png(width: int, height: int, raw_rows: bytes, color_type: int) -> bytes:
    def chunk(kind: bytes, payload: bytes) -> bytes:
        checksum = zlib.crc32(kind + payload) & 0xFFFFFFFF
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", checksum)

    header = struct.pack(">IIBBBBB", width, height, 8, color_type, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(raw_rows, 9)) + chunk(b"IEND", b"")


def load_reference_path(path: str | None = None, config_path: str | Path | None = None) -> dict[str, Any]:
    relative = str(rel_path(path or current_mpc_reference_path(config_path)))
    target = abs_path(relative)
    rows = list(csv.DictReader(target.read_text(encoding="utf-8").splitlines()))
    points = []
    for index, row in enumerate(rows):
        try:
            x = float(row.get("x_m", ""))
            y = float(row.get("y_m", ""))
        except ValueError:
            continue
        points.append(
            {
                "index": index,
                "s_m": _float_or_none(row.get("s_m")),
                "x_m": x,
                "y_m": y,
                "psi_rad": _float_or_none(row.get("psi_rad")),
                "kappa_radpm": _float_or_none(row.get("kappa_radpm")),
                "vx_mps": _float_or_none(row.get("vx_mps")),
                "ax_mps2": _float_or_none(row.get("ax_mps2")),
            }
        )
    if len(points) < 2:
        raise ValueError(f"reference path has too few points: {relative}")
    return {
        "path": relative,
        "package_path": str(Path(relative).relative_to(MPC_ROOT)),
        "points": points,
        "stats": path_stats(points),
    }


def _float_or_none(value: Any) -> float | None:
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def path_stats(points: list[dict[str, Any]]) -> dict[str, Any]:
    length = 0.0
    min_segment = None
    for a, b in zip(points, points[1:]):
        segment = math.hypot(float(b["x_m"]) - float(a["x_m"]), float(b["y_m"]) - float(a["y_m"]))
        length += segment
        min_segment = segment if min_segment is None else min(min_segment, segment)
    return {"count": len(points), "length_m": length, "min_segment_m": min_segment}


def path_editor_state(path: str | None = None, config_path: str | Path | None = None) -> dict[str, Any]:
    active_config_path = reference_config_path(config_path)
    source = load_reference_path(path, active_config_path)
    return {
        "config_path": str(active_config_path),
        "circular": mpc_reference_is_circular(active_config_path),
        "map": read_map_yaml(config_path=active_config_path),
        "source": source,
        "available_paths": available_reference_paths(active_config_path),
        "default_target_path": default_reference_target(source["path"]),
    }


def default_reference_target(source_path: str) -> str:
    source = abs_path(source_path)
    parent = source.parent
    stem = source.stem
    if is_manual_reference_path(source):
        return str(source.relative_to(REPO_ROOT))
    return str((parent / f"{stem}_manual.csv").relative_to(REPO_ROOT))


def is_manual_reference_path(path: Path) -> bool:
    return bool(re.search(r"(?:^|_)manual(?:_v\d+)?$", path.stem))


def save_reference_path(body: dict[str, Any]) -> dict[str, Any]:
    config_path = reference_config_path(str(body.get("config_path") or "") or None)
    source_path = str(body.get("source_path") or current_mpc_reference_path(config_path))
    target_path = str(body.get("target_path") or default_reference_target(source_path))
    target_relative = validate_reference_target(target_path)
    points = normalize_path_points(body.get("points") or [])
    csv_text, computed_points, warnings = build_reference_path_csv(
        points,
        circular=mpc_reference_is_circular(config_path),
    )
    target = abs_path(target_relative)
    old = target.read_text(encoding="utf-8") if target.exists() else ""
    changed = old != csv_text
    backup = backup_file(target_relative) if changed and target.exists() else None
    if changed:
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(csv_text, encoding="utf-8")
    config_changed = False
    if body.get("switch_config"):
        config_changed = update_mpc_reference_path(target_relative, config_path)
    if changed or config_changed:
        state = read_state()
        state["dirty_since"] = now_iso()
        state["last_edited_path"] = target_relative
        write_state(state)
    return {
        "changed": changed,
        "config_changed": config_changed,
        "config_path": str(config_path),
        "path": target_relative,
        "package_path": str(Path(target_relative).relative_to(MPC_ROOT)),
        "backup": str(backup.relative_to(REPO_ROOT)) if backup else None,
        "points": computed_points,
        "stats": path_stats(computed_points),
        "warnings": warnings,
    }


def validate_reference_target(path: str) -> str:
    candidate = Path(str(path))
    if not candidate.is_absolute() and candidate.parts and candidate.parts[0] in {"env", "maps"}:
        candidate = MPC_ROOT / candidate
    relative = rel_path(candidate)
    if relative.suffix.lower() != ".csv":
        raise ValueError("reference path target must be a CSV file")
    relative.relative_to(MPC_ROOT)
    allowed_roots = (MPC_ROOT / "env", MPC_ROOT / "maps")
    if not any(relative == root or root in relative.parents for root in allowed_roots):
        raise PermissionError("reference path must be saved under multi_purpose_mpc_ros/env or maps")
    return str(relative)


def normalize_path_points(raw_points: list[Any]) -> list[dict[str, float]]:
    points = []
    for raw in raw_points:
        if not isinstance(raw, dict):
            continue
        try:
            x = float(raw.get("x_m"))
            y = float(raw.get("y_m"))
        except (TypeError, ValueError):
            continue
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        points.append(
            {
                "x_m": x,
                "y_m": y,
                "vx_mps": _finite_or_default(raw.get("vx_mps"), 0.0),
                "ax_mps2": _finite_or_default(raw.get("ax_mps2"), 0.0),
            }
        )
    if len(points) < 2:
        raise ValueError("reference path needs at least two points")
    return points


def _finite_or_default(value: Any, default: float) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return default
    return numeric if math.isfinite(numeric) else default


def build_reference_path_csv(points: list[dict[str, float]], circular: bool) -> tuple[str, list[dict[str, Any]], list[str]]:
    warnings = path_warnings(points)
    distances = [0.0]
    for a, b in zip(points, points[1:]):
        distances.append(distances[-1] + math.hypot(b["x_m"] - a["x_m"], b["y_m"] - a["y_m"]))
    headings = []
    count = len(points)
    for index, point in enumerate(points):
        if index < count - 1:
            target = points[index + 1]
        elif circular:
            target = points[0]
        else:
            target = points[index - 1]
        headings.append(math.atan2(target["y_m"] - point["y_m"], target["x_m"] - point["x_m"]))
    computed: list[dict[str, Any]] = []
    for index, point in enumerate(points):
        previous_index = (index - 1) % count if circular else max(0, index - 1)
        previous = points[previous_index]
        previous_distance = max(
            math.hypot(point["x_m"] - previous["x_m"], point["y_m"] - previous["y_m"]),
            1e-6,
        )
        heading_delta = normalize_angle(headings[index] - headings[previous_index])
        kappa = heading_delta / previous_distance
        computed.append(
            {
                "index": index,
                "s_m": distances[index],
                "x_m": point["x_m"],
                "y_m": point["y_m"],
                "psi_rad": headings[index],
                "kappa_radpm": kappa,
                "vx_mps": point["vx_mps"],
                "ax_mps2": point["ax_mps2"],
            }
        )
    lines = ["s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2\n"]
    for point in computed:
        lines.append(
            ",".join(
                [
                    _fmt_float(point["s_m"]),
                    _fmt_float(point["x_m"]),
                    _fmt_float(point["y_m"]),
                    _fmt_float(point["psi_rad"]),
                    _fmt_float(point["kappa_radpm"]),
                    _fmt_float(point["vx_mps"]),
                    _fmt_float(point["ax_mps2"]),
                ]
            )
            + "\n"
        )
    return "".join(lines), computed, warnings


def path_warnings(points: list[dict[str, float]]) -> list[str]:
    warnings = []
    min_segment = None
    for a, b in zip(points, points[1:]):
        segment = math.hypot(b["x_m"] - a["x_m"], b["y_m"] - a["y_m"])
        min_segment = segment if min_segment is None else min(min_segment, segment)
    if min_segment is not None and min_segment < 0.2:
        warnings.append(f"minimum segment is short: {min_segment:.3f} m")
    return warnings


def _fmt_float(value: float) -> str:
    return f"{float(value):.7f}"


def normalize_angle(value: float) -> float:
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def mpc_reference_is_circular(config_path: str | Path | None = None) -> bool:
    cfg = current_mpc_config(config_path)
    reference_path = cfg.get("reference_path") or {}
    return bool(reference_path.get("circular", True))


def update_mpc_reference_path(target_relative: str, config_path: str | Path | None = None) -> bool:
    target_package_path = str(Path(target_relative).relative_to(MPC_ROOT))
    config_relative = str(reference_config_path(config_path))
    config_path = abs_path(config_relative)
    text = config_path.read_text(encoding="utf-8")
    lines = text.splitlines(keepends=True)
    in_reference = False
    changed = False
    for index, line in enumerate(lines):
        stripped = line.strip()
        indent = len(line) - len(line.lstrip(" "))
        if re.match(r"^reference_path:\s*(#.*)?$", stripped):
            in_reference = True
            continue
        if in_reference and indent == 0 and stripped and not stripped.startswith("#"):
            break
        if in_reference and re.match(r"^\s*csv_path\s*:", line) and not stripped.startswith("#"):
            newline = "\n" if line.endswith("\n") else ""
            body = line[:-1] if newline else line
            updated = re.sub(
                r'^(\s*csv_path\s*:\s*)(".*?"|\'.*?\'|[^#\n]*)(\s*#.*)?$',
                lambda match: f'{match.group(1)}"{target_package_path}"{match.group(3) or ""}',
                body,
            )
            lines[index] = updated + newline
            changed = lines[index] != line
            break
    if not changed:
        return False
    updated_text = "".join(lines)
    validate_content(config_relative, updated_text)
    backup_file(config_relative)
    config_path.write_text(updated_text, encoding="utf-8")
    return True


def selected_method() -> str:
    state = read_state()
    selected = state.get("selected_control_method")
    methods = parse_control_methods()
    if isinstance(selected, str) and selected in methods:
        return selected
    default = parse_control_default()
    return default if default in methods else (methods[0] if methods else "mpc")


def set_selected_method(method: str) -> dict[str, Any]:
    methods = parse_control_methods()
    if method not in methods:
        raise ValueError(f"unknown control_method: {method}")
    state = read_state()
    changed = state.get("selected_control_method") != method
    state["selected_control_method"] = method
    write_state(state)
    return {"method": method, "changed": changed, "files": catalog_files(method)}


def snapshot_files(command_id: str, method: str) -> Path:
    snapshot_dir = HISTORY_DIR / "snapshots" / command_id
    files = catalog_files(method)
    manifest: list[dict[str, str]] = []
    for item in files:
        source = abs_path(item["path"])
        if not source.exists():
            continue
        target = snapshot_dir / item["path"]
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        data = source.read_bytes()
        manifest.append(
            {
                "path": item["path"],
                "sha256": hashlib.sha256(data).hexdigest(),
                "label": item["label"],
                "kind": item["kind"],
            }
        )
    snapshot_dir.mkdir(parents=True, exist_ok=True)
    (snapshot_dir / "manifest.json").write_text(
        json.dumps({"control_method": method, "files": manifest}, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return snapshot_dir


def append_history(entry: dict[str, Any]) -> None:
    ensure_dirs()
    with HISTORY_FILE.open("a", encoding="utf-8") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")


def load_history() -> list[dict[str, Any]]:
    if not HISTORY_FILE.exists():
        return []
    rows: list[dict[str, Any]] = []
    for line in HISTORY_FILE.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows[-200:]


def command_env(method: str) -> dict[str, str]:
    env = os.environ.copy()
    env["CONTROL_METHOD"] = method
    return env


def normalize_npc_count(value: Any) -> int:
    try:
        count = int(value)
    except (TypeError, ValueError):
        count = 0
    if not 0 <= count <= 3:
        raise ValueError("npc_count must be between 0 and 3")
    return count


def _clean_option_text(value: Any, field: str, limit: int = 240) -> str:
    text = str(value or "").strip()
    if "\n" in text or "\r" in text or "\0" in text:
        raise ValueError(f"{field} must be a single line")
    return text[:limit]


def _choice(value: Any, allowed: set[str], field: str) -> str:
    text = _clean_option_text(value, field, 80)
    if text and text not in allowed:
        raise ValueError(f"unknown {field}: {text}")
    return text


def _int_option(value: Any, field: str, minimum: int, maximum: int) -> str:
    text = _clean_option_text(value, field, 40)
    if not text:
        return ""
    try:
        number = int(text)
    except ValueError as exc:
        raise ValueError(f"{field} must be an integer") from exc
    if not minimum <= number <= maximum:
        raise ValueError(f"{field} must be between {minimum} and {maximum}")
    return str(number)


def _float_option(value: Any, field: str, minimum: float | None = None, maximum: float | None = None) -> str:
    text = _clean_option_text(value, field, 40)
    if not text:
        return ""
    try:
        number = float(text)
    except ValueError as exc:
        raise ValueError(f"{field} must be a number") from exc
    if minimum is not None and number < minimum:
        raise ValueError(f"{field} must be >= {minimum}")
    if maximum is not None and number > maximum:
        raise ValueError(f"{field} must be <= {maximum}")
    return text


def _laps_option(value: Any) -> str:
    text = _clean_option_text(value, "laps", 40)
    if not text:
        return ""
    lowered = text.lower()
    if lowered in {"unlimited", "inf", "0"}:
        return lowered
    try:
        number = int(text)
    except ValueError as exc:
        raise ValueError("laps must be an integer or unlimited") from exc
    if number < 0:
        raise ValueError("laps must be >= 0")
    return str(number)


def normalize_safety_gate(value: Any, required: bool = False) -> str | None:
    gate_id = _clean_option_text(value, "safety_gate", 40)
    if not gate_id:
        if required:
            raise ValueError("safety_gate is required")
        return None
    if gate_id not in SAFETY_GATES:
        raise ValueError(f"unknown safety gate: {gate_id}")
    return gate_id


def normalize_simulator_options(raw: Any) -> dict[str, Any]:
    data = raw if isinstance(raw, dict) else {}
    return {
        "start_mode": _choice(data.get("start_mode"), {"off", "sync", "count"}, "start_mode"),
        "start_count_seconds": _int_option(data.get("start_count_seconds"), "start_count_seconds", 0, 10),
        "laps": _laps_option(data.get("laps")),
        "timeout": _float_option(data.get("timeout"), "timeout", 0.0, None),
        "simulator_npcs": _int_option(data.get("simulator_npcs"), "simulator_npcs", 0, 3),
        "boosts": _int_option(data.get("boosts"), "boosts", 0, 5),
        "camera": _choice(data.get("camera"), {"true", "false"}, "camera"),
        "lidar": _choice(data.get("lidar"), {"true", "false"}, "lidar"),
        "sound": _choice(data.get("sound"), {"true", "false"}, "sound"),
        "collisions": _choice(data.get("collisions"), {"true", "false"}, "collisions"),
        "wall_recovery": _choice(data.get("wall_recovery"), {"true", "false"}, "wall_recovery"),
        "ranking": _choice(data.get("ranking"), {"true", "false"}, "ranking"),
        "steer_source": _choice(
            data.get("steer_source"),
            {"ackermann", "actuation", "actuation-longitudinal-only"},
            "steer_source",
        ),
        "manual_mode": _choice(data.get("manual_mode"), {"true", "false"}, "manual_mode"),
        "scenario": _clean_option_text(data.get("scenario"), "scenario"),
        "vehicle_poses": _clean_option_text(data.get("vehicle_poses"), "vehicle_poses"),
        "json_path": _clean_option_text(data.get("json_path"), "json_path"),
        "replay0": _clean_option_text(data.get("replay0"), "replay0"),
        "multiplay_mode": _choice(data.get("multiplay_mode"), {"server", "client", "host"}, "multiplay_mode"),
        "multiplay_address": _clean_option_text(data.get("multiplay_address"), "multiplay_address", 160),
        "multiplay_port": _int_option(data.get("multiplay_port"), "multiplay_port", 1, 65535),
        "multiplay_name": _clean_option_text(data.get("multiplay_name"), "multiplay_name", 80),
        "multiplay_send_hz": _float_option(data.get("multiplay_send_hz"), "multiplay_send_hz", 0.1, 1000.0),
        "raw_args": _clean_option_text(data.get("raw_args"), "raw_args", 1000),
    }


def awsim_option_keys_from_tokens(tokens: list[str]) -> list[str]:
    keys: list[str] = []
    for token in tokens:
        if token.startswith("--"):
            keys.append(token)
        elif token.startswith("-") and token != "-" and not token[1:].replace(".", "", 1).isdigit():
            keys.append(token)
    return keys


def awsim_option_keys_from_text(text: str) -> list[str]:
    return awsim_option_keys_from_tokens(str(text or "").split())


def validate_raw_awsim_args(raw_args: str, reserved_options: set[str]) -> None:
    raw_keys = awsim_option_keys_from_text(raw_args)
    duplicate_raw = {key for key in raw_keys if raw_keys.count(key) > 1}
    duplicate_reserved = set(raw_keys) & reserved_options
    duplicates = sorted(duplicate_raw | duplicate_reserved)
    if duplicates:
        joined = ", ".join(duplicates)
        raise ValueError(f"raw_args duplicates AWSIM options already managed by GUI/launch: {joined}")


def simulator_option_args(
    options: dict[str, Any],
    allow_scenario: bool = True,
    include_session_options: bool = True,
    skip_options: set[str] | None = None,
    blocked_raw_options: set[str] | None = None,
) -> str:
    args: list[str] = []
    skip_options = skip_options or set()
    blocked_raw_options = blocked_raw_options or set()

    def add(name: str, value: Any) -> None:
        text = str(value or "").strip()
        if text:
            args.extend([name, text])

    if include_session_options:
        add("--start-mode", options.get("start_mode"))
        add("--start-count-seconds", options.get("start_count_seconds"))
        add("--laps", options.get("laps"))
        add("--timeout", options.get("timeout"))
    add("--npcs", options.get("simulator_npcs"))
    add("--boosts", options.get("boosts"))
    add("--steer-source", options.get("steer_source"))

    for key, name in SIMULATOR_BOOL_OPTIONS.items():
        if key in skip_options:
            continue
        add(name, options.get(key))

    if allow_scenario:
        add("--scenario", options.get("scenario"))
    add("--vehicle-poses", options.get("vehicle_poses"))
    add("--json_path", options.get("json_path"))
    add("--replay0", options.get("replay0"))

    add("--multiplay", options.get("multiplay_mode"))
    add("--multiplay-address", options.get("multiplay_address"))
    add("--multiplay-port", options.get("multiplay_port"))
    add("--multiplay-name", options.get("multiplay_name"))
    add("--multiplay-send-hz", options.get("multiplay_send_hz"))

    parts: list[str] = []
    if args:
        parts.append(" ".join(args))
    if options.get("raw_args"):
        reserved_options = set(AWSIM_FIXED_OPTIONS)
        reserved_options.update(awsim_option_keys_from_tokens(args))
        reserved_options.update(blocked_raw_options)
        if not allow_scenario:
            reserved_options.add("--scenario")
        validate_raw_awsim_args(str(options["raw_args"]), reserved_options)
        parts.append(str(options["raw_args"]))
    return " ".join(parts).strip()


def awsim_extra_args(
    headless: bool,
    options: dict[str, Any],
    allow_scenario: bool = True,
    include_session_options: bool = True,
    blocked_raw_options: set[str] | None = None,
) -> str:
    parts: list[str] = []
    skip_options: set[str] = set()
    blocked_raw_options = set(blocked_raw_options or set())
    if headless:
        parts.append(AWSIM_HEADLESS_ARGS)
        skip_options.update({"camera", "lidar"})
        blocked_raw_options.update(awsim_option_keys_from_text(AWSIM_HEADLESS_ARGS))
    option_args = simulator_option_args(
        options,
        allow_scenario=allow_scenario,
        include_session_options=include_session_options,
        skip_options=skip_options,
        blocked_raw_options=blocked_raw_options,
    )
    if option_args:
        parts.append(option_args)
    return " ".join(parts).strip()


def env_assignments(pairs: dict[str, str]) -> str:
    return "".join(f"{key}={shell_quote(value)} " for key, value in pairs.items() if value is not None)


def evalwrap_label(method: str, note: str, headless: bool, npc_count: int) -> str:
    profile = "headless-eval" if headless else "gui-eval"
    if npc_count:
        profile = f"{profile}-{npc_count}npc"
    label = note.strip() or f"{method}-{profile}"
    return label[:80]


def command_for(
    action: str,
    method: str,
    build_first: bool,
    note: str = "",
    headless: bool = False,
    npc_count: int = 0,
    simulator_options: dict[str, Any] | None = None,
    safety_gate: str | None = None,
) -> str:
    npc_count = normalize_npc_count(npc_count)
    simulator_options = normalize_simulator_options(simulator_options)
    total_vehicles = npc_count + 1
    eval_prefix = eval_env_prefix(method, headless, total_vehicles, simulator_options)
    if action == "build":
        return "make autoware-build"
    if action == "dev":
        run = command_for_dev(method, total_vehicles, headless, simulator_options)
        return f"make autoware-build && {run}" if build_first else run
    if action == "eval":
        label = evalwrap_label(method, note, headless, npc_count)
        run = f"{eval_prefix}tools/evalwrap run --label {shell_quote(label)}"
        if note.strip():
            run += f" --note {shell_quote(note.strip())}"
        if not build_first:
            run += " --skip-build"
        return run
    if action == "quick-eval":
        run = f"{eval_prefix}make eval"
        return f"make autoware-build && {run}" if build_first else run
    if action == "gate":
        gate_id = normalize_safety_gate(safety_gate, required=True)
        gate_extra_args = awsim_extra_args(
            headless,
            simulator_options,
            allow_scenario=False,
            include_session_options=False,
            blocked_raw_options={"--safety-gate"},
        )
        pairs = {"CONTROL_METHOD": method}
        if simulator_options.get("timeout"):
            pairs["AWSIM_TIMEOUT"] = str(simulator_options["timeout"])
        if simulator_options.get("start_count_seconds"):
            pairs["AWSIM_START_COUNT_SECONDS"] = str(simulator_options["start_count_seconds"])
        pairs["AWSIM_EXTRA_ARGS"] = ""
        pairs["GATE_EXTRA_ARGS"] = gate_extra_args
        run = f"{env_assignments(pairs)}make {gate_id}"
        return f"make autoware-build && {run}" if build_first else run
    if action == "ingest":
        label = evalwrap_label(method, note, headless, npc_count).replace("-eval", "-log")
        run = f"tools/evalwrap ingest --label {shell_quote(label)} --path output/latest"
        if note.strip():
            run += f" --note {shell_quote(note.strip())}"
        return run
    if action == "down":
        return "make down"
    raise ValueError(f"unknown action: {action}")


def dev_target(total_vehicles: int) -> str:
    if not 1 <= total_vehicles <= 4:
        raise ValueError("total vehicles must be between 1 and 4")
    return "dev" if total_vehicles == 1 else f"dev{total_vehicles}"


AWSIM_HEADLESS_ARGS = "-batchmode -nographics --camera false --lidar false"


def eval_env_prefix(method: str, headless: bool, total_vehicles: int, simulator_options: dict[str, Any]) -> str:
    if not 1 <= total_vehicles <= 4:
        raise ValueError("total vehicles must be between 1 and 4")
    pairs = {
        "CONTROL_METHOD": method,
        "AWSIM_VEHICLES": str(total_vehicles),
    }
    if simulator_options.get("start_mode"):
        pairs["AWSIM_START_MODE"] = str(simulator_options["start_mode"])
    if simulator_options.get("start_count_seconds"):
        pairs["AWSIM_START_COUNT_SECONDS"] = str(simulator_options["start_count_seconds"])
    if simulator_options.get("laps"):
        pairs["AWSIM_LAPS"] = str(simulator_options["laps"])
    if simulator_options.get("timeout"):
        pairs["AWSIM_TIMEOUT"] = str(simulator_options["timeout"])
    extra_args = awsim_extra_args(headless, simulator_options, include_session_options=False)
    pairs["AWSIM_EXTRA_ARGS"] = extra_args
    return env_assignments(pairs)


def command_for_dev(method: str, total_vehicles: int, headless: bool, simulator_options: dict[str, Any]) -> str:
    if not 1 <= total_vehicles <= 4:
        raise ValueError("total vehicles must be between 1 and 4")
    pairs = {
        "ROSBAG": "true",
        "CONTROL_METHOD": method,
    }
    if simulator_options.get("start_mode"):
        pairs["AWSIM_START_MODE"] = str(simulator_options["start_mode"])
    if simulator_options.get("start_count_seconds"):
        pairs["AWSIM_START_COUNT_SECONDS"] = str(simulator_options["start_count_seconds"])
    if simulator_options.get("laps"):
        pairs["AWSIM_LAPS"] = str(simulator_options["laps"])
    if simulator_options.get("timeout"):
        pairs["AWSIM_TIMEOUT"] = str(simulator_options["timeout"])
    extra_args = awsim_extra_args(headless, simulator_options, include_session_options=False)
    pairs["AWSIM_EXTRA_ARGS"] = extra_args
    return f"{env_assignments(pairs)}make {dev_target(total_vehicles)}"


def shell_quote(value: str) -> str:
    return "'" + value.replace("'", "'\"'\"'") + "'"


def start_command(
    action: str,
    method: str,
    build_first: bool,
    note: str,
    headless: bool = False,
    npc_count: int = 0,
    simulator_options: dict[str, Any] | None = None,
    safety_gate: str | None = None,
) -> CommandState:
    global active_process, active_state
    if method not in parse_control_methods():
        raise ValueError(f"unknown control_method: {method}")
    npc_count = normalize_npc_count(npc_count)
    simulator_options = normalize_simulator_options(simulator_options)
    safety_gate = normalize_safety_gate(safety_gate, required=True) if action == "gate" else None
    with command_lock:
        if active_process is not None and active_process.poll() is None:
            raise RuntimeError("command is already running")
        ensure_dirs()
        command_id = datetime.now().strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8]
        log_path = COMMAND_DIR / f"{command_id}-{action}.log"
        snapshot_dir = snapshot_files(command_id, method) if action in {"dev", "eval", "quick-eval", "ingest", "build", "gate"} else None
        command = command_for(action, method, build_first, note, headless, npc_count, simulator_options, safety_gate)
        log = log_path.open("w", encoding="utf-8", buffering=1)
        log.write(f"$ {command}\n")
        process = subprocess.Popen(
            ["bash", "-lc", command],
            cwd=REPO_ROOT,
            env=command_env(method),
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        state = CommandState(
            id=command_id,
            action=action,
            command=command,
            control_method=method,
            note=note,
            headless=headless,
            npc_count=npc_count,
            simulator_options=simulator_options,
            safety_gate=safety_gate,
            status="running",
            started_at=now_iso(),
            finished_at=None,
            returncode=None,
            log_path=str(log_path.relative_to(REPO_ROOT)),
            pid=process.pid,
            snapshot_dir=str(snapshot_dir.relative_to(REPO_ROOT)) if snapshot_dir else None,
        )
        active_process = process
        active_state = state
        append_history({"type": "command_start", **asdict(state)})
        thread = threading.Thread(target=_watch_command, args=(process, log, state), daemon=True)
        thread.start()
        return state


def _watch_command(process: subprocess.Popen[str], log: Any, state: CommandState) -> None:
    global active_process, active_state
    returncode = process.wait()
    log.write(f"\n[exit] {returncode}\n")
    log.close()
    with command_lock:
        state.status = "finished"
        state.returncode = returncode
        state.finished_at = now_iso()
        state.pid = None
        append_history({"type": "command_finish", **asdict(state)})
        store = read_state()
        store["last_command"] = asdict(state)
        if state.action == "build" and returncode == 0:
            store["last_build_at"] = state.finished_at
            store.pop("dirty_since", None)
        elif state.action in {"dev", "eval", "quick-eval", "ingest", "gate"} and returncode == 0:
            store["last_run_at"] = state.finished_at
            if "make autoware-build" in state.command or (
                state.action == "eval" and "--skip-build" not in state.command
            ):
                store["last_build_at"] = state.finished_at
                store.pop("dirty_since", None)
        write_state(store)
        active_process = None
        active_state = state


def stop_active_command() -> None:
    global active_process
    with command_lock:
        if active_process is None or active_process.poll() is not None:
            return
        os.killpg(active_process.pid, signal.SIGINT)


def command_status() -> dict[str, Any]:
    with command_lock:
        state = active_state
        running = active_process is not None and active_process.poll() is None
    if state is None:
        last = read_state().get("last_command")
        return {"running": False, "command": last, "log_tail": ""}
    return {"running": running, "command": asdict(state), "log_tail": tail_text(REPO_ROOT / state.log_path, 240)}


def tail_text(path: Path, lines: int = 200) -> str:
    if not path.exists():
        return ""
    data = path.read_text(encoding="utf-8", errors="replace").splitlines()
    return "\n".join(data[-lines:])


def save_preset(name: str, method: str, note: str) -> dict[str, Any]:
    safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip())[:80].strip("._-")
    if not safe_name:
        raise ValueError("preset name is empty")
    if method not in parse_control_methods():
        raise ValueError(f"unknown control_method: {method}")
    files = []
    for item in catalog_files(method):
        source = abs_path(item["path"])
        if source.exists():
            files.append({**item, "content": source.read_text(encoding="utf-8")})
    payload = {
        "name": safe_name,
        "note": note,
        "control_method": method,
        "created_at": now_iso(),
        "files": files,
    }
    target = PRESET_DIR / f"{safe_name}.json"
    target.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return {"name": safe_name, "path": str(target.relative_to(REPO_ROOT)), "file_count": len(files)}


def list_presets() -> list[dict[str, Any]]:
    result = []
    for path in sorted(PRESET_DIR.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            continue
        result.append(
            {
                "name": data.get("name", path.stem),
                "note": data.get("note", ""),
                "control_method": data.get("control_method", ""),
                "created_at": data.get("created_at", ""),
                "file_count": len(data.get("files", [])),
            }
        )
    return result


def restore_preset(name: str) -> dict[str, Any]:
    path = PRESET_DIR / f"{re.sub(r'[^A-Za-z0-9_.-]+', '_', name)}.json"
    if not path.exists():
        raise FileNotFoundError(f"preset not found: {name}")
    data = json.loads(path.read_text(encoding="utf-8"))
    changed = []
    for item in data.get("files", []):
        relative = str(rel_path(item["path"]))
        content = str(item.get("content", ""))
        result = write_file(relative, content, make_backup=True)
        if result["changed"]:
            changed.append(relative)
    method = str(data.get("control_method") or selected_method())
    set_control_method_default(method)
    return {"name": name, "control_method": method, "changed": changed}


def discover_output_runs() -> list[dict[str, Any]]:
    rows = []
    for summary in sorted((REPO_ROOT / "output").glob("*/d*/result-summary.json"), key=lambda p: p.stat().st_mtime, reverse=True):
        try:
            data = json.loads(summary.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            continue
        vehicle = (data.get("vehicles") or [{}])[0]
        laps = vehicle.get("laps") or data.get("laps") or []
        rows.append(
            {
                "run_dir": str(summary.parents[1].relative_to(REPO_ROOT)),
                "domain": summary.parent.name,
                "finished": vehicle.get("finished"),
                "lap_count": vehicle.get("lap_count") or data.get("num_laps"),
                "best_lap_sec": vehicle.get("min_lap_time") or data.get("min_time"),
                "avg_lap_sec": vehicle.get("avg_lap_time"),
                "total_lap_time": vehicle.get("total_lap_time") or data.get("total_lap_time"),
                "laps": laps,
                "summary_path": str(summary.relative_to(REPO_ROOT)),
                "autoware_log": str((summary.parent / "autoware.log").relative_to(REPO_ROOT)),
                "mtime": summary.stat().st_mtime,
            }
        )
    return rows[:100]


def discover_reports() -> list[dict[str, Any]]:
    reports = []
    for report in sorted((REPO_ROOT / "analysis/runs").glob("*/report/index.html"), key=lambda p: p.stat().st_mtime, reverse=True):
        run_dir = report.parents[1]
        metrics_path = run_dir / "processed/metrics.json"
        motion_log_path = run_dir / "processed/motion_log.csv"
        vehicle_timeseries_path = run_dir / "processed/vehicle_timeseries.csv"
        control_timeseries_path = run_dir / "processed/control_timeseries.csv"
        metrics: dict[str, Any] = {}
        if metrics_path.exists():
            try:
                metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                metrics = {}
        domains = metrics.get("domains") or {}
        domain = next(iter(domains.values()), {})
        motion_log_available = motion_log_path.exists() or vehicle_timeseries_path.exists() or control_timeseries_path.exists()
        reports.append(
            {
                "run_id": run_dir.name,
                "report_path": str(report.relative_to(REPO_ROOT)),
                "best_lap_sec": domain.get("best_lap_sec"),
                "avg_lap_sec": domain.get("avg_lap_sec"),
                "lap_count": domain.get("lap_count"),
                "judgement": domain.get("judgement"),
                "low_speed_time_sec": domain.get("low_speed_time_sec"),
                "max_speed_mps": domain.get("max_speed_mps"),
                "motion_log_available": motion_log_available,
                "motion_log_path": str(motion_log_path.relative_to(REPO_ROOT)) if motion_log_path.exists() else None,
                "vehicle_timeseries_path": str(vehicle_timeseries_path.relative_to(REPO_ROOT)) if vehicle_timeseries_path.exists() else None,
                "control_timeseries_path": str(control_timeseries_path.relative_to(REPO_ROOT)) if control_timeseries_path.exists() else None,
                "mtime": report.stat().st_mtime,
            }
        )
    return reports[:100]


def motion_log_payload(run_id: str, domain_id: str | None = None, limit: int = 1600) -> dict[str, Any]:
    safe_run_id = _safe_run_id(run_id)
    run_dir = REPO_ROOT / "analysis/runs" / safe_run_id
    if not run_dir.is_dir():
        raise FileNotFoundError(f"analysis run not found: {safe_run_id}")

    processed_dir = run_dir / "processed"
    motion_path = processed_dir / "motion_log.csv"
    vehicle_path = processed_dir / "vehicle_timeseries.csv"
    control_path = processed_dir / "control_timeseries.csv"
    motion_rows = _read_csv_dicts(motion_path)
    if not motion_rows:
        motion_rows = _combine_motion_rows(_read_csv_dicts(vehicle_path), _read_csv_dicts(control_path))

    domains = sorted({str(row.get("domain_id") or "") for row in motion_rows if row.get("domain_id")})
    selected_domain = domain_id if domain_id in domains else (domains[0] if domains else "")
    if selected_domain:
        motion_rows = [row for row in motion_rows if row.get("domain_id") == selected_domain]

    motion_rows = sorted(motion_rows, key=lambda row: _float_or_none(row.get("time_sec")) or 0.0)
    sampled_rows = _sample_rows(motion_rows, max(100, min(limit, 6000)))
    first_time = next((_float_or_none(row.get("time_sec")) for row in motion_rows if _float_or_none(row.get("time_sec")) is not None), None)
    points = [_motion_point(row, first_time) for row in sampled_rows]
    points = [point for point in points if point["time_sec"] is not None]

    return {
        "run_id": safe_run_id,
        "domain_id": selected_domain,
        "domains": domains,
        "sample_count": len(motion_rows),
        "display_count": len(points),
        "paths": {
            "motion_log": str(motion_path.relative_to(REPO_ROOT)) if motion_path.exists() else None,
            "vehicle_timeseries": str(vehicle_path.relative_to(REPO_ROOT)) if vehicle_path.exists() else None,
            "control_timeseries": str(control_path.relative_to(REPO_ROOT)) if control_path.exists() else None,
        },
        "stats": _motion_stats(motion_rows, first_time),
        "points": points,
    }


def _safe_run_id(run_id: str) -> str:
    safe = run_id.strip()
    if not safe or safe in {".", ".."} or "/" in safe or "\\" in safe:
        raise PermissionError(f"invalid run_id: {run_id}")
    return safe


def _read_csv_dicts(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as handle:
        return [dict(row) for row in csv.DictReader(handle)]


def _combine_motion_rows(vehicle_rows: list[dict[str, str]], control_rows: list[dict[str, str]]) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    if vehicle_rows:
        for vehicle in vehicle_rows:
            control = _nearest_csv_row(control_rows, _float_or_none(vehicle.get("time_sec")))
            rows.append(
                {
                    "run_id": vehicle.get("run_id", ""),
                    "domain_id": vehicle.get("domain_id", ""),
                    "time_sec": vehicle.get("time_sec", ""),
                    "speed_mps": vehicle.get("speed_mps", ""),
                    "acceleration_mps2": vehicle.get("acceleration_mps2", ""),
                    "steering_rad": vehicle.get("steering_rad", ""),
                    "target_speed_mps": control.get("target_speed_mps", "") if control else "",
                    "command_accel_mps2": control.get("accel_mps2", "") if control else "",
                    "command_steer_rad": control.get("steer_rad", "") if control else "",
                    "throttle": control.get("throttle", "") if control else "",
                    "brake": control.get("brake", "") if control else "",
                }
            )
        return rows

    for control in control_rows:
        rows.append(
            {
                "run_id": control.get("run_id", ""),
                "domain_id": control.get("domain_id", ""),
                "time_sec": control.get("time_sec", ""),
                "speed_mps": "",
                "acceleration_mps2": "",
                "steering_rad": "",
                "target_speed_mps": control.get("target_speed_mps", ""),
                "command_accel_mps2": control.get("accel_mps2", ""),
                "command_steer_rad": control.get("steer_rad", ""),
                "throttle": control.get("throttle", ""),
                "brake": control.get("brake", ""),
            }
        )
    return rows


def _nearest_csv_row(rows: list[dict[str, str]], time_sec: float | None, tolerance_sec: float = 0.25) -> dict[str, str] | None:
    if time_sec is None:
        return None
    best: dict[str, str] | None = None
    best_delta = tolerance_sec
    for row in rows:
        row_time = _float_or_none(row.get("time_sec"))
        if row_time is None:
            continue
        delta = abs(row_time - time_sec)
        if delta <= best_delta:
            best = row
            best_delta = delta
    return best


def _sample_rows(rows: list[dict[str, str]], limit: int) -> list[dict[str, str]]:
    if len(rows) <= limit:
        return rows
    step = max(1, math.ceil(len(rows) / limit))
    return rows[::step]


def _motion_point(row: dict[str, str], first_time: float | None) -> dict[str, float | None]:
    absolute_time = _float_or_none(row.get("time_sec"))
    time_sec = absolute_time - first_time if absolute_time is not None and first_time is not None else absolute_time
    return {
        "time_sec": time_sec,
        "speed_mps": _float_or_none(row.get("speed_mps")),
        "acceleration_mps2": _float_or_none(row.get("acceleration_mps2")),
        "steering_rad": _float_or_none(row.get("steering_rad")),
        "target_speed_mps": _float_or_none(row.get("target_speed_mps")),
        "command_accel_mps2": _float_or_none(row.get("command_accel_mps2")),
        "command_steer_rad": _float_or_none(row.get("command_steer_rad")),
    }


def _motion_stats(rows: list[dict[str, str]], first_time: float | None) -> dict[str, float | int | None]:
    times = [_float_or_none(row.get("time_sec")) for row in rows]
    times = [value for value in times if value is not None]
    speeds = [_float_or_none(row.get("speed_mps")) for row in rows]
    speeds = [value for value in speeds if value is not None]
    accels = [_float_or_none(row.get("acceleration_mps2")) for row in rows]
    accels = [value for value in accels if value is not None]
    steer_values = [_float_or_none(row.get("steering_rad")) for row in rows]
    steer_values = [value for value in steer_values if value is not None]
    command_steers = [_float_or_none(row.get("command_steer_rad")) for row in rows]
    command_steers = [value for value in command_steers if value is not None]
    duration = max(times) - min(times) if times else None
    return {
        "duration_sec": duration,
        "samples": len(rows),
        "max_speed_mps": max(speeds) if speeds else None,
        "max_abs_acceleration_mps2": max((abs(value) for value in accels), default=None),
        "max_abs_steering_rad": max((abs(value) for value in steer_values), default=None),
        "max_abs_command_steering_rad": max((abs(value) for value in command_steers), default=None),
        "start_time_sec": first_time,
    }


def _float_or_none(value: Any) -> float | None:
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def docker_ps() -> str:
    try:
        result = subprocess.run(
            ["make", "ps"],
            cwd=REPO_ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=10,
            check=False,
        )
        return result.stdout
    except Exception as exc:  # pragma: no cover - diagnostic fallback
        return str(exc)


def app_state() -> dict[str, Any]:
    ensure_dirs()
    methods = parse_control_methods()
    method = selected_method()
    state = read_state()
    return {
        "repo_root": str(REPO_ROOT),
        "methods": methods,
        "selected_control_method": method,
        "xml_default_control_method": parse_control_default(),
        "files": catalog_files(method),
        "catalog": {name: catalog_files(name) for name in methods if name in CATALOG},
        "dirty_since": state.get("dirty_since"),
        "last_build_at": state.get("last_build_at"),
        "last_run_at": state.get("last_run_at"),
        "command": command_status(),
        "presets": list_presets(),
        "safety_gates": [
            {"id": gate_id, **gate}
            for gate_id, gate in SAFETY_GATES.items()
        ],
    }


class Handler(BaseHTTPRequestHandler):
    server_version = "TuningGUI/0.1"

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        try:
            if parsed.path == "/":
                self._send_index()
            elif parsed.path in {"/app.js", "/styles.css"}:
                content_type = "application/javascript; charset=utf-8" if parsed.path.endswith(".js") else "text/css; charset=utf-8"
                self._send_file(STATIC_DIR / parsed.path.lstrip("/"), content_type, no_store=True)
            elif parsed.path == "/api/state":
                self._json(app_state())
            elif parsed.path == "/api/file":
                params = parse_qs(parsed.query)
                path = str(params.get("path", [""])[0])
                relative = str(rel_path(path))
                if relative not in allowed_paths():
                    raise PermissionError(f"not editable by tuning GUI: {relative}")
                target = abs_path(relative)
                self._json(
                    {
                        "path": relative,
                        "content": target.read_text(encoding="utf-8") if target.exists() else "",
                        "exists": target.exists(),
                        "mtime": target.stat().st_mtime if target.exists() else None,
                    }
                )
            elif parsed.path == "/api/diff":
                params = parse_qs(parsed.query)
                self._json({"diff": file_diff(params.get("path", [""])[0])})
            elif parsed.path == "/api/structured":
                params = parse_qs(parsed.query)
                path = str(params.get("path", [""])[0])
                relative = str(rel_path(path))
                if relative not in allowed_paths():
                    raise PermissionError(f"not editable by tuning GUI: {relative}")
                target = abs_path(relative)
                self._json(structured_rows(relative, target.read_text(encoding="utf-8") if target.exists() else ""))
            elif parsed.path == "/api/command":
                self._json(command_status())
            elif parsed.path == "/api/history":
                self._json({"commands": load_history(), "outputs": discover_output_runs(), "reports": discover_reports()})
            elif parsed.path == "/api/motion-log":
                params = parse_qs(parsed.query)
                run_id = str(params.get("run_id", [""])[0])
                domain_id = params.get("domain", [None])[0]
                limit_raw = params.get("limit", ["1600"])[0]
                try:
                    limit = int(limit_raw)
                except ValueError:
                    limit = 1600
                self._json(motion_log_payload(run_id, domain_id, limit))
            elif parsed.path == "/api/docker-ps":
                self._json({"output": docker_ps()})
            elif parsed.path == "/api/presets":
                self._json({"presets": list_presets()})
            elif parsed.path == "/api/path-editor":
                params = parse_qs(parsed.query)
                path = params.get("path", [None])[0]
                config_path = params.get("config_path", [None])[0]
                self._json(path_editor_state(path, config_path))
            elif parsed.path == "/api/path-editor/map.png":
                params = parse_qs(parsed.query)
                relative = rel_path(unquote(params.get("path", [""])[0]))
                if relative.suffix.lower() != ".pgm":
                    raise PermissionError("map image must be a PGM file")
                relative.relative_to(MPC_ROOT)
                data = pgm_to_png(abs_path(relative))
                self.send_response(200)
                self.send_header("Content-Type", "image/png")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            elif parsed.path == "/files":
                params = parse_qs(parsed.query)
                relative = str(rel_path(unquote(params.get("path", [""])[0])))
                if not relative.startswith(("analysis/", "output/", "tools/tuning_gui/runtime/", "tools/tuning_gui/history/")):
                    raise PermissionError("file is outside readonly GUI roots")
                self._send_file(abs_path(relative), self._content_type(relative))
            else:
                self.send_error(HTTPStatus.NOT_FOUND)
        except Exception as exc:
            self._error(exc)

    def do_POST(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        try:
            body = self._read_json()
            if parsed.path == "/api/selected-control-method":
                result = set_selected_method(str(body.get("method", "")))
                self._json(result)
            elif parsed.path == "/api/control-method":
                with command_lock:
                    running = active_process is not None and active_process.poll() is None
                if running:
                    raise RuntimeError("editing is locked while a command is running")
                result = set_control_method_default(str(body.get("method", "")))
                if body.get("auto_rebuild"):
                    start_command("build", result["method"], False, "auto rebuild after control_method change")
                self._json(result)
            elif parsed.path == "/api/file":
                with command_lock:
                    running = active_process is not None and active_process.poll() is None
                if running:
                    raise RuntimeError("editing is locked while a command is running")
                result = write_file(str(body.get("path", "")), str(body.get("content", "")))
                if result["changed"] and body.get("auto_rebuild"):
                    start_command("build", selected_method(), False, f"auto rebuild after editing {body.get('path')}")
                self._json(result)
            elif parsed.path == "/api/diff":
                self._json({"diff": file_diff(str(body.get("path", "")), str(body.get("content", "")))})
            elif parsed.path == "/api/validate":
                validate_content(str(body.get("path", "")), str(body.get("content", "")))
                self._json({"ok": True})
            elif parsed.path == "/api/structured/parse":
                self._json(structured_rows(str(body.get("path", "")), str(body.get("content", ""))))
            elif parsed.path == "/api/structured/apply":
                self._json(
                    apply_structured_rows(
                        str(body.get("path", "")),
                        str(body.get("content", "")),
                        list(body.get("rows") or []),
                    )
                )
            elif parsed.path == "/api/run":
                action = str(body.get("action", ""))
                method = str(body.get("control_method") or selected_method())
                build_first = bool(body.get("build_first", action in {"dev", "eval", "quick-eval"}))
                note = str(body.get("note", ""))
                headless = bool(body.get("headless", False))
                npc_count = normalize_npc_count(body.get("npc_count", 0))
                simulator_options = normalize_simulator_options(body.get("simulator_options"))
                safety_gate = normalize_safety_gate(body.get("safety_gate"), required=True) if action == "gate" else None
                state = start_command(action, method, build_first, note, headless, npc_count, simulator_options, safety_gate)
                self._json(asdict(state))
            elif parsed.path == "/api/stop":
                stop_active_command()
                self._json({"ok": True})
            elif parsed.path == "/api/presets":
                result = save_preset(str(body.get("name", "")), str(body.get("control_method") or selected_method()), str(body.get("note", "")))
                self._json(result)
            elif parsed.path == "/api/presets/restore":
                with command_lock:
                    running = active_process is not None and active_process.poll() is None
                if running:
                    raise RuntimeError("editing is locked while a command is running")
                result = restore_preset(str(body.get("name", "")))
                if body.get("auto_rebuild"):
                    start_command("build", result["control_method"], False, "auto rebuild after preset restore")
                self._json(result)
            elif parsed.path == "/api/path-editor/load":
                config_path = str(body.get("config_path") or "") or None
                path = str(body.get("path", "") or current_mpc_reference_path(config_path))
                self._json(path_editor_state(path, config_path))
            elif parsed.path == "/api/path-editor/save":
                with command_lock:
                    running = active_process is not None and active_process.poll() is None
                if running:
                    raise RuntimeError("editing is locked while a command is running")
                result = save_reference_path(body)
                if (result["changed"] or result["config_changed"]) and body.get("auto_rebuild"):
                    start_command("build", selected_method(), False, f"auto rebuild after editing {result['path']}")
                self._json(result)
            else:
                self.send_error(HTTPStatus.NOT_FOUND)
        except Exception as exc:
            self._error(exc)

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[{datetime.now().strftime('%H:%M:%S')}] {fmt % args}")

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length") or 0)
        if length == 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def _json(self, payload: Any, status: int = 200) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_index(self) -> None:
        path = STATIC_DIR / "index.html"
        if not path.exists() or not path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        text = path.read_text(encoding="utf-8")
        text = text.replace('href="/styles.css"', f'href="/styles.css?v={self._asset_version("styles.css")}"')
        text = text.replace('src="/app.js"', f'src="/app.js?v={self._asset_version("app.js")}"')
        self._send_bytes(text.encode("utf-8"), "text/html; charset=utf-8", no_store=True)

    def _asset_version(self, filename: str) -> int:
        path = STATIC_DIR / filename
        try:
            return int(path.stat().st_mtime)
        except FileNotFoundError:
            return 0

    def _send_file(self, path: Path, content_type: str, *, no_store: bool = False) -> None:
        if not path.exists() or not path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        data = path.read_bytes()
        self._send_bytes(data, content_type, no_store=no_store)

    def _send_bytes(self, data: bytes, content_type: str, *, no_store: bool = False) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        if no_store:
            self.send_header("Cache-Control", STATIC_CACHE_CONTROL)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _error(self, exc: Exception) -> None:
        status = HTTPStatus.BAD_REQUEST
        if isinstance(exc, PermissionError):
            status = HTTPStatus.FORBIDDEN
        elif isinstance(exc, FileNotFoundError):
            status = HTTPStatus.NOT_FOUND
        elif isinstance(exc, RuntimeError) and "running" in str(exc):
            status = HTTPStatus.CONFLICT
        self._json({"error": str(exc)}, int(status))

    def _content_type(self, path: str) -> str:
        if path.endswith(".html"):
            return "text/html; charset=utf-8"
        if path.endswith(".json"):
            return "application/json; charset=utf-8"
        if path.endswith(".csv"):
            return "text/csv; charset=utf-8"
        if path.endswith(".log") or path.endswith(".txt"):
            return "text/plain; charset=utf-8"
        return "application/octet-stream"


def main() -> int:
    ensure_dirs()
    host = os.environ.get("TUNING_GUI_HOST", "127.0.0.1")
    port = int(os.environ.get("TUNING_GUI_PORT", "8765"))
    server = ThreadingHTTPServer((host, port), Handler)
    print(f"tuning GUI: http://{host}:{port}")
    print(f"repo root: {REPO_ROOT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("stopping tuning GUI")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
