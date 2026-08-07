from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy


def _load_vehicle_state_qos():
    node_path = (
        Path(__file__).resolve().parents[1]
        / "autostart_orchestrator_py/autostart_orchestrator_node.py"
    )
    spec = spec_from_file_location("autostart_orchestrator_qos_node", node_path)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module._vehicle_state_qos


def test_vehicle_state_subscription_accepts_latched_awsim_state():
    qos = _load_vehicle_state_qos()()

    assert qos.depth == 10
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
