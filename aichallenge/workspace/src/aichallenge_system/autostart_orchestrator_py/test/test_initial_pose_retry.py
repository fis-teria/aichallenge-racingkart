import threading
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace


def _load_module():
    node_path = (
        Path(__file__).resolve().parents[1]
        / "autostart_orchestrator_py/autostart_orchestrator_node.py"
    )
    spec = spec_from_file_location("autostart_orchestrator_retry_node", node_path)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_MODULE = _load_module()


class _Logger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass

    def error(self, _message):
        pass


class _InitializationHarness:
    _STATE_REQUEST_CONTROL_MODE = "REQUEST_CONTROL_MODE"
    _STATE_WAIT_INITIAL_POSE = "WAIT_INITIAL_POSE"
    _do_start_initialization = (
        _MODULE.AutostartOrchestrator._do_start_initialization
    )

    def __init__(self, responses, *, timeout_sec=1.0, interval_sec=0.001):
        self._responses = list(responses)
        self._active = True
        self._cli_initial_pose = object()
        self._parameters = {
            "initial_pose_service_timeout_sec": timeout_sec,
            "initial_pose_retry_interval_sec": interval_sec,
            "initial_pose_service": "/set_initial_pose",
            "control_mode_request_topic": "/control_mode",
        }
        self.call_count = 0
        self.control_mode_count = 0

    def get_parameter(self, name):
        return SimpleNamespace(value=self._parameters[name])

    def get_logger(self):
        return _Logger()

    def _set_workflow_state(self, _state, _detail):
        pass

    def _race_initialization_active(self, _generation):
        return self._active

    def _wait_for_service(self, _client, timeout_sec=None):
        return timeout_sec is not None

    def _call_trigger_before_start(self, _client, _generation, _deadline):
        self.call_count += 1
        return self._responses.pop(0)

    def _wait_initial_pose_retry_interval(
        self, _generation, _deadline, _interval_sec
    ):
        return self._active

    def _publish_control_mode(self):
        self.control_mode_count += 1
        return True, "published"


def test_initial_pose_retry_succeeds_on_third_gnss_attempt():
    harness = _InitializationHarness(
        [
            (False, "no GNSS data received yet"),
            (False, "no GNSS data received yet"),
            (True, "initialized"),
        ]
    )

    assert harness._do_start_initialization(True, False, 7) is True
    assert harness.call_count == 3
    assert harness.control_mode_count == 0


def test_fatal_initial_pose_failure_is_not_retried_or_sent_to_control():
    harness = _InitializationHarness([(False, "invalid heading csv")])

    assert harness._do_start_initialization(True, True, 9) is False
    assert harness.call_count == 1
    assert harness.control_mode_count == 0


def test_start_invalidation_discards_success_before_control_mode():
    harness = _InitializationHarness([(True, "initialized")])

    def invalidate_on_response(_client, _generation, _deadline):
        harness.call_count += 1
        harness._active = False
        return True, "late initialized"

    harness._call_trigger_before_start = invalidate_on_response
    assert harness._do_start_initialization(True, True, 10) is False
    assert harness.call_count == 1
    assert harness.control_mode_count == 0


def test_invalid_retry_budget_fails_closed_without_service_call():
    harness = _InitializationHarness(
        [(True, "initialized")], timeout_sec=float("nan")
    )

    assert harness._do_start_initialization(True, True, 11) is False
    assert harness.call_count == 0
    assert harness.control_mode_count == 0


class _InactiveLatch:
    def initialization_active(self, _generation):
        return False


class _CountingClient:
    def __init__(self):
        self.call_count = 0

    def call_async(self, _request):
        self.call_count += 1
        raise AssertionError("invalidated initialization must not send a request")


class _TriggerBeforeStartHarness:
    _call_trigger_before_start = (
        _MODULE.AutostartOrchestrator._call_trigger_before_start
    )

    def __init__(self):
        self._race_arm_lock = threading.Lock()
        self._race_arm_latch = _InactiveLatch()


def test_invalidated_generation_is_rechecked_before_service_request():
    harness = _TriggerBeforeStartHarness()
    client = _CountingClient()

    ok, message = harness._call_trigger_before_start(client, 12, float("inf"))

    assert ok is False
    assert message == "initialization invalidated before request"
    assert client.call_count == 0
