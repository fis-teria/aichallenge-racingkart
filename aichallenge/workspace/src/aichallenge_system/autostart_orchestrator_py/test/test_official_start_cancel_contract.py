from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


def test_accepted_official_start_cancel_forces_fresh_false_publication() -> None:
    source = (
        Path(__file__).resolve().parents[1]
        / "autostart_orchestrator_py/autostart_orchestrator_node.py"
    ).read_text(encoding="utf-8")

    start = source.index("    def _on_official_start(")
    end = source.index("    def _publish_race_arm(", start)
    callback = source[start:end]
    assert "force_publish = response.success" in callback
    assert "self._publish_race_arm(should_arm, force=force_publish)" in callback


def test_cancel_while_already_false_still_publishes_fresh_false() -> None:
    node_path = (
        Path(__file__).resolve().parents[1]
        / "autostart_orchestrator_py/autostart_orchestrator_node.py"
    )
    spec = spec_from_file_location("autostart_cancel_node", node_path)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)

    class Latch:
        @staticmethod
        def force_disarm() -> bool:
            return False

    class Logger:
        @staticmethod
        def info(_message: str) -> None:
            return None

    class Condition:
        def __enter__(self) -> "Condition":
            return self

        def __exit__(self, *_unused: object) -> None:
            return None

        @staticmethod
        def notify_all() -> None:
            return None

    class Dummy:
        _cond = Condition()
        _last_vehicle_state = "Start"
        _race_arm_latch = Latch()
        publications: list[tuple[bool, bool]] = []

        def _publish_race_arm(self, armed: bool, *, force: bool = False) -> None:
            self.publications.append((armed, force))

        @staticmethod
        def get_logger() -> Logger:
            return Logger()

    request = module.SetBool.Request()
    request.data = False
    response = module.SetBool.Response()
    dummy = Dummy()
    result = module.AutostartOrchestrator._on_official_start(
        dummy, request, response
    )

    assert result.success is True
    assert result.message == "official_start_cancelled"
    assert dummy.publications == [(False, True)]
