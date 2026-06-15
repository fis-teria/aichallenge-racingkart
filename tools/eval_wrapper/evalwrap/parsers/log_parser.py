from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path


KEYWORDS = ("ERROR", "WARN", "Exception", "exception", "Traceback", "node died", "died")


@dataclass
class LogExcerpt:
    path: str
    line_no: int
    text: str


@dataclass
class ParsedLogs:
    excerpts: list[LogExcerpt] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


def parse_logs(domain_dir: Path, limit: int = 80) -> ParsedLogs:
    parsed = ParsedLogs()
    candidates = [domain_dir / "autoware.log", *sorted((domain_dir / "ros" / "log").glob("**/*.log"))]
    for path in candidates:
        if not path.exists() or not path.is_file():
            continue
        try:
            with path.open("r", encoding="utf-8", errors="replace") as handle:
                for line_no, line in enumerate(handle, start=1):
                    if any(keyword in line for keyword in KEYWORDS):
                        parsed.excerpts.append(
                            LogExcerpt(path=str(path.relative_to(domain_dir)), line_no=line_no, text=line.rstrip())
                        )
                        if len(parsed.excerpts) >= limit:
                            return parsed
        except Exception as exc:  # noqa: BLE001
            parsed.warnings.append(f"failed to read log {path.name}: {exc}")
    return parsed
