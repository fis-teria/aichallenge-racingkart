from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any


def load_config(path: str | Path) -> dict[str, Any]:
    source = Path(path)
    text = source.read_text(encoding="utf-8")
    try:
        import yaml

        data = yaml.safe_load(text)
    except ImportError:
        data = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError("experiment configuration must be a mapping")
    validate_config(data)
    return data


def validate_config(config: dict[str, Any]) -> None:
    for section in ("run", "evaluator", "baseline", "search_space", "fitness"):
        if section not in config or not isinstance(config[section], dict):
            raise ValueError(f"missing mapping: {section}")
    run = config["run"]
    for name in ("population_size", "elite_count"):
        if int(run[name]) <= 0:
            raise ValueError(f"run.{name} must be positive")
    if int(run["generations"]) < 0:
        raise ValueError("run.generations must be zero (unlimited) or positive")
    if int(run["elite_count"]) >= int(run["population_size"]):
        raise ValueError("elite_count must be smaller than population_size")
    evaluator = config["evaluator"]
    if evaluator.get("mode") == "shared_awsim_batch":
        environments = evaluator.get("environments", [])
        if environments:
            domains = [
                int(domain)
                for environment in environments
                for domain in environment.get("vehicle_domain_ids", [])
            ]
            admin_domains = [
                int(environment.get("admin_domain_id", 0))
                for environment in environments
            ]
            if len(admin_domains) != len(set(admin_domains)):
                raise ValueError(
                    "shared_awsim_batch requires unique environment admin domains"
                )
        else:
            domains = [int(value) for value in evaluator.get("vehicle_domain_ids", [])]
        if not domains or len(domains) != len(set(domains)):
            raise ValueError(
                "shared_awsim_batch requires unique vehicle domains"
            )
        if int(run.get("parallel_workers", 1)) != len(domains):
            raise ValueError(
                "run.parallel_workers must equal the shared AWSIM domain count"
            )
    surrogate = config.get("surrogate", {})
    if surrogate:
        if int(surrogate.get("candidate_pool_multiplier", 1)) < 1:
            raise ValueError("surrogate.candidate_pool_multiplier must be at least 1")
        exploit = float(surrogate.get("exploit_fraction", 0.6))
        uncertainty = float(surrogate.get("uncertainty_fraction", 0.2))
        if exploit < 0.0 or uncertainty < 0.0 or exploit + uncertainty > 1.0:
            raise ValueError(
                "surrogate selection fractions must be non-negative and sum to at most 1"
            )
    baseline = config["baseline"]
    for name, spec in config["search_space"].items():
        low, high = float(spec["min"]), float(spec["max"])
        if not low < high:
            raise ValueError(f"invalid bounds for {name}")
        if name not in baseline:
            raise ValueError(f"baseline is missing {name}")


def canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def digest(value: Any) -> str:
    return "sha256:" + hashlib.sha256(canonical_json(value).encode()).hexdigest()
