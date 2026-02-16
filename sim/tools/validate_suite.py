#!/usr/bin/env python3
from __future__ import annotations

import json
import shutil
import subprocess
import tempfile
import textwrap
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
BASE_CONFIG = ROOT / "config"
OUT_ROOT = ROOT / "output" / "validate_suite"


def _write_scenario(path: Path, body: str) -> None:
    path.write_text(textwrap.dedent(body).strip() + "\n", encoding="utf-8")


def _prepare_config(overrides: dict[str, float]) -> Path:
    td = Path(tempfile.mkdtemp(prefix="sim_cfg_"))
    for name in ("robot.yaml", "noise.yaml", "faults.yaml", "run.yaml"):
        shutil.copy2(BASE_CONFIG / name, td / name)

    robot = yaml.safe_load((td / "robot.yaml").read_text(encoding="utf-8"))
    robot.update(overrides)
    (td / "robot.yaml").write_text(yaml.safe_dump(robot, sort_keys=False), encoding="utf-8")
    return td


def _run_case(name: str, cfg_dir: Path, scenario: Path, duration: float) -> dict[str, object]:
    out = OUT_ROOT / name
    if out.exists():
        shutil.rmtree(out)

    cmd = [
        "python3",
        "run.py",
        "--config-dir",
        str(cfg_dir),
        "--scenario",
        str(scenario),
        "--out",
        str(out),
        "--duration",
        str(duration),
        "--no-realtime",
    ]
    p = subprocess.run(cmd, cwd=ROOT, text=True, capture_output=True)
    if p.returncode != 0:
        return {
            "name": name,
            "ok": False,
            "error": (p.stderr or p.stdout).strip(),
        }

    report = json.loads((out / "report.json").read_text(encoding="utf-8"))
    diag = report.get("diagnostics", {})
    over = diag.get("overshoot_check", {})
    rev = diag.get("reverse_motor_check", {})
    return {
        "name": name,
        "ok": True,
        "max_speed": report.get("max_speed_mps", 0.0),
        "max_slip": report.get("max_slip_mps", 0.0),
        "turn_overshoot_deg": over.get("max_turn_overshoot_deg", 0.0),
        "reverse_events": rev.get("event_count", 0),
        "issue_count": diag.get("issue_count", 0),
        "issues": diag.get("issues", []),
        "report": str(out / "report.json"),
    }


def main() -> None:
    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    scenario_dir = Path(tempfile.mkdtemp(prefix="sim_suite_scn_"))

    baseline_scn = scenario_dir / "baseline.yaml"
    _write_scenario(
        baseline_scn,
        """
        segments:
          - duration_s: 1.8
            left_cmd: 0.65
            right_cmd: 0.65
          - duration_s: 0.7
            left_cmd: 0.45
            right_cmd: -0.45
          - duration_s: 1.4
            left_cmd: 0.75
            right_cmd: 0.75
          - duration_s: 0.5
            left_cmd: 0.0
            right_cmd: 0.0
        """,
    )

    turn_stress_scn = scenario_dir / "turn_stress.yaml"
    _write_scenario(
        turn_stress_scn,
        """
        segments:
          - duration_s: 0.9
            left_cmd: 0.9
            right_cmd: -0.9
          - duration_s: 0.9
            left_cmd: -0.9
            right_cmd: 0.9
          - duration_s: 0.6
            left_cmd: 0.0
            right_cmd: 0.0
        """,
    )

    results: list[dict[str, object]] = []

    # Normal expected-good run.
    cfg_ok = _prepare_config({})
    results.append(_run_case("baseline_ok", cfg_ok, baseline_scn, 4.4))

    # Should trigger overshoot issue.
    cfg_over = _prepare_config({})
    results.append(_run_case("turn_stress", cfg_over, turn_stress_scn, 2.6))

    # Should trigger direction mismatch by forcing inverted motor torque sign.
    cfg_bad_motor = _prepare_config({"stall_torque_nm": -0.35})
    results.append(_run_case("motor_direction_fault", cfg_bad_motor, baseline_scn, 2.5))

    # Evaluate suite pass/fail rules.
    suite_ok = True
    reasons: list[str] = []

    baseline = next((r for r in results if r["name"] == "baseline_ok" and r.get("ok")), None)
    turn = next((r for r in results if r["name"] == "turn_stress" and r.get("ok")), None)
    motor = next((r for r in results if r["name"] == "motor_direction_fault" and r.get("ok")), None)

    if baseline is None or baseline.get("issue_count", 1) != 0:
        suite_ok = False
        reasons.append("baseline should be clean")

    if turn is None or "turn_overshoot_high" not in turn.get("issues", []):
        suite_ok = False
        reasons.append("turn stress should trigger overshoot")

    if motor is None or "motor_direction_mismatch_detected" not in motor.get("issues", []):
        suite_ok = False
        reasons.append("motor fault should trigger direction mismatch")

    summary = {
        "suite_ok": suite_ok,
        "reasons": reasons,
        "results": results,
        "output_root": str(OUT_ROOT),
    }

    (OUT_ROOT / "suite_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
