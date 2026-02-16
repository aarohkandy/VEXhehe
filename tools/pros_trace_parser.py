#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path


def parse_line(line: str) -> tuple[str, dict[str, str]] | None:
    line = line.strip()
    if not line.startswith("MOTION_TRACE"):
        return None

    parts = line.split(",")
    tag = parts[0]
    data: dict[str, str] = {}
    for item in parts[1:]:
        if "=" not in item:
            continue
        k, v = item.split("=", 1)
        data[k.strip()] = v.strip()
    return tag, data


def main() -> None:
    p = argparse.ArgumentParser(description="Parse PROS MOTION_TRACE logs")
    p.add_argument("log", type=str, help="Path to terminal log file")
    args = p.parse_args()

    path = Path(args.log)
    if not path.exists():
        raise SystemExit(f"log not found: {path}")

    starts = []
    ends = []
    samples = []
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        parsed = parse_line(line)
        if not parsed:
            continue
        tag, data = parsed
        if tag == "MOTION_TRACE_BEGIN":
            starts.append(data)
        elif tag == "MOTION_TRACE_END":
            ends.append(data)
        else:
            samples.append(data)

    summary = {
        "starts": len(starts),
        "samples": len(samples),
        "ends": len(ends),
        "end_events": ends,
    }
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
