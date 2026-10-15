# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Observe one bounded cohort of fresh LAND cases; passes do not establish a timeout fix."""

import json
import sys

from runtime_land_fixture import run_case


def main() -> int:
    for sample in range(1, 21):
        print(json.dumps({"event": "runtime_land_timing_sample", "sample": sample}), flush=True)
        run_case("accepted-without-mode", "unknown", True, mode_delay=None)
    print(json.dumps({"event": "runtime_land_timing_cohort", "result": "passed", "samples": 20}), flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception:
        print('{"event":"runtime_land_timing_cohort","result":"failed"}', file=sys.stderr, flush=True)
        raise SystemExit(1)
