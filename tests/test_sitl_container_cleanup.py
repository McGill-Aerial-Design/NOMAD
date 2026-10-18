# SPDX-License-Identifier: Apache-2.0
"""Execute CI shell lifecycles without a Docker daemon or an aircraft."""

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
STEPS = [
    ("sitl", "Qualify runtime authority and independent source on isolated Copter", 1),
    ("sitl", "Qualify installed CLI Copter LAND on three clean isolated simulators", 3),
    ("quadplane-observation", "Prove disarmed simulated RC fault delivery on two clean instances", 2),
]
FAKE_COMMANDS = r"""
docker() {
  local action="$1"
  shift
  if [[ "$action" == "compose" ]]; then
    if [[ "$FAIL_CREATE" == 1 ]]; then
      printf 'create_failed\n' >> "$EVENTS"
      return 7
    fi
    if [[ -e "$OWNER" ]]; then
      printf 'name_conflict\n' >> "$EVENTS"
      return 23
    fi
    local count=0
    if [[ -f "$CREATES" ]]; then read -r count < "$CREATES"; fi
    count=$((count + 1))
    printf '%s\n' "$count" > "$CREATES"
    printf 'owned-%s\n' "$count" > "$OWNER"
    printf 'create owned-%s\n' "$count" >> "$EVENTS"
    cat "$OWNER"
    return 0
  fi
  if [[ "$action" != stop && "$action" != rm ]]; then return 66; fi
  if [[ "$action" == rm ]]; then
    if [[ "$1" != --volumes ]]; then return 67; fi
    shift
  fi
  if [[ ! -f "$OWNER" || "$(cat "$OWNER")" != "$1" ]]; then
    printf 'unowned_cleanup\n' >> "$EVENTS"
    return 65
  fi
  printf '%s %s\n' "$action" "$1" >> "$EVENTS"
  if [[ "$action" == rm ]]; then
    if [[ "$FAIL_REMOVE" == 1 ]]; then return 11; fi
    rm "$OWNER"
  fi
}
sleep() { :; }
pixi() {
  printf 'probe\n' >> "$EVENTS"
  if [[ "$FAIL_PROBE" == 1 ]]; then return 9; fi
}
"""


@pytest.mark.skipif(sys.platform == "win32", reason="CI cleanup shell requires native Bash")
@pytest.mark.parametrize("job,name,trials", STEPS, ids=["authority", "land", "quadplane"])
@pytest.mark.parametrize("failure", ["none", "create", "probe", "remove"])
def test_owned_container_lifecycle(tmp_path, job, name, trials, failure):
    bash = shutil.which("bash")
    assert bash is not None
    workflow = yaml.safe_load((ROOT / ".github/workflows/sitl.yml").read_text(encoding="utf-8"))
    script = next(step["run"] for step in workflow["jobs"][job]["steps"] if step.get("name") == name)
    owner, events = tmp_path / "owner", tmp_path / "events"
    environment = dict(
        os.environ,
        OWNER=owner.as_posix(),
        EVENTS=events.as_posix(),
        CREATES=(tmp_path / "creates").as_posix(),
        RUNNER_TEMP=tmp_path.as_posix(),
        FAIL_CREATE=str(int(failure == "create")),
        FAIL_PROBE=str(int(failure == "probe")),
        FAIL_REMOVE=str(int(failure == "remove")),
    )
    result = subprocess.run(
        [bash, "-c", FAKE_COMMANDS + script], env=environment, capture_output=True, text=True, timeout=5
    )
    observed = events.read_text(encoding="utf-8").splitlines()
    if failure == "create":
        assert result.returncode == 7 and observed == ["create_failed"]
    elif failure == "remove":
        assert result.returncode == 11
        assert observed == ["create owned-1", "probe", "stop owned-1", "rm owned-1", "stop owned-1", "rm owned-1"]
    else:
        assert result.returncode == (9 if failure == "probe" else 0), result.stderr
        completed = 1 if failure == "probe" else trials
        expected = [
            event
            for n in range(1, completed + 1)
            for event in [f"create owned-{n}", "probe", f"stop owned-{n}", f"rm owned-{n}"]
        ]
        assert observed == expected, observed
    assert owner.exists() == (failure == "remove")
