#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Builds only inside the owned workspace; never installs binaries or system packages.
set -euo pipefail
sim_root=${1:-"$HOME/nomad-simulation"}
target=${2:-gazebo}
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
bash "$script_dir/prepare_linux.sh" "$sim_root" --check
mkdir -p "$sim_root/build" "$sim_root/logs"
export PYTHONNOUSERSITE=1
unset PYTHONPATH
export PIP_CACHE_DIR="$sim_root/cache/pip"
case "$target" in
    ardupilot)
        if [[ ! -x "$sim_root/python/bin/python" ]]; then
            python3 -m venv "$sim_root/python"
        fi
        "$sim_root/python/bin/python" -m pip install --disable-pip-version-check setuptools==75.3.2 wheel==0.45.1
        "$sim_root/python/bin/python" -m pip install --disable-pip-version-check --no-build-isolation \
            empy==3.3.4 pexpect==4.9.0 ptyprocess==0.7.0
        "$sim_root/python/bin/python" -m pip freeze > "$sim_root/logs/python-packages.txt"
        export PATH="$sim_root/python/bin:$PATH"
        cd "$sim_root/sources/ardupilot"
        ./waf configure --board sitl --no-submodule-update --disable-tests 2>&1 | tee "$sim_root/logs/ap-configure.log"
        ./waf copter -j4 2>&1 | tee "$sim_root/logs/ap-build.log"
        ./waf plane -j4 2>&1 | tee "$sim_root/logs/ap-plane-build.log"
        sha256sum build/sitl/bin/arducopter build/sitl/bin/arduplane > "$sim_root/logs/ardupilot.sha256"
        python3 "$script_dir/build_manifest.py" "$sim_root" ardupilot \
            "$sim_root/sources/ardupilot/build/sitl/bin/arducopter" \
            "$sim_root/sources/ardupilot/build/sitl/bin/arduplane"
        ;;
    gazebo)
        export GZ_VERSION=harmonic
        # The pinned checkout stays clean; recorded NOMAD patches apply to an exported build copy.
        patched="$sim_root/build/ardupilot_gazebo-patched"
        rm -rf -- "$patched" && mkdir -p "$patched"
        git -C "$sim_root/sources/ardupilot_gazebo" archive HEAD | tar -x -C "$patched"
        for patch_file in "$script_dir"/patches/ardupilot_gazebo-*.patch; do
            patch -d "$patched" -p1 --forward --batch < "$patch_file"
        done
        rm -rf -- "$sim_root/build/gazebo-plugin/CMakeCache.txt" "$sim_root/build/gazebo-plugin/CMakeFiles"
        cmake -S "$patched" -B "$sim_root/build/gazebo-plugin" \
            -DCMAKE_BUILD_TYPE=RelWithDebInfo 2>&1 | tee "$sim_root/logs/gazebo-configure.log"
        cmake --build "$sim_root/build/gazebo-plugin" --parallel 4 2>&1 | tee "$sim_root/logs/gazebo-build.log"
        cmake -S "$script_dir/../../mission_planner/tests/simulation/gazebo" -B "$sim_root/build/nomad-test-system" \
            -DCMAKE_BUILD_TYPE=RelWithDebInfo
        cmake --build "$sim_root/build/nomad-test-system" --parallel 4
        python3 "$script_dir/build_manifest.py" "$sim_root" ardupilot_gazebo "$sim_root/build/gazebo-plugin/libArduPilotPlugin.so"
        python3 - "$sim_root" "$script_dir/../../mission_planner/tests/simulation/gazebo" <<'PY'
import hashlib, json, sys
from pathlib import Path
workspace, source = map(Path, sys.argv[1:])
files = {name: hashlib.sha256((source / name).read_bytes()).hexdigest()
         for name in ("CMakeLists.txt", "NomadTestSystem.cc")}
binary = "build/nomad-test-system/libNomadTestSystem.so"
files[binary] = hashlib.sha256((workspace / binary).read_bytes()).hexdigest()
(workspace / "logs/nomad-test-system-build.json").write_text(json.dumps({"schema": 1, "files": files}, indent=2) + "\n")
PY
        ;;
    last-letter)
        cmake -S "$sim_root/sources/last_letter" -B "$sim_root/build/last-letter" \
            -DCMAKE_BUILD_TYPE=Release -DLAST_LETTER_BUILD_ARDUPILOT=ON \
            -DLAST_LETTER_BUILD_PYTHON=OFF -DLAST_LETTER_BUILD_TESTS=OFF \
            -DFETCHCONTENT_SOURCE_DIR_YAML_CPP="$sim_root/sources/yaml_cpp" \
            -DFETCHCONTENT_SOURCE_DIR_DATA_TAMER="$sim_root/sources/data_tamer" \
            -DFETCHCONTENT_SOURCE_DIR_NLOPT="$sim_root/sources/nlopt" -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
            2>&1 | tee "$sim_root/logs/last-letter-configure.log"
        cmake --build "$sim_root/build/last-letter" --parallel 4 2>&1 | tee "$sim_root/logs/last-letter-build.log"
        python3 "$script_dir/build_manifest.py" "$sim_root" last_letter "$sim_root/build/last-letter/last_letter_ardupilot"
        ;;
    *) echo 'Expected ardupilot, gazebo or last-letter' >&2; exit 2 ;;
esac
bytes=$(du -s -B1 "$sim_root" | cut -f1)
[[ "$bytes" -lt 10737418240 ]] || { echo '10 GiB source/build ceiling exceeded' >&2; exit 1; }
printf 'Owned source/build bytes: %s\n' "$bytes"
