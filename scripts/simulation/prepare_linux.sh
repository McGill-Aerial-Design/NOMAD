#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Source acquisition only, in a dedicated owned directory; no system installation.
set -euo pipefail
sim_root=${1:-"$HOME/nomad-simulation"}
mode=${2:---prepare}
case "$mode" in --prepare|--check) ;; *) echo 'Expected --prepare or --check' >&2; exit 2;; esac
[[ "$sim_root" = /* && "$sim_root" != / && ! -L "$sim_root" ]] || {
    echo 'An absolute non-symlink workspace is required' >&2; exit 2;
}
if [[ ! -f "$sim_root/.nomad-simulation-owned" ]]; then
    [[ "$mode" = --prepare ]] || { echo 'Owned workspace missing' >&2; exit 1; }
    if [[ -d "$sim_root" && -n "$(ls -A "$sim_root")" ]]; then
        echo 'Refusing a nonempty unowned workspace' >&2; exit 1
    fi
    mkdir -p "$sim_root"
    printf 'NOMAD simulation sources v1\n' > "$sim_root/.nomad-simulation-owned"
fi
[[ $(cat "$sim_root/.nomad-simulation-owned") = 'NOMAD simulation sources v1' ]] || exit 1
sim_root=$(realpath "$sim_root")
[[ $(du -s -B1 "$sim_root" | cut -f1) -lt 10737418240 ]] || {
    echo '10 GiB source/build ceiling exceeded' >&2; exit 1;
}
for source in sources sources/ardupilot{,/.git} sources/ardupilot_gazebo{,/.git} sources/last_letter{,/.git} \
    sources/yaml_cpp{,/.git} sources/data_tamer{,/.git} sources/nlopt{,/.git}; do
    resolved=$(realpath -m -- "$sim_root/$source")
    [[ "$resolved" = "$sim_root/"* ]] || {
        echo "Source path escapes the owned workspace: $source" >&2; exit 1;
    }
done

checkout() {
    local name=$1 url=$2 revision=$3
    local directory="$sim_root/sources/$name"
    if [[ "$mode" = --prepare ]]; then
        if [[ ! -d "$directory/.git" ]]; then
            [[ ! -e "$directory" ]] || { echo "Unowned source path: $name" >&2; exit 1; }
            git init -q "$directory"
            git -C "$directory" remote add origin "$url"
        fi
        [[ $(git -C "$directory" remote get-url origin) = "$url" ]] || exit 1
        [[ -z $(git -C "$directory" status --porcelain) ]] || {
            echo "Source has local changes: $name" >&2; exit 1;
        }
        if ! git -C "$directory" cat-file -e "$revision^{commit}" 2>/dev/null; then
            git -C "$directory" fetch --depth=1 origin "$revision"
        fi
        git -C "$directory" checkout -q --detach "$revision"
    fi
    [[ $(git -C "$directory" rev-parse HEAD) = "$revision" ]] || exit 1
    [[ -z $(git -C "$directory" status --porcelain) ]] || exit 1
    printf '%s %s\n' "$name" "$revision"
}

checkout ardupilot https://github.com/ArduPilot/ardupilot.git fb773567896014da007bca58358704d4a0d8516f
checkout ardupilot_gazebo https://github.com/ArduPilot/ardupilot_gazebo.git 80dc9664dda7efd6c5d1a71a79198d1d6b708c0e
checkout last_letter https://github.com/Georacer/last_letter.git 13baa815e971abbcc568bea1e265026b44686273
checkout yaml_cpp https://github.com/jbeder/yaml-cpp.git 56e3bb550c91fd7005566f19c079cb7a503223cf
checkout data_tamer https://github.com/PickNikRobotics/data_tamer.git ae41fff484dcda1ad96527c003ecef3f787067e6
checkout nlopt https://github.com/stevengj/nlopt.git 88c424d4f458412787df96fcc95218acbca224fd

submodules=(modules/waf modules/mavlink modules/DroneCAN/DSDL modules/DroneCAN/dronecan_dsdlc
    modules/DroneCAN/libcanard modules/DroneCAN/pydronecan modules/littlefs modules/lwip)
if [[ "$mode" = --prepare ]]; then
    git -C "$sim_root/sources/ardupilot" submodule update --init --recursive --depth=1 -- "${submodules[@]}"
fi
state=$(git -C "$sim_root/sources/ardupilot" submodule status --recursive -- "${submodules[@]}")
if printf '%s\n' "$state" | grep -qE '^[-+U]'; then
    echo 'Required submodule is absent or differs from its reviewed gitlink' >&2; exit 1
fi
printf '%s\n' "$state"
printf 'Owned source bytes: %s\n' "$(du -s -B1 "$sim_root" | cut -f1)"
