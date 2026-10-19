#!/usr/bin/env bash
# Symlink this repo's Gazebo models and worlds and PX4 airframes into a PX4-Autopilot checkout,
# so the repo is the single source of truth, and apply the fixes in sim/patches. Apart from
# those patches, PX4 stays a stock checkout.
#
# Usage: sim/link_into_px4.sh [path/to/PX4-Autopilot]   (default: ~/PX4-Autopilot)
#
# Existing real files/dirs at a link target are moved aside to <target>.bak-<timestamp>,
# never deleted. Re-running is safe: correct links are left alone.
# Patches already applied are skipped.
# Rebuild PX4 afterwards (make px4_sitl) so the airframes and patched code are built.

set -euo pipefail

PX4_DIR="${1:-$HOME/PX4-Autopilot}"
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODELS_DST="$PX4_DIR/Tools/simulation/gz/models"
AIRFRAMES_DST="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
WORLDS_DST="$PX4_DIR/Tools/simulation/gz/worlds"
STAMP="$(date +%Y%m%d-%H%M%S)"

[[ -d "$MODELS_DST" && -d "$AIRFRAMES_DST" ]] || {
    echo "Not a PX4-Autopilot checkout: $PX4_DIR" >&2; exit 1; }

link() {  # link <source in repo> <target in PX4>
    local src="$1" dst="$2"
    if [[ -L "$dst" && "$(readlink -f "$dst")" == "$(readlink -f "$src")" ]]; then
        echo "ok      $dst"; return
    fi
    if [[ -e "$dst" || -L "$dst" ]]; then
        mv "$dst" "$dst.bak-$STAMP"
        echo "backup  $dst -> $dst.bak-$STAMP"
    fi
    ln -s "$src" "$dst"
    echo "linked  $dst"
}

for model in "$SIM_DIR"/models/*/; do
    model="${model%/}"
    link "$model" "$MODELS_DST/$(basename "$model")"
done

for airframe in "$SIM_DIR"/airframes/*; do
    link "$airframe" "$AIRFRAMES_DST/$(basename "$airframe")"
done

# PX4 loads worlds/<name>.sdf, and Gazebo topics (bridge_config.yaml) use <world name>,
# so the two must match. Skip files where they don't (e.g. scratch saves from the GUI).
for world in "$SIM_DIR"/worlds/*.sdf; do
    [[ -e "$world" ]] || continue
    file_name="$(basename "$world" .sdf)"
    world_name="$(grep -oPm1 "<world name=['\"]\K[^'\"]+" "$world" || true)"
    if [[ "$world_name" != "$file_name" ]]; then
        echo "skip    $(basename "$world"): <world name='$world_name'> does not match the file name" >&2
        continue
    fi
    link "$world" "$WORLDS_DST/$(basename "$world")"
done

# PX4 only installs airframes listed in this CMakeLists, inside the custom-model range.
CMAKE="$AIRFRAMES_DST/CMakeLists.txt"
for airframe in "$SIM_DIR"/airframes/*; do
    name="$(basename "$airframe")"
    if ! grep -q "^[[:space:]]*$name\$" "$CMAKE"; then
        sed -i "s|^\([[:space:]]*\)# \[22000, 22999\] Reserve for custom models|\1$name\n&|" "$CMAKE"
        echo "cmake   added $name to $CMAKE"
    fi
done

# Local fixes to PX4 and its Gazebo models submodule.
apply_patch() {  # apply_patch <git repo> <patch>
    local repo="$1" patch="$2"
    if git -C "$repo" apply --reverse --check "$patch" 2>/dev/null; then
        echo "ok      patch $(basename "$patch")"
    elif git -C "$repo" apply "$patch"; then
        echo "patched $repo with $(basename "$patch")"
    else
        echo "Patch $(basename "$patch") does not apply to $repo" >&2; exit 1
    fi
}

# Gazebo magnetometer: correct frame and units (backport of PX4 PR #28877 and
# PX4-gazebo-models PR #161). Without it the EKF heading is wrong whenever the drone tilts.
apply_patch "$PX4_DIR" "$SIM_DIR/patches/px4-autopilot-gz-magnetometer.patch"
apply_patch "$PX4_DIR/Tools/simulation/gz" "$SIM_DIR/patches/px4-gazebo-models-mag-noise-tesla.patch"
