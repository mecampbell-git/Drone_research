#!/usr/bin/env bash
# Symlink this repo's Gazebo models and PX4 airframes into a PX4-Autopilot checkout,
# so the repo is the single source of truth and PX4 itself stays a stock checkout.
#
# Usage: sim/link_into_px4.sh [path/to/PX4-Autopilot]   (default: ~/PX4-Autopilot)
#
# Existing real files/dirs at a link target are moved aside to <target>.bak-<timestamp>,
# never deleted. Re-running is safe: correct links are left alone.
# Rebuild PX4 afterwards (make px4_sitl) so the airframes are copied into build/.

set -euo pipefail

PX4_DIR="${1:-$HOME/PX4-Autopilot}"
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODELS_DST="$PX4_DIR/Tools/simulation/gz/models"
AIRFRAMES_DST="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
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

# PX4 only installs airframes listed in this CMakeLists, inside the custom-model range.
CMAKE="$AIRFRAMES_DST/CMakeLists.txt"
for airframe in "$SIM_DIR"/airframes/*; do
    name="$(basename "$airframe")"
    if ! grep -q "^[[:space:]]*$name\$" "$CMAKE"; then
        sed -i "s|^\([[:space:]]*\)# \[22000, 22999\] Reserve for custom models|\1$name\n&|" "$CMAKE"
        echo "cmake   added $name to $CMAKE"
    fi
done
