#!/usr/bin/env bash
# Tear down everything the simulation launch files start: ros2 launch, PX4 SITL, Gazebo
# (server and GUI), the Micro XRCE-DDS Agent, this workspace's ROS nodes, the camera bridge,
# and the gnome-terminal windows the launch files open.
#
# Usage: sim/kill_sim.sh
#
# Windows are found by DRONE_SIM_WINDOW=1, which the launch files export inside each window.
# Their shells end in `exec bash`, an interactive bash that ignores SIGTERM, so they get SIGHUP.

set -u

PATTERNS=(
    'ros2 launch (px4_bringup|px4_offboard)'
    'ros2 run px4_offboard'
    '/lib/px4_offboard/'                # every px4_offboard node
    'make px4_sitl'
    'px4_sitl_default/bin/px4'
    'gz sim'
    'MicroXRCEAgent'
    '/lib/ros_gz_bridge/parameter_bridge'
    '/lib/joy/joy_node'
)

# Never signal this script or the shells it runs in (e.g. when run from a sim window).
declare -A SPARE
pid=$$
while [[ "$pid" -gt 1 ]]; do
    SPARE[$pid]=1
    pid=$(ps -o ppid= -p "$pid" | tr -d ' ')
done

unspared() {  # filter stdin PIDs, dropping SPARE
    local p
    sort -u | while read -r p; do [[ -z "${SPARE[$p]:-}" ]] && echo "$p"; done
}

matching() {  # PIDs of every pattern match
    local p
    for p in "${PATTERNS[@]}"; do pgrep -f -- "$p"; done | unspared
}

sim_windows() {  # shells started by gnome-terminal that carry the marker
    local p
    for p in $(pgrep -P "$(pgrep -d, -x gnome-terminal-)" -x bash 2>/dev/null | unspared); do
        tr '\0' '\n' < "/proc/$p/environ" 2>/dev/null | grep -qx 'DRONE_SIM_WINDOW=1' && echo "$p"
    done
}

signal() {  # signal <SIG> <pids...>
    local sig="$1"; shift
    [[ $# -gt 0 ]] && kill -s "$sig" "$@" 2>/dev/null
}

# 1. Ctrl-C the launch files first, so they shut their own nodes down.
signal INT $(pgrep -f 'ros2 launch (px4_bringup|px4_offboard)' | unspared)
sleep 1

# 2. Everything else, and the windows.
signal TERM $(matching)
signal HUP $(sim_windows)

# 3. Wait up to 5 s, then SIGKILL whatever is left.
for _ in $(seq 10); do
    [[ -z "$(matching)$(sim_windows)" ]] && break
    sleep 0.5
done
left="$(matching) $(sim_windows)"
if [[ -n "${left// }" ]]; then
    echo "Force-killing:"; ps -o pid=,args= -p $left | cut -c1-120
    signal KILL $left
fi

echo "Simulation stopped."
