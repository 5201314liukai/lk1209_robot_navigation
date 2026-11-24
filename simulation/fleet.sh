#!/usr/bin/env bash
# Run both LK1209s in the same house. Press Ctrl+C to stop the fleet.
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
gui=false
nav_profile=classic
while (( $# )); do
    case "$1" in
        --gui) gui=true; shift ;;
        --nav-profile)
            case "${2:-}" in classic|smac_rpp|lk_astar) nav_profile=$2 ;; *) echo "Expected classic, smac_rpp or lk_astar." >&2; exit 2 ;; esac
            shift 2 ;;
        --help|-h) echo "Usage: bash simulation/fleet.sh [--gui] [--nav-profile classic|smac_rpp|lk_astar]"; exit 0 ;;
        *) echo "Unknown fleet option: $1" >&2; exit 2 ;;
    esac
done

source /opt/ros/humble/setup.bash
gz_ws=${ROS_GZ_WS:-"$HOME/ros_gz_harmonic_ws"}
if [[ -f "$gz_ws/install/setup.bash" ]]; then source "$gz_ws/install/setup.bash"; fi
export GZ_VERSION=harmonic PYTHONNOUSERSITE=1
lock_dir=/run/user/$UID
[[ -d "$lock_dir" && -w "$lock_dir" ]] || lock_dir=/tmp
# Share the singleton's lock: two /clock providers cannot share one ROS domain.
exec 9>"$lock_dir/lk1209-sim-${UID}-${ROS_DOMAIN_ID:-0}.lock"
flock -n 9 || { echo "An LK1209 simulation is using this ROS domain." >&2; exit 1; }
if gz service -l | grep -Fx /gazebo/worlds >/dev/null; then
    echo "This Gazebo partition already contains a server." >&2; exit 1
fi
cd "$root/ros2_ws"
colcon build --packages-up-to robot_navigation robot_description --parallel-workers 1
source install/setup.bash
nav_share=$(ros2 pkg prefix --share robot_navigation)
description_share=$(ros2 pkg prefix --share robot_description)
models="$root/simulation/vendor/turtlebot3_house/models"
export GZ_SIM_RESOURCE_PATH="$models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export SDF_PATH="$models${SDF_PATH:+:$SDF_PATH}"
logs="$root/simulation/logs/fleet"
mkdir -p "$logs"
run=$(mktemp -d "${TMPDIR:-/tmp}/lk1209-fleet.XXXXXXXX")
pids=()
cleanup() {
    trap - EXIT INT TERM HUP
    for signal in INT TERM KILL; do
        for pid in "${pids[@]}"; do
            kill -"$signal" -- "-$pid" 2>/dev/null || true
        done
        [[ "$signal" == KILL ]] || sleep 2
    done
    wait || true
    rm -f -- "$run"/robot{1,2}{.urdf,_nav.yaml,_safety.yaml}
    rmdir "$run" || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

gz_flags=(-s -r)
if [[ -z "${DISPLAY:-}" ]]; then gz_flags+=(--headless-rendering); fi
setsid gz sim "${gz_flags[@]}" "$root/simulation/worlds/turtlebot3_house.sdf" \
    9>&- >"$logs/gazebo.log" 2>&1 &
server=$!
pids+=("$server")
for ((i=0;i<60;i++)); do
    kill -0 "$server" 2>/dev/null || { tail "$logs/gazebo.log" >&2; exit 1; }
    if gz service -l | grep -Fx /world/default/create >/dev/null; then break; fi
    sleep 1
done
/usr/bin/python3 "$root/simulation/fleet_model.py" "$run" "$nav_share/config" \
    "$description_share/urdf/robot.urdf.xacro"
if $gui; then
    setsid gz sim -g 9>&- >"$logs/gui.log" 2>&1 &
    pids+=("$!")
fi
setsid ros2 launch robot_navigation fleet_navigation.launch.py project_root:="$root" runtime_dir:="$run" nav_profile:="$nav_profile" \
    9>&- >"$logs/navigation.log" 2>&1 &
navigation=$!
pids+=("$navigation")
echo "Starting $nav_profile fleet navigation for robot1 and robot2. Log: $logs/navigation.log"
wait -n "$server" "$navigation" || exit $?
# Both processes must stay alive while a fleet session is running.
echo "Fleet navigation or the Gazebo server exited; stopping the session." >&2
exit 1
