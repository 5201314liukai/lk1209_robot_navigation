#!/bin/bash

# ==================================================
# LK1209 ROBOT AUTONOMOUS NAVIGATION
# ROS2 Humble + Gazebo Harmonic + Nav2
#
# Usage:
#   ./start_robot.sh slam
#   ./start_robot.sh slam --world turtlebot3_house
#   ./start_robot.sh navigation /path/to/map.yaml [--world turtlebot3_house]
# ==================================================

set -Eeo pipefail

# ==================================================
# 1. Project paths
# ==================================================

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/ros2_ws"
HARMONIC_WS="${HARMONIC_WS:-$HOME/ros_gz_harmonic_ws}"

WORLD_KEY=robot_world
NAV_PROFILE=classic
LIDAR_MODE=2d
LOG_DIR="$ROOT/simulation/logs"

NAV_PKG="$WS/src/robot_navigation"
DESC_PKG="$WS/src/robot_description"

usage()
{
    echo "Usage: $0 [slam | navigation MAP.yaml] [--world WORLD] [--nav-profile classic|smac_rpp|lk_astar] [--lidar 2d|3d]"
    echo "Default: slam --world robot_world"
    echo "Example: $0 slam --world turtlebot3_house"
}

positional=()
while (( $# )); do
    case "$1" in
        --world)
            if [[ ! ${2:-} =~ ^[a-zA-Z0-9_-]+$ || ${2:-} == -* ]]; then
                echo "Expected a world file name without a directory or .sdf after --world." >&2
                usage >&2
                exit 2
            fi
            WORLD_KEY=$2
            shift 2
            ;;
        --nav-profile)
            case "${2:-}" in classic|smac_rpp|lk_astar) NAV_PROFILE=$2 ;; *) echo "Expected classic, smac_rpp or lk_astar." >&2; exit 2 ;; esac
            shift 2 ;;
        --lidar)
            case "${2:-}" in 2d|3d) LIDAR_MODE=$2 ;; *) echo "Expected 2d or 3d after --lidar." >&2; exit 2 ;; esac
            shift 2 ;;
        -h|--help) usage; exit 0 ;;
        -*) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
        *) positional+=("$1"); shift ;;
    esac
done

MODE=${positional[0]:-slam}
if [[ "$MODE" == slam && "$NAV_PROFILE" != classic ]]; then
    echo "--nav-profile $NAV_PROFILE requires navigation mode." >&2
    exit 2
fi
MAP_FILE=${positional[1]:-}
WORLD="$ROOT/simulation/worlds/$WORLD_KEY.sdf"
SPAWN_X=0
SPAWN_Y=0
if [[ "$WORLD_KEY" == "turtlebot3_house" ]]; then
    # Spawn inside the main room, facing the doorway to the east room.
    SPAWN_X=-2.0
    SPAWN_Y=0.45
fi

export GZ_VERSION=harmonic

# ==================================================
# 2. Process IDs
# ==================================================

GZ_PID=""
GUI_PID=""
ROBOT_PID=""
BRIDGE_PID=""
CLOUD_PID=""
GUARD_PID=""
MONITOR_PID=""
NAV_PID=""

# ==================================================
# 3. Logging
# ==================================================

info()
{
    echo -e "\033[32m[INFO]\033[0m $1"
}

error()
{
    echo -e "\033[31m[ERROR]\033[0m $1"
}

# ==================================================
# 4. Cleanup
# ==================================================

cleanup()
{
    trap - EXIT HUP INT TERM
    # A GUI closed long ago may already have released its PID.
    if [[ -n "$GUI_PID" ]] && [[ $(ps -o ppid= -p "$GUI_PID" | tr -d " ") != "$$" ]]; then
        GUI_PID=""
    fi

    echo ""
    info "Stopping LK1209 navigation system..."

    for PID in "$NAV_PID" "$MONITOR_PID" "$CLOUD_PID" "$BRIDGE_PID" "$GUARD_PID" "$ROBOT_PID" "$GUI_PID" "$GZ_PID"
    do
        if [ -n "$PID" ]; then
            kill -INT -- "-$PID" 2>/dev/null || true
        fi
    done

    sleep 3

    for PID in "$NAV_PID" "$MONITOR_PID" "$CLOUD_PID" "$BRIDGE_PID" "$GUARD_PID" "$ROBOT_PID" "$GUI_PID" "$GZ_PID"
    do
        if [ -n "$PID" ]; then
            kill -TERM -- "-$PID" 2>/dev/null || true
        fi
    done

    info "Shutdown completed."
}

# ==================================================
# 5. Check mode and files
# ==================================================

info "Checking project files..."

case "$MODE" in
    slam)       expected_args=1 ;;
    navigation) expected_args=2 ;;
    *) error "Unknown mode: $MODE"; usage >&2; exit 2 ;;
esac
if (( ${#positional[@]} > expected_args )); then
    error "Too many arguments."
    usage >&2
    exit 2
fi

if [ ! -f "$WORLD" ]; then
    error "Gazebo world not found: $WORLD"
    exit 1
fi

WORLD_NAME=$(python3 - "$WORLD" <<'PY_WORLD'
import sys
import xml.etree.ElementTree as ET

try:
    worlds = ET.parse(sys.argv[1]).getroot().findall("world")
except (OSError, ET.ParseError) as exc:
    sys.exit(f"Cannot read {sys.argv[1]}: {exc}")
if len(worlds) != 1 or not worlds[0].get("name"):
    sys.exit(f"Expected one named <world> in {sys.argv[1]}")
print(worlds[0].get("name"))
PY_WORLD
)

if [ ! -f "$DESC_PKG/package.xml" ]; then
    error "robot_description package not found."
    exit 1
fi

if [ ! -f "$NAV_PKG/package.xml" ]; then
    error "robot_navigation package not found."
    exit 1
fi

if [ "$MODE" = "navigation" ]; then

    if [ -z "$MAP_FILE" ] || [ ! -f "$MAP_FILE" ]; then
        error "Navigation mode requires an existing map YAML."
        exit 1
    fi
    MAP_FILE=$(realpath -- "$MAP_FILE")

fi

mkdir -p "$LOG_DIR"

# ==================================================
# 6. Load environments
# ==================================================

info "Loading ROS2 Humble..."

source /opt/ros/humble/setup.bash

info "Loading Gazebo Harmonic..."

source "$HARMONIC_WS/install/setup.bash"

# ==================================================
# 7. Build project packages
# ==================================================

# Both modes publish the same ROS topics. Serialize startup within one ROS domain.
lock_dir="/run/user/$UID"
[[ -d "$lock_dir" && -w "$lock_dir" ]] || lock_dir=/tmp
exec 9>"$lock_dir/lk1209-sim-${UID}-${ROS_DOMAIN_ID:-0}.lock"
if ! flock -n 9; then
    error "LK1209 is already starting or running in this ROS domain."
    exit 1
fi
if gz service -l 2>/dev/null | grep -Fx /gazebo/worlds >/dev/null; then
    error "A Gazebo server already exists in this transport partition. Close that session before starting another."
    exit 1
fi

info "Building robot_description and robot_navigation..."

cd "$WS"

colcon build \
    --packages-up-to robot_description robot_navigation \
    --parallel-workers 1

source "$WS/install/setup.bash"

if [[ "$WORLD_KEY" == "turtlebot3_house" ]]; then
    HOUSE_MODELS="$ROOT/simulation/vendor/turtlebot3_house/models"
    if [[ ! -r "$HOUSE_MODELS/turtlebot3_house/model.sdf" ]]; then
        error "TurtleBot3 House model missing: $HOUSE_MODELS/turtlebot3_house/model.sdf"
        exit 1
    fi
    export GZ_SIM_RESOURCE_PATH="$HOUSE_MODELS${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
    export SDF_PATH="$HOUSE_MODELS${SDF_PATH:+:$SDF_PATH}"
fi

# ==================================================
# 8. Check Nav2 packages
# ==================================================

for PKG in \
    ros_gz_sim \
    ros_gz_bridge \
    nav2_bringup \
    slam_toolbox
do
    if ! ros2 pkg prefix "$PKG" >/dev/null 2>&1; then
        error "Package unavailable: $PKG"
        exit 1
    fi
done

# ==================================================
# 9. Check Gazebo world
# ==================================================

info "Checking Gazebo world..."

gz sdf -k "$WORLD"

# ==================================================
# 10. Start Gazebo
# ==================================================

trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

info "Starting Gazebo Harmonic ($WORLD_NAME)..."

desktop=true
gz_flags=(-s -r)
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    desktop=false
    gz_flags+=(--headless-rendering)
fi
setsid gz sim "${gz_flags[@]}" "$WORLD" \
    > "$LOG_DIR/gazebo.log" 2>&1 9>&- &

GZ_PID=$!

READY=0

for i in {1..60}
do

    if ! kill -0 "$GZ_PID" 2>/dev/null; then
        error "Gazebo exited unexpectedly."
        tail -n 40 "$LOG_DIR/gazebo.log"
        exit 1
    fi

    if gz service -l 2>/dev/null | grep -Fx \
        "/world/$WORLD_NAME/create" >/dev/null; then
        READY=1
        break
    fi

    sleep 1

done

if [ "$READY" -ne 1 ]; then
    error "Gazebo startup timeout."
    exit 1
fi

info "Gazebo started."

# ==================================================
# 11. Start robot description and RViz2
# ==================================================

info "Starting robot description and RViz2..."

setsid ros2 launch robot_description robot_display.launch.py \
    lidar_mode:="$LIDAR_MODE" rviz:="$desktop" \
    > "$LOG_DIR/robot.log" 2>&1 9>&- &

ROBOT_PID=$!

READY=0

for i in {1..40}
do

    if ! kill -0 "$ROBOT_PID" 2>/dev/null; then
        error "Robot description process exited."
        tail -n 40 "$LOG_DIR/robot.log"
        exit 1
    fi

    INFO=$(ros2 topic info /robot_description 2>/dev/null || true)

    if echo "$INFO" | grep -q "Publisher count: 1"; then
        READY=1
        break
    fi

    sleep 1

done

if [ "$READY" -ne 1 ]; then
    error "Robot description unavailable."
    exit 1
fi

# ==================================================
# 12. Spawn robot
# ==================================================

info "Spawning LK1209 robot..."

ros2 run ros_gz_sim create \
    -world "$WORLD_NAME" \
    -topic /robot_description \
    -name lk1209_robot \
    -x "$SPAWN_X" -y "$SPAWN_Y" -z 0.10

PYTHONNOUSERSITE=1 /usr/bin/python3 "$ROOT/simulation/gazebo_view.py" --world "$WORLD_NAME"
info "Robot spawned and visible in the Gazebo scene."

# A newly connected GUI receives a full snapshot including the spawned robot.
if $desktop; then
    info "Opening Gazebo GUI..."
    setsid gz sim -g > "$LOG_DIR/gazebo_gui.log" 2>&1 9>&- &
    GUI_PID=$!
    if ! PYTHONNOUSERSITE=1 /usr/bin/python3 "$ROOT/simulation/gazebo_view.py" --spawn "$SPAWN_X" "$SPAWN_Y" --gui-pid "$GUI_PID"; then
        echo "[WARN] Gazebo camera setup could not be verified; see $LOG_DIR/gazebo_gui.log" >&2
    fi
    if ! kill -0 "$GUI_PID" 2>/dev/null; then
        echo "[WARN] Gazebo GUI is closed. Reconnect with: gz sim -g" >&2
        GUI_PID=""
    fi
fi

# ==================================================
# 13. Check Gazebo LiDAR
# ==================================================

info "Checking Gazebo LiDAR..."

lidar_topic=/scan
lidar_type=gz.msgs.LaserScan
lidar_bridge='/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
if [[ "$LIDAR_MODE" == 3d ]]; then
    lidar_topic=/lidar/points
    lidar_type=gz.msgs.PointCloudPacked
    lidar_bridge='/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
fi
READY=0

for i in {1..30}
do

    SCAN_INFO=$(gz topic -i -t "$lidar_topic" 2>/dev/null || true)

    if echo "$SCAN_INFO" | grep -Fq "$lidar_type"; then
        READY=1
        break
    fi

    sleep 1

done

if [ "$READY" -ne 1 ]; then
    error "Gazebo LiDAR unavailable."
    exit 1
fi


# ==================================================
# 14. Start ROS2-Gazebo bridges
# ==================================================

info "Starting simulator velocity guard..."
GUARD_CONFIG="$(ros2 pkg prefix --share robot_navigation)/config/nav2_params.yaml"
guard_args=()
if [[ "$MODE" == "navigation" ]]; then
    monitor_config="$(ros2 pkg prefix --share robot_navigation)/config/sim_collision_monitor.yaml"
    guard_args=(--ros-args --params-file "$monitor_config")
fi
setsid /usr/bin/python3 "$ROOT/simulation/sim_cmd_vel_guard.py" "$GUARD_CONFIG" "${guard_args[@]}" \
    > "$LOG_DIR/cmd_vel_guard.log" 2>&1 9>&- &
GUARD_PID=$!

bridge_velocity=/lk1209/sim/cmd_vel_checked
if [[ "$MODE" == "navigation" ]]; then
    bridge_velocity=/lk1209/sim/cmd_vel_safe
    monitor_exec="$(ros2 pkg prefix nav2_collision_monitor)/lib/nav2_collision_monitor/collision_monitor"
    monitor_args=(--ros-args --params-file "$monitor_config")
    if [[ "$LIDAR_MODE" == 3d ]]; then
        monitor_args+=(-p 'observation_sources:=[cloud]' -p cloud.type:=pointcloud
            -p cloud.topic:=/lidar/obstacles -p cloud.min_height:=-0.05 -p cloud.max_height:=0.35)
    fi
    setsid "$monitor_exec" "${monitor_args[@]}" \
        > "$LOG_DIR/collision_monitor.log" 2>&1 9>&- &
    MONITOR_PID=$!
fi

info "Starting ROS2-Gazebo bridges..."


setsid ros2 run ros_gz_bridge parameter_bridge \
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
    '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
    "$lidar_bridge" \
    '/model/lk1209_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
    '/model/lk1209_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' \
    --ros-args \
    -r "/cmd_vel:=$bridge_velocity" \
    -r /model/lk1209_robot/odometry:=/odom \
    -r /model/lk1209_robot/tf:=/tf \
    > "$LOG_DIR/bridge.log" 2>&1 9>&- &

BRIDGE_PID=$!

if [[ "$LIDAR_MODE" == 3d ]]; then
    cloud_exec="$(ros2 pkg prefix obstacle_detection)/lib/obstacle_detection/cloud_obstacles"
    setsid "$cloud_exec" --ros-args -p use_sim_time:=true \
        > "$LOG_DIR/lidar_cloud.log" 2>&1 9>&- &
    CLOUD_PID=$!
fi

sleep 3

if ! kill -0 "$GUARD_PID" 2>/dev/null; then
    error "Simulator velocity guard failed."
    tail -n 20 "$LOG_DIR/cmd_vel_guard.log"
    exit 1
fi

if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
    error "ROS-Gazebo bridge failed."
    cat "$LOG_DIR/bridge.log"
    exit 1
fi

if [[ -n "$MONITOR_PID" ]]; then
    # A lost DDS service reply can follow a successful transition. Read back
    # the lifecycle state before issuing another transition.
    for attempt in {1..8}; do
        state=$(timeout 7 ros2 lifecycle get /collision_monitor 2>/dev/null) || continue
        case "$state" in
            'active [3]') break ;;
            'unconfigured [1]') transition=1 ;;
            'inactive [2]') transition=3 ;;
            *) sleep 1; continue ;;
        esac
        timeout 8 ros2 service call /collision_monitor/change_state \
            lifecycle_msgs/srv/ChangeState "{transition: {id: $transition}}" >/dev/null || true
    done
    if [[ "${state:-}" != 'active [3]' ]]; then
        error "Collision monitor did not reach active state."
        exit 1
    fi
fi
info "ROS2-Gazebo bridges started."

# ==================================================
# 15. Start SLAM or Navigation
# ==================================================

if [ "$MODE" = "slam" ]; then

    info "Starting SLAM Toolbox..."

    setsid ros2 launch robot_navigation slam.launch.py \
        lidar_mode:="$LIDAR_MODE" \
        > "$LOG_DIR/slam.log" 2>&1 9>&- &

    NAV_PID=$!

    info "SLAM mapping mode started."

else

    info "Starting Nav2 localization and navigation..."

    setsid ros2 launch robot_navigation navigation.launch.py \
        map:="$MAP_FILE" \
        world:="$WORLD_KEY" \
        nav_profile:="$NAV_PROFILE" \
        lidar_mode:="$LIDAR_MODE" \
        > "$LOG_DIR/navigation.log" 2>&1 9>&- &

    NAV_PID=$!

    info "Nav2 navigation mode started."

fi

# ==================================================
# 16. Startup information
# ==================================================

echo ""
echo "================================================"
echo "          LK1209 ROBOT SYSTEM"
echo "================================================"

info "Gazebo Harmonic       : STARTED"
if [[ -n "$GUI_PID" ]] && kill -0 "$GUI_PID" 2>/dev/null; then
    info "Gazebo GUI            : OPEN"
else
    info "Gazebo GUI            : CLOSED (gz sim -g to reconnect)"
fi
info "Robot Model           : LOADED"
if $desktop; then info "RViz2                 : STARTED"; fi
info "LiDAR                 : $LIDAR_MODE ($lidar_topic)"
info "Odometry              : /odom"
info "Gazebo TF             : /tf"
info "Velocity              : /cmd_vel"
info "Navigation Mode       : $MODE"
info "World                 : $WORLD_NAME ($WORLD)"

if [ "$MODE" = "navigation" ]; then
    info "Map                   : $MAP_FILE"
fi

echo "------------------------------------------------"
echo "Logs: $LOG_DIR"
echo "================================================"
echo "Press Ctrl+C to stop the system."

# ==================================================
# 17. Keep system running
# ==================================================

# Loss of the checked command path must also shut down the simulation.
control_pids=("$GUARD_PID" "$BRIDGE_PID")
if [[ -n "$MONITOR_PID" ]]; then control_pids+=("$MONITOR_PID"); fi
if [[ -n "$CLOUD_PID" ]]; then control_pids+=("$CLOUD_PID"); fi
for PID in "${control_pids[@]}"; do
    if ! kill -0 "$PID" 2>/dev/null; then
        error "The simulator velocity command path exited during startup."
        exit 1
    fi
done
status=0
wait -n -p stopped_pid "$GZ_PID" "${control_pids[@]}" || status=$?
if [[ ${stopped_pid:-} != "$GZ_PID" ]]; then
    error "The simulator velocity command path exited; stopping the simulation."
    exit 1
fi
exit "$status"

