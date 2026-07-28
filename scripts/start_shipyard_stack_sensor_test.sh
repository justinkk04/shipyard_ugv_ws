#!/usr/bin/env bash
set -e

# ============================================================
# Shipyard UGV startup script - V2 mission worlds
# Starts Gazebo, spawns rover, and bridges ROS 2 / Gazebo topics.
# Usage:
#   ./start_shipyard_stack.sh                 # default clear-route world
#   ./start_shipyard_stack.sh avoidable       # avoidable-obstacle world
#   ./start_shipyard_stack.sh blocked         # blocked-route safe-stop world
# ============================================================

MODE="${1:-clear}"
WS="$HOME/shipyard_ugv_ws"

MODEL="$WS/src/models/shipyard_rover/model.sdf"

if [ "$MODE" = "blocked" ]; then

    WORLD="$WS/src/worlds/shipyard_corridor_blocked_world.sdf"
    WORLD_NAME="shipyard_corridor_blocked_world"

elif [ "$MODE" = "avoidable" ]; then

    WORLD="$WS/src/worlds/shipyard_corridor_avoidable_world.sdf"
    WORLD_NAME="shipyard_corridor_avoidable_world"

elif [ "$MODE" = "clear" ]; then

    WORLD="$WS/src/worlds/shipyard_corridor_clear_world.sdf"
    WORLD_NAME="shipyard_corridor_clear_world"

else

    echo "ERROR: Unknown mode: $MODE"
    echo "Use one of: clear, avoidable, blocked"
    exit 1
    
fi

SPAWN_X="-4.0"
SPAWN_Y="0.0"
SPAWN_Z="0.25"

cleanup() {
    echo ""
    echo "Stopping Shipyard UGV stack..."
    if [ -n "${GAZEBO_PID:-}" ] && kill -0 "$GAZEBO_PID" 2>/dev/null; then
        kill "$GAZEBO_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "============================================"
echo "Starting Shipyard UGV simulation"
echo "Mode:       $MODE"
echo "World file: $WORLD"
echo "Model file: $MODEL"
echo "World name: $WORLD_NAME"
echo "Spawn pose: x=$SPAWN_X, y=$SPAWN_Y, z=$SPAWN_Z"
echo "============================================"

if [ ! -f "$WORLD" ]; then
    echo "ERROR: World file not found: $WORLD"
    exit 1
fi

if [ ! -f "$MODEL" ]; then
    echo "ERROR: Model file not found: $MODEL"
    echo "Run: find ~/shipyard_ugv_ws/src -name model.sdf"
    exit 1
fi

source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi

echo "Starting Gazebo..."
ign gazebo -r "$WORLD" &
GAZEBO_PID=$!

sleep 5

echo "Spawning rover into world: $WORLD_NAME"
ign service -s "/world/$WORLD_NAME/create" \
    --reqtype ignition.msgs.EntityFactory \
    --reptype ignition.msgs.Boolean \
    --timeout 300 \
    --req "sdf_filename: \"$MODEL\", name: \"shipyard_rover\", pose: {position: {x: $SPAWN_X, y: $SPAWN_Y, z: $SPAWN_Z}}"

echo "Rover spawn request sent."
sleep 2

echo "Starting ROS/Gazebo bridges separately..."

ros2 run ros_gz_bridge parameter_bridge \
    /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist &
CMD_BRIDGE_PID=$!

ros2 run ros_gz_bridge parameter_bridge \
    /odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry &
ODOM_BRIDGE_PID=$!

ros2 run ros_gz_bridge parameter_bridge \
    /scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan &
SCAN_BRIDGE_PID=$!

ros2 run ros_gz_bridge parameter_bridge \
    /imu@sensor_msgs/msg/Imu@ignition.msgs.IMU &
IMU_BRIDGE_PID=$!

echo "Bridge PIDs:"
echo "cmd_vel bridge: $CMD_BRIDGE_PID"
echo "odom bridge:    $ODOM_BRIDGE_PID"
echo "scan bridge:    $SCAN_BRIDGE_PID"
echo "imu bridge:     $IMU_BRIDGE_PID"

echo ""
echo "To simulate LiDAR failure later, run:"
echo "kill $SCAN_BRIDGE_PID"

wait