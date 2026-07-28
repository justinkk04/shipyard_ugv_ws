#!/usr/bin/env bash
set -e

# ============================================================
# Shipyard UGV startup script - U-shaped mission worlds
# Usage:
#   ./start_shipyard_stack_u.sh          # normal U-shaped route
#   ./start_shipyard_stack_u.sh blocked  # blocked-route safe-stop test
# ============================================================

MODE="${1:-normal}"
WS="$HOME/shipyard_ugv_ws"
MODEL="$WS/src/models/shipyard_rover/model.sdf"

if [ "$MODE" = "blocked" ]; then
    WORLD="$WS/src/worlds/shipyard_corridor_u_blocked_world.sdf"
    WORLD_NAME="shipyard_corridor_u_blocked"
else
    WORLD="$WS/src/worlds/shipyard_corridor_u_custom.sdf"
    WORLD_NAME="shipyard_corridor_u"
fi

# Start zone is centered at (-5, -3). Robot faces +x by default.
SPAWN_X="-5.0"
SPAWN_Y="-3.0"
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
echo "Starting Shipyard UGV U-route simulation"
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

echo "Starting ROS/Gazebo bridge..."
ros2 run ros_gz_bridge parameter_bridge \
    /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
    /odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry \
    /scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan \
    /imu@sensor_msgs/msg/Imu@ignition.msgs.IMU
