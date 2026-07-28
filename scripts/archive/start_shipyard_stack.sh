#!/usr/bin/env bash

set -e

# ============================================================
# Shipyard UGV startup script
# Starts Gazebo corridor world, spawns rover, and bridges topics
# ============================================================

# Choose world mode:
#   normal  = avoidable obstacle corridor
#   blocked = blocked-route safe-stop test
MODE="${1:-normal}"

WS="$HOME/shipyard_ugv_ws"

# Your current folder structure
MODEL="$WS/src/models/shipyard_rover/model.sdf"

if [ "$MODE" = "blocked" ]; then
    WORLD="$WS/src/worlds/shipyard_corridor_blocked_world.sdf"
    WORLD_NAME="shipyard_corridor_blocked"
else
    WORLD="$WS/src/worlds/shipyard_corridor_world.sdf"
    WORLD_NAME="shipyard_corridor"
fi

echo "============================================"
echo "Starting Shipyard UGV simulation"
echo "Mode:       $MODE"
echo "World file: $WORLD"
echo "Model file: $MODEL"
echo "World name: $WORLD_NAME"
echo "============================================"

# Check files exist
if [ ! -f "$WORLD" ]; then
    echo "ERROR: World file not found:"
    echo "$WORLD"
    exit 1
fi

if [ ! -f "$MODEL" ]; then
    echo "ERROR: Model file not found:"
    echo "$MODEL"
    echo
    echo "Run this to find your model:"
    echo "find ~/shipyard_ugv_ws/src -name model.sdf"
    exit 1
fi

# Source ROS 2
source /opt/ros/humble/setup.bash

if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi

# Start Gazebo
echo "Starting Gazebo..."
ign gazebo -r "$WORLD" &

GAZEBO_PID=$!

# Give Gazebo time to load
sleep 5

# Spawn rover
echo "Spawning rover into world: $WORLD_NAME"

ign service -s "/world/$WORLD_NAME/create" \
    --reqtype ignition.msgs.EntityFactory \
    --reptype ignition.msgs.Boolean \
    --timeout 300 \
    --req "sdf_filename: \"$MODEL\", name: \"shipyard_rover\", pose: {position: {x: -4.0, y: 0.0, z: 0.25}}"

echo "Rover spawn request sent."

sleep 2

# Start bridge
echo "Starting ROS/Gazebo bridge..."

ros2 run ros_gz_bridge parameter_bridge \
    /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
    /odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry \
    /scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan \
    /imu@sensor_msgs/msg/Imu@ignition.msgs.IMU

# If bridge exits, stop Gazebo too
kill $GAZEBO_PID