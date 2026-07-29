#!/bin/bash
# Launch robot_state_publisher + joint_state_publisher + RViz2
# to view the SDF model. Ctrl+C to kill everything.

SDF_FILE="$HOME/shipyard_ugv_ws/src/models/shipyard_rover/model.sdf"

echo "=== Launching RViz Model Viewer ==="
echo "SDF: $SDF_FILE"
echo ""

# Read the SDF file content
ROBOT_DESC=$(cat "$SDF_FILE")

# Launch robot_state_publisher in background
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$ROBOT_DESC" &
RSP_PID=$!
echo "robot_state_publisher PID: $RSP_PID"

sleep 2

# Launch joint_state_publisher in background
ros2 run joint_state_publisher joint_state_publisher &
JSP_PID=$!
echo "joint_state_publisher PID: $JSP_PID"

sleep 1

# Launch RViz2 in foreground
echo ""
echo "Launching RViz2..."
echo "  1. Set Fixed Frame to 'base_footprint'"
echo "  2. Click Add -> RobotModel"
echo "  3. Set Description Topic to /robot_description"
echo ""
rviz2

# When RViz is closed, clean up
echo "Shutting down..."
kill $RSP_PID 2>/dev/null
kill $JSP_PID 2>/dev/null
wait
echo "Done."
