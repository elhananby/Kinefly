#!/bin/bash
set -e

# Setup ROS environment
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/root/catkin/devel/setup.bash"

# Set the RIG environment variable
export RIG=${RIG:-default}

# Start roscore in the background
roscore &

# Wait for roscore to be up
until rostopic list ; do sleep 0.5; done

# Check if the main launch file exists
MAIN_LAUNCH_FILE="/root/catkin/src/Kinefly/launch/main.launch"
if [ ! -f "$MAIN_LAUNCH_FILE" ]; then
    echo "Error: Main launch file $MAIN_LAUNCH_FILE does not exist."
    echo "Available launch files:"
    find /root/catkin/src/Kinefly/launch -name "*.launch"
    exit 1
fi

# Check if the rig-specific folder exists
RIG_FOLDER="/root/catkin/src/Kinefly/launch/${RIG}"
if [ ! -d "$RIG_FOLDER" ]; then
    echo "Error: Rig folder $RIG_FOLDER does not exist."
    echo "Available rig folders:"
    ls -d /root/catkin/src/Kinefly/launch/*/
    exit 1
fi

# Launch Kinefly
echo "Launching Kinefly with RIG=${RIG}"
roslaunch Kinefly main.launch

# Wait for any process to exit
wait -n

# Exit with status of process that exited first
exit $?