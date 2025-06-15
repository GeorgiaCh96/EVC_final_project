#!/bin/bash

# Step 1: Clean previous build (optional)
cd /home/ubuntu/traffic_ws
rm -rf build/ devel/

# Step 2: Source ROS
source /opt/ros/noetic/setup.bash

# Step 3: Rebuild the workspace
catkin_make

# Step 4: Source the workspace
source devel/setup.bash

# Step 5: Export ROS environment variables
export ROS_MASTER_URI=http://192.168.8.2:11311
export ROS_IP=192.168.8.141
export DISPLAY=192.168.8.141:0

# Step 6: Add PYTHONPATH to .bashrc if not already added
grep -qxF 'export PYTHONPATH=$PYTHONPATH:/home/ubuntu/traffic_ws/devel/lib/python3.8/dist-packages' ~/.bashrc || \
  echo 'export PYTHONPATH=$PYTHONPATH:/home/ubuntu/traffic_ws/devel/lib/python3.8/dist-packages' >> ~/.bashrc

# Step 7: Add workspace source to .bashrc if not already added
grep -qxF 'source ~/traffic_ws/devel/setup.bash' ~/.bashrc || \
  echo 'source ~/traffic_ws/devel/setup.bash' >> ~/.bashrc

# Step 8: Apply .bashrc changes (only if necessary)
source ~/.bashrc

# Step 9: Start interactive shell
exec bash
