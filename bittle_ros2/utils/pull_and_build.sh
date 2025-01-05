#!/bin/bash

# Navigate to the bittle_ros2 directory
cd ~/ros2_ws/src/bittle_ros2 || exit 1

# Pull the latest changes from the git repository
echo "Pulling latest changes from git repository..."
git pull || exit 1

# Navigate back to the ros2_ws directory
cd ~/ros2_ws || exit 1

# Build the bittle_ros2 package
echo "Building the bittle_ros2 package..."
colcon build --packages-select bittle_ros2 || exit 1

# Source the setup.bash file
echo "Sourcing the setup.bash file..."
source install/setup.bash || exit 1

echo "Pulled and built bittle_ros2 successfully"
