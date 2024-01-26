# bittle_ROS2
ROS2 package for Bittle Robot

# Set up Workspace on Raspberry Pi
### launch docker container
```bash
docker run -it --net=host --device=/dev/ttyAMA0 [image_id]
```
### source your workspace
```bash
source source /opt/ros/humble/setup.bash
```
### make ros2 workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
### clone the bittle_ros2 package

