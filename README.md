# bittle_ROS2
ROS2 package for Bittle Robot

# install ros2 using docker
https://docs.ros.org/en/foxy/How-To-Guides/Installing-on-Raspberry-Pi.html

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
```bash
git clone https://github.com/gravesreid/bittle_ros2.git
```
### Install colcon
```bash
sudo apt install python3-colcon-common-extensions
```
### build the package
```bash
cd ~/ros2_ws
colcon build
```

### Source the environment
```bash
source install/setup.bash
```

