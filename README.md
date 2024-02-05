# bittle_ROS2
ROS2 package for Bittle Robot

## Run bittle controller after you have configured both the Raspberry Pi and Desktop for ROS2
1) make sure you source your build
```bash
source ~/ros2_ws/install/setup.bash
```
2) make sure your domain is set
```bash
export ROS_DOMAIN_ID=1
```

3) launch the server node
```bash
ros2 launch bittle_ros2 bittle_teleop_server_launch.py
```
An rqt GUI should open up on your desktop

4) source the build on the pi
```bash
source ~/ros2_ws/install/setup.bash
```

5) make sure your domain is set
```bash
export ROS_DOMAIN_ID=1
```

6) launch the robot node
```bash
ros2 launch bittle_ros2 bittle_teleop_robot_launch.py
```

## run camera node after initial setup
1) Source build
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   ```
2) Make sure domain is set (on pc and pi-same domain id for both)
```bash
export ROS_DOMAIN_ID=1
```
3) run camera node on pi
```bash
ros2 run v4l2_camera v4l2_camera_node
```

4) run visualization tool on desktop/pc
```bash
ros2 run rqt_image_view rqt_image_view
```
5) select image view in GUI that pops up. Choose /image_raw/compressed topic for low latency streaming

# install ros2 using docker
https://docs.ros.org/en/foxy/How-To-Guides/Installing-on-Raspberry-Pi.html

## You can download the latest docker image with the setup steps already completed here:
```bash
docker pull gravesreid/bittle_ros_humble:latest
```

# Set up Workspace on Raspberry Pi
### get docker image id
```bash
docker images
```
### launch docker container
```bash
docker run -it --net=host --device=/dev/ttyAMA0 [image_id]
```
### source your workspace
```bash
source /opt/ros/humble/setup.bash
```
## if you didn't pull the bittle_ros_humble docker image
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
### make sure dependencies are updated and installed
```bash
rosdep update
rosdep install --from-paths src -r -y
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

# set up raspberry pi camera library instructions
from: https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304
1) clone packages
   ```bash
   cd ~/ros2_ws/src
   ```
   ```bash
   git clone --branch humble https://gitlab.com/boldhearts/ros2_v4l2_camera.git
   ```
   ```bash
   git clone --branch humble https://github.com/ros-perception/vision_opencv.git
   ```
   ```bash
   git clone --branch humble https://github.com/ros-perception/image_common.git
   ```
   ```bash
   git clone --branch humble https://github.com/ros-perception/image_transport_plugins.git
   ```
2) install dependencies
   ```bash
   cd ..
   rosdep install --from-paths src -r -y
   ```
3) Build packages
   ```bash
   colcon build --packages-up-to v4l2_camera image_transport_plugins
   ```
4) Source workspace
   ```bash
   source install/setup.bash
   ```
# set up intel realsense D405 camera wrapper
1) install package
```bash
sudo apt install ros-humble-librealsense2*
```
2) source workspace
```bash
cd ~/ros2_ws
source install/setup.bash
```


