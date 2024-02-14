# bittle_ROS2
ROS2 package for Bittle Robot. There are launch files for Object detection based control, RQT robot controller GUI Control, and Generic USB video game controller control. All options have video streaming from Bittle to the desktop

## Prerequisites
Before you begin, ensure you have the following installed:
- Petoi Bittle Robot
- Raspberry pi (Zero 2, 4 or 5)
- ROS2 (Tested on Humble)
- Python 3.5 or higher
- Git
- Docker (On raspberry pi)

## Desktop/Laptop setup

## Raspberry pi setup
1) Configure raspberry pi as detailed here in the readme: https://github.com/gravesreid/autonomous-bittle.git

### install ros2 using docker taken from:
https://docs.ros.org/en/foxy/How-To-Guides/Installing-on-Raspberry-Pi.html

#### Steps to install:
```bash
 curl -fsSL https://get.docker.com -o get-docker.sh
 sudo sh get-docker.sh
```
#### so you don't need to use sudo
```bash
sudo usermod -aG docker $USER
```
```bash
newgrp docker
```
Check that docker works
```bash
docker run hello-world
```

#### You can download the latest docker image with the setup steps already completed here:
```bash
docker pull gravesreid/bittle_ros_humble:latest
```

### Set up Workspace on Raspberry Pi
#### get docker image id
```bash
docker images
```
#### launch docker container
```bash
docker run -it --net=host --privileged [image_id]
```
#### source ros2
```bash
source /opt/ros/humble/setup.bash
```
#### source your workspace
```bash
cd ~/ros2_ws
source install/setup.bash
```
### if you didn't pull the bittle_ros_humble docker image
### this is what you need to do to set up your desktop ros2 workspace also
#### make ros2 workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
#### clone the bittle_ros2 package
```bash
git clone https://github.com/gravesreid/bittle_ros2.git
```
#### clone bittle_msgs package
```bash
git clone https://github.com/gravesreid/bittle_msgs.git
```
#### Clone joy package
```bash
git clone --branch ros2 https://github.com/ros-drivers/joystick_drivers.git
```
#### Install colcon
```bash
sudo apt install python3-colcon-common-extensions
```
#### make sure dependencies are updated and installed
```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src -r -y
```
#### build the package
```bash
cd ~/ros2_ws
colcon build
```

#### Source the environment
```bash
source install/setup.bash
```

### set up raspberry pi camera library instructions
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
### If using realsense camera: set up intel realsense D405 camera wrapper
1) install package
```bash
sudo apt install ros-humble-librealsense2*
```
2) source workspace
```bash
cd ~/ros2_ws
source install/setup.bash
```


### Run bittle controller after you have configured both the Raspberry Pi and Desktop for ROS2
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

### run usb camera node after initial setup
To list which port the camera is attached to:
```bash
v4l2-ctl --list-devices
```
```bash
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file src/usb_cam/config/params.yaml
```

### run raspberry pi camera node after initial setup
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
5) select image view in GUI that pops up. Choose /image_raw/compressed topic for low latency streaming. Make sure you have compressed image transport on your desktop:
```bash
sudo apt-get install ros-humble-compressed-image-transport
```




