# Build this project in a docker container in arm-based macbook
## docker setip
1. start colima, mac use colima to run docker
'''
colima start
'''

2. pull ubuntu:20.04 image, it automatically use arm64 based image, since my mac is arm64
'''
docker pull ubuntu:20.04
'''

3. start container 
'''
docker run -it \
  -v "/Users/jtao/Documents/Study/Computer Engineering/WS2026/Robotics/git/workspace_assignment4:/workspace" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  ubuntu:20.04 /bin/bash
'''

## restart the container
'''
docker restart <container_id>
docker exec -it <container_id> /bin/bash
'''

## build 
### /mapping package
1. Install curl and gnupg
'''
apt install -y curl gnupg
'''

2. Install ROS

'''
echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt update
apt install -y ros-noetic-desktop-full
'''

3. Install C++ compiler and build tools
'''
apt install -y build-essential
'''

4. First take other packages out of the path, only keep /mapping package
'''
cd /workspace
catkin_make
'''
5. Test it
'''
source devel/setup.bash
roslaunch mapping mapping.launch
'''

### /rbo_create package
1. install dependencies
'''
apt install -y libwxgtk3.0-gtk3-dev
'''
2. build
'''
cd /workspace
catkin_make
'''
### /localization package
1. install dependencies, it map_server package of ROS Noetic
'''
apt install -y ros-noetic-map-server
'''
2. build
'''
cd /workspace
catkin_make
'''

### /create_gui package

1. it needs wxWidgets - GUI library and opengl library
'''
apt install -y libwxgtk3.0-gtk3-
apt install -y libgl1-mesa-glx libglu1-mesa mesa-utils
'''
2. build
'''
cd /workspace
catkin_make
'''

## run the project
First source the workspace
'''
source /opt/ros/noetic/setup.bash
source devel/setup.bash
'''
### run mapping package
'''
roslaunch mapping mapping.launch
'''
### run localization package
'''
roslaunch localization mcl.launch
'''