# Build this project in a docker container in arm-based macbook
## Setup X11 server
1. Install XQuartz
```bash
brew install --cask xquartz
```

2. Start XQuartz


Go to XQuartz → Settings → Security tab and make sure "Allow connections from network clients" is **checked**.

Then **quit and restart XQuartz** for the settings to take effect.

4. Allow X11 connections from any host
```bash
xhost +
```

You should see: `access control disabled, clients can connect from any host`

## docker setup
1. start colima, mac use colima to run docker
```bash
colima start
```

2. pull ubuntu:20.04 image, it automatically use arm64 based image, since my mac is arm64
```bash
docker pull ubuntu:20.04
```

3. start container

**Before running:** Make sure you've completed all X11 setup steps above (especially `xhost +`).

```bash
docker run -it \
  -v "/Users/jtao/Documents/Projects/simple_slam_ros1/robotics_slam_task_ubuntu20_on_armbased_mac:/workspace" \
  -e DISPLAY=host.docker.internal:0 \
  --name simple_slam_ros1 \
  ubuntu:20.04 /bin/bash
```

**Note:** We use `host.docker.internal:0` for DISPLAY because Docker on macOS (via Colima) doesn't properly sync Unix socket files. This uses network-based X11 forwarding instead.

Try to test x11
```bash
apt-get update && apt-get install -y x11-apps
xeyes
```

If you see a window with eyes, then X11 is working.

## restart the container
```bash
docker restart simple_slam_ros1
docker exec -it simple_slam_ros1 /bin/bash
```

## build 
### /mapping package
1. Install curl and gnupg
```bash
apt install -y curl gnupg
```

2. Install ROS

```bash
echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt update
apt install -y ros-noetic-desktop-full
```

3. Install C++ compiler and build tools
```bash
apt install -y build-essential
```

4. Compile the workspace
Source ros environment
```bash
source /opt/ros/noetic/setup.bash
```

Install dependencies for mapping package
```bash
apt install -y libwxgtk3.0-gtk3-dev libgl1-mesa-glx libglu1-mesa mesa-utils
apt install -y ros-noetic-map-server
```

Then compile the workspace
```bash
cd /workspace
catkin_make
```
5. Test it
```bash
source devel/setup.bash
roslaunch mapping mapping.launch
```

## run the project
First source the workspace
```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```
### run mapping package
```bash
roslaunch mapping mapping.launch
```
### run localization package
```bash
roslaunch localization mcl.launch
```