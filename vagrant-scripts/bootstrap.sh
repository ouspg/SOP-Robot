#!/usr/bin/env bash

sudo -u vagrant echo "export WORKSPACE='/workspace/'" >> /home/vagrant/.bashrc
sudo -u vagrant echo "cd /workspace" >> /home/vagrant/.bashrc
sudo -u vagrant echo "source /opt/ros/foxy/setup.bash" >> /home/vagrant/.bashrc

# Enable X11 Forwarding
# echo "X11Forwarding yes" >> /etc/ssh/sshd_config
# echo "export LIBGL_ALWAYS_INDIRECT=1" >> /home/vagrant/.bashrc
# Note: if using LIBGL_ALWAYS_INDIRECT=1, rviz2 does not run! (Failed to create an OpenGL context)

mkdir /workspace/
chown -R vagrant /workspace/

apt update

# setup GNOME desktop
apt-get install -y ubuntu-desktop virtualbox-guest-dkms virtualbox-guest-utils virtualbox-guest-x11

# Install ROS1
#sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
#apt-get update
#apt-get install -y ros-noetic-desktop-full ros-noetic-moveit python3-catkin-tools python3-catkin-lint python3-rosdep python3-roslaunch \
#  ros-noetic-moveit-visual-tools ros-noetic-dynamixel-sdk
#apt-get install -y gitg vim meld terminator

#rosdep init
#rosdep fix-permissions

# sudo -u vagrant rosdep update

# Install ROS2
apt install curl gnupg2 lsb-release software-properties-common

curl -s 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc' | apt-key add -
sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# add-apt-repository ppa:deadsnakes/ppa
#   python3.7 \

apt update

apt install -y \
  build-essential \
  cmake \
  git \
  libyaml-cpp-dev \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
# install some pip packages needed for testing
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest
# install Fast-RTPS dependencies
apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
apt install --no-install-recommends -y \
  libcunit1-dev

# Install ROS2 Foxy
apt install -y \
  ros-foxy-desktop ros-foxy-dynamixel-sdk

# curl -L -o /tmp/ros2-foxy.tar.bz2 https://github.com/ros2/ros2/releases/download/release-foxy-20201211/ros2-foxy-20201211-linux-focal-amd64.tar.bz2

# mkdir -p /ros2_foxy
# cd /ros2_foxy

# tar -xf /tmp/ros2-foxy.tar.bz2 -C /ros2_foxy
# rm /tmp/ros2-foxy.tar.bz2

# chown -R vagrant /ros2_foxy/

# sudo -u vagrant rosdep update
# sudo -u vagrant rosdep install --from-paths ros2-linux/share --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps osrf_testing_tools_cpp poco_vendor rmw_connext_cpp rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers"

# Setup Dynamixel U2D2 (https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/#u2d2)
wget https://raw.githubusercontent.com/ROBOTIS-GIT/dynamixel-workbench/master/99-dynamixel-workbench-cdc.rules -P /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger

# Change python3 symlink from python3.8 to python3.7 (ROS2 does not work with python 3.8 currently)
# rm /usr/bin/python3
# ln -s /usr/bin/python3.7 /usr/bin/python3

# Clone submodules
cd /workspace && git submodule update --init --recursive

source /opt/ros/foxy/setup.bash

# Install package dependencies
apt install -y \
  ros-foxy-test-msgs ros-foxy-control-msgs \
  ros-foxy-realtime-tools ros-foxy-xacro ros-foxy-angles \
  v4l-utils


# Install ros2_control (https://github.com/ros-controls/ros2_control)

mkdir -p /ros2_control_ws/src
cd /ros2_control_ws
wget https://raw.githubusercontent.com/ros-controls/ros2_control/master/ros2_control/ros2_control.repos
vcs import src < ros2_control.repos

colcon build

source /ros2_control_ws/install/setup.bash
sudo -u vagrant echo "source /ros2_control_ws/install/setup.bash" >> /home/vagrant/.bashrc

# Allow access to shared folders
adduser vagrant vboxsf

# Install moveit2 (https://moveit.ros.org/install-moveit2/source/)
mkdir -p /moveit2_ws/src
cd /moveit2_ws
wget https://raw.githubusercontent.com/ros-planning/moveit2/main/moveit2.repos
vcs import src < moveit2.repos

rosdep init
rosdep update
rosdep install -r --from-paths src --ignore-src --rosdistro foxy -y

colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release

# Note: Moveit2 hardware_interface conflicts with ros2_controller hardware_interface package, so remove it after build for now
rm -rf /moveit2_ws/install/hardware_interface/

source /moveit2_ws/install/setup.bash
sudo -u vagrant echo "source /moveit2_ws/install/setup.bash" >> /home/vagrant/.bashrc

# Install opencv_cam (https://github.com/clydemcqueen/opencv_cam)
mkdir -p /opencv_cam_ws/src
cd /opencv_cam_ws/src
git clone https://github.com/clydemcqueen/opencv_cam.git
git clone https://github.com/ptrmu/ros2_shared.git
cd ..

rosdep install --from-paths src --ignore-src --rosdistro foxy -r -y

colcon build
source /opencv_cam_ws/install/setup.bash
sudo -u vagrant echo "source /opencv_cam_ws/install/setup.bash" >> /home/vagrant/.bashrc

# Install workspace package dependencies
cd /workspace
#rosdep install --from-paths src --ignore-src --rosdistro foxy -r -y