# yumi_ros
ABB YuMi IRB 14000 ROS Architeture
> This repo is a variant of the original [KTH-RPL](https://github.com/kth-ros-pkg/yumi), that works with `ROS-Noetic`.

## ROS Installation

Please follow the steps in [ROS-wiki](http://wiki.ros.org/ROS/Installation) to install the corresponding ROS distribution.
- in my case Ubuntu `20.04`: [ROS-Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

## Please read this [wiki](https://github.com/kth-ros-pkg/yumi/wiki) before to start.

## Dependencies

```
sudo apt-get install \
        python3-pip \
        protobuf-compiler \
        protobuf-c-compiler \
        ros-$ROS_DISTRO-control-toolbox \
        ros-$ROS_DISTRO-controller-interface \
        ros-$ROS_DISTRO-controller-manager \
        ros-$ROS_DISTRO-effort-controllers \
        ros-$ROS_DISTRO-force-torque-sensor-controller \
        ros-$ROS_DISTRO-gazebo-ros-control \
        ros-$ROS_DISTRO-joint-limits-interface \
        ros-$ROS_DISTRO-joint-state-publisher \
        ros-$ROS_DISTRO-joint-state-controller \
        ros-$ROS_DISTRO-joint-trajectory-controller \
        ros-$ROS_DISTRO-moveit-commander \
        ros-$ROS_DISTRO-moveit-core \
        ros-$ROS_DISTRO-moveit-planners \
        ros-$ROS_DISTRO-moveit-ros-move-group \
        ros-$ROS_DISTRO-moveit-ros-planning \
        ros-$ROS_DISTRO-moveit-ros-visualization \
        ros-$ROS_DISTRO-moveit-simple-controller-manager \
        ros-$ROS_DISTRO-position-controllers \
        ros-$ROS_DISTRO-rqt-joint-trajectory-controller \
        ros-$ROS_DISTRO-transmission-interface \
        ros-$ROS_DISTRO-velocity-controllers
```
Then:
```
pip3 install --user pyftpdlib
pip3 install --user --upgrade pyassimp

```

## Quick Start

```
mkdir -p ~/yumi_ws/src && cd ~/yumi_ws/src
git clone --recursive https://github.com/bhomaidan1990/yumi_ros.git
git clone https://github.com/ros-industrial/abb_driver.git
git clone https://github.com/ros-industrial/industrial_core.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin b -DCMAKE_BUILD_TYPE=RELEASE
. devel/setup.bash
```

### Network Setup:

Please be sure to follow the [Network Setup Instructions](https://github.com/kth-ros-pkg/yumi/wiki/Network-setup)

To run RVIZ with moveit:
- After you connect to YuMi (which should be running and motors on in Automatic mode):
- > `roslaunch yumi_moveit_config myLaunch.launch`
