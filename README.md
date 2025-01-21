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
        ros-$ROS_DISTRO-velocity-controllers \
        ros-$ROS_DISTRO-hector-xacro-tools
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
```

### Network Setup:

Please be sure to follow the [Network Setup Instructions](https://github.com/kth-ros-pkg/yumi/wiki/Network-setup)

## Running the Robot

Please set the main pointer at the FlexPendant to main for the `Rob_L` and `Rob_R` tasks, then run them by presseng on the play button on the flexpendant lower right side for each of them (you can access them from `Program Editor`)

To run RVIZ with moveit:
- After you connect to YuMi (which should be running and motors on in Automatic mode):
```
cd ~/yumi_ws && catkin b -DCMAKE_BUILD_TYPE=RELEASE && . devel/setup.bash
cd src/yumi_ros/yumi_description/urdf/
rosrun xacro xacro yumi.urdf.xacro arms_interface:=VelocityJointInterface grippers_interface:=EffortJointInterface yumi_setup:=robot_centric -o yumi.urdf
roslaunch yumi_moveit_config myLaunch.launch
```
