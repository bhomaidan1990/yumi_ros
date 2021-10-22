# yumi_ros
ABB YuMi IRB 14000 ROS Architeture

## Please read this [wiki](https://github.com/kth-ros-pkg/yumi/wiki) before to start.

## Quick Start

```
git clone --recursive https://github.com/bhomaidan1990/yumi_ros.git
cd yumi_ros
git clone https://github.com/ros-industrial/abb_driver.git
git clone https://github.com/ros-industrial/industrial_core.git
mkdir src && mv * src/
catkin b -DCMAKE_BUILD_TYPE=RELEASE
. devel/setup.bash
```

To run RVIZ with moveit:
- > `roslaunch yumi_moveit_config myLaunch.launch`
