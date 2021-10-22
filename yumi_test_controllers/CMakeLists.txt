cmake_minimum_required(VERSION 2.8.3)
project(yumi_test_controllers)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  angles
  control_msgs
  control_toolbox
  controller_interface
  forward_command_controller
  realtime_tools
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    angles
    control_msgs
    control_toolbox
    controller_interface
    forward_command_controller
    realtime_tools
    sensor_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include 
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
)


add_executable(test_traj_vel_control src/test_traj_vel_control.cpp)
target_link_libraries(test_traj_vel_control ${catkin_LIBRARIES})


add_executable(test_vel_control src/test_vel_control.cpp)
target_link_libraries(test_vel_control ${catkin_LIBRARIES})
