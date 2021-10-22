cmake_minimum_required(VERSION 2.8.3)
project(yumi_cameras)

add_compile_options(-std=c++11)
# find_package(abb_rws_interface QUIET)


# if (abb_rws_interface_FOUND)
#   message("--   ABB RWS library found!")
#   message("--   yumi_cameras package will be built")

#   find_package(catkin REQUIRED COMPONENTS abb_rws_interface roscpp)
#   catkin_package(
#     CATKIN_DEPENDS
#       abb_rws_interface
#       roscpp
#     INCLUDE_DIRS include
#     LIBRARIES ${PROJECT_NAME}
#   )
#   include_directories(include ${catkin_INCLUDE_DIRS})
  
#   add_executable(rws_continuous_trigger src/rws_continuous_trigger.cpp)
#   target_link_libraries(rws_continuous_trigger ${catkin_LIBRARIES})
# else()
#   message("--   ABB RWS library not found")
#   message("--   This package is highly dependent on RWS, and its files cannot be built without it")
# endif()