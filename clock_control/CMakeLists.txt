cmake_minimum_required(VERSION 3.0.2)
project(clock_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  gazebo_dev
  std_msgs
  sensor_msgs
  gazebo
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include 
 CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp gazebo_dev std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


add_library(clock_plugin SHARED src/clock_plugin.cpp)
target_link_libraries(clock_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS
clock_plugin
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


add_library(diff_p SHARED src/diff_p.cpp)
target_link_libraries(diff_p ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS
diff_p
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


