cmake_minimum_required(VERSION 3.0.2)
project(simulation_env)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  xacro
)
catkin_python_setup()
catkin_package()


catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
