cmake_minimum_required(VERSION 2.8.3)
project(frontier_exploration_turtlebot)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  move_base_msgs
)

catkin_package()

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(frontierExplorer 
	src/main.cpp 
	src/CollisionDetector.cpp 
	src/PathPlanning.cpp
	)
	
target_link_libraries(frontierExplorer
	${catkin_LIBRARIES}
)


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")
  add_rostest_gtest(
  frontierExplorerTest
  test/Test.launch
  test/main.cpp
  test/CollisionDetectorTest.cpp
  test/PathPlanningTest.cpp

  src/CollisionDetector.cpp
  src/PathPlanning.cpp
  )
  target_link_libraries(
  frontierExplorerTest
  ${catkin_LIBRARIES}
  )
  add_dependencies(
  frontierExplorerTest
  frontierExplorer
  ${catkin_EXPORTED_TARGETS}
  )
endif()
