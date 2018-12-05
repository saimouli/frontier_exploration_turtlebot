cmake_minimum_required(VERSION 2.8.3)
project(frontier_exploration_turtlebot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  move_base_msgs
)

catkin_package(
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(frontier_exploration_turtlebot 
	src/main.cpp 
	src/CollisionDetector.cpp 
	src/Guidance.cpp 
	src/PathPlanning.cpp
	)
	
target_link_libraries(frontier_exploration_turtlebot
	${catkin_LIBRARIES}
)
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")

  add_rostest_gtest(
    Test
    test/GuidanceTest.cpp
    test/CollisionDetectorTest.cpp
    test/PathPlanningTest.cpp
    src/CollisionDetector.cpp
    src/Guidance.cpp
    src/PathPlanning.cpp
  )

  target_link_libraries(
    Test
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    Test
    frontier_exploration_turtlebot
    ${catkin_EXPORTED_TARGETS}
  )
endif()
