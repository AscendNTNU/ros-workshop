cmake_minimum_required(VERSION 2.8.3)
project(ros_workshop)

##Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
  	${catkin_INCLUDE_DIRS}
)

add_library(ascend_drone src/AscendDrone.cpp)
target_link_libraries(ascend_drone ${catkin_LIBRARIES})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(control_node src/control_node.cpp)
add_executable(perception_node src/perception_node.cpp)
add_executable(ai_node src/ai_node.cpp)
add_executable(test_ros src/test.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(perception_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ai_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(control_node
	${catkin_LIBRARIES}
	ascend_drone
)
## Specify libraries to link a library or executable target against
target_link_libraries(perception_node
	${catkin_LIBRARIES}
	ascend_drone
)
## Specify libraries to link a library or executable target against
target_link_libraries(ai_node
	${catkin_LIBRARIES}
	ascend_drone
)

target_link_libraries(test_ros
	${catkin_LIBRARIES}
)
