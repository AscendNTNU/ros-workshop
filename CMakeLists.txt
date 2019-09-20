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
  gazebo_msgs
  gazebo_ros
  roscpp
  rospy
  std_msgs
)

find_package(gazebo REQUIRED)


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
	CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp rospy
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
add_executable(control_node src/control.cpp)
add_executable(perception_node src/perception.cpp)
#add_executable(ai_node src/ai_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(perception_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(ai_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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



## Add gazebo plugins
include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
add_library(drone_controller_plugin SHARED src/simulator/drone_controller.cc)
target_link_libraries(drone_controller_plugin ${GAZEBO_LIBRARIES})
add_library(drone_props_spinner_plugin SHARED src/simulator/drone_props.cc)
target_link_libraries(drone_props_spinner_plugin  ${GAZEBO_LIBRARIES})
#message(STATUS "GAZEBO LIBS: " ${GAZEBO_LIBRARIES})

add_executable(fake_mavros_node src/simulator/fake_mavros.cpp)
add_dependencies(fake_mavros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fake_mavros_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

