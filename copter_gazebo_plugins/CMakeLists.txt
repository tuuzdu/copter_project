cmake_minimum_required(VERSION 2.8.3)
project(copter_gazebo_plugins)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  control_toolbox
)
include_directories(include ${catkin_INCLUDE_DIRS})

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

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
  DEPENDS 
    roscpp 
    gazebo_ros 
)

###########
## Build ##
###########

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(flyer_plugin src/flyer_plugin.cpp src/wind.cpp)
target_link_libraries(flyer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#add_library(px4flow_plugin src/px4flow_plugin.cpp)
#target_link_libraries(px4flow_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(copter_gazebo_ros_imu src/gazebo_ros_imu.cpp)
target_link_libraries(copter_gazebo_ros_imu ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(hector_gazebo_ros_imu ${PROJECT_NAME}_generate_messages_cpp)

add_library(copter_gazebo_ros_sonar src/gazebo_ros_sonar.cpp)
target_link_libraries(copter_gazebo_ros_sonar ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
