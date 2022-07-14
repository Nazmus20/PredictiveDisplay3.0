cmake_minimum_required(VERSION 2.8.3)
project(freq_sweep)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  #geometry_msgs
  roscpp
  rospy
  joy
  #std_msgs
#  sensor_msgs
  #tf2
  #tf2_ros
  #genmsg
  #cv_bridge
  #image_transport
  #sensor_msgs
  #smach
  #smach_ros
  #mavros_msgs

)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES detection
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf2 tf2_ros piksi_rtk_msgs cv_bridge image_transport sensor_msgs smach smach_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
${catkin_INCLUDE_DIRS}
/usr/local/include
/usr/include
${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

 add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} )
