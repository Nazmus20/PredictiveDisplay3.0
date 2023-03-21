cmake_minimum_required(VERSION 2.8.3)
project(predictor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  genmsg
  cv_bridge
  image_transport
  sensor_msgs
  smach
  smach_ros
  mavros_msgs

)
#find_package( OpenCV REQUIRED )
find_package(OpenCV 4.2.0 REQUIRED)
find_package(Boost REQUIRED)

#NO_MODULE # should be optional, tells CMake to use config mode
#PATHS /usr/local /usr/share # look here
#NO_DEFAULT_PATH) # and don't look anywhere else
find_package(CUDA REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html


################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)


## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf2 tf2_ros piksi_rtk_msgs cv_bridge image_transport sensor_msgs smach smach_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
/usr/local/include
/usr/include
${OpenCV_INCLUDE_DIRS}
)

###################################################################################
add_library(${PROJECT_NAME}_HistogramMatching src/HistogramMatching.cpp)
add_library(${PROJECT_NAME}_HomographyCalculation src/HomographyCalculation.cpp)
add_library(${PROJECT_NAME}_ImageStitching src/ImageStitching.cpp)
#add_library(${PROJECT_NAME}_testing src/testing.cpp)
#####################################################################################

#################################################################################### 
add_executable(${PROJECT_NAME}_delay src/delay.cpp)
add_executable(${PROJECT_NAME}_new src/Stitcher_new.cpp)
add_executable(${PROJECT_NAME}_main src/main.cpp)
add_executable(${PROJECT_NAME}_main2 src/main2.cpp) 
#add_executable(${PROJECT_NAME}_HistogramMatching src/HistogramMatching.cpp)
# add_executable(${PROJECT_NAME}_testing src/testing.cpp)
########################################################################################

################################################################################################
target_link_libraries(${PROJECT_NAME}_new ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME}_HistogramMatching ${PROJECT_NAME}_ImageStitching ${PROJECT_NAME}_HomographyCalculation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_main2 ${PROJECT_NAME}_HistogramMatching ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_ImageStitching ${PROJECT_NAME}_HistogramMatching ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_delay ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
# target_link_libraries(${PROJECT_NAME}_testing ${catkin_LIBRARIES}  )
######################################################################################################

