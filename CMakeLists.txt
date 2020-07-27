cmake_minimum_required(VERSION 2.8.3)
project(visual_servoing)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS
actionlib
  actionlib_msgs
  dynamic_reconfigure
  pluginlib
  geometry_msgs
  message_generation
  rosconsole
  roscpp
  rospy
  sensor_msgs
  control_msgs
  std_msgs
  tf
  image_transport
  cmake_modules
  dvs_msgs
  geometry_msgs
  cv_bridge
)
 
add_message_files(
   FILES
)
 add_service_files(
   FILES
  object_test.srv
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
 )

include_directories( ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GSTREAMER_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
include_directories(${OPENCV_INCLUDE_DIR})
link_directories(${catkin_LIBRARY_DIRS} ${Eigen_LIBRARY_DIRS})
include_directories ("${PROJECT_SOURCE_DIR}/include/")

include_directories ("/home/abdulla/OneDrive/KU_research/codes/visual_servoing/src/visual_servoing_davis/include/")
include_directories ("/home/abdulla/OneDrive/KU_research/codes/visual_servoing/src/rpg_dvs_ros/davis_ros_driver/include/")
find_package(Boost REQUIRED COMPONENTS thread system)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

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
  LIBRARIES multi_robot_system
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs message_generation rosconsole roscpp rospy sensor_msgs std_msgs tf control_msgs  cmake_modules )

###########
## Build ##
###########
include_directories ("/home/user/catkin_ws/src/baxter_gripper_server/include/")

include_directories ("${PROJECT_SOURCE_DIR}/include/")
find_package(Boost REQUIRED COMPONENTS thread system)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
include_directories(
  ${catkin_INCLUDE_DIRS}
)
find_package (VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
link_directories(${VTK_LIBRARY_DIRS})
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIR})
find_package(actionlib REQUIRED)
include_directories(${actionlib_INCLUDE_DIRS})





add_executable(visual_servo_node src/VisualServoing.cpp src/visual_servoing_node.cpp src/CornerDetectorHARRIS.cpp src/harris_fixed_distinct_queue.cpp src/harris_distinct_queue.cpp)
add_executable(test_eigen src/eigen_test.cpp)
target_link_libraries(visual_servo_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARIES}) 
target_link_libraries(test_eigen ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARIES}) 
add_dependencies(visual_servo_node ${catkin_LIBRARIES}  ${Eigen_LIBRARIES}  )








# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
#add_executable(testing_corner_edge src/test_node.cpp src/test_data_processing.cpp )
#target_link_libraries(data_processing_test_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
#add_dependencies(data_processing_test_node ${catkin_LIBRARIES}  ${Eigen_LIBRARIES})
#############
## Install ##
#############