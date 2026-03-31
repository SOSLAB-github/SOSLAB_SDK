cmake_minimum_required(VERSION 3.10)
project(ml)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -O0)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  image_transport
  sensor_msgs
  cv_bridge
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

## SOSLAB Libraries
set( SOSLAB_INCLUDE_DIR 
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/nlohmann_json" 
)
set( SOSLAB_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/libsoslab/libLidar_x64_release.so")
file(GLOB SOSLAB_SO_FILES
  ${SOSLAB_LIBRARIES}
)


# set(soslab_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libsoslab")
# set(soslab_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/libsoslab/libsoslab_core.so" "${CMAKE_CURRENT_SOURCE_DIR}/libsoslab/libsoslab_ml.so")

## System dependencies are found with CMake's conventions
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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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

set(SOSLAB_IMPORTED_TARGETS "")
foreach(_so ${SOSLAB_SO_FILES})
  get_filename_component(_so_name ${_so} NAME)
  # Turn libXXX.so(.ver) into target name "soslab::XXX"
  string(REGEX REPLACE "^lib" "" _base "${_so_name}")
  string(REGEX REPLACE "\\.so(\\..*)?$" "" _base "${_base}")
  string(REPLACE "-" "_" _base "${_base}")
  string(REPLACE "." "_" _base "${_base}")

  set(_tgt "soslab::${_base}")
  add_library(${_tgt} SHARED IMPORTED GLOBAL)
  set_target_properties(${_tgt} PROPERTIES
    IMPORTED_LOCATION "${_so}"
    INTERFACE_INCLUDE_DIRECTORIES "${SOSLAB_INCLUDE_DIR}"
  )
  list(APPEND SOSLAB_IMPORTED_TARGETS ${_tgt})
endforeach()

catkin_package(
  CATKIN_DEPENDS roscpp pcl_ros pcl image_transport sensor_msgs cv_bridge
  DEPENDS Boost
  INCLUDE_DIRS ${SOSLAB_INCLUDE_DIR}
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/nlohmann_json
  libsoslab
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${SOSLAB_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ml.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} src/ml_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${SOSLAB_IMPORTED_TARGETS}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )

install(FILES ${SOSLAB_SO_FILES} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
