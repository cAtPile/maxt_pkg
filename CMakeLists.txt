cmake_minimum_required(VERSION 3.0.2)
project(maxt_pkg)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Compile as C++14, supported in ROS Noetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  actionlib_msgs
  message_generation
  mavros
  mavros_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  behaviortree_cpp_v3
  std_msgs
)

## System dependencies are found with CMake's conventions
add_action_files(FILES Guidance.action ugv_mover.action)
add_service_files(FILES PlayAudio.srv)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GeographicLib REQUIRED geographiclib)
find_package(gazebo REQUIRED)

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
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES maxt_pkg
  CATKIN_DEPENDS actionlib actionlib_msgs message_runtime roscpp mavros mavros_msgs geometry_msgs sensor_msgs tf2 tf2_ros behaviortree_cpp_v3 std_msgs
  DEPENDS Boost Eigen3 GeographicLib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

# Native target motion avoids synchronous ROS service calls during simulation.
link_directories(${GAZEBO_LIBRARY_DIRS} ${GeographicLib_LIBRARY_DIRS})
add_library(maxt_moving_target_plugin SHARED src/sim/moving_target_plugin.cpp)
target_include_directories(maxt_moving_target_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(maxt_moving_target_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/audio_caller.cpp
  src/maxt_nodes/play_audio.cpp
  src/maxt_core.cpp
  src/mavkit/callback.cpp
  src/mavkit/set.cpp
  src/mavkit/timer.cpp
  src/mavkit/get_pose.cpp
  src/mavkit/reach_check.cpp
  src/maxt_mav/maxt_mav_cb.cpp
  src/maxt_mav/maxt_mav_con.cpp
  src/maxt_mav/maxt_mav_request.cpp
  src/maxt_nodes/guidance.cpp
  src/maxt_nodes/ugv_cowork_node.cpp
  src/maxt_nodes/capture_pose_node.cpp
  src/maxt_nodes/send_geo_pose_node.cpp

  src/maxt_nodes/takeoff.cpp
  src/maxt_nodes/waitstep.cpp
  src/maxt_nodes/hover.cpp
  src/maxt_nodes/traverse_path_generator.cpp
  src/maxt_nodes/get_region_waypoint.cpp
  src/maxt_nodes/found_check.cpp
  src/maxt_nodes/round.cpp
  src/maxt_nodes/connect_check.cpp

  src/maxt_nodes/touch_down.cpp

  src/maxt_nodes/la_lo_nav.cpp
  src/maxt_nodes/re_heading.cpp
  src/maxt_nodes/nav.cpp
  src/maxt_nodes/nav_raw.cpp
  src/maxt_nodes/base_point_azimuth_fix.cpp
  src/maxt_nodes/azimuth_fix_nav.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(maxt_test_node nodes/maxt_test_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(maxt_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${GeographicLib_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}

)

target_link_libraries(maxt_test_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

catkin_install_python(PROGRAMS script/moving_target.py script/guidance_test_server.py script/aruco_center.py script/ugv_sim.py script/ugv_controller.py
  script/fast_mission/fast_mission.py
  script/fake_audio.py
  script/fake_reciver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS maxt_test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME} maxt_moving_target_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN "*~" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch" PATTERN "*.xml" PATTERN "*.yaml"
  PATTERN "*~" EXCLUDE
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_stop_curve test/unit/test_stop_curve.cpp)

  catkin_add_gtest(test_global_nav_curve test/unit/test_global_nav_curve.cpp)
  if(TARGET test_global_nav_curve)
    target_link_libraries(test_global_nav_curve ${GeographicLib_LIBRARIES})
  endif()

  catkin_add_gtest(test_calibrate_tools test/unit/test_calibrate_tools.cpp)
  if(TARGET test_calibrate_tools)
    target_link_libraries(test_calibrate_tools ${catkin_LIBRARIES})
  endif()

  add_executable(nav_breakpoint_tree_test test/nav_breakpoint_tree_test.cpp)
  add_dependencies(nav_breakpoint_tree_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(nav_breakpoint_tree_test ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_test(NAME nav_breakpoint_tree
    COMMAND nav_breakpoint_tree_test
      ${CMAKE_CURRENT_SOURCE_DIR}/config/mission/region_traverse_test.xml
      ${CMAKE_CURRENT_SOURCE_DIR}/config/mission/sim_geo_treverse.xml)

  add_executable(ugv_cowork_tree_test test/ugv_cowork_tree_test.cpp)
  add_dependencies(ugv_cowork_tree_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(ugv_cowork_tree_test ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_test(NAME ugv_cowork_tree
    COMMAND ugv_cowork_tree_test ${CMAKE_CURRENT_SOURCE_DIR}/config/mission/ugv_cowork_test.xml)
endif()

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_maxt_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

install(DIRECTORY action
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
