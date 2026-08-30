cmake_minimum_required(VERSION 3.0.2)
project(maxt_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  behaviortree_cpp_v3
  geometry_msgs
  mavros_msgs
  roscpp
  tf2
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS behaviortree_cpp_v3 geometry_msgs mavros_msgs roscpp tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/maxt_core.cpp
  src/maxt_mav/maxt_mav_cb.cpp
  src/maxt_mav/maxt_mav_con.cpp
  src/maxt_mav/maxt_mav_request.cpp
  src/maxt_nodes/connect_check.cpp
  src/maxt_nodes/goto.cpp
  src/maxt_nodes/hover.cpp
  src/maxt_nodes/quintic_nav.cpp
  src/maxt_nodes/round.cpp
  src/maxt_nodes/takeoff.cpp
  src/maxt_nodes/touch_down.cpp
  src/maxt_nodes/waitstep.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(maxt_test_node
  nodes/maxt_test_node.cpp
)

target_link_libraries(maxt_test_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} maxt_test_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch" PATTERN "*.xml"
)
