cmake_minimum_required(VERSION 2.8.3)
project(bezier_replanner)
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  autoware_msgs
  autoware_build_flags
  std_msgs
  tf
  python_qt_binding
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/BezierReplanner.cfg
)

catkin_package(CATKIN_DEPENDS
  roscpp
  rospy
  autoware_msgs
  autoware_build_flags
  std_msgs
  tf
  python_qt_binding
  dynamic_reconfigure
)

include_directories(${catkin_INCLUDE_DIRS} include)
include_directories(/usr/include/eigen3)

add_executable(${PROJECT_NAME}_node
  src/Bezier.h
  src/bezier_replanner.cpp
  src/bezier_replanner_node.cpp
)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME bezier_replanner PREFIX "")
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/panel
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
