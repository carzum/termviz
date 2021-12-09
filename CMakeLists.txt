cmake_minimum_required(VERSION 3.0.2)
project(termviz)

find_package(
  catkin REQUIRED
  COMPONENTS
  geometry_msgs
  visualization_msgs
  sensor_msgs
  nav_msgs
  tf2_msgs)

file(GLOB_RECURSE SRCS src/*)

#geometry_msgs_DIR:PATH=/opt/ros/noetic/share/geometry_msgs/cmake

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/bin/termviz
    COMMAND ${CMAKE_COMMAND} -E env CARGO_HOME=${CMAKE_CURRENT_BINARY_DIR}/cargo ROSRUST_MSG_PATH=${geometry_msgs_DIR}/../../../:${visualization_msgs_DIR}/../../../:${sensor_msgs_DIR}/../../../:${nav_msgs_DIR}/../../../:${tf2_msgs_DIR}/../../../ cargo install --path=${PROJECT_SOURCE_DIR} --root=${CMAKE_CURRENT_BINARY_DIR} --no-track
    DEPENDS ${SRCS} Cargo.toml
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
add_custom_target(cargo-target ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/bin/termviz)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/bin/termviz
        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
