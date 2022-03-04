cmake_minimum_required(VERSION 3.8)
project(can_node_test)

include_directories(
  ${CMAKE_SOURCE_DIR}/include
)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(toml REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(dependencies
  toml
  ament_index_cpp
  rclcpp
)

add_executable(can_node_test embed_protocol_test/can_node_test.cpp)
ament_target_dependencies(can_node_test ${dependencies})
target_compile_definitions(can_node_test PRIVATE
  "PASER_PATH=\"${CMAKE_SOURCE_DIR}/test/embed_protocol_test/parser\""
)
add_executable(sensor_simulator_node embed_protocol_test/sensor_simulator_node.cpp)
ament_target_dependencies(sensor_simulator_node ${dependencies})
target_compile_definitions(sensor_simulator_node PRIVATE
  "PASER_PATH=\"${CMAKE_SOURCE_DIR}/test/embed_protocol_test/parser\""
)

ament_add_gtest(
  embed_protocol_test embed_protocol_test/embed_protocol_test.cpp
  TIMEOUT 20
)
ament_target_dependencies(embed_protocol_test ${dependencies})

target_compile_definitions(embed_protocol_test PRIVATE
  "PASER_PATH=\"${CMAKE_SOURCE_DIR}/test/embed_protocol_test/parser\""
  "COMMON_PROTOCOL_TEST=true"
)
