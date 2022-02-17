cmake_minimum_required(VERSION 3.8)
project(can_node_test)

include_directories(
  ${CMAKE_SOURCE_DIR}/include
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(toml REQUIRED)

set(dependencies
  toml
)

add_executable(can_node_one embed_protocol_test/can_node_one.cpp)
ament_target_dependencies(can_node_one rclcpp std_msgs toml)
target_compile_definitions(can_node_one PRIVATE
  "PASER_PATH=\"${CMAKE_SOURCE_DIR}/test/embed_protocol_test/parser\""
)
add_executable(can_node_two embed_protocol_test/can_node_two.cpp)
ament_target_dependencies(can_node_two rclcpp std_msgs toml)
target_compile_definitions(can_node_two PRIVATE
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
