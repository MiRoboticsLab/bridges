cmake_minimum_required(VERSION 3.8)
project(bes_transmit)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(protocol REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(mqttc REQUIRED)
find_package(cpp_httplib REQUIRED)
find_package(ament_index_cpp REQUIRED)
pkg_search_module(UUID REQUIRED uuid)

set(dependencies
  rclcpp
  std_msgs
  protocol
  cyberdog_common
  mqttc
  cpp_httplib
  ament_index_cpp
)
set(executable_name bes_transmit_waiter)

include_directories(include)

add_executable(${executable_name} src/main.cpp src/bes_transmit_center.cpp)
target_include_directories(${executable_name} PUBLIC ${UUID_INCLUDE_DIRS})
ament_target_dependencies(${executable_name}
  ${dependencies}
)
target_link_libraries(${executable_name}
  ${UUID_LIBRARIES}
)

install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
