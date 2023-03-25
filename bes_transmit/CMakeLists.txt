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
find_package(OpenSSL REQUIRED)
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

include_directories(
  include
  ${OPENSSL_INCLUDE_DIR}
)

add_executable(${executable_name} src/main.cpp src/bes_transmit_center.cpp)
target_include_directories(${executable_name} PUBLIC ${UUID_INCLUDE_DIRS})
ament_target_dependencies(${executable_name}
  ${dependencies}
)
target_link_libraries(${executable_name}
  ${UUID_LIBRARIES}
  ${OPENSSL_LIBRARIES}
)

add_library(file_uploading SHARED
  src/file_uploading.cpp
)
target_include_directories(file_uploading PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(file_uploading PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(file_uploading
  cyberdog_common
)
ament_export_targets(file_uploading HAS_LIBRARY_TARGET)

install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS file_uploading
  EXPORT file_uploading
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)

install(
  DIRECTORY include/file_uploading/
  DESTINATION include/file_uploading/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
