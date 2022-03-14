cmake_minimum_required(VERSION 3.8)
project(protocol)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# include(compile_grpc.cmake.in)
include(compile_ros.cmake.in)

set(protocol_list)                # 协议列表
get_ros_protocol_file("msg")      # 加载 msg 协议
get_ros_protocol_file("srv")      # 加载 srv 协议
# get_ros_protocol_file("action") # 加载 action 协议
rosidl_generate_interfaces(${PROJECT_NAME} ${protocol_list}
  DEPENDENCIES builtin_interfaces sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime builtin_interfaces sensor_msgs std_msgs)

ament_package()