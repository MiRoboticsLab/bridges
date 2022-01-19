cmake_minimum_required(VERSION 3.5)
project(athena_cyberdog_app)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(./common.cmake)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(include)
# Proto file
get_filename_component(rg_proto "./protos/cyberdog_app.proto" ABSOLUTE)
get_filename_component(rg_proto_path "${rg_proto}" PATH)

# Generated sources
set(rg_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/cyberdog_app.pb.cc")
set(rg_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/cyberdog_app.pb.h")
set(rg_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/cyberdog_app.grpc.pb.cc")
set(rg_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/cyberdog_app.grpc.pb.h")

set(executable_name app_server)
set(library_name ${executable_name}_core)
set(executable_name_action app_action_client)
set(dependencies
  builtin_interfaces
  rclcpp
  rclcpp_action
  std_msgs
)

add_custom_command(
      OUTPUT "${rg_proto_srcs}" "${rg_proto_hdrs}" "${rg_grpc_srcs}" "${rg_grpc_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${rg_proto_path}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${rg_proto}"
  DEPENDS "${rg_proto}")

# Include generated *.pb.h files
include_directories("${CMAKE_CURRENT_BINARY_DIR}")
# rg_grpc_proto
add_library(rg_grpc_proto
  ${rg_grpc_srcs}
  ${rg_grpc_hdrs}
  ${rg_proto_srcs}
  ${rg_proto_hdrs})

target_link_libraries(rg_grpc_proto
  ${_REFLECTION}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF})


add_library(${library_name}
  src/cyberdog_app.cpp
  src/cyberdog_app_client.cpp
  src/cyberdog_app_server.cpp
  src/timer_interval.cpp
)

target_link_libraries(${library_name}
    rg_grpc_proto
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF})

ament_target_dependencies(${library_name}
  ${dependencies}
)

# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_executable(${executable_name}
  src/main.cpp
)

#add_executable(${executable_name_action}
#  src/action_clients.cpp
#)

#ament_target_dependencies(${executable_name_action}
#  ${dependencies}
#)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name}
  ${library_name}
  #
)

#target_link_libraries(${executable_name_action}
#  ${library_name}
#  lcm
#)


install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#install(TARGETS ${executable_name_action}
#  RUNTIME DESTINATION lib/${PROJECT_NAME}
#)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
