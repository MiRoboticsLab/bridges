cmake_minimum_required(VERSION 3.8)
project(cyberdog_ota)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


option(BUILD_TESTING "" ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(protocol REQUIRED)

find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

find_package(Boost REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

find_package(OpenSSL REQUIRED)
include_directories(${OPENSSL_INCLUDE_DIR})

find_package(workflow)
include_directories(${WORKFLOW_INCLUDE_DIR})

set(ALL_OTA_SRCS
  src/crc32c.cpp
  src/client.cpp
  src/client_interface.cpp
  src/command_interface.cpp
  src/command_mcu.cpp
  src/command_nx.cpp
  src/factory_interface.cpp
  src/factory.cpp
  src/file_system.cpp
  src/json_parser.cpp
  src/manager_mcu.cpp
  src/manager_nx.cpp
  src/manager.cpp
  src/md5_util.cpp
  src/ota_node.cpp
  src/cybredog_ota_main.cpp
  src/ota_options.cpp
  src/protocol_interface.cpp
  src/protocol_app_message.cpp 
  src/protocol_server_message.cpp 
  src/rbtree.cpp
  src/server_interface.cpp
  src/server.cpp
  src/string_util.cpp
  src/string.cpp
  src/system_info.cpp
  src/threading.cpp
  src/timer.cpp
  src/utils.cpp
  src/version.cpp
)

set(ROS_DEPENDENCIES
  rclcpp 
  std_msgs
  std_srvs
  cyberdog_common
  protocol
)

add_executable(${PROJECT_NAME} ${ALL_OTA_SRCS})
ament_target_dependencies(${PROJECT_NAME} ${ROS_DEPENDENCIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${GLOG_INCLUDE_DIRS}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${Boost_INCLUDE_DIRS}") 
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${OPENSSL_INCLUDE_DIR}")

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${GLOG_LIBRARIES})
target_link_libraries(${PROJECT_NAME} OpenSSL::SSL OpenSSL::Crypto curl)
target_link_libraries(${PROJECT_NAME} ${LIB})


target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17



install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)


  include_directories(include)

  # TCP client
  add_executable(tcp_client_test test/TCPClient.cpp test/tcp_client_test.cpp)

  # TCP server
  add_executable(tcp_server_test test/TCPServer.cpp test/tcp_server_test.cpp)
  target_link_libraries(tcp_server_test pthread)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
