cmake_minimum_required(VERSION 3.5)
project(android_grpc)

# include_directories(
#   ${CMAKE_SOURCE_DIR}/include
# )
include_directories("${CMAKE_CURRENT_BINARY_DIR}/../..")

# message('--------------------------------')
# message(${CMAKE_CURRENT_BINARY_DIR})
# message('--------------------------------')

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find_package(cyberdog_grpc REQUIRED)

set(executable_name dog_server)
set(library_name app_server_core)
# set(dependencies
#   cyberdog_grpc
# )

add_executable(${executable_name}
  android_grpc_simulator/main.cpp
)

# ament_target_dependencies(${executable_name}
#   ${dependencies}
# )

target_link_libraries(${executable_name}
  ${library_name}
)

target_include_directories(${executable_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)