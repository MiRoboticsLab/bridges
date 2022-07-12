# Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# 功能说明: 统计该工程下指定类型的 lcm 文件
# :param target: 待统计协议类型
# :type target: 字符串
# :param ARGN: 包含 lcm 定义的接口文件类型
# :type ARGN: 字符串列表
# :param LOG: 如果设置 LOG 则打印日志
# :type LOG: 选项
# 举例:
# 1. get_lcm_protocol_file(${PROJECT_NAME} "*")      # 加载所有 lcm 文件
# 2. get_lcm_protocol_file(${PROJECT_NAME} "motion") # 加载所有以 motion 结尾的 lcm 文件
# 3. get_lcm_protocol_file(${PROJECT_NAME} "*" LOG)  # 加载所有 lcm 协议，并输出日志
#
macro(get_lcm_protocol_file target)
  cmake_parse_arguments(_ARG "LOG" "" "" ${ARGN})
  file(GLOB_RECURSE _lcm_protocol_list RELATIVE ${PROJECT_SOURCE_DIR} lcm/*${_ARG_UNPARSED_ARGUMENTS}.lcm)  # 以相对路径方式递归包含目标文件
  list(LENGTH _lcm_protocol_list _lcm_protocol_list_number)
  if(_lcm_protocol_list_number)
    list(APPEND lcm_protocol_list ${_lcm_protocol_list})
    if(_ARG_LOG)
      message("\n ┏━[${_ARG_UNPARSED_ARGUMENTS}]> 搜索到${_lcm_protocol_list_number}个${_ARG_UNPARSED_ARGUMENTS}协议文件")
      set(_loop_var_index 1)
      foreach(_loop_var IN LISTS _lcm_protocol_list)
        message(" ┠─[${_loop_var_index}]─> ${_loop_var}")
        math(EXPR _loop_var_index "(${_loop_var_index}+1)")
      endforeach()
      list(LENGTH lcm_protocol_list lcm_protocol_list_number)
      message(" ┗━[lcm]> 共计${lcm_protocol_list_number}个 lcm 协议文件")
    endif()
  endif()
endmacro()

#
# 功能说明: 编译指定的 lcm 文件生成相应语言接口文件(默认 c++)
# :param target: 待统计协议类型
# :type target: 字符串
# :param ARGN: lcm 文件/列表
# :type ARGN: 字符串列表
# :param C: 如果设置 C 则生成 c 接口文件
# :type C: 选项
# :param PYTHON: 如果设置 PYTHON 则生成 python 接口文件
# :type PYTHON: 选项
# :param LOG: 如果设置 LOG 则打印日志
# :type LOG: 选项
# 举例:
#   准备输入: set(lcm_protocol_list)
#            get_lcm_protocol_file(${PROJECT_NAME} "*")
# 1. 编译指定的 lcm 文件，输出 c++ 接口文件
#     lcm_generate_interfaces(${PROJECT_NAME} ${lcm_protocol_list})
# 2. 编译指定的 lcm 文件，输出 c++ 接口文件，并输出日志
#     lcm_generate_interfaces(${PROJECT_NAME} ${lcm_protocol_list} LOG)
# 3. 编译指定的 lcm 文件，输出 c++、C 接口文件，
#     lcm_generate_interfaces(${PROJECT_NAME} ${lcm_protocol_list} C)
# 4. 编译指定的 lcm 文件，输出 c++、C、python 接口文件
#     lcm_generate_interfaces(${PROJECT_NAME} ${lcm_protocol_list} C PYTHON)
#
macro(lcm_generate_interfaces target)
  cmake_parse_arguments(_ARG "C;PYTHON;LOG" "" "" ${ARGN})
  if(NOT _ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "lcmidl_generate_interfaces() 没有任何接口文件的情况下调用了")
  endif()
  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR "lcmidl_generate_interfaces() 必须在 ament_package() 之前调用")
  endif()
  set(_interface_tuples "")
  foreach(_file ${_ARG_UNPARSED_ARGUMENTS})
    if(IS_ABSOLUTE "${_file}")
      string(FIND "${_file}" ":" _index)
      if(_index EQUAL -1)
        message(FATAL_ERROR "lcmidl_generate_interfaces() 传递的绝对文件 '${_file}' 必须表示为绝对基本路径，用冒号分隔接口文件的相对路径")
      endif()
      string(REGEX REPLACE ":([^:]*)$" "/\\1" _abs_file "${_file}")
      if(NOT EXISTS "${_abs_file}")
        message(FATAL_ERROR "lcmidl_generate_interfaces() 传递的文件 '${_abs_file}' 不存在")
      endif()
      list(APPEND _interface_tuples "${_file}")
    else()
      set(_abs_file "${CMAKE_CURRENT_SOURCE_DIR}/${_file}")
      if(NOT EXISTS "${_abs_file}")
        message(FATAL_ERROR "lcmidl_generate_interfaces() 传递的文件 '${_file}' 相对于 CMAKE_CURRENT_SOURCE_DIR '${CMAKE_CURRENT_SOURCE_DIR}' 不存在")
      endif()
      list(APPEND _interface_tuples "${CMAKE_CURRENT_SOURCE_DIR}:${_file}")
    endif()
  endforeach()
  set(_interface_files "")
  foreach(_tuple ${_interface_tuples})
    string(REGEX REPLACE ":([^:]*)$" "/\\1" _abs_interface "${_tuple}")
    list(APPEND _interface_files "${_abs_interface}")
    stamp("${_abs_interface}")
  endforeach()
  set(_ARG_CPP true)   # 默认生成 hpp
  if(_ARG_LOG)
    message("\n ┏━[缓存路径]>:${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_core/stamps")
    message(" ┗━[安装路径]>:${CMAKE_INSTALL_PREFIX}")
  endif()
  execute_process(COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/include/protocol/lcm/detail)
  if(_ARG_C)
    if(_ARG_LOG)
      message("\n ┏━[.h 文件路径]>:${CMAKE_INSTALL_PREFIX}/include/protocol/lcm")
      message(" ┗━[.c 文件路径]>:${CMAKE_INSTALL_PREFIX}/include/protocol/lcm/detail")
    endif()
    execute_process(COMMAND lcm-gen -c ${_interface_files} --c-hpath ${CMAKE_INSTALL_PREFIX}/include/protocol/lcm --c-cpath ${CMAKE_INSTALL_PREFIX}/include/protocol/lcm/detail)
  endif()
  if(_ARG_CPP)
    if(_ARG_LOG)
      message("\n ● [.hpp 文件路径]>:${CMAKE_INSTALL_PREFIX}/include/protocol/lcm")
    endif()
    execute_process(COMMAND lcm-gen -x ${_interface_files} --cpp-hpath ${CMAKE_INSTALL_PREFIX}/include/protocol/lcm)
  endif()
  if(_ARG_PYTHON)
    if(_ARG_LOG)
      message("\n ● [.py 文件路径]>:${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/protocol/lcm")
    endif()
    execute_process(COMMAND lcm-gen -p ${_interface_files} --ppath ${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/protocol/lcm)
  endif()
endmacro()
