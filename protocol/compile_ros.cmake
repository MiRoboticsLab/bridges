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
# 功能说明: 统计该工程下指定类型的 ros 文件
# :param target: 待统计协议类型
# :type target: 字符串
# :param ARGN: 包含 topic、service、action 定义的接口文件类型
# :type ARGN: 字符串列表
# :param LOG: 如果设置 LOG 则打印日志
# :type LOG: 选项
# 举例:
# 1. get_ros_protocol_file(${PROJECT_NAME} "msg")         # 加载所有 msg 文件
# 2. get_ros_protocol_file(${PROJECT_NAME} "srv")         # 加载所有以 srv 文件
# 3. get_ros_protocol_file(${PROJECT_NAME} "action")      # 加载所有 action 文件
# 4. get_ros_protocol_file(${PROJECT_NAME} "msg" LOG)     # 加载所有 msg 协议，并输出日志
#
macro(get_ros_protocol_file target)
  cmake_parse_arguments(_ARG "LOG" "" "" ${ARGN})
  file(GLOB_RECURSE _ros_protocol_list RELATIVE ${PROJECT_SOURCE_DIR} ros/${_ARG_UNPARSED_ARGUMENTS}/*.${_ARG_UNPARSED_ARGUMENTS})  # 以相对路径方式递归包含目标文件
  list(LENGTH _ros_protocol_list _ros_protocol_list_number)
  if(_ros_protocol_list_number)
    list(APPEND ros_protocol_list ${_ros_protocol_list})
    if(_ARG_LOG)
      message("\n ┏━[${_ARG_UNPARSED_ARGUMENTS}]> 搜索到${_ros_protocol_list_number}个${_ARG_UNPARSED_ARGUMENTS}协议文件")
      set(_loop_var_index 1)
      foreach(_loop_var IN LISTS _ros_protocol_list)
        message(" ┠─[${_loop_var_index}]─> ${_loop_var}")
        math(EXPR _loop_var_index "(${_loop_var_index}+1)")
      endforeach()
      list(LENGTH ros_protocol_list ros_protocol_list_number)
      message(" ┗━[ROS]> 共计${ros_protocol_list_number}个 ROS 协议文件")
    endif()
  endif()
endmacro()

#
# 功能说明: 打印当前时间
# :param target: 消息内容
# :type target: 字符串
# :param LOG: 如果设置 LOG 则打印日志
# :type LOG: 选项
# 举例:
# 1. print_time("msg")         # 打印当前时间
#
function(print_time msg)
  string(TIMESTAMP COMPILE_TIME_1 %Y%m%d_%H%M%S)
  message(" 🌹 [${msg}]> ${COMPILE_TIME_1}")
endfunction()
