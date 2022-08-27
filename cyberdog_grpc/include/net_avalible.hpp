// Copyright (c) 2021 Xiaomi Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NET_AVALIBLE_HPP_
#define NET_AVALIBLE_HPP_
#include <stdio.h>
#include <unistd.h>

#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include "cyberdog_common/cyberdog_log.hpp"
class NetChecker
{
public:
  NetChecker()
  : need_run_(true), thread_(nullptr), need_ping_(false)
  {
    run();
  }
  ~NetChecker()
  {
    {
      std::unique_lock<std::mutex> lck(ip_mutex_);
      need_run_ = false;
      cv_.notify_all();
    }
    if (thread_ && thread_->joinable()) {
      thread_->join();
    }
  }
  void pause(bool pau = true)
  {
    std::unique_lock<std::mutex> lck(ip_mutex_);
    need_ping_ = !pau;
    cv_.notify_one();
  }
  void set_ip(const std::string & ip, bool start = true)
  {
    {
      std::unique_lock<std::mutex> lck(ip_mutex_);
      if (ip != ip_) {
        ip_ = ip;
      }
    }
    if (start && !ip.empty()) {
      pause(false);
    }
  }

private:
  void run()
  {
    if (!thread_) {
      thread_ = std::make_shared<std::thread>(&NetChecker::run_, this);
    }
  }

  void run_()
  {
    FILE * fp;
    char buf[128];
    std::unique_lock<std::mutex> lck(ip_mutex_);
    while (need_run_) {
      if (need_ping_) {
        cv_.wait_for(lck, std::chrono::seconds(10));
      } else {
        cv_.wait(lck);
      }
      if (!need_run_) {
        break;
      }
      if (!need_ping_) {
        continue;
      }
      std::string cmd = "ping -c 1 -w 2 " + ip_;
      if ((fp = popen(cmd.c_str(), "r")) == NULL) {
        ERROR("failed to popen");
        continue;
      }
      while (fgets(buf, sizeof(buf), fp) != NULL) {
        INFO_STREAM(buf);
      }
      pclose(fp);
    }
  }
  std::shared_ptr<std::thread> thread_;
  std::string ip_;
  bool need_run_;
  bool need_ping_;
  std::mutex ip_mutex_;
  std::condition_variable cv_;
};

#endif  // NET_AVALIBLE_HPP_
