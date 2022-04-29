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

#include "cyberdog_common/cyberdog_log.hpp"
class NetChecker
{
public:
  NetChecker()
  : need_run_(true), thread_(NULL), need_ping(true) {}
  ~NetChecker()
  {
    need_run_ = false;
    if (thread_ != NULL) {
      thread_->join();
    }
  }
  void pause()
  {
    need_ping = false;
    ip_ = "";
  }
  void set_ip(std::string ip)
  {
    if (ip != ip_) {
      ip_ = ip;
      need_ping = true;
      run();
    }
  }

private:
  void run()
  {
    if (thread_ == NULL) {
      thread_ = std::make_shared<std::thread>(&NetChecker::run_, this);
    }
  }

  void run_()
  {
    FILE * fp;
    char buf[128];
    while (need_run_) {
      if (need_ping) {
        std::string cmd = "ping -c 1 -w 2 " + ip_;
        if ((fp = popen(cmd.c_str(), "r")) == NULL) {
          ERROR("failed to popen");
          return;
        }

        while (fgets(buf, sizeof(buf), fp) != NULL) {
          INFO_STREAM(buf);
        }
        pclose(fp);
      }
      sleep(10);
    }
  }
  std::shared_ptr<std::thread> thread_;
  std::string ip_;
  bool need_run_;
  bool need_ping;
};

#endif  // NET_AVALIBLE_HPP_
