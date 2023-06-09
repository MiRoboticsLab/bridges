// Copyright (c) 2023 Xiaomi Corporation
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


#ifndef TIME_INTERVAL_HPP_
#define TIME_INTERVAL_HPP_

#include <stdio.h>
#include <time.h>
#include <limits>
#include <fstream>
#include <iostream>
#include "cyberdog_common/cyberdog_log.hpp"

class TimeInterval
{
public:
  TimeInterval();
  void init();
  bool check(unsigned int seconds);
  bool check_once(uint32_t seconds);

private:
  time_t first, second;
};
#endif  // TIME_INTERVAL_HPP_
