// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#ifndef PROTOCOL__CAN__TOML_ANALYSIS_HPP_
#define PROTOCOL__CAN__TOML_ANALYSIS_HPP_

#include <string>
#include <vector>
#include "toml/toml.hpp"

namespace cyberdog
{
namespace embed
{
enum class Field
{
  VARFIELD,
  ARRAYFIELD,
  CMDFIELD,
  STRUCTFIELD
};

typedef struct _base_region
{
  std::string description;
} base_region;

typedef struct _var_region : base_region
{
  std::string var_name;
  std::string can_id;
  std::string var_type;
  std::vector<uint8_t> parser_param;
  std::string parser_type;
  double var_zoom;
} var_region;


template<typename TDataClass>
class TomlAnalysis
{
public:
  explicit TomlAnalysis(std::string file_name)
  {
    try {
      v = toml::parse(file_name);
    } catch (const toml::exception & e) {
      std::cerr << e.what() << '\n';
    } catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
    } catch (...) {
      std::cerr << "toml analysis unkown exception." << '\n';
    }
  }

private:
  toml::value v;
};  // class CanParser
}  // namespace embed
}  // namespace cyberdog

#endif  // PROTOCOL__CAN__TOML_ANALYSIS_HPP_
