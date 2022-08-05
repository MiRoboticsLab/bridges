// Copyright (c) 2022 Xiaomi Corporation
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

#ifndef TRANSMIT_FILES_HPP_
#define TRANSMIT_FILES_HPP_

#include <sys/time.h>
#include <string>
#include <sstream>
#include "./cyberdog_app.grpc.pb.h"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "protocol/srv/camera_service.hpp"

#define CHUNK_SIZE 4194304  // 4MB

namespace carpo_cyberdog_app
{
class TransmitFiles
{
public:
  static bool ReturnCameraFile(
    ::grpc::ServerWriter<::grpcapi::FileChunk> * writer,
    uint8_t result,
    const std::string & msg)
  {
    ::grpcapi::FileChunk chunk;
    if (result != 0) {
      SendErrorCode(result, writer);
      return false;
    }
    std::string file_name, file_name_with_path;
    size_t file_size = 0;
    if (!parseCameraServiceResponseString(msg, file_size, file_name)) {
      ERROR("Not able to parse msg from camera_service.");
      SendErrorCode(8, writer);
      return false;
    }
    chunk.set_file_name(file_name);
    chunk.set_file_size(uint32_t(file_size));
    INFO_STREAM("The file name is " << file_name << ", size is " << uint32_t(file_size));
    file_name_with_path = "/home/mi/Camera/" + file_name;
    char data[CHUNK_SIZE];
    std::ifstream infile;
    infile.open(file_name_with_path, std::ifstream::in | std::ifstream::binary);
    if (!infile.is_open()) {
      ERROR_STREAM("Failed to open file: " << file_name_with_path);
      SendErrorCode(9, writer);
      return false;
    }
    chunk.set_error_code(0);
    int chunk_num = 0;
    timeval start, end;
    INFO_STREAM("Start sending file: " << file_name);
    bool unexpected_interruption = false;
    gettimeofday(&start, NULL);
    while (!infile.eof()) {
      infile.read(data, CHUNK_SIZE);
      chunk.set_buffer(data, infile.gcount());
      int trial_times = 0;
      while (!writer->Write(chunk)) {
        ERROR("Not able to send file chunk. Please check max message size settings.");
        if (trial_times++ > 2) {
          unexpected_interruption = true;
          break;
        }
      }
      if (unexpected_interruption) {
        break;
      }
      INFO_STREAM("Finish sending chunk num " << chunk_num++);
    }
    infile.close();
    if (unexpected_interruption) {
      return false;
    }
    gettimeofday(&end, NULL);
    INFO_STREAM(
      "Finish sending file: " << file_name_with_path <<
        ", it takes: " << double(end.tv_sec - start.tv_sec) + double(end.tv_usec - start.tv_usec) /
        1000000 << " seconds.");
    remove(file_name_with_path.c_str());
    return true;
  }

  static void SendErrorCode(uint32_t code, ::grpc::ServerWriter<::grpcapi::FileChunk> * writer)
  {
    ::grpcapi::FileChunk chunk;
    chunk.set_error_code(code);
    writer->Write(chunk);
  }

private:
  static bool parseCameraServiceResponseString(
    const std::string & str,
    size_t & file_size,
    std::string & file_name)
  {
    std::stringstream ss;
    size_t comma_index = str.find(',', 0);
    if (comma_index == string::npos) {
      ERROR("error while parsing camera_service respose msg");
      return false;
    }
    file_name = str.substr(0, comma_index);
    ss << str.substr(comma_index + 1);
    ss >> file_size;
    return true;
  }
};  // class TransmitFiles
}  // namespace carpo_cyberdog_app
#endif  // TRANSMIT_FILES_HPP_"
