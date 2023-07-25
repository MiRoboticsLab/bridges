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

#ifndef TRANSMIT_FILES_HPP_
#define TRANSMIT_FILES_HPP_

#include <sys/time.h>
#include <string>
#include <sstream>
#include <set>
#include <mutex>
#include <atomic>
#include <cstring>
#include "./cyberdog_app.grpc.pb.h"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "protocol/srv/camera_service.hpp"

#define CHUNK_SIZE 4194304  // 4MB

namespace cyberdog
{
namespace bridges
{
/**
 * @brief Class for transmit files via gRPC
 */
class TransmitFiles
{
public:
  /**
   * @brief Get and send a file produced from camera
   * @param writer FileChunk writer
   * @param result Result from camera service
   * @param msg Camsera service response message
   * @return true Successfully sent file
   * @return false Failed to send file
   */
  static bool ReturnCameraFile(
    ::grpc::ServerWriter<::grpcapi::FileChunk> * writer,
    uint8_t result,
    const std::string & msg)
  {
    if (result != 0) {
      SendErrorCode(result, writer);
      return false;
    }
    std::string file_name;
    size_t file_size = 0;
    if (!ParseCameraServiceResponseString(msg, file_size, file_name)) {
      ERROR("Not able to parse msg from camera_service.");
      SendErrorCode(8, writer);
      return false;
    }
    return SendFile(writer, file_name, "/home/mi/Camera/", file_size);
  }

  static int ListUnuploadedfiles(const std::string & path, std::set<std::string> & files)
  {
    std::string cmd("ls ");
    cmd += path;
    FILE * fp;
    char buf[128] {'\0'};
    if ((fp = popen(cmd.c_str(), "r")) == NULL) {
      printf("failed to popen");
      return -1;
    }
    while (fgets(buf, sizeof(buf), fp) != NULL) {
      std::string file_name(buf);
      file_name = file_name.substr(0, file_name.length() - 1);
      files.insert(file_name);
      memset(buf, '\0', sizeof(buf));
    }
    return pclose(fp);
  }

  /**
   * @brief Send error code with the writer
   * @param code Error code
   * @param writer FileChunk writer
   */
  static void SendErrorCode(uint32_t code, ::grpc::ServerWriter<::grpcapi::FileChunk> * writer)
  {
    ERROR("Sendfile error: %d", code);
    ::grpcapi::FileChunk chunk;
    chunk.set_error_code(code);
    writer->Write(chunk);
  }

  /**
   * @brief Send file implementation
   * @param writer FileChunk writer
   * @param file_name File name
   * @param name_prefix Folder of the file
   * @param file_size File size
   * @param file_set Container to record file names that are transmiting
   * @return true Successfully sent file
   * @return false Failed to send file
   */
  static bool SendFile(
    ::grpc::ServerWriter<::grpcapi::FileChunk> * writer,
    const std::string & file_name,
    const std::string & name_prefix = "/home/mi/Camera/",
    size_t file_size = 0,
    std::set<std::string> * file_set = nullptr)
  {
    static std::set<std::string> uncomplete_files;
    static std::mutex file_set_mutex;
    static std::string default_folder = "/home/mi/Camera/";
    if (!writer && file_set) {  // check undownloaded files
      std::unique_lock<std::mutex> lock(file_set_mutex);
      if (file_set->empty()) {
        for (auto & file : uncomplete_files) {
          file_set->insert(file);
          INFO_STREAM("Add auto saved file: " << file << " to output list.");
        }
      } else {
        *file_set = uncomplete_files;
        INFO("Get uncomplete list.");
      }
      ListUnuploadedfiles(default_folder, *file_set);
      return true;
    } else if (!writer) {
      ERROR("Illegal calling SendFile!");
      return false;
    }

    thread_counts_++;
    ::grpcapi::FileChunk chunk;
    chunk.set_file_name(file_name);
    INFO_STREAM("The file name is " << file_name << ", size is " << uint32_t(file_size));
    std::string file_name_with_path = name_prefix + file_name;
    char data[CHUNK_SIZE];
    std::ifstream infile;
    infile.open(file_name_with_path, std::ifstream::in | std::ifstream::binary);
    if (!infile.is_open()) {
      ERROR_STREAM("Failed to open file: " << file_name_with_path);
      SendErrorCode(9, writer);
      return false;
    }
    if (file_size == 0) {
      infile.seekg(0, infile.end);
      file_size = infile.tellg();
      infile.seekg(0, infile.beg);
    }
    chunk.set_file_size(uint32_t(file_size));
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
        file_set_mutex.lock();
        uncomplete_files.insert(file_name);
        file_set_mutex.unlock();
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

    std::unique_lock<std::mutex> lock(file_set_mutex);
    if (!uncomplete_files.empty()) {
      auto itr = uncomplete_files.find(file_name);
      if (itr != uncomplete_files.end()) {
        uncomplete_files.erase(itr);
      }
    }
    thread_counts_--;
    return true;
  }
  /**
   * @brief Parse camera service response message string
   * @param str Message string
   * @param file_size Parsed file size
   * @param file_name Parsed file name
   * @return true Successfully parsed
   * @return false Failed to parse
   */
  static bool ParseCameraServiceResponseString(
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

  static std::atomic_int thread_counts_;
};  // class TransmitFiles
}  // namespace bridges
}  // namespace cyberdog
#endif  // TRANSMIT_FILES_HPP_
