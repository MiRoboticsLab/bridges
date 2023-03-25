// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#ifndef FILE_UPLOADING__FILE_UPLOADING_H_
#define FILE_UPLOADING__FILE_UPLOADING_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Upload a file to backend server
 * @param file_name File name with full path
 * @param id User ID
 * @param http_result_code Response from HTTP
 * @return Error code from enum cyberdog::bridge::Backend_Http::ErrorCode
 */
int uploadFile(const char * file_name, int id, int * http_result_code);

/**
 * @brief Upload a string message to backend server
 * @param info String message
 * @param id User ID
 * @param http_result_code Response from HTTP
 * @return Error code from enum cyberdog::bridge::Backend_Http::ErrorCode
 */
int sendWarningInfo(const char * info, int id, int * http_result_code);

#ifdef __cplusplus
}
#endif

#endif  // FILE_UPLOADING__FILE_UPLOADING_H_
