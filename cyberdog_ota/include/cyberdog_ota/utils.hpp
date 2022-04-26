#ifndef CYBERDOG_OTA_UTILS_HPP_
#define CYBERDOG_OTA_UTILS_HPP_

#include <string>

namespace cyberdog
{

void Strrevn(char* s, const int len);

long long FileSize(const char* path);
bool buf_to_file(const char* path, const char* buf, const size_t len, const char* mode, long int offset = 0);

bool file_to_string(const char* path, std::string& contents, long int offset = 0);
bool string_to_file(const char* path, const std::string& contents, const char* mode, long int offset = 0);
bool del_file(const char* path);

}  // namespace cyberdog

#endif  // CYBERDOG_OTA_UTILS_HPP_