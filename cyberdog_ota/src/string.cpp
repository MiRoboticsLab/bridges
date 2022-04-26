#include "cyberdog_ota/string.hpp"

#include <algorithm>
#include <cstdarg>
#include <fstream>
#include <sstream>

#include <boost/algorithm/string.hpp>

namespace cyberdog {
namespace {

void StringAppendV(std::string* dst, const char* format, va_list ap) 
{
  // First try with a small fixed size buffer.
  static const int kFixedBufferSize = 1024;
  char fixed_buffer[kFixedBufferSize];

  // It is possible for methods that use a va_list to invalidate
  // the data in it upon use.  The fix is to make a copy
  // of the structure before using it and use that copy instead.
  va_list backup_ap;
  va_copy(backup_ap, ap);
  int result = vsnprintf(fixed_buffer, kFixedBufferSize, format, backup_ap);
  va_end(backup_ap);

  if (result < kFixedBufferSize) {
    if (result >= 0) {
      // Normal case - everything fits.
      dst->append(fixed_buffer, result);
      return;
    }

    if (result < 0) {
      // Just an error.
      return;
    }
  }

  // Increase the buffer size to the size requested by vsnprintf,
  // plus one for the closing \0.
  const int variable_buffer_size = result + 1;
  std::unique_ptr<char[]> variable_buffer(new char[variable_buffer_size]);

  // Restore the va_list before we use it again.
  va_copy(backup_ap, ap);
  result =
      vsnprintf(variable_buffer.get(), variable_buffer_size, format, backup_ap);
  va_end(backup_ap);

  if (result >= 0 && result < variable_buffer_size) {
    dst->append(variable_buffer.get(), result);
  }
}

bool IsNotWhiteSpace(const int character) 
{
  return character != ' ' && character != '\n' && character != '\r' &&
         character != '\t';
}

}  // namespace

std::string StringPrintf(const char* format, ...) 
{
  va_list ap;
  va_start(ap, format);
  std::string result;
  StringAppendV(&result, format, ap);
  va_end(ap);
  return result;
}

std::string StringReplace(const std::string& str, const std::string& old_str,
                          const std::string& new_str) {
  if (old_str.empty()) {
    return str;
  }
  size_t position = 0;
  std::string mod_str = str;
  while ((position = mod_str.find(old_str, position)) != std::string::npos) {
    mod_str.replace(position, old_str.size(), new_str);
    position += new_str.size();
  }
  return mod_str;
}

std::string StringGetAfter(const std::string& str, const std::string& key) {
  if (key.empty()) {
    return str;
  }
  std::size_t found = str.rfind(key);
  if (found != std::string::npos) {
    return str.substr(found + key.length(),
                      str.length() - (found + key.length()));
  }
  return "";
}

std::vector<std::string> StringSplit(const std::string& str,
                                     const std::string& delim) {
  std::vector<std::string> elems;
  boost::split(elems, str, boost::is_any_of(delim), boost::token_compress_on);
  return elems;
}

bool StringStartsWith(const std::string& str, const std::string& prefix) {
  return !prefix.empty() && prefix.size() <= str.size() &&
         str.substr(0, prefix.size()) == prefix;
}

void StringLeftTrim(std::string* str) {
  str->erase(str->begin(),
             std::find_if(str->begin(), str->end(), IsNotWhiteSpace));
}

void StringRightTrim(std::string* str) {
  str->erase(std::find_if(str->rbegin(), str->rend(), IsNotWhiteSpace).base(),
             str->end());
}

void StringTrim(std::string* str) {
  StringLeftTrim(str);
  StringRightTrim(str);
}

void StringToLower(std::string* str) {
  std::transform(str->begin(), str->end(), str->begin(), ::tolower);
}

void StringToUpper(std::string* str) {
  std::transform(str->begin(), str->end(), str->begin(), ::toupper);
}

bool StringContains(const std::string& str, const std::string& sub_str) {
  return str.find(sub_str) != std::string::npos;
}

}  // namespace cyberdog
