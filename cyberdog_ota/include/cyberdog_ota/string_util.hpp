#ifndef CYBERDOG_OTA_STRINGUTIL_H_
#define CYBERDOG_OTA_STRINGUTIL_H_

#include <string>
#include <vector>

/**
 * @file   StringUtil.h
 * @brief  String toolbox
 */

// static class
class StringUtil
{
public:
	static size_t url_decode(char *str, size_t len);
	static void url_decode(std::string& str);
	static std::string url_encode(const std::string& str);
	static std::string url_encode_component(const std::string& str);
	static std::vector<std::string> split(const std::string& str, char sep);
	static std::string strip(const std::string& str);
	static bool start_with(const std::string& str, const std::string& prefix);

	//this will filter any empty result, so the result vector has no empty string
	static std::vector<std::string> split_filter_empty(const std::string& str, char sep);
};

#endif  // CYBERDOG_OTA_STRINGUTIL_H_

