#ifndef CYBERDOG_OTA_MD5UTIL_HPP
#define CYBERDOG_OTA_MD5UTIL_HPP

#include <stdint.h>
#include <string>
#include <utility>

/**
 * @file   MD5Util.h
 * @brief  MD5 toolbox
 */

// static class
class MD5Util
{
public:
	//128 bit binary data
	static std::string md5_bin(const std::string& str);
	//128 bit hex string style, lower case
	static std::string md5_string_32(const std::string& str);
	//64  bit hex string style, lower case
	static std::string md5_string_16(const std::string& str);

	//128 bit integer style
	static std::pair<uint64_t, uint64_t> md5_integer_32(const std::string& str);
	//64  bit integer style
	static uint64_t md5_integer_16(const std::string& str);
};

#endif // CYBERDOG_OTA_MD5UTIL_HPP

