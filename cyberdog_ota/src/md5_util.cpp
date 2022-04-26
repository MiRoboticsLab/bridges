#include <openssl/md5.h>
#include <string.h>
#include <string>
#include "cyberdog_ota/md5_util.hpp"

static inline void __md5(const std::string& str, unsigned char *md)
{
	MD5_CTX ctx;
	MD5_Init(&ctx);
	MD5_Update(&ctx, str.c_str(), str.size());
	MD5_Final(md, &ctx);
}

std::string MD5Util::md5_bin(const std::string& str)
{
	unsigned char md[16];

	__md5(str, md);
	return std::string((const char *)md, 16);
}

static inline char __hex_char(int v)
{
	return v < 10 ? '0' + v : 'a' + v - 10;
}

static inline void __plain_hex(char *s, int ch)
{
	*s = __hex_char(ch / 16);
	*(s + 1) = __hex_char(ch % 16);
}

std::string MD5Util::md5_string_32(const std::string& str)
{
	unsigned char md[16];
	char out[32];

	__md5(str, md);
	for (int i = 0; i < 16; i++)
		__plain_hex(out + (i * 2), md[i]);

	return std::string((const char *)out, 32);
}

std::string MD5Util::md5_string_16(const std::string& str)
{
	unsigned char md[16];
	char out[16];

	__md5(str, md);
	for (int i = 0; i < 8; i++)
		__plain_hex(out + (i * 2), md[i + 4]);

	return std::string((const char *)out, 16);
}

std::pair<uint64_t, uint64_t> MD5Util::md5_integer_32(const std::string& str)
{
	unsigned char md[16];
	std::pair<uint64_t, uint64_t> res;

	__md5(str, md);
	memcpy(&res.first, md, sizeof (uint64_t));
	memcpy(&res.second, md + 8, sizeof (uint64_t));
	return res;
}

uint64_t MD5Util::md5_integer_16(const std::string& str)
{
	unsigned char md[16];
	uint64_t res;

	__md5(str, md);
	memcpy(&res, md + 4, sizeof (uint64_t));
	return res;
}

