

#ifndef CYBERDOG_OTA_CRC32C_HPP_
#define CYBERDOG_OTA_CRC32C_HPP_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t crc32c(uint32_t crc, const void *buf, size_t len);

void crc32c_global_init (void);

#ifdef __cplusplus
}
#endif

#endif  // CYBERDOG_OTA_CRC32C_HPP_
