/* 小容量bootloader向けにtableを使わないCRC32Cを実装する。 */
#include "boot_crc32c.h"

#include <stdint.h>

uint32_t boot_crc32c(const void * data, size_t size)
{
  const uint8_t * bytes = (const uint8_t *)data;
  uint32_t crc = UINT32_MAX;
  for (size_t index = 0; index < size; index++) {
    crc ^= bytes[index];
    for (unsigned int bit = 0; bit < 8U; bit++) {
      crc = (crc >> 1U) ^ ((crc & 1U) != 0U ? UINT32_C(0x82F63B78) : 0U);
    }
  }
  return ~crc;
}
