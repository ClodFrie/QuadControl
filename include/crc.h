#ifndef CRC_H
#define CRC_H

#include <stddef.h>

unsigned crc8(unsigned crc, unsigned char const *data, size_t len);

#endif