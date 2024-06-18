#include "main.h"
#ifndef CRC8_H
#define CRC8_H

void crc8_init(uint8_t poly);
uint8_t crc8_calc(uint8_t *data, uint8_t len);

#endif //CRC8_H
