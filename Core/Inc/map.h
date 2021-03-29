#ifndef __MAP_H
#define __MAP_H

#include "main.h"

//Ptr size must be 4
HAL_StatusTypeDef GetDeviceId(uint8_t *data);

HAL_StatusTypeDef DisplayEmoji(uint8_t emoji, uint16_t duration_time, char forever_flag);

//Ptr size must be 64
void DisplayBuffer(void);

void SetBuffer(void);

#endif
