#ifndef __GRAPHICS_H
#define __GRAPHICS_H

#include "main.h"

//Ptr size must be 4
HAL_StatusTypeDef GetDeviceId(uint8_t *data);

HAL_StatusTypeDef DisplayEmoji(uint8_t emoji, uint16_t duration_time, char forever_flag);

//Ptr size must be 64
void DisplayBuffer(void);

void PaintTiles(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);

#endif
