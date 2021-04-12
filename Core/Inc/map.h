#ifndef __MAP_H
#define __MAP_H

#include "main.h"

typedef enum {
	CL_SEA = 0xA7,
	CL_SHIP = 0x5B,
	CL_TEMP_SHIP = 0x2A,
	CL_SHOT = 0xFF,
	CL_SELECTION = 0xFE,
	CL_HIT = 0x01
} Color;

typedef enum {
	horizontal = 0x01,
	vertical = 0x02,
} orientation;

typedef struct {
	uint8_t x:3;
	uint8_t y:3;
} position;

typedef struct {
	uint8_t enabled:1;
	uint8_t x:3;//0-7
	uint8_t y:3;//0-7
	uint8_t size:3;//0-7
	orientation orientation;
} ship;

typedef enum {
	SH_None,
	SH_Miss,
	SH_Hit
} ShotInfo;

void AddShip(ship ship);
void AddShot(uint8_t x, uint8_t y, ShotInfo info);
void AddLocalShot(uint8_t x, uint8_t y);
void FillOutOwnMap(void);

void PaintSea(void);
void PaintShips(void);
void PaintTempShip(void);
void PaintLocalShots(void);
void PaintDistantShots(void);
void PaintCursor(void);

position GetCursor(void);
void SetCursor(position pos);

ship GetNextShip(void);
ship GetTemporaryShipLocation(void);
void SetTemporaryShipLocation(ship ship);

#endif
