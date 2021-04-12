#include "map.h"
#include "graphics.h"

const uint8_t ships_to_place_count = 8;
uint8_t ships_to_place[ships_to_place_count] = {3, 3, 2, 2, 1, 1, 1, 1};
uint8_t currently_placed_ship = 0;

ship own_ships[8];
ship temporary_ship;
uint8_t ship_count = 0;


uint8_t shots_got[8];

position cursor;




ShotInfo shots_fired[8][8];


void AddShip(ship ship) {
	//TODO: Check Placement
	own_ships[ship_count++] = ship;
}

void AddShot(uint8_t x, uint8_t y, ShotInfo info) {
	shots_fired[y][x] = info;
}

void AddLocalShot(uint8_t x, uint8_t y) {
	shots_got[y] |= 0x1 >> x;
}

uint8_t CheckIfHit(uint8_t x, uint8_t y) {
	for (int i=0; i<ship_count; i++)
	{
		int w = 1;
		int h = 1;
		if(own_ships[i].orientation & horizontal)
		{
			w = own_ships[i].size;
		}
		else
		{
			h = own_ships[i].size;
		}
		
		if (x >= own_ships[i].x && x < own_ships[i].x + w
		 && y >= own_ships[i].y && y < own_ships[i].y + h)
		return 1;
	}
	return 0;
}

void drawShip(ship ship, Color color) {
		int w = 1;
		int h = 1;
		if(ship.orientation & horizontal)
		{
			w = ship.size;
		}
		else
		{
			h = ship.size;
		}
		
		PaintTiles(ship.x,
							 ship.y,
							 w,
							 h,
							 color);
}

void PaintSea() {
	PaintTiles(0, 0, 8, 8, CL_SEA);
}

void PaintShips() {
	for (int i=0; i<ship_count; i++)
	{
		 drawShip(own_ships[i], CL_SHIP);
	}
}

void PaintTempShip() {
	drawShip(temporary_ship, CL_TEMP_SHIP);
}

void PaintLocalShots() {
	//Place shots_fired
	for(int y = 0; y<8; y++) {
		for (int x = 0; x<8; x++) {
			if ( (shots_got[y] >> x) & 0x1) 
			{
				if(CheckIfHit(x,y))
					PaintTiles(x,y,1,1,CL_HIT);
				else
					PaintTiles(x,y,1,1,CL_SHOT);
			}
		}
	}
}


void PaintDistantShots() {
	for(int y = 0; y<8; y++) {
		for (int x = 0; x<8; x++) {
			if(shots_fired[y][x] != SH_None)
			{
				if(shots_fired[y][x] == SH_Miss)
					PaintTiles(x,y,1,1,CL_SHOT);
				else
					PaintTiles(x,y,1,1,CL_HIT);
			}
		}
	}
}

uint8_t cursor_time;
void PaintCursor() {
	if(cursor_time++ % 2)
		PaintTiles(cursor.x, cursor.y, 1, 1, CL_SELECTION);
}

position GetCursor()
{
	return cursor;
}

void SetCursor(position pos)
{
	cursor = pos;
}

ship GetNextShip() 
{
	ship result;
	
	result.x = 0;
	result.y = 0;
	result.orientation = horizontal;
	result.size = ships_to_place[currently_placed_ship];
	
	if (currently_placed_ship++ == ships_to_place_count) 
		result.enabled = 0;
	else
		result.enabled = 1;
	
	return result;
}

void SetTemporaryShipLocation(ship ship) {
	temporary_ship = ship;
}

ship GetTemporaryShipLocation() {
	return temporary_ship;
}

