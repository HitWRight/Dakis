#include "map.h"

uint8_t own_ships[8][8];
uint8_t shots_fired[8][8];

typedef enum {
	horizontal = 0,
	vertical,
} orientation;

typedef struct {
	uint8_t x:3;
	uint8_t y:3;
	orientation rotation;
} current_placement;

void FillOutMap() {
	
}
