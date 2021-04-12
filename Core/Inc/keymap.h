#ifndef __KEYMAP_H
#define __KEYMAP_H

#include "main.h"

typedef enum {
	KC_None = 0x0,
	KC_Left = 0x1,
	KC_Right = 0x2,
	KC_Up = 0x4,
	KC_Down = 0x8,
	KC_Shoot = 0x10,
	KC_Alt = 0x20
} KeyClick;

KeyClick GetKeyClick(void); 

#endif
