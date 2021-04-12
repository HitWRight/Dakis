#include "keymap.h"

#define DOWN 0
#define UP 1

uint8_t last_key_state[6] = {UP, UP, UP, UP, UP, UP};

KeyClick GetKeyClick()
{
	KeyClick result = KC_None;
	uint8_t key_state;
	
	key_state = HAL_GPIO_ReadPin(GPIOA, Mygtukas_1_Pin);
	if (last_key_state[0] != key_state && last_key_state[0] == DOWN ) 
		result = KC_Left;
	last_key_state[0] = key_state;
	
	key_state = HAL_GPIO_ReadPin(GPIOA, Mygtukas_2_Pin);
	if (last_key_state[1] != key_state && last_key_state[1] == DOWN ) 
		result = KC_Up;
	last_key_state[1] = key_state;
	
	key_state = HAL_GPIO_ReadPin(GPIOA, Mygtukas_3_Pin);
	if (last_key_state[2] != key_state && last_key_state[2] == DOWN ) 
		result = KC_Down;
	last_key_state[2] = key_state;
	
	key_state = HAL_GPIO_ReadPin(GPIOA, Mygtukas_4_Pin);
	if (last_key_state[3] != key_state && last_key_state[3] == DOWN ) 
		result = KC_Right;
	last_key_state[3] = key_state;
	
	key_state = HAL_GPIO_ReadPin(GPIOA, Mygtukas_5_Pin);
	if (last_key_state[4] != key_state && last_key_state[4] == DOWN ) 
		result = KC_Shoot;
	last_key_state[4] = key_state;
	
	key_state = HAL_GPIO_ReadPin(GPIOA, Mygtukas_6_Pin);
	if (last_key_state[5] != key_state && last_key_state[5] == DOWN ) 
		result = KC_Alt;
	last_key_state[5] = key_state;
	
	return result;
}
