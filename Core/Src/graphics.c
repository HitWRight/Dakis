#include "graphics.h"

const uint16_t GROVE_TWO_RGB_LED_MATRIX_DEF_I2C_ADDR = 0x65 << 1; //Poslinkis bit'o i kaire nes to nedaro HAL biblioteka automatiskai
uint8_t _buffer[8][8];

//Ptr size must be 4
HAL_StatusTypeDef GetDeviceId(uint8_t *data)
{
	I2C_HandleTypeDef hnd = GetI2CHandle();
	HAL_StatusTypeDef ret;
	const char I2C_CMD_GET_DEV_ID = 0;
	
	uint8_t command[1] = {I2C_CMD_GET_DEV_ID};
	
	ret = HAL_I2C_Master_Transmit(&hnd, GROVE_TWO_RGB_LED_MATRIX_DEF_I2C_ADDR, command, 1, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		Error_Handler();
	} 
	
	ret = HAL_I2C_Master_Receive(&hnd, GROVE_TWO_RGB_LED_MATRIX_DEF_I2C_ADDR, data, 4, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		Error_Handler();
	}
	
	return ret;
}

HAL_StatusTypeDef DisplayEmoji(uint8_t emoji, uint16_t duration_time, char forever_flag) 
{
    I2C_HandleTypeDef hnd = GetI2CHandle();
	
    uint8_t data[5] = {0, };

  	data[0] = 0x02; //I2C_CMD_DISP_EMOJI
    data[1] = emoji; //emojiID
    data[2] = (uint8_t)(duration_time & 0xff); //duration time low
    data[3] = (uint8_t)((duration_time >> 8) & 0xff); //duration time high
    data[4] = forever_flag; // forever flag
	
		HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hnd, GROVE_TWO_RGB_LED_MATRIX_DEF_I2C_ADDR, data, 5, 20);

		return ret;
}



void DisplayBuffer() {
	I2C_HandleTypeDef hnd = GetI2CHandle();
	uint8_t data[72] = {0, };
	data[0] = 0x05; //I2C_CMD_DISP_CUSTOM
  data[1] = 0x0; //duration time low bits
  data[2] = 0x0; //duration time high bits
  data[3] = 0x1; //forever_flag
  data[4] = 0x1; //frames number
	
	data[5] = 0x0; //frameID
	
	//Copy buffer info
	for (int j = 0; j < 64; j++) {
      data[8 + j] = _buffer[j/8][j%8];
	}
	
	
	uint8_t *data_ptr = data;
	
	HAL_I2C_Master_Transmit(&hnd, GROVE_TWO_RGB_LED_MATRIX_DEF_I2C_ADDR, data_ptr, 24, HAL_MAX_DELAY);
	HAL_Delay(1);
	data_ptr += 24;
	data_ptr--;
	data_ptr[0] = 0x81;
	HAL_I2C_Master_Transmit(&hnd, GROVE_TWO_RGB_LED_MATRIX_DEF_I2C_ADDR, data_ptr, 25, HAL_MAX_DELAY);
	HAL_Delay(1);
	data_ptr += 25;
	data_ptr--;
	data_ptr[0] = 0x81;
	HAL_I2C_Master_Transmit(&hnd, GROVE_TWO_RGB_LED_MATRIX_DEF_I2C_ADDR, data_ptr, 25, HAL_MAX_DELAY);
}


void SetBuffer() {
	for (int y = 0; y <= 7; y++)
	{	
		for (int x = 0; x <= 7; x++)
		{
			_buffer[y][x] = 0xa7; //juros spalva (melyna)
			
			if (y == 3 && x > 3 && x < 7) {
				_buffer[y][x] = 0x5b; //laivo spalva (zalia)
			}
			
			if (y < 4 && x == 2) {
				_buffer[y][x] = 0x5b; //laivo spalva (zalia)
			}
				
			if (y == 1 && x > 1 && x < 7) {
				_buffer[y][x] = 0xff; //suvio vietos spalva (juoda (nedega))
			}
			
			if (y == 1 && x == 2) {
				_buffer[y][x] = 0x01; //pataikyto laivo spalva (raudona)
			}
		}
	}
}

void PaintTiles(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
	for (int xx = x; xx<x+w; xx++)
	{
		for (int yy = y; yy<y+h; yy++) 
		{
			_buffer[yy][xx] = color;
		}
	}
}
