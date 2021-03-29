#include "graphics.h"

const uint16_t GROVE_TWO_RGB_LED_MATRIX_DEF_I2C_ADDR = 0x65 << 1; //Leftshift due to STM32
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
		const char I2C_CMD_DISP_EMOJI = 2;
	
    uint8_t data[5] = {0, };

  	data[0] = 0x02; //I2C_CMD_DISP_CUSTOM
    data[1] = emoji;
    data[2] = (uint8_t)(duration_time & 0xff);
    data[3] =  (uint8_t)((duration_time >> 8) & 0xff);
    data[4] = forever_flag;
	
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
	for (int j = 0; j < 64; j++) {
		_buffer[j/8][j%8] = 0xa7;
		if (j/8 == 3 && j%8 > 3 && j%8 < 7) {
			_buffer[j/8][j%8] = 0x5b;
		}
		
		if (j/8 == 1 && j%8 > 1 && j%8 < 7) {
			_buffer[j/8][j%8] = 0xff;
		}
		
		if (j/8 < 4 && j%8 == 2) {
			_buffer[j/8][j%8] = 0x5b;
		}
		
		if (j/8 == 1 && j%8 == 2) {
			_buffer[j/8][j%8] = 0x01;
		}
		
	
	}
}

//void GroveTwoRGBLedMatrixClass::displayFrames(uint8_t* buffer, uint16_t duration_time, bool forever_flag,
//        uint8_t frames_number) {
//    uint8_t data[72] = {0, };
//    // max 5 frames in storage
//    if (frames_number > 5) {
//        frames_number = 5;
//    } else if (frames_number == 0) {
//        return;
//    }

//    data[0] = I2C_CMD_DISP_CUSTOM;
//    data[1] = 0x0;
//    data[2] = 0x0;
//    data[3] = 0x0;
//    data[4] = frames_number;

//    for (int i = frames_number - 1; i >= 0; i--) {
//        data[5] = i;
//        for (int j = 0; j < 64; j++) {
//            data[8 + j] = buffer[j + i * 64];
//        }
//        if (i == 0) {
//            // display when everything is finished.
//            data[1] = (uint8_t)(duration_time & 0xff);
//            data[2] = (uint8_t)((duration_time >> 8) & 0xff);
//            data[3] = forever_flag;
//        }
//        i2cSendBytes(currentDeviceAddress, data, 24);
//        delay(1);
//        i2cSendContinueBytes(currentDeviceAddress, data + 24, 24);
//        delay(1);
//        i2cSendContinueBytes(currentDeviceAddress, data + 48, 24);
//    }
// 