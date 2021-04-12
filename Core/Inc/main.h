#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "nrf24.h"


/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define CE_nRF_Pin GPIO_PIN_1
#define CE_nRF_GPIO_Port GPIOC
#define CSN_nRF_Pin GPIO_PIN_2
#define CSN_nRF_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define Mygtukas_1_Pin GPIO_PIN_8
#define Mygtukas_1_GPIO_Port GPIOA
#define Mygtukas_2_Pin GPIO_PIN_9
#define Mygtukas_2_GPIO_Port GPIOA
#define Mygtukas_3_Pin GPIO_PIN_10
#define Mygtukas_3_GPIO_Port GPIOA
#define Mygtukas_4_Pin GPIO_PIN_11
#define Mygtukas_4_GPIO_Port GPIOA
#define Mygtukas_5_Pin GPIO_PIN_12
#define Mygtukas_5_GPIO_Port GPIOA
#define Mygtukas_6_Pin GPIO_PIN_1
#define Mygtukas_6_GPIO_Port GPIOA

#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA

void nRF24_CSN_L(void);
uint8_t nRF24_LL_RW(uint8_t reg);
void nRF24_LL_RW_MB(uint8_t reg, uint8_t *pBuf, uint8_t count);
void nRF24_CSN_H(void);
void nRF24_CE_L(void);
void nRF24_CE_H(void);
I2C_HandleTypeDef GetI2CHandle(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
