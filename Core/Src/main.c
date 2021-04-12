#include "main.h"
#include "connectivity.h"
#include "graphics.h"
#include "map.h"
#include "keymap.h"

SPI_HandleTypeDef hspi1;
I2C_HandleTypeDef hi2c1;

uint8_t message_buffer[32];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);

void Init() {
	HAL_Init(); //HAL bibliotekos inicializacija (Generuotas kodas)
  SystemClock_Config(); //Sisteminio laikrodžio daugikliu nustatymas (Generuotas kodas) 
	
  MX_GPIO_Init(); //Mygtuku ir CE CSN pinu konfiguravimas
  MX_I2C1_Init(); //I2C sasajos inicializacija skirta komunikacijai su matrica
  MX_SPI1_Init(); //SPI sasajos inicializacija skirta komunikacijai su NRF24L01
}

typedef enum {
	ShipPlacement,
	EnemyMap,
	YourMap
} GameState;

typedef enum {
	Turn_Yours,
	Turn_Enemies,
} Turn;

int main(void)
{ 
  Init();
	HAL_Delay(250);
	Turn turn = Turn_Yours;
	SetAsReceiver();
	
	GameState gamestate = ShipPlacement;
	
	ship ship;
	ship = GetNextShip();
	SetTemporaryShipLocation(ship);	

	
	Shoot_Position data;


	while(1) {
		data = ReceiveData();
		if (data.enabled) {
			AddLocalShot(data.x, data.y);
			turn = Turn_Yours;
		}		
		
		switch (gamestate) {
			case ShipPlacement:
			{
				uint8_t keys_clicked = GetKeyClick();
				ship = GetTemporaryShipLocation();
				
				if (keys_clicked & KC_Shoot) 
				{
					AddShip(ship);
					ship = GetNextShip();
					if (ship.enabled == 0)
					{
						gamestate = EnemyMap;
						break;
					}
					SetTemporaryShipLocation(ship);	
				}

				if (keys_clicked & KC_Alt) 
				{
					if (ship.orientation == vertical)
						ship.orientation = horizontal;
					else
						ship.orientation = vertical;
				}
				
				if (keys_clicked & KC_Left)	
						ship.x--;
					
					if (keys_clicked & KC_Right)
						ship.x++;
					
					if (keys_clicked & KC_Up)
						ship.y--;
					
					if (keys_clicked & KC_Down)
						ship.y++;
				
				SetTemporaryShipLocation(ship);		

				
				
				PaintSea();
				PaintShips();
				PaintTempShip();
				break;
			}		
			case EnemyMap:
			{
				uint8_t keys_clicked = GetKeyClick();
				position cursor = GetCursor();
				
				if (keys_clicked & KC_Left)
					cursor.x--;
				
				if (keys_clicked & KC_Right)
					cursor.x++;
				
				if (keys_clicked & KC_Up)
					cursor.y--;
				
				if (keys_clicked & KC_Down)
					cursor.y++;
				
				if(cursor.x < 8 && cursor.y < 8)
				{
					SetCursor(cursor);
					if (keys_clicked & KC_Shoot && turn == Turn_Yours)
					{
						Shoot_Position sp;
						sp.enabled = 1;
						sp.x = cursor.x;
						sp.y = cursor.y;
						SetAsTransmitter();
						SendData(sp);
						SetAsReceiver();
						
						AddShot(cursor.x, cursor.y, SH_Miss);
						
						//turn = Turn_Enemies;
					}
				}
				
				if (keys_clicked & KC_Alt)
				{
					gamestate = YourMap;
				}
					
				PaintSea();
				PaintDistantShots();
				if(turn == Turn_Yours)
					PaintCursor();
			}
			break;
			case YourMap:
			{
				uint8_t keys_clicked = GetKeyClick();
				if (keys_clicked & KC_Alt)
				{
					gamestate = EnemyMap;
				}
				
				PaintSea();
				PaintShips();
				PaintLocalShots();
				break;
			}
		}
		
		DisplayBuffer();	
		HAL_Delay(100);
	}
	
	//receiver();
	//transmitter();

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E; //100kHz
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  */
static void MX_SPI1_Init(void) {
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CE_nRF_Pin|CSN_nRF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CE_nRF_Pin CSN_nRF_Pin LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = CE_nRF_Pin|CSN_nRF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Mygtukas_1_Pin Mygtukas_2_Pin Mygtukas_3_Pin Mygtukas_4_Pin
                           Mygtukas_5_Pin */
  GPIO_InitStruct.Pin = Mygtukas_1_Pin|Mygtukas_2_Pin|Mygtukas_3_Pin|Mygtukas_4_Pin
                          |Mygtukas_5_Pin|Mygtukas_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

void print_error(char* error, const char *format, ...) {
	 sprintf((char*)message_buffer, error, format);
}

uint8_t nRF24_LL_RW(uint8_t reg) {
	uint8_t result;
	//Write register address
  if (HAL_SPI_TransmitReceive(&hspi1, &reg, &result, 1, 10) != HAL_OK)
	{
		print_error("Error reading Register: %i", (char*)hspi1.ErrorCode);
	}
	
	return result;
}

//   count - number of bytes to read
void nRF24_LL_RW_MB(uint8_t reg, uint8_t *pBuf, uint8_t count) {
	nRF24_CSN_L();
	
	uint8_t tx[32] = { reg, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF};
	uint8_t rx[32] = {0};
	
	if (HAL_SPI_TransmitReceive(&hspi1, tx, rx, count, 10) != HAL_OK)
	{
		print_error("Error reading Register: %i", (char*)hspi1.ErrorCode);
	}
	for(int i=0;i<count;i++)
	{
		*pBuf++ = rx[i];
	}	
	
	nRF24_CSN_H();
}


//Lower CE PIN
void nRF24_CE_L(void) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}

//Raise CE PIN
void nRF24_CE_H(void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
}

//lower CSN pin
void nRF24_CSN_L(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
}

//Raise CSN pin
void nRF24_CSN_H(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
}

I2C_HandleTypeDef GetI2CHandle()
{
	return hi2c1;
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
