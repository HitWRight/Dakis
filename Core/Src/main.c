#include "main.h"
#include "graphics.h"

SPI_HandleTypeDef hspi1;
I2C_HandleTypeDef hi2c1;

uint8_t message_buffer[32];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);

uint8_t nRF24_payload[32];

// Pipe number
nRF24_RXResult pipe;

uint32_t i, j, k;

// Length of received payload
uint8_t payload_length;

// Helpers for transmit mode demo

// Timeout counter (depends on the CPU speed)
// Used for not stuck waiting for IRQ
#define nRF24_WAIT_TIMEOUT         (uint32_t)0x000FFFFF

// Result of packet transmission
typedef enum {
    nRF24_TX_ERROR  = (uint8_t)0x00, // Unknown error
    nRF24_TX_SUCCESS,                // Packet has been transmitted successfully
    nRF24_TX_TIMEOUT,                // It was timeout during packet transmit
    nRF24_TX_MAXRT                   // Transmit failed with maximum auto retransmit count
} nRF24_TXResult;

nRF24_TXResult tx_res;

// Function to transmit data packet
// input:
//   pBuf - pointer to the buffer with data to transmit
//   length - length of the data buffer in bytes
// return: one of nRF24_TX_xx values
nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length) {
    volatile uint32_t wait = nRF24_WAIT_TIMEOUT;
    uint8_t status;

    // Deassert the CE pin (in case if it still high)
    nRF24_CE_L();

    // Transfer a data from the specified buffer to the TX FIFO
    nRF24_WritePayload(pBuf, length);

    // Start a transmission by asserting CE pin (must be held at least 10us)
    nRF24_CE_H();

    // Poll the transceiver status register until one of the following flags will be set:
    //   TX_DS  - means the packet has been transmitted
    //   MAX_RT - means the maximum number of TX retransmits happened
    // note: this solution is far from perfect, better to use IRQ instead of polling the status
    do {
        status = nRF24_GetStatus();
        if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
            break;
        }
    } while (wait--);

    // Deassert the CE pin (Standby-II --> Standby-I)
    nRF24_CE_L();

    if (!wait) {
        // Timeout
        return nRF24_TX_TIMEOUT;
    }

    // Clear pending IRQ flags
    nRF24_ClearIRQFlags();

    if (status & nRF24_FLAG_MAX_RT) {
        // Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
        return nRF24_TX_MAXRT;
    }

    if (status & nRF24_FLAG_TX_DS) {
        // Successful transmission
        return nRF24_TX_SUCCESS;
    }

    // Some banana happens, a payload remains in the TX FIFO, flush it
    nRF24_FlushTX();

    return nRF24_TX_ERROR;
}


void transmitter() {
    // RX/TX disabled
    nRF24_CE_L();
	
		HAL_Delay(100);

    // Configure the nRF24L01+
    while (!nRF24_Check()) {      
			HAL_Delay(100);
    }
		
    // Initialize the nRF24L01 to its default state
    nRF24_Init();
		
		// This is simple transmitter (to one logic address):
    //   - TX address: '0xE7 0x1C 0xE3'
    //   - payload: 5 bytes
    //   - RF channel: 115 (2515MHz)
    //   - data rate: 250kbps (minimum possible, to increase reception reliability)
    //   - CRC scheme: 2 byte

    // The transmitter sends a 5-byte packets to the address '0xE7 0x1C 0xE3' without Auto-ACK (ShockBurst disabled)

    // Disable ShockBurst for all RX pipes
    nRF24_DisableAA(0xFF);

    // Set RF channel
    nRF24_SetRFChannel(115);

    // Set data rate
    nRF24_SetDataRate(nRF24_DR_250kbps);

    // Set CRC scheme
    nRF24_SetCRCScheme(nRF24_CRC_2byte);

    // Set address width, its common for all pipes (RX and TX)
    nRF24_SetAddrWidth(3);

    // Configure TX PIPE
    static const uint8_t nRF24_ADDR[] = { 0xE7, 0x1C, 0xE3 };
    nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR); // program TX address

    // Set TX power (maximum)
    nRF24_SetTXPower(nRF24_TXPWR_0dBm);

    // Set operational mode (PTX == transmitter)
    nRF24_SetOperationalMode(nRF24_MODE_TX);

    // Clear any pending IRQ flags
    nRF24_ClearIRQFlags();

    // Wake the transceiver
    nRF24_SetPowerMode(nRF24_PWR_UP);


    // The main loop
//    int j = 0;
//    int payload_length = 6;
//		uint8_t payload[] = "Hello";
//    while (1) {
//       // Prepare data packet
//			for (i = 0; i < payload_length; i++) {
//    		nRF24_payload[i] = payload[i];

//    	}

//        // Transmit a packet
//			tx_res = nRF24_TransmitPacket(nRF24_payload, payload_length);
//        switch (tx_res) {
//            case nRF24_TX_SUCCESS:
//                break;
//            case nRF24_TX_TIMEOUT:
//                break;
//            case nRF24_TX_MAXRT:
//                break;
//            default:
//                break;
//        }
//				
//        //HAL_Delay(1);
//    }
}

void receiver() {
    // RX/TX disabled
    nRF24_CE_L();

    if (!nRF24_Check()) {
        while (1) { }
    }

    // Initialize the nRF24L01 to its default state
    nRF24_Init();

/***************************************************************************/

    // This is simple receiver with one RX pipe:
    //   - pipe#1 address: '0xE7 0x1C 0xE3'
    //   - payload: 5 bytes
    //   - RF channel: 115 (2515MHz)
    //   - data rate: 250kbps (minimum possible, to increase reception reliability)
    //   - CRC scheme: 2 byte

    // The transmitter sends a 5-byte packets to the address '0xE7 0x1C 0xE3' without Auto-ACK (ShockBurst disabled)

    // Disable ShockBurst for all RX pipes
    nRF24_DisableAA(0xFF);

    // Set RF channel
    nRF24_SetRFChannel(115);

    // Set data rate
    nRF24_SetDataRate(nRF24_DR_250kbps);

    // Set CRC scheme
    nRF24_SetCRCScheme(nRF24_CRC_2byte);

    // Set address width, its common for all pipes (RX and TX)
    nRF24_SetAddrWidth(3);

    // Configure RX PIPE#1
    static const uint8_t nRF24_ADDR[] = { 0xE7, 0x1C, 0xE3 };
    nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR); // program address for RX pipe #1
    nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 5); // Auto-ACK: disabled, payload length: 5 bytes

    // Set operational mode (PRX == receiver)
    nRF24_SetOperationalMode(nRF24_MODE_RX);

    // Wake the transceiver
    nRF24_SetPowerMode(nRF24_PWR_UP);

    // Put the transceiver to the RX mode
    nRF24_CE_H();


    // The main loop
    while (1) {
        //
        // Constantly poll the status of the RX FIFO and get a payload if FIFO is not empty
        //
        // This is far from best solution, but it's ok for testing purposes
        // More smart way is to use the IRQ pin :)
        //
        if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
            // Get a payload from the transceiver
            pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);

            // Clear all pending IRQ flags
            nRF24_ClearIRQFlags();

						HAL_Delay(1);
        }
    }

}

void Init() {
	HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
}

void Check() {
	
}

void Connect() {
}

void ReadUserInput() {
	
}

int main(void)
{ 
  Init();
	Check();
	Connect();
	SetBuffer();
	DisplayBuffer();
	

	while(1) {
		ReadUserInput();
		
		//Display();
		for (int i=0; i<5; i++)
		{

			DisplayEmoji(i, 1000, 0);
			HAL_Delay(2000);
		}
		
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
  hi2c1.Init.Timing = 0x2000090E;
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
  HAL_GPIO_WritePin(GPIOC, CE_nRF_Pin|CSN_nRF_Pin|LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CE_nRF_Pin CSN_nRF_Pin LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = CE_nRF_Pin|CSN_nRF_Pin|LD4_Pin|LD3_Pin;
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
                          |Mygtukas_5_Pin;
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
