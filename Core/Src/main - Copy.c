#include "main.h"

#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 



I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

uint8_t message_buffer[32];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);

typedef enum {
	CONFIG = 0x00,
	EN_AA = 0x01,
	EN_RXADDR = 0x02,
	SETUP_AW = 0x03,
	SETUP_RETR = 0x04,
	RF_CH = 0x05,
	RF_SETUP = 0x06,	
	STATUS = 0x07,
	OBSERVE_TX = 0x08,
	CD = 0x09,
	RX_ADDR_P0 = 0x0A,
	RX_ADDR_P1 = 0x0B,
	RX_ADDR_P2 = 0x0C,
	RX_ADDR_P3 = 0x0D,
	RX_ADDR_P4 = 0x0E,
	RX_ADDR_P5 = 0x0F,
	TX_ADDR = 0x10,
	RX_PW_P0 = 0x11,
	RX_PW_P1 = 0x12,
	RX_PW_P2 = 0x13,
	RX_PW_P3 = 0x14,
	RX_PW_P4 = 0x15,
	RX_PW_P5 = 0x16,
	FIFO_STATUS = 0x17,
	DYNPD = 0x1C,
	FEATURE = 0x1D,
	NOOP = 0xFF
} NRFRegister_t;

void print_error(char* error, const char *format, ...) {
	 sprintf((char*)message_buffer, error, format);
}

uint8_t read_register(NRFRegister_t reg) {
	//lower CSN pin
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	
	//Write register address
	uint8_t tx[4] = { reg };
	uint8_t rx[4] = { 0 };
  if (HAL_SPI_TransmitReceive(&hspi1, tx, rx, 1, 10) != HAL_OK)
	{
		print_error("Error reading Register: %i", (char*)hspi1.ErrorCode);
	}
	
	//Read Register value
	if (HAL_SPI_TransmitReceive(&hspi1, tx, rx, 1, 10) != HAL_OK)
	{
		print_error("Error reading Register: %i", (char*)hspi1.ErrorCode);
	}	
	
	//Raise CSN pin
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);

	return rx[0];
}

uint8_t read_multi_byte_register(uint8_t reg, uint8_t *buf, uint8_t size) {
	//lower CSN pin
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	
	//Write register address
	uint8_t tx[4] = { reg };
	uint8_t rx[4] = { 0 };
  if (HAL_SPI_TransmitReceive(&hspi1, tx, rx, 1, 10) != HAL_OK)
	{
		print_error("Error reading Register: %i", (char*)hspi1.ErrorCode);
	}
	
	//Read Register value
	if (HAL_SPI_Receive(&hspi1, buf, size, 10) != HAL_OK)
	{
		print_error("Error reading Register: %i", (char*)hspi1.ErrorCode);
	}	
	
	//Raise CSN pin
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);

	return rx[0];
}

void write_register(NRFRegister_t reg, uint8_t* value, uint8_t size) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	uint8_t tx[4] = { reg | 0x20};
	uint8_t rx[4] = { 0 };
	
	if (HAL_SPI_TransmitReceive(&hspi1, tx, rx, 1, 10) != HAL_OK)
	{
		print_error("Error writing Register: %i", (char*)hspi1.ErrorCode);
	}

	if (HAL_SPI_TransmitReceive(&hspi1, value, rx, size, 10) != HAL_OK)
	{
		print_error("Error writing Register: %i", (char*)hspi1.ErrorCode);
	}
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
}

const uint8_t PRIM_RX_FLAG = 0x00;
const uint8_t PWR_UP_FLAG = 0x01;


void NRF24L01_PowerUp() {
	//NRF24L01 Init
	HAL_Delay(11); //10.3ms wait after powerup

	//Enter Power up state
	uint8_t config_value = read_register(CONFIG);
	config_value = config_value | (0xFF & PWR_UP_FLAG);
	write_register(CONFIG, &config_value, 1);
	
	HAL_Delay(2); //1.5ms wait after flag change
}

void NRF24L01_RX_Mode() {
	//Enter RX_Mode
	uint8_t config_value = read_register(CONFIG);
	config_value = config_value | PRIM_RX_FLAG;
	write_register(CONFIG, &config_value, 1);
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET); //Raise Chip Enable
	
	HAL_Delay(1); // Wait more than 130us
}

void NRF24L01_Read_Payload() {
	uint8_t command = 0x61; //R_RX_PAYLOAD
	
  uint8_t data_ready = read_register(RX_PW_P0) & 0x1F; //First 5 bits, dictates how many bytes are ready
	
	if (data_ready > 0)
	{
		print_error("FUCK YES: %i", (char*)data_ready);
	}
	//Get Number of bytes in PIPE FIFO 
}

//TODO
void NRF24L01_Send_Data() {
	//Add data to FIFO
	uint8_t tx[32] = {0xA0, 'H', 'e', 'l', 'l', 'o', '\0'}; //W_TX_Payload
	uint8_t rx[32];
	if (HAL_SPI_TransmitReceive(&hspi1, tx, rx, 7, 10) != HAL_OK)
	{
		print_error("Error writing Register: %i", (char*)hspi1.ErrorCode);
	}

	//Set PRIM_RX flag to 0
	uint8_t config_value = read_register(CONFIG);
	config_value = config_value & ~PRIM_RX_FLAG;
	write_register(CONFIG, &config_value, 1);
	
	//CE set to high for more than 10 us
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_Delay(1);
	//Wait 130 us for transition to TX Mode
	
	//Wait for TX FIFO for being empty
	while((read_register(FIFO_STATUS) & 0x04) == 0)
	{
		HAL_Delay(1);
	}
	
	//CE set to low
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); //Lower Chip Enable
}

typedef enum {
	DR_250kbps = (uint8_t)0x20, // 250kbps data rate
	DR_1Mbps   = (uint8_t)0x00, // 1Mbps data rate
	DR_2Mbps   = (uint8_t)0x08  // 2Mbps data rate
} DataRate_t;

void NRF24L01_SetDataRate(DataRate_t data_rate) {
	uint8_t reg;

	reg  = read_register(RF_SETUP);
	reg &= ~0x28; // 0b0010_1000
	reg |= data_rate;
	write_register(RF_SETUP, &reg, 1);
}

typedef enum {
	CRC_off   = (uint8_t)0x00, // CRC disabled
	CRC_1byte = (uint8_t)0x08, // 1-byte CRC
	CRC_2byte = (uint8_t)0x0c  // 2-byte CRC
} CRCScheme_t;


void NRF24L01_SetCRCScheme(CRCScheme_t scheme) {
	uint8_t reg;

	// Configure EN_CRC[3] and CRCO[2] bits of the CONFIG register
	reg = read_register(CONFIG);
	reg &= ~0x0C; // 0b0000_1100
	reg |= (scheme & 0x0C);
	write_register(CONFIG, &reg, 1);
}

typedef enum {
	Width_3 = 1,
	Width_4 = 2,
	Width_5 = 3
} AddressWidth_t;	

// Set of address widths
// input:
//   addr_width - RX/TX address field width, value from 3 to 5
// note: this setting is common for all pipes
void NRF24L01_SetAddrWidth(AddressWidth_t addr_width) {
	uint8_t addr_width_cmd = (uint8_t) addr_width;
	write_register(SETUP_AW, &addr_width_cmd, 1);
}

// Enumeration of RX pipe addresses and TX address
typedef enum {
	PIPE0 = (uint8_t)0x00, // pipe0
	PIPE1 = (uint8_t)0x01, // pipe1
	PIPE2 = (uint8_t)0x02, // pipe2
	PIPE3 = (uint8_t)0x03, // pipe3
	PIPE4 = (uint8_t)0x04, // pipe4
	PIPE5 = (uint8_t)0x05, // pipe5
	PIPETX = (uint8_t)0x06  // TX address (not a pipe in fact)
} PipeAddress_t;

#define nRF24_REG_RX_ADDR_P0       (uint8_t)0x0A // Receive address data pipe 0
#define nRF24_REG_RX_ADDR_P1       (uint8_t)0x0B // Receive address data pipe 1
#define nRF24_REG_RX_ADDR_P2       (uint8_t)0x0C // Receive address data pipe 2
#define nRF24_REG_RX_ADDR_P3       (uint8_t)0x0D // Receive address data pipe 3
#define nRF24_REG_RX_ADDR_P4       (uint8_t)0x0E // Receive address data pipe 4
#define nRF24_REG_RX_ADDR_P5       (uint8_t)0x0F // Receive address data pipe 5
#define nRF24_REG_TX_ADDR          (uint8_t)0x10 // Transmit address

static const uint8_t nRF24_ADDR_REGS[7] = {
		nRF24_REG_RX_ADDR_P0,
		nRF24_REG_RX_ADDR_P1,
		nRF24_REG_RX_ADDR_P2,
		nRF24_REG_RX_ADDR_P3,
		nRF24_REG_RX_ADDR_P4,
		nRF24_REG_RX_ADDR_P5,
		nRF24_REG_TX_ADDR
};

// RF output power in TX mode
typedef enum {
	TXPWR_18dBm = (uint8_t)0x00, // -18dBm
	TXPWR_12dBm = (uint8_t)0x02, // -12dBm
	TXPWR_6dBm  = (uint8_t)0x04, //  -6dBm
	TXPWR_0dBm  = (uint8_t)0x06  //   0dBm
} TXPwr_t;

void NRF24L01_SetTXPower(TXPwr_t tx_pwr) {
	uint8_t reg;

	// Configure RF_PWR[2:1] bits of the RF_SETUP register
	reg  = read_register(RF_SETUP);
	reg &= ~0x06; //0b0000_0110
	reg |= tx_pwr;
	write_register(RF_SETUP, &reg, 1);
}

// Set static RX address for a specified pipe
// input:
//   pipe - pipe to configure address, one of nRF24_PIPEx values
//   addr - pointer to the buffer with address
// note: pipe can be a number from 0 to 5 (RX pipes) and 6 (TX pipe)
// note: buffer length must be equal to current address width of transceiver
// note: for pipes[2..5] only first byte of address will be written because
//       other bytes of address equals to pipe1
// note: for pipes[2..5] only first byte of address will be written because
//       pipes 1-5 share the four most significant address bytes
void NRF24L01_SetAddr(PipeAddress_t pipe, const uint8_t *addr) {
	uint8_t *l_addr = (uint8_t*) addr; //copy
	uint8_t addr_width;

	// RX_ADDR_Px register
	switch (pipe) {
		case PIPETX:
		case PIPE0:
		case PIPE1:
			// Get address width
			addr_width = read_register(SETUP_AW) + 1;
			// Write address in reverse order (LSByte first)
			uint8_t tx[4];		
			for(int i=0; i<addr_width; i++) {
				tx[addr_width - i - 1] = addr[i];
			}
			
			//Set CSN low
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
			write_register(nRF24_ADDR_REGS[pipe], tx, addr_width);
			//Set CSN high
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
			
			break;	
		case PIPE2:
		case PIPE3:
		case PIPE4:
		case PIPE5:
			// Write address LSBbyte (only first byte from the addr buffer)
			write_register(nRF24_ADDR_REGS[pipe], l_addr, 1);
			break;
		default:
			// Incorrect pipe number -> do nothing
			break;
	}
}

// Set automatic retransmission parameters
// input:
//   ard - auto retransmit delay x * 250uS 
//   arc - count of auto retransmits, value form 0 to 15 
// note: zero arc value means that the automatic retransmission disabled
void NRF24L01_SetAutoRetr(uint8_t auto_retransmit_delay, uint8_t auto_retransmit_count) {
	// Set auto retransmit settings (SETUP_RETR register)
	uint8_t reg = (uint8_t)((auto_retransmit_delay << 4) | (auto_retransmit_count & 0x0F));
	write_register(SETUP_RETR, &reg, 1);
}

// Enable the auto retransmit (a.k.a. enhanced ShockBurst) for the specified RX pipe
// input:
//   pipe - number of the RX pipe, value from 0 to 5
void NRF24L01_EnableAA(PipeAddress_t pipe) {
	uint8_t reg;

	// Set bit in EN_AA register
	reg  = read_register(EN_AA);
	reg |= (1 << pipe);
	write_register(EN_AA, &reg, 1);
}

// Transceiver mode
typedef enum {
	nRF24_MODE_RX = (uint8_t)0x01, // PRX
	nRF24_MODE_TX = (uint8_t)0x00  // PTX
} Mode_t;



const uint8_t PRIM_RX = 0x01; // PRIM_RX bit in CONFIG register

// Set transceiver operational mode
// input:
//   mode - operational mode, one of Mode_t values
// Should not be chanced when device is Powered (PWR_UP flag)
void NRF24L01_SetOperationalMode(Mode_t mode) {
	uint8_t reg;
	
	// Configure PRIM_RX bit of the CONFIG register
	reg  = read_register(CONFIG);
	reg &= ~PRIM_RX;
	reg |= (mode & PRIM_RX);
	write_register(CONFIG, &reg, 1);
}

// Clear any pending IRQ flags
void NRF24L01_ClearIRQFlags() {
	uint8_t reg;
	// Clear RX_DR, TX_DS and MAX_RT bits of the STATUS register
	reg  = read_register(STATUS);
	reg |= 0x70; //MASK_STATUS_IRQ
	write_register(STATUS, &reg, 1);
}

typedef enum {
	DPL_ON,
	DPL_OFF
} DPLMode_t;
// Set transceiver DynamicPayloadLength feature for all the pipes
// input:
//   mode - status, one of nRF24_DPL_xx values
void NRF24L01_SetDynamicPayloadLength(DPLMode_t mode) {
	uint8_t reg;
	reg  = read_register(FEATURE);
	if(mode) {
		reg = reg | 0x04; // nRF24_FEATURE_EN_DPL  
		write_register(FEATURE, &reg, 1);
		reg = 0x1F;
		write_register(DYNPD, &reg, 1);
	} else {
		reg = reg &~ 0x04;
		write_register(FEATURE, &reg, 1); // nRF24_FEATURE_EN_DPL  
		reg = 0x00;
		write_register(DYNPD, &reg, 1);
	}
}

typedef enum {
	PWR_UP,
	PWR_DOWN
} PowerMode_t;

void NRF24L01_SetPowerMode(PowerMode_t mode) {
	uint8_t reg;

	reg = read_register(CONFIG);
	if (mode == PWR_UP) {
		// Set the PWR_UP bit of CONFIG register to wake the transceiver
		// It goes into Stanby-I mode with consumption about 26uA
		reg |= 0x01;
	} else {
		// Clear the PWR_UP bit of CONFIG register to put the transceiver
		// into power down mode with consumption about 900nA
		reg &= ~0x01;
	}
	write_register(CONFIG, &reg, 1);
}

void NRF24L01_Init() {
	// Write to registers their initial values
//	nRF24_WriteReg(nRF24_REG_CONFIG, 0x08);
//	nRF24_WriteReg(nRF24_REG_EN_AA, 0x3F);
//	nRF24_WriteReg(nRF24_REG_EN_RXADDR, 0x03);
//	nRF24_WriteReg(nRF24_REG_SETUP_AW, 0x03);
//	nRF24_WriteReg(nRF24_REG_SETUP_RETR, 0x03);
//	nRF24_WriteReg(nRF24_REG_RF_CH, 0x02);
//	nRF24_WriteReg(nRF24_REG_RF_SETUP, 0x0E);
//	nRF24_WriteReg(nRF24_REG_STATUS, 0x00);
//	nRF24_WriteReg(nRF24_REG_RX_PW_P0, 0x00);
//	nRF24_WriteReg(nRF24_REG_RX_PW_P1, 0x00);
//	nRF24_WriteReg(nRF24_REG_RX_PW_P2, 0x00);
//	nRF24_WriteReg(nRF24_REG_RX_PW_P3, 0x00);
//	nRF24_WriteReg(nRF24_REG_RX_PW_P4, 0x00);
//	nRF24_WriteReg(nRF24_REG_RX_PW_P5, 0x00);
//	nRF24_WriteReg(nRF24_REG_DYNPD, 0x00);
//	nRF24_WriteReg(nRF24_REG_FEATURE, 0x00);

//	// Clear the FIFO's
//	nRF24_FlushRX();
//	nRF24_FlushTX();

//	// Clear any pending interrupt flags
//	nRF24_ClearIRQFlags();

//	// Deassert CSN pin (chip release)
//	nRF24_CSN_H();

	// Set RF channel
	uint8_t channel = 98;
	write_register(RF_CH, &channel, 1);
	
	// Set data rate
	//NRF24L01_SetDataRate(DR_2Mbps);
	NRF24L01_SetDataRate(DR_250kbps);
	
	// Set CRC scheme
  NRF24L01_SetCRCScheme(CRC_2byte);
	
	// Set address width, its common for all pipes (RX and TX)
  NRF24L01_SetAddrWidth(Width_3);
	

	
	// Set TX power (maximum)
  NRF24L01_SetTXPower(TXPWR_0dBm);
	
  // Clear any pending IRQ flags
  NRF24L01_ClearIRQFlags();


	



    
}


void nRF24_WritePayload(uint8_t *pBuf, uint8_t length) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	
	uint8_t tx[1] = { 0xA0 }; // nRF24_CMD_W_TX_PAYLOAD  
	uint8_t rx[32] = { 0 };
	
	if (HAL_SPI_TransmitReceive(&hspi1, tx, rx, 1, 10) != HAL_OK)
	{
		print_error("Error writing Register: %i", (char*)hspi1.ErrorCode);
	}
	
	if (HAL_SPI_TransmitReceive(&hspi1, pBuf, rx, length, 10) != HAL_OK)
	{
		print_error("Error writing Register: %i", (char*)hspi1.ErrorCode);
	}
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
}

typedef enum  {
	TX_SUCCESS,
	TX_TIMEOUT,
  TX_MAXRT,
	TX_ERROR
} TXResult_t;
	
// Function to transmit data packet
// input:
//   pBuf - pointer to the buffer with data to transmit
//   length - length of the data buffer in bytes
// return: one of nRF24_TX_xx values
TXResult_t NRF24L01_TransmitPacket(uint8_t *pBuf, uint8_t length) {
	volatile uint32_t wait = 5000;
	uint8_t status;

	// Deassert the CE pin (in case if it still high)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	
	// Transfer a data from the specified buffer to the TX FIFO
	nRF24_WritePayload(pBuf, length);

	// Start a transmission by asserting CE pin (must be held at least 10us)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

	// Poll the transceiver status register until one of the following flags will be set:
	//   TX_DS  - means the packet has been transmitted
	//   MAX_RT - means the maximum number of TX retransmits happened
	// note: this solution is far from perfect, better to use IRQ instead of polling the status
	const uint8_t FLAG_TX_DS = 0x20; // TX_DS bit (data sent TX FIFO interrupt)
	const uint8_t FLAG_MAX_RT = 0x10; 
	
	while(wait) {
		status = read_register(STATUS);
		if (status & (FLAG_TX_DS | FLAG_MAX_RT)) {
			break;
		}
		wait--;
	};

	// Deassert the CE pin (Standby-II --> Standby-I)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	
	if (!wait) {
		// Timeout
		return TX_TIMEOUT;
	}

	// Clear pending IRQ flags
  NRF24L01_ClearIRQFlags();

	if (status & FLAG_MAX_RT) {
		// Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
		return TX_MAXRT;
	}

	if (status & FLAG_TX_DS) {
		// Successful transmission
		return TX_SUCCESS;
	}

	// Some banana happens, a payload remains in the TX FIFO, flush it
	uint8_t nop = 0xFF;
	write_register(0xE1, &nop, 1);

	return TX_ERROR;
}

void Send() {
	NRF24L01_Init();
	
	// Configure TX PIPE
  static const uint8_t nRF24_ADDR[] = { 'E', 'S', 'B' };
	NRF24L01_SetAddr(PIPETX, nRF24_ADDR); // program TX address
	NRF24L01_SetAddr(PIPE0, nRF24_ADDR); // program address for pipe#0, must be same as TX (for Auto-ACK)
	
	// Configure auto retransmit: 10 retransmissions with pause of 2500us in between
  NRF24L01_SetAutoRetr(10, 10);

  // Enable Auto-ACK for pipe#0 (for ACK packets)
  NRF24L01_EnableAA(PIPE0);

  // Enable DPL
  NRF24L01_SetDynamicPayloadLength(DPL_ON);

  // Set operational mode (PTX == transmitter)
  NRF24L01_SetOperationalMode(nRF24_MODE_TX);
	
  // Wake the transceiver
	NRF24L01_SetPowerMode(PWR_UP);

  // The main loop
	int8_t payload[] = "HELLO";
	int8_t len = 6;

  while (1) {
		// Transmit a packet
		uint8_t result = NRF24L01_TransmitPacket(payload, len);
		if(result != TX_SUCCESS)
		{
			print_error("FUCK ME: %s", (char*)result);
		}
			//Wait 1s
		HAL_Delay(1000);
	}
}

// Configure a specified RX pipe
// input:
//   pipe - number of the RX pipe, value from 0 to 5
//   aa_state - state of auto acknowledgment, one of nRF24_AA_xx values
//   payload_len - payload length in bytes
void NRF24L01_SetRXPipe(PipeAddress_t pipe, uint8_t aa_state, uint8_t payload_len) {
	const uint8_t MASK_EN_RX = 0x3F;
	const uint8_t MASK_RX_PW = 0x3F;
	
	uint8_t reg;

	// Enable the specified pipe (EN_RXADDR register)
	reg = (read_register(EN_RXADDR) | (1 << pipe)) & MASK_EN_RX;
	write_register(EN_RXADDR, &reg, 1);

	// Set RX payload length (RX_PW_Px register)
	reg = payload_len & MASK_RX_PW;
	write_register(nRF24_ADDR_REGS[pipe], &reg, 1);

	// Set auto acknowledgment for a specified pipe (EN_AA register)
	reg = read_register(EN_AA);
	if (aa_state == 1) {
		reg |=  (1 << pipe);
	} else {
		reg &= ~(1 << pipe);
	}
	
	write_register(EN_AA, &reg, 1);
}

// Result of RX FIFO reading
typedef enum {
	nRF24_RX_PIPE0  = (uint8_t)0x00, // Packet received from the PIPE#0
	nRF24_RX_PIPE1  = (uint8_t)0x01, // Packet received from the PIPE#1
	nRF24_RX_PIPE2  = (uint8_t)0x02, // Packet received from the PIPE#2
	nRF24_RX_PIPE3  = (uint8_t)0x03, // Packet received from the PIPE#3
	nRF24_RX_PIPE4  = (uint8_t)0x04, // Packet received from the PIPE#4
	nRF24_RX_PIPE5  = (uint8_t)0x05, // Packet received from the PIPE#5
	nRF24_RX_EMPTY  = (uint8_t)0xff  // The RX FIFO is empty
} nRF24_RXResult;

#define nRF24_REG_RX_PW_P0         (uint8_t)0x11 // Number of bytes in RX payload in data pipe 0
#define nRF24_REG_RX_PW_P1         (uint8_t)0x12 // Number of bytes in RX payload in data pipe 1
#define nRF24_REG_RX_PW_P2         (uint8_t)0x13 // Number of bytes in RX payload in data pipe 2
#define nRF24_REG_RX_PW_P3         (uint8_t)0x14 // Number of bytes in RX payload in data pipe 3
#define nRF24_REG_RX_PW_P4         (uint8_t)0x15 // Number of bytes in RX payload in data pipe 4
#define nRF24_REG_RX_PW_P5         (uint8_t)0x16 // Number of bytes in RX payload in data pipe 5

static const uint8_t nRF24_RX_PW_PIPE[6] = {
		nRF24_REG_RX_PW_P0,
		nRF24_REG_RX_PW_P1,
		nRF24_REG_RX_PW_P2,
		nRF24_REG_RX_PW_P3,
		nRF24_REG_RX_PW_P4,
		nRF24_REG_RX_PW_P5
};

//Read payload
static nRF24_RXResult nRF24_ReadPayloadGeneric(uint8_t *pBuf, uint8_t *length) {
	uint8_t pipe;
	
	const uint8_t nRF24_MASK_RX_P_NO = 0x0E;
	// Extract a payload pipe number from the STATUS register
	pipe = (read_register(STATUS) & nRF24_MASK_RX_P_NO) >> 1;

	// RX FIFO empty?
	if (pipe < 6) {
		// Get payload length
		*length = read_register(nRF24_RX_PW_PIPE[pipe]);
		// Read a payload from the RX FIFO
		if (*length) {
			read_multi_byte_register(0x61, pBuf, *length);
		}
		
		return ((nRF24_RXResult)pipe);
	}

	// The RX FIFO is empty
	*length = 0;

	return nRF24_RX_EMPTY;
}


//TODO
void Receive() {
    // This is simple receiver with Enhanced ShockBurst:
    //   - RX address: 'ESB'
    //   - payload: 10 bytes
    //   - RF channel: 40 (2440MHz)
    //   - data rate: 2Mbps
    //   - CRC scheme: 2 byte

    // The transmitter sends a 10-byte packets to the address 'ESB' with Auto-ACK (ShockBurst enabled)
	
		NRF24L01_Init();
	
    // Configure RX PIPE
    static const uint8_t nRF24_ADDR[] = {'E', 'S', 'B'};
		
    NRF24L01_SetAddr(PIPE1, nRF24_ADDR); // program address for pipe
    NRF24L01_SetRXPipe(PIPE1, 1, 10); // Auto-ACK: enabled, payload length: 10 bytes

    // Set operational mode (PRX == receiver)
    NRF24L01_SetOperationalMode(nRF24_MODE_RX);

    // Clear any pending IRQ flags
    NRF24L01_ClearIRQFlags();

    // Wake the transceiver
    NRF24L01_SetPowerMode(PWR_UP);

    // Put the transceiver to the RX mode
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

    // The main loop
		
		uint8_t payload[32];
		uint8_t payload_length = 10;

    while (1) {
        //
        // Constantly poll the status of the RX FIFO and get a payload if FIFO is not empty
        //
        // This is far from best solution, but it's ok for testing purposes
        // More smart way is to use the IRQ pin :)
        //
        if (read_register(FIFO_STATUS) != 0x03) {
          // Get a payload from the transceiver
          nRF24_ReadPayloadGeneric(payload, &payload_length);

          // Clear all pending IRQ flags
          NRF24L01_ClearIRQFlags();

					HAL_Delay(1);
        }
    }
}

int main(void)
{ 
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
0.
  MX_I2C1_Init();
  MX_SPI1_Init();

	Send();
	//Receive();
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
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
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
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
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
