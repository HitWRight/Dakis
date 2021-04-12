#include "connectivity.h"

uint8_t nRF24_payload[32];
nRF24_RXResult pipe;
uint32_t i, j, k;
uint8_t payload_length;

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



void SetAsTransmitter() {
    // RX/TX disabled
    nRF24_CE_L();
	
		HAL_Delay(100);

    // Configure the nRF24L01+
    if (!nRF24_Check()) {      
			Error_Handler();
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
    
}

void SetAsReceiver() {

		// RX/TX disabled
    nRF24_CE_L();

		HAL_Delay(100);
    if (!nRF24_Check()) {
        Error_Handler();
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
}

nRF24_TXResult tx_res;

void SendData(Shoot_Position pos){
    int payload_length = 2;
		uint8_t payload[2] = { pos.x, pos.y };
		
		// Prepare data packet
		for (i = 0; i < payload_length; i++) {
			nRF24_payload[i] = payload[i];
		}

	  // Transmit a packet
		tx_res = nRF24_TransmitPacket(nRF24_payload, payload_length);
		switch (tx_res) {
			case nRF24_TX_SUCCESS:
							break;
					case nRF24_TX_TIMEOUT:
							break;
					case nRF24_TX_MAXRT:
							break;
					default:
							break;
			}
}

Shoot_Position ReceiveData()
{    
	Shoot_Position result;
	result.enabled = 0;
   if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
		 
     // Get a payload from the transceiver
     pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);
		 result.enabled = 1;
		 result.x = nRF24_payload[0];
		 result.y = nRF24_payload[1];

     // Clear all pending IRQ flags
     nRF24_ClearIRQFlags();

		 HAL_Delay(1);
   }
}
