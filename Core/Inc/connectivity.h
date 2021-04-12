#ifndef __CONNECTIVITY_H
#define __CONNECTIVITY_H

#include "nrf24.h"

#define nRF24_WAIT_TIMEOUT         (uint32_t)0x000FFFFF

// Result of packet transmission
typedef enum {
    nRF24_TX_ERROR  = (uint8_t)0x00, // Unknown error
    nRF24_TX_SUCCESS,                // Packet has been transmitted successfully
    nRF24_TX_TIMEOUT,                // It was timeout during packet transmit
    nRF24_TX_MAXRT                   // Transmit failed with maximum auto retransmit count
} nRF24_TXResult;


typedef struct {
	uint8_t enabled:1;
	uint8_t x:3;
	uint8_t y:3;
} Shoot_Position;

typedef enum {
	Miss,
	Hit,
	Destroy
} Hit_Response;


// Function to transmit data packet
// input:
//   pBuf - pointer to the buffer with data to transmit
//   length - length of the data buffer in bytes
// return: one of nRF24_TX_xx values
nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length);
void SetAsTransmitter(void);
void SetAsReceiver(void);
void SendData(Shoot_Position pos);
Shoot_Position ReceiveData(void);

#endif

