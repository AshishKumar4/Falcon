
/*
Tested with 16MHz pro-mini, QuadX and https://github.com/gcopeland/RF24
Motors use pins 9,6,5,3 instead of 9,10,11,3
SPI connections (left is SPI, right is arduino):q
  CE      7
  CSN     8
  MOSI   11
  MISO   12
  SCK    13
You can change CE and CSN in SPI_RC.cpp
*/

#ifndef CUSTOM_RX_H_
#define CUSTOM_RX_H_

#include "config.h"

#if defined(SPI_RX)
#if defined(NRF24_RX)

// The sizeof this struct should not exceed 32 bytes

void NRF24_Init();
void NRF24_Read_RC();

#elif defined(RPI_SPI_RX)

void RPI_SPI_Init();
void RPI_SPI_Read_RC();

#endif

extern int16_t SPI_rcData[RC_CHANS];

void SPI_Init();
void SPI_Read_RC();

#endif

#if defined(I2C_RX)

extern int16_t I2C_rcData[RC_CHANS];

void I2C_RX_Init();
void I2C_RX_Read_RC();

#endif

#endif /* SPI_RX_H_ */
