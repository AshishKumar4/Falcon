#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Falcon.h"
#include "CUSTOM_RX.h"
#include "Serial.h"

#include <SPI.h>

/*****************************************************************************************/
#if defined(SPI_RX)
/*****************************************************************************************/

#define CPACKET_MAGIC 110
#define REQ_SIGNAL 251
#define REQ2_SIGNAL 101
#define ACCEPT_SIGNAL 252
#define ACK_GOT_PACKET 250
#define RPACKET_MAGIC 120
#define FALSE_PACKET 145

int16_t SPI_rcData[RC_CHANS];

struct ControlPackets
{
  unsigned char magic;
  unsigned char throttle;
  unsigned char pitch;
  unsigned char roll;
  unsigned char yaw;
  unsigned char aux1;
  unsigned char aux2;
  unsigned char switches;
  //unsigned char random[9];
  unsigned char checksum;
};

struct ResponsePackets
{
  unsigned char magic;
  unsigned char alt;
  unsigned char pitch;
  unsigned char roll;
  unsigned char yaw;
  unsigned char lat;
  unsigned char lon;
  unsigned char heading;
  //unsigned char random[8];
  //unsigned char flags;
  unsigned char checksum;
};

ControlPackets rfCusData;
//ResponsePackets rfResData;

void SPI_Init()
{
#if defined(NRF24_RX)
  NRF24_Init();
#elif defined(RPI_SPI_RX)
  RPI_SPI_Init();
#endif
}

void SPI_Read_RC()
{
#if defined(NRF24_RX)
  NRF24_Read_RC();
#elif defined(RPI_SPI_RX)
  RPI_SPI_Read_RC();
#endif
}

/*****************************************************************************************/
/*****************************************************************************************/
/*****************************************************************************************/

#if defined(NRF24_RX)

#include <RF24.h>
#include <nRF24L01.h>

// Single radio pipe address for the 2 nodes to communicate.
static const uint64_t pipe = 0xE8E8F0F0E1LL;

RF24 radio(A2, 10); // CE, CSN

void resetRF24Data()
{
  rfCusData.throttle = 0;
  rfCusData.yaw = 128;
  rfCusData.pitch = 128;
  rfCusData.roll = 128;
  rfCusData.aux1 = 0;
  rfCusData.aux2 = 0;
  rfCusData.switches = 0;
}

void resetResponsePackets()
{
  rfResData.magic = RPACKET_MAGIC;
  rfResData.lat = 0;
  rfResData.lon = 0;
  rfResData.heading = 0;
  rfResData.pitch = 0;
  rfResData.roll = 0;
  rfResData.alt = 0;
}

void NRF24_Init()
{

  resetRF24Data();
  resetResponsePackets();

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(1); // Ensure autoACK is enabled
  radio.enableAckPayload();

  radio.openReadingPipe(1, pipe);
  radio.startListening();
}

void NRF_Write_TELE()
{
  // radio.writeAckPayload(1, serialBufferTX, TX_BUFFER_SIZE);
}

void NRF24_Read_RC()
{

  static unsigned long lastRecvTime = 0;

  rfResData.magic = RPACKET_MAGIC;
  rfResData.lat = 35.62;
  rfResData.lon = 139.68;
  rfResData.heading = att.heading;
  rfResData.pitch = att.angle[PITCH];
  rfResData.roll = att.angle[ROLL];
  rfResData.alt = alt.EstAlt;

  memcpy(&rfResData.flags, &f, 1); // first byte of status flags

  unsigned long now = millis();
  while (radio.available())
  {
#if defined(NRF_TELEMETRY_CUSTOM)
    //radio.writeAckPayload(1, serialBufferTX, TX_BUFFER_SIZE);
    radio.writeAckPayload(1, &rfResData, sizeof(ResponsePackets));
#else
    radio.writeAckPayload(1, &rfResData, sizeof(ResponsePackets));
#endif
    radio.read(&rfCusData, sizeof(ControlPackets));
    lastRecvTime = now;
  }
  if (now - lastRecvTime > 1000)
  {
    // signal lost?
    resetRF24Data();
  }

  SPI_rcData[THROTTLE] = map(rfCusData.throttle, 0, 255, 1000, 2000);
  SPI_rcData[YAW] = map(rfCusData.yaw, 0, 255, 1000, 2000);
  SPI_rcData[PITCH] = map(rfCusData.pitch, 0, 255, 1000, 2000);
  SPI_rcData[ROLL] = map(rfCusData.roll, 0, 255, 1000, 2000);
  SPI_rcData[AUX1] = map(rfCusData.aux1, 0, 255, 1000, 2000);
  SPI_rcData[AUX2] = map(rfCusData.aux2, 0, 255, 1000, 2000);
}

/*****************************************************************************************/
#elif defined(RPI_SPI_RX)
/*****************************************************************************************/

int rcPins[] = {A2, 12, 10, 11, A1, A3}; //{A0, A1, A3, A6, A7, A2};

unsigned char checksum(unsigned char *buf, int len)
{
  unsigned char tt = 0;
  for (int i = 0; i < len - 1; i++)
  {
    tt ^= buf[i];
  }
  return tt;
}

unsigned char buff[256]; //(unsigned char *)&rfCusData;
//unsigned char *rbuff = (unsigned char *)&rfResData;

volatile int index = 0;
volatile boolean process = true;

void RPI_SPI_Init()
{
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // SPI.begin();
  index = 0;
  process = true;
  SPI.attachInterrupt();
  /*for (int i = 0; i < sizeof(ControlPackets); i++)
  {
    SPDR = rbuff[i];
  }*/
  //SPDR = ACCEPT_SIGNAL; // Recieved Signal
  //Serial.begin(115200);

  SPI_rcData[THROTTLE] = map(rfCusData.throttle, 0, 255, 1000, 2000);
  SPI_rcData[YAW] = map(rfCusData.yaw, 0, 255, 1000, 2000);
  SPI_rcData[PITCH] = map(rfCusData.pitch, 0, 255, 1000, 2000);
  SPI_rcData[ROLL] = map(rfCusData.roll, 0, 255, 1000, 2000);
  SPI_rcData[AUX1] = map(rfCusData.aux1, 0, 255, 1000, 2000);
  SPI_rcData[AUX2] = map(rfCusData.aux2, 0, 255, 1000, 2000);
}

volatile static uint8_t tval = 0, chnl = 0, csum = 0, tbf = 0;
volatile static uint8_t tmpVal = 0, tmpChnl = 0;
volatile static uint8_t flg1 = 0, flg2 = 0;
volatile static uint8_t g;

int chnl_tbl[] = {THROTTLE, PITCH, ROLL, YAW};

ISR(SPI_STC_vect)
{
  g = SPDR;
  buff[0] += g;
  SPDR = g;
}

void tes()
{
  if (index == 0) // Value
  {
    SPDR = (g ^ 0xff);
    if (flg1 == 0)
    {
      tmpVal = g;
    }
    ++index;
  }
  else if (index == 1) // Channel
  {
    SPDR = (g ^ 0xff);
    if (flg2 == 0)
    {
      tmpChnl = g;
    }
    ++index;
  }
  else if (index == 2)
  {
    SPDR = 0x25;
    /*tbf = ((g&0xf0) >> 4)&0x0f;
    g = g & 0x0f;
    if(tbf > g && !flg1)   // correct value recieved*/
    if (g == tmpVal || flg1 == 1)
    {
      flg1 = 1;
    }
    else
    {
      flg1 = 0; // We got a incorrect value
    }
    ++index;
  }
  else if (index == 3)
  {
    SPDR = 0x23;
    /*tbf = ((g&0xf0) >> 4)&0x0f;
    g = g & 0x0f;*/
    if (g == tmpChnl || flg2 == 1) //if(tbf > g && !flg2)   // correct channel recieved
    {
      flg2 = 1;
    }
    else
    {
      flg2 = 0; // We got a incorrect channel
    }
    if (flg1 == 1 && flg2 == 1) // We have recieved the right channel and value
    {
      flg1 = 0;
      flg2 = 0;
      buff[tmpChnl] = tmpVal;
      //SPI_rcData[chnl_tbl[tmpChnl%4]] = map(tmpVal, 0, 255, 1000, 2000);
      //SPI_rcData[0] = map(43, 0, 255, 1000, 2000);
    }
    index = 0;
  }
}

void RPI_SPI_Read_RC()
{
  uint8_t oldSREG;
  oldSREG = SREG;
  cli(); // Let's disable interrupts
  SPI_rcData[THROTTLE] = buff[10] + 1000;//map(buff[0], 0, 255, 1000, 2000);
  SPI_rcData[YAW] = map(buff[1], 0, 255, 1000, 2000);
  SPI_rcData[PITCH] = map(buff[2], 0, 255, 1000, 2000);
  SPI_rcData[ROLL] = map(buff[3], 0, 255, 1000, 2000);
  SPI_rcData[AUX1] = map(buff[4], 0, 255, 1000, 2000);
  SPI_rcData[AUX2] = map(buff[5], 0, 255, 1000, 2000);
  SREG = oldSREG; // Let's restore interrupt state
  /*while((SPSR & (1<<SPIF)) != 0)
  {
    unsigned char g = SPDR;
    SPDR = g + 10;
  }*/
}

#endif      // End of RPI_SPI Definitions
/*****************************************************************************************/
#endif      //  End of SPI Definitions
/*****************************************************************************************/


/*****************************************************************************************/
#if defined(I2C_RX)     // I2C Definitions
/*****************************************************************************************/

#define CPACKET_MAGIC 110
#define REQ_SIGNAL 251
#define REQ2_SIGNAL 101
#define ACCEPT_SIGNAL 252
#define ACK_GOT_PACKET 250
#define RPACKET_MAGIC 120
#define FALSE_PACKET 145

int16_t I2C_rcData[RC_CHANS];
uint8_t buff[8];

void I2C_RX_Init()
{
  buff[0] = 127;
} 

void I2C_RX_Read_RC()
{
  uint8_t oldSREG;
  oldSREG = SREG;
  cli(); // Let's disable interrupts
  I2C_rcData[THROTTLE] = map(buff[0], 0, 255, 1000, 2000);
  I2C_rcData[YAW] = map(buff[1], 0, 255, 1000, 2000);
  I2C_rcData[PITCH] = map(buff[2], 0, 255, 1000, 2000);
  I2C_rcData[ROLL] = map(buff[3], 0, 255, 1000, 2000);
  I2C_rcData[AUX1] = map(buff[4], 0, 255, 1000, 2000);
  I2C_rcData[AUX2] = map(buff[5], 0, 255, 1000, 2000);
  SREG = oldSREG; // Let's restore interrupt state*/
}

/*****************************************************************************************/
#endif
/*****************************************************************************************/

