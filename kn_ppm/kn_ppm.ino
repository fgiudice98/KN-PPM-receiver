#include <util/atomic.h>
#include <avr/io.h>
#include "iface_nrf24l01.h"

#include <TinyPpmGen.h>
#include <Rcul.h>

#define CH_MAX_NB  8

#define PPM_PERIOD_US         20000

// ############ Wiring ################
//SPI Comm.pins with nRF24L01
#define MOSI_pin  1  // MOSI - D3
#define SCK_pin   3  // SCK  - D4
#define CS_pin    2  // CE   - D5
#define MISO_pin  4 // MISO - A0
#define CE_pin    5 // CS   - A1

// SPI outputs
#define  MISO_on (PINB & _BV(4)) // PC0*/
#define MOSI_on PORTB |= 0b00000010  // PD3
#define MOSI_off PORTB &= ~0b00000010// PD3
#define SCK_on PORTB |= 0b00001000   // PD4
#define SCK_off PORTB &= ~0b00001000 // PD4
#define CS_on PORTB |= 0b00000100    // PD5
#define CS_off PORTB &= ~0b00000100  // PD5
#define CE_on PORTB |= 0b00100000    // PC1
#define CE_off PORTB &= ~0b00100000  // PC1
// SPI input
#define  MISO_on (PINB & 0b00010000) // PC0

#define RF_POWER TX_POWER_5mW //TX_POWER_80mW 

bool bind = false, use1mbps = true, started = false, lost = false;
byte hopping_channels[4], data[16];
byte tx_addr[5];
byte ch_index = 2, ch_indexl = 1;
unsigned long timing = 0, timing2 = 0;
int t, a, e, r, twayr = 0, twaya = 0;
byte sw, th, id, dr, td, tr, ta;
int deadline = 13;

byte rssic = 0, rssir = 0;
int rssi = 0;

void cleanArray(byte *arr, byte num) {
  for (byte i = 0; i < num; i++) {
    arr[i] = 0;
  }
}

void setup() {
  delay(500);

  pinMode(MOSI_pin, OUTPUT);
  pinMode(SCK_pin, OUTPUT);
  pinMode(CS_pin, OUTPUT);
  pinMode(CE_pin, OUTPUT);
  pinMode(MISO_pin, INPUT);
  MOSI_on;
  SCK_on;
  CS_on;
  CE_on;
  delay(1);
  MOSI_off;
  SCK_off;
  CS_off;
  delay(250);

  cleanArray(tx_addr, 5);
  cleanArray(hopping_channels, 4);

  for (byte i = 0; i < 2; i++) {
    NRF24L01_Activate(0x73);
    NRF24L01_Reset();
    NRF24L01_Initialize();

    NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP)); //enable Crc / crc endcoding 2bytes / power up

    NRF24L01_SetTxRxMode(RX_EN);  //reset and switch to rx mode

    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);    //disable auto acknowledgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01); //enable rx on data pipe 0
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);  //rx-tx address width of 5bytes
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0); //auto-retransmit delay 250us
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);  //clear interupts (rx, tx and max number of retransmit)
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, 0x20);  //32bytes rx payload in data pipe 0

    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 1);            //enable dynamic payload length on data pipe 0
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, _BV(NRF2401_1D_EN_DPL)); //enable dynamic payload length

    NRF24L01_SetPower(RF_POWER);          //set rf power                    //

    NRF24L01_SetBitrate(NRF24L01_BR_1M);
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, (const byte*)"KNDZK", 5);
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, 83);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x40);
  }

  while (!bind) {
    cleanArray(data, 16);
    if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & 0x40) {
      NRF24L01_ReadPayload(data, 16);
      NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x40);
      if (data[0] == 'K' && data[1] == 'N' && data[2] == 'D' && data[3] == 'Z') {
        bind = true;
        tx_addr[0] = data[4];
        tx_addr[1] = data[5];
        tx_addr[2] = data[6];
        tx_addr[3] = data[7];
        tx_addr[4] = 'K';
        hopping_channels[0] = data[8];
        hopping_channels[1] = data[9];
        hopping_channels[2] = data[10];
        hopping_channels[3] = data[11];
        if (data[15] == 1) {
          use1mbps = true;
        } else {
          use1mbps = false;
        }
      }
    }
  }

  if (use1mbps) {
    NRF24L01_SetBitrate(NRF24L01_BR_1M);
  } else {
    NRF24L01_SetBitrate(NRF24L01_BR_250K);
  }
  NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, tx_addr, 5);
  cleanArray(data, 16);

  TinyPpmGen.begin(TINY_PPM_GEN_NEG_MOD, CH_MAX_NB, PPM_PERIOD_US);

  TinyPpmGen.setChWidth_us(1, 1000);
  for (byte i = 2; i < 9; i++) {
    TinyPpmGen.setChWidth_us(i, 1500);
  }
}

void loop() {
  unsigned long m = millis();
  if (!started) {
    timing = m;
    timing2 = m;
  }

  if ((m - timing2) >= 500) {
    lost = true;
    TinyPpmGen.suspend();
    timing2 = m;
  }

  if ((m - timing) > deadline) {
    ch_index++;
    rssic++;
    timing = m;
    if (deadline == 13) {
      deadline = 10;
    }
  }

  if (rssic >= 16) {
    rssi = (rssir * 100) / 16;
    if (!lost) {
      TinyPpmGen.setChWidth_us(8, 1000 + (rssi * 10));
    }
    rssir = 0;
    rssic = 0;
  }

  if (ch_index >= 4) {
    ch_index = 0;
  }

  if (ch_index != ch_indexl) {
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_channels[ch_index]);
  }
  ch_indexl = ch_index;

  if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & 0x40) {
    cleanArray(data, 16);
    NRF24L01_ReadPayload(data, 16);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x40);

    t = data[0] << 8;
    t += data[1];
    a = data[2] << 8;
    a += data[3];
    e = data[4] << 8;
    e += data[5];
    r = data[6] << 8;
    r += data[7];
    ta = data[9];
    tr = data[11];
    sw = data[12];
    th = bitRead(sw, 1);
    id = bitRead(sw, 2);
    dr = bitRead(sw, 0);
    td = bitRead(sw, 6);

    if (tr < 100) {
      twayr = -1;
    } else if (tr > 100) {
      twayr = 1;
    } else {
      twayr = 0;
    }

    if (ta < 100) {
      twaya = -1;
    } else if (tr > 100) {
      twaya = 1;
    } else {
      twaya = 0;
    }

    if (lost) {
      TinyPpmGen.resume();
    }
    TinyPpmGen.setChWidth_us(1, 988 + t);
    TinyPpmGen.setChWidth_us(2, 988 + a);
    TinyPpmGen.setChWidth_us(3, 988 + e);
    TinyPpmGen.setChWidth_us(4, 988 + r);
    TinyPpmGen.setChWidth_us(5, 1000 + (th * 600) + (td * 400));
    TinyPpmGen.setChWidth_us(6, 1000 + (id * 600) + (twaya * 200));
    TinyPpmGen.setChWidth_us(7, 1200 + (dr * 600) + (twayr * 200));

    ch_index++;
    rssic++;
    rssir++;
    timing = millis();
    timing2 = timing;
    started = true;
    lost = false;
    deadline = 13;
  }
}
