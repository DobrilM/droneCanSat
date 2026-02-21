#include "mspBase.h"
#include "config.h"
#include "global.h"

enum MSPType {
  IDLE,
  DOLLAR,
  M,
  ARROW,
  SIZE,
  CMD,
  PAYLOAD,
  CHECKSUM
};

uint8_t payload[16];

MSPType charType = IDLE;
uint8_t dataSize = 0;
uint8_t cmd = 0;
uint8_t checksum = 0;
uint8_t arrayptr = 0;

void mspSetup() {
  //msp init
  Serial1.begin(115200);
}


void mspCmd(uint8_t cmd, uint8_t* payload, uint8_t size) {
  uint8_t checksum = 0;
  Serial1.write('$');
  Serial1.write('M');
  Serial1.write('<');
  
  Serial1.write(size);
  checksum ^= size;

  Serial1.write(cmd);
  checksum ^= cmd;

  for (uint8_t i = 0; i < size; i++) {
      Serial1.write(payload[i]);
      checksum ^= payload[i];
  }
  Serial1.write(checksum);
}


void parsePacket(uint8_t cmd) {
    switch (cmd) {
    case GPS_GET:
    fix  = payload[0];
    numSat = payload[1];

    latitude = *(uint32_t*)&payload[2];
    longitude = *(uint32_t*)&payload[6];
    altGPS = (*(int16_t*)&payload[10]);
    break;
    
    case BATT_GET:
    battVolt = *(uint16_t*)&payload[0];
    break;
    case NAV_STAT:
    navStat = payload[0];
    break;
  }
}


void parseMSP(uint8_t readChar) {
  switch (charType) {
  case IDLE: charType = (readChar == '$') ? DOLLAR : IDLE; break;
  case DOLLAR:
  charType = (readChar == 'M') ? M : IDLE; break;
  case M:
  charType = (readChar == '>') ? ARROW : IDLE; break;
  case ARROW:
  dataSize = readChar; charType = SIZE; break;
  case SIZE: 
  cmd = readChar; 
  charType = CMD; arrayptr = 0; break;
  case CMD: 
    if (arrayptr < dataSize) {
      payload[arrayptr++] = readChar;
    } else {
      charType = CHECKSUM;
    }
  break;
  case CHECKSUM: 
    parsePacket(cmd);
    charType = IDLE;
    break;
  }
}
