#include "MSP.h"
#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_SleepyDog.h>
#include <Adafruit_BNO08x.h>
//#include <SD.h>

//radio definition
#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4
#define RF95_FREQ 433.0 //CHANGE BEFORE LAUNCH!!!!!
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//bmp define
#define BMP_CS 16
#define PRESSURE_SEA 1014.5 //CHANGE BEFORE LAUNCH!!!!!
Adafruit_BMP3XX bmp;

//imu define
#define BNO08X_RESET 18
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
float accX, accY, accZ;
//change 

//sd define
//#define SD_CS 15
//File logs;

//Note: MSP is connected to Serial1

MSP msp;

msp_set_raw_rc_t rc;
msp_raw_gps_t gps;
msp_analog_t batt;
msp_nav_status_t navStat;
msp_inav_status_t inavStat;

struct message {

  uint32_t packetCounter; //IMPLEMENT!!!!

  int16_t temp;
  int16_t alt; 
  int32_t pressure;

  uint8_t fix;
  uint8_t numSat;
  uint32_t rawLat;
  uint32_t rawLong;
  int16_t gpsAlt;

  uint16_t battVolt;

  int16_t accX;
  int16_t accY;
  int16_t accZ;
  //send accy

  uint8_t navMode;
  uint8_t navState;
  uint8_t navError;

  uint8_t state;

  uint8_t geozoneStatus;
};

float temperature, altitude, pressure;

uint8_t fix;
uint8_t numSat;
uint32_t rawLat;
uint32_t rawLong;
int16_t gpsAlt;
uint32_t fc_arming_flags;
uint16_t battVolt;

uint8_t navMode;
uint8_t navState;
uint8_t navError;

float latitude;
float longitude;

//geozone stuff
bool geozoneStatus;

struct point {
  float x;
  float y;
};

//update!!!
constexpr point borders[4] = {

  //actual coords launch site
  {52.402008,5.924794},
  {52.424292,5.966989},
  {52.414117, 5.979726},
  {52.395261,5.927807}, 

  //Patrik's house
  // {52.26943825232155, 4.607911997462024},
  // {52.26966098030295, 4.608252639857821},
  
  // //{52.26946605905007, 4.608035402425242},
  // //{52.26978285972307, 4.608522477454779},
  // {52.27000198501937, 4.608016763695906},
  // {52.26971329827905, 4.607593366196109},
};

typedef enum : uint8_t {
  STANDBY=0,
  ASCENDING,
  DESCENDING,
  WAITFORFIX,
  RTWP,
  FAILSAFE,
} state_t;

state_t state;
//fix all thresholds
#define HEIGHT_ASCENT -10

float lastHeight = 0;

uint8_t MSPReqTurn = 0;
unsigned long beforeFix = 0;
unsigned long setNavStateTime = 0;

uint32_t packetCounter = 0;
uint8_t calledSum =0;

bool rcOn = true;
bool checkChange();
//
const msp_set_raw_rc_t  valuesDisarmed = {
  1500,
  1500,
  1000,
  1500,
  1000,
  2000, //angle
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
};
const msp_set_raw_rc_t  valuesArmed = {
  1500,
  1500,
  1000,
  1500,
  2000,//arm
  2000,//angle
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
};

const msp_set_raw_rc_t  valuesArmedAltHold = {
  1500,
  1500,
  1750, //throttle
  1500,
  2000,//arm
  2000,//angle
  1500,//alt_hold
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
};

const msp_set_raw_rc_t  valuesArmedWpNav = {
  1500,
  1500,
  1000,
  1500,
  2000,//arm
  2000,//angle
  1000,
  2000,//wpnav
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
};




void setup() {
  //Serial 0 init (only used for the initializing of the arduino) 
  Serial.begin(115200);
  while(!Serial); //remove for release
  Serial.print("test");

  //rf95 setup
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  if (!rf95.init()) {
    Serial.println("rf doesnt init");
    while (1);
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Freq cannot be set");
    while (1);
  }
  rf95.setTxPower(23, false);
  Serial.print("radio init");

  //bmp init
  if (!bmp.begin_SPI(BMP_CS)) { 
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  Serial.print("bmp init");

  /*if (!SD.begin(SD_CS)) {
    Serial.println("SD not available");
  }*/

  if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }

  Serial1.begin(115200);
  msp.begin(Serial1);
  for (int i=0; i < MSP_MAX_SUPPORTED_CHANNELS; i++) {
    rc.channel[i] = 1000;
  }
  rc.channel[2] = 1000;
  Serial.print("msp initialized");
}

message makeMessage() {
  message p{};

  p.packetCounter = packetCounter;

  p.accX = accX * 100;
  p.accY = accY * 100;
  p.accZ = accZ * 100;
  p.temp = temperature * 100; //conv to int
  p.alt = altitude * 100;
  p.pressure = pressure * 100;

  p.fix = fix;
  p.numSat = numSat;
  p.rawLat = rawLat;
  p.rawLong = rawLong;
  p.gpsAlt = gpsAlt;

  p.battVolt = battVolt;

  p.navMode = navMode;
  p.navState = navState;
  p.navError = navError;

  p.state = state;
  p.geozoneStatus = geozoneStatus;
  return p;
}

//imu
void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    Serial.println("Could not enable linear acceleration");
  }
}

//geozone
bool vecCheck(point a, point b, point c) {
  return ((b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x) < 0);
}

bool geozoneCheck(point coordinates) {
  uint8_t geoChecksum = 0;
  uint8_t polygonCount = sizeof(borders)/sizeof(borders[0]);
  for (uint8_t i =0; i < polygonCount-1; i++) {
    geoChecksum += vecCheck(borders[i], borders[i+1], coordinates);
    //Serial.print(geoChecksum);
  }
  geoChecksum += vecCheck(borders[polygonCount-1], borders[0], coordinates);
  //Serial.println(geoChecksum);
  return (geoChecksum == polygonCount);
}


  void standbyMode(float h) {
    if (h > HEIGHT_ASCENT) {
      if(checkChange()) {
        state =ASCENDING;
        calledSum =0;
      } 
    }
  }
  void ascending(float lastH, float h) {
    if (h - lastH < 0) {
      if(checkChange()) {
        state =DESCENDING;
        calledSum =0;
      } 
    }
  }
  void descending(float h, unsigned long now) {
    if (h < 900) {
      beforeFix = now;
      rc = valuesArmed;
    if(checkChange()) {
        state =WAITFORFIX;
        calledSum =0;
      } 
    }
  }

  void waitForFix(unsigned long now) {
    rc = valuesArmedAltHold;
    if (now - beforeFix >30000 && fix <2) {
      if(checkChange()) {
        state =FAILSAFE;
        calledSum =0;
      }
    } else if (fix > 1) {
      if(checkChange()) {
        state =RTWP;
        calledSum =0;
      } 
      setNavStateTime = now;

    }
  }

  void rtwpMode(uint8_t navError, unsigned long now) {
    rc = valuesArmedWpNav;

    if(navError == 4 || now - setNavStateTime > 50000 ) { // so for some reason the guys 
      rc.channel[6] = 1000; //wp nav off                  // that made the msp protocol thought that it was a good decision to put the
      if(checkChange()) {
        state =FAILSAFE;
        calledSum =0;
      }                                          //"mission complete" flag as an error (MSP_NAV_STATUS_ERROR_FINISH = 4, see lib)
    }

  }

void failsafe() {
  rcOn = false;
}

bool checkChange() {
  return (calledSum < 6 && state != FAILSAFE);
}

void modeLogic(unsigned long now) {
  //failsafe check
  // if (fix >0 && !geozoneStatus) {
  //   state = FAILSAFE;
  // }
  if (state >3 && navError > 0) {
    state = FAILSAFE;
  }
  // if (battVolt < 11.0) {
  //   state = FAILSAFE;
  // }
  calledSum++;

  //mode selection
  switch (state) {
        case STANDBY: standbyMode(altitude); break;
        case ASCENDING: ascending(lastHeight, altitude); break;
        case DESCENDING: descending(altitude, now); break;
        case WAITFORFIX: waitForFix(now); break;
        case RTWP: rtwpMode(navState, now); break;
        case FAILSAFE: failsafe(); break;
  }

}

void sendSerial() {
   //Serial.print(fc_arming_flags);
    //Serial.println("radio sent");
    Serial.print(temperature);
    Serial.print(pressure);
    Serial.print(altitude);
    Serial.print(";");
    Serial.print(packetCounter);
    Serial.print(";");
    Serial.print(accX);
    Serial.print(";");
    Serial.print(accY);
    Serial.print(";");
    Serial.print(accZ);
    Serial.print(";");
    Serial.print(geozoneStatus);
    Serial.print(";");
    char message[128];
    sprintf(message, "%i;%i;%i;%i;%.7f;%.7f;%i;%i;%i;%i;%i;%i", fix, numSat, rawLat, rawLong, latitude, longitude, gpsAlt, battVolt, navMode, navState, navError,state);
    Serial.println(message);
      switch (state) {
        case 0: Serial.println("radio off");break;
        case 1: Serial.println("ascending");break;
        case 2: Serial.println("descending");break;
        case 3: Serial.println("waiting for fix");break;
        case 4: Serial.println("going to wp"); break;
        case 5: Serial.println("failsafe"); break;
      }
}

void loop() {
  unsigned long now = millis();
  static unsigned long lastMSP = 0;
  static unsigned long lastMSPReq = 0;
  static unsigned long lastSerial = 0;
  
  //read bmp

  if (bmp.performReading()) {
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure() / 100.0;   //hPa
    altitude =  44330.0 * (1.0 - pow(pressure / PRESSURE_SEA, 0.1903));
  } 

  //read imu

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  
  if (! bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  switch (sensorValue.sensorId) {  
    case SH2_LINEAR_ACCELERATION:
      accX = sensorValue.un.linearAcceleration.x;
      accY = sensorValue.un.linearAcceleration.y;
      accZ = sensorValue.un.linearAcceleration.z;
      break;
  }

  //read msp data and geozone 
  if (now - lastMSPReq > 500) {
        if (msp.request(MSP_RAW_GPS, &gps, sizeof(gps))) {
          fix = gps.fixType;
          numSat = gps.numSat;
          rawLat = gps.lat;
          rawLong = gps.lon;
          latitude = rawLat / 1e7f;
          longitude = rawLong / 1e7f; 
          gpsAlt = gps.alt;
        }

        if(msp.request(MSP_ANALOG, &batt, sizeof(batt))) {
          battVolt = batt.vbat;
        }

        if(msp.request(MSP_NAV_STATUS, &navStat, sizeof(navStat))) {
          navMode = navStat.mode;
          navState = navStat.state;
          navError = navStat.error;
        }

    lastMSPReq = now;
    point coordinates = {latitude, longitude};
    geozoneStatus = geozoneCheck(coordinates);

    //hardcode for testing
    //fix =0;
    //geozoneStatus = 1;

    lastHeight = altitude;
  }

  //failsafe logica and mode selection
  modeLogic(now);

  if (now - lastMSP > 50 && rcOn) {
    msp.command(MSP_SET_RAW_RC, &rc, sizeof(rc));
    lastMSP = now;
  }


  //serial, radio and sd
  if (now - lastSerial > 1000) {
    digitalWrite(LED_BUILTIN, HIGH);
    //make packet
    message pkt = makeMessage();

    //if (state >=2) {
      //send through radio
      rf95.send((uint8_t*)&pkt, sizeof(pkt));
      rf95.waitPacketSent();
    //}
    // write to sd
    // logs = SD.open("log.txt", FILE_WRITE);
    // logs.write((uint8_t*)&pkt, sizeof(pkt));
    // logs.close();

    sendSerial();

    lastSerial = now;
    digitalWrite(LED_BUILTIN, LOW);
    packetCounter++;
  }

}