#include "MSP.h"
#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_SleepyDog.h>
#include <Adafruit_BNO08x.h>

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

//bno define
#define BNO08X_RESET 18
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
float accX, accY, accZ;

//sd define
//#define SD_CS 15
//File logs;

//msp define
MSP msp;

//msp data define
msp_set_raw_rc_t rc;
msp_raw_gps_t gps;
msp_analog_t batt;
msp_nav_status_t navStat;
msp_inav_status_t inavStat;


//message structure define
struct message {
  
  //number of packet
  uint32_t packetCounter;

  //bmp
  int16_t temp;
  int16_t alt; 
  int32_t pressure;

  //gps
  uint8_t fix;
  uint8_t numSat;
  uint32_t rawLat;
  uint32_t rawLong;
  int16_t gpsAlt;

  //battery voltage
  uint16_t battVolt;

  //bno
  int16_t accX;
  int16_t accY;
  int16_t accZ;

  //navigation 
  uint8_t navMode;
  uint8_t navState;
  uint8_t navError;

  //cansat mode
  uint8_t state;

  //geozone
  uint8_t geozoneStatus;
};

//global var definition
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

bool geozoneStatus;


//geozone point typedefine
struct point {
  float x;
  float y;
};

//border define
constexpr point borders[4] = {

  //actual coords launch site
  {52.402008,5.924794},
  {52.424292,5.966989},
  {52.414117, 5.979726},
  {52.395261,5.927807}, 

};

//mode typedefine
typedef enum : uint8_t {
  STANDBY=0,
  ASCENDING,
  DESCENDING,
  WAITFORFIX,
  RTWP,
  FAILSAFE,
  BEFOREROCKET,
} state_t;

state_t state = BEFOREROCKET;

//transition and temp variables
float lastHeight = 0;
uint8_t MSPReqTurn = 0;
unsigned long beforeFix = 0;
unsigned long setNavStateTime = 0;
unsigned long missionStart =0;
uint8_t calledSum =0;
bool rcOn = true;
bool checkChange();

//packetcounter
uint32_t packetCounter = 0;

//rc channel values
const msp_set_raw_rc_t  valuesDisarmed = {
  1500,
  1500,
  1000,
  1500,
  2000, //angle
  1000, //nav wp
  1000, //nav alt/pos hold
  1000, //arm
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
};

//rc values fc armed
const msp_set_raw_rc_t  valuesArmed = {
  1500,
  1500,
  1000,
  1500,
  2000, //angle
  1000, //nav wp
  1000, //nav alt/pos hold
  2000, //arm
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
};

//rc values fc armed + altitude hold on
const msp_set_raw_rc_t  valuesArmedAltHold = {
  1500,
  1500,
  1750, //throttle
  1500,
  2000, //angle
  1000, //nav wp
  1500, //nav alt/pos hold
  2000, //arm
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
};

//rc values fc armed + waypoint navigation on
const msp_set_raw_rc_t  valuesArmedWpNav = {
  1500,
  1500,
  1000,
  1500,
  2000, //angle
  2000, //nav wp
  1000, //nav alt/pos hold
  2000, //arm
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
  //rf95 setup
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  if (!rf95.init()) {
    while (1);
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    while (1);
  }
  rf95.setTxPower(23, false);

  //bmp init
  if (!bmp.begin_SPI(BMP_CS)) { 
    while (1);
  }
  
  //bno init
  if (!bno08x.begin_I2C()) {
    while (1) { delay(10); }
  }

  //msp init
  Serial1.begin(115200);
  delay(2000); // give FC time to boot
  msp.begin(Serial1);

  //rc channel init
  for (int i=0; i < MSP_MAX_SUPPORTED_CHANNELS; i++) {
    rc.channel[i] = 1000;
  }
  rc.channel[2] = 1000;
  rc.channel[4] = 2000; //angle

}

//message creation
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

//bno
void setReports(void) {
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
  }
}


//geozone

//check individual vector
bool vecCheck(point a, point b, point c) {
  return ((b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x) < 0);
}

//check all borders
bool geozoneCheck(point coordinates) {
  uint8_t geoChecksum = 0;
  uint8_t polygonCount = sizeof(borders)/sizeof(borders[0]);
  for (uint8_t i =0; i < polygonCount-1; i++) {
    geoChecksum += vecCheck(borders[i], borders[i+1], coordinates);
  }
  geoChecksum += vecCheck(borders[polygonCount-1], borders[0], coordinates);
  return (geoChecksum == polygonCount);
}



//mode selections

  //standby
  void standbyMode(float h) {
    if (h > 50) {
      if(checkChange()) {
        state =ASCENDING;
        calledSum =0;
      } 
    }
  }

  //ascend
  void ascending(float lastH, float h, unsigned long now) {
    if (h - lastH < 0) {
      if(checkChange()) {
        missionStart = now;
        state =DESCENDING;
        calledSum =0;
      } 
    }
  }

  //descend
  void descending(float h, unsigned long now) {
    if (h < 900 && maxHeight-h > 50) {
      beforeFix = now;
      rc = valuesArmed;
    if(checkChange()) {
        state =WAITFORFIX;
        calledSum =0;
      } 
    }
  }

  //wait for fix in the air
  void waitForFix(unsigned long now) {
    rc = valuesArmedAltHold;

    //if timeout
    if (now - beforeFix >30000 && fix <2) {
      if(checkChange()) {
        state =FAILSAFE;
        calledSum =0;
      }
      //if fix
    } else if (fix > 1) {
      if(checkChange()) {
        state =RTWP;
        calledSum =0;
      } 
      setNavStateTime = now;
    }
  }

//return to waypoint 
void rtwpMode(uint8_t navError, unsigned long now) {
    rc = valuesArmedWpNav;

    if(navError == 4) {
      rc.channel[6] = 1000;
      if(checkChange()) {
        state =FAILSAFE;
        calledSum =0;
      }
    }

  }

//test before putting in the rocket (for 1 minute)
void tempTestMode(unsigned long now) {
    static unsigned long beforeRocket = now;

    if (now - beforeRocket > 60000) {
      state = STANDBY;
    }
  }

//turn on failsafe on fc
void failsafe() {
  rcOn = false;
}

//3 sec delay between mode switch
bool checkChange() {
  return (calledSum < 6 && state != FAILSAFE);
}

void modeLogic(unsigned long now) {

  //failsafe check

  //if outside geozone, while having gps fix
  if (fix >0 && !geozoneStatus) {
    state = FAILSAFE;
  }

  //if there is navigation error
  if (state >3 && navError > 0) {
    state = FAILSAFE;
  }

  //if battery is dead
  if (battVolt < 11.0) {
    state = FAILSAFE;
  }
  calledSum++;
  
  //if we are longer in the air then allowed
  if (now - missionStart > 80000) {
    state =FAILSAFE;
  }

  //mode selection
  switch (state) {
        case STANDBY: standbyMode(altitude); break;
        case ASCENDING: ascending(lastHeight, altitude, now); break;
        case DESCENDING: descending(altitude, now); break;
        case WAITFORFIX: waitForFix(now); break;
        case RTWP: rtwpMode(navState, now); break;
        case FAILSAFE: failsafe(); break;
        case BEFOREROCKET: tempTestMode(now); break;
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

        //gps
        if (msp.request(MSP_RAW_GPS, &gps, sizeof(gps))) {
          fix = gps.fixType;
          numSat = gps.numSat;
          rawLat = gps.lat;
          rawLong = gps.lon;
          latitude = rawLat / 1e7f;
          longitude = rawLong / 1e7f; 
          gpsAlt = gps.alt;
        }

        //battery
        if(msp.request(MSP_ANALOG, &batt, sizeof(batt))) {
          battVolt = batt.vbat;
        }
        
        //navigation data
        if(msp.request(MSP_NAV_STATUS, &navStat, sizeof(navStat))) {
          navMode = navStat.mode;
          navState = navStat.state;
          navError = navStat.error;
        }


    lastMSPReq = now;

    //geozone
    point coordinates = {latitude, longitude};
    geozoneStatus = geozoneCheck(coordinates);

    lastHeight = altitude;
    if(altitude > maxHeight ) {
      maxHeight = altitude;
    }
  }

  //failsafe logic and mode selection
  modeLogic(now);

  //rc msp sending
  if (now - lastMSP > 50 && rcOn) {
    msp.command(MSP_SET_RAW_RC, &rc, sizeof(rc));
    lastMSP = now;
  }


  //radio
  if (now - lastSerial > 1000) {
    digitalWrite(LED_BUILTIN, HIGH);

    //make packet
    message pkt = makeMessage();

    //if outside rocket or before rocket send radio
    if (state >=2) {
      //send through radio
      rf95.send((uint8_t*)&pkt, sizeof(pkt));
      rf95.waitPacketSent();
    }

    lastSerial = now;
    digitalWrite(LED_BUILTIN, LOW);
    packetCounter++;
  }

}