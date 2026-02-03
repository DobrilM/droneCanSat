#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_BMP280.h>

//radio definition
#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4
#define RF95_FREQ 433.4 //CHANGE BEFORE LAUNCH!!!!!
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//bmp definition
#define BMP_CS 19
#define PRESSURE_SEA 1013.25 //CHANGE BEFORE LAUNCH!!!!!
Adafruit_BMP280 bmp(BMP_CS);

//Note: MSP is connected to Serial1

//msp command codes
constexpr uint8_t GPS_GET = 106;
constexpr uint8_t GYRO_GET = 102;
constexpr uint8_t NAV_STAT = 121;
constexpr uint8_t BATT_GET = 130;
constexpr uint8_t RC_CMD = 200;

//structure of the byte array for the message sent through radio
struct message {
  int16_t temp;
  int16_t alt; 
  int32_t pressure;
  uint8_t fix;
  uint8_t numSat;
  uint32_t latitude;
  uint32_t longitude;
  int16_t altGPS;
  uint16_t battVolt;
  uint8_t navStat;
  uint8_t status;
};

//type of character read by msp
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

//byte array for reading msp
uint8_t payload[16];

//other variables needed for reading msp
MSPType type = IDLE;
uint8_t dataSize = 0;
uint8_t cmd = 0;
uint8_t checksum = 0;
uint8_t arrayptr = 0;

//gps reading
uint8_t fix, numSat;
uint32_t latitude, longitude;
int16_t altGPS;

//gyro reading
float accX, accY, accZ;

//battery reading
uint16_t battVolt;

//reading mission status
uint8_t navStat;

//used for timing of both radio and 
unsigned long lastRadio = 0;
unsigned long lastMSP = 0;

//values representing all channels for simulating rc input to fc
uint16_t rcValues[16];

//state of the CanSat (0 is waiting for launch, 1 is wating for descent, 2 is moving to the set waypoint, 3 is waiting to be retrieved)
int status = 0;

//check for when the CanSat is done with the mission
bool landing = 0;

//setup function
void setup() {
  //Serial 0 init (only used for the initializing of the arduino)
  Serial.begin(9600);

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

  //bmp init
  if (!bmp.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  //msp init
  Serial1.begin(115200);

  //initializing values for the rc controlling of the fc 
  for (int i = 0; i < 16; i++) {
    rcValues[i] = 1500;
  }
  rcValues[2] = 1000; //throttle needs to be low for arming
}

//assigning all data to byte array, ready to be sent through radio
message makeMessage(float temperature, float altitude, float pressure) {
  message p{};
  p.temp = temperature * 100; //conv to int
  p.alt = altitude * 100;
  p.pressure = pressure * 100;
  p.fix = fix;
  p.numSat = numSat;
  p.latitude = latitude;
  p.longitude = longitude;
  p.altGPS = altGPS;
  p.battVolt = battVolt;
  p.navStat = navStat;
  p.status = status;

  return p;
};

//writing command to fc using msp
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

//parsing the retrieved packet, using the read command as a baseline
void parsePacket(uint8_t cmd) {
    switch (cmd) {
    case GPS_GET:
    fix  = payload[0];
    numSat = payload[1];

    latitude = *(uint32_t*)&payload[2];
    longitude = *(uint32_t*)&payload[6];
    altGPS = (*(int16_t*)&payload[10]);
    break;
    case GYRO_GET: //accel wont be sent, so thats why the convertion is made (analog acceleration -> g-force -> acceleration (m/s^2))
    accX  = *(int16_t*)&payload[6]/1670.13;
    accY  = *(int16_t*)&payload[8]/1670.13;
    accZ  = *(int16_t*)&payload[10]/1670.13; 
    break;
    case BATT_GET:
    battVolt = *(uint16_t*)&payload[0];
    break;
    case NAV_STAT:
    navStat = payload[0];
    break;

  }
}
//parsing each byte one by one and creating a packet of data
void parseMSP(uint8_t readChar) {
  switch (type) {
  case IDLE: type = (readChar == '$') ? DOLLAR : IDLE; break;
  case DOLLAR: type = (readChar == 'M') ? M : IDLE; break;
  case M: type = (readChar == '>') ? ARROW : IDLE; break;
  case ARROW: dataSize = readChar; checksum ^= readChar; type = SIZE; break;
  case SIZE: cmd = readChar; checksum ^= readChar; type = CMD; arrayptr = 0; break;
  case CMD: 
    if (arrayptr < dataSize) {
      payload[arrayptr] = readChar;
      arrayptr++;
      checksum ^= readChar;
    } else {
      type = CHECKSUM;
    }
  break;
  case CHECKSUM: 
    if (checksum == readChar) {
      parsePacket(cmd);
    }
    checksum = 0;
    type = IDLE;
    break;

  }
}

//reading gps
void mspReadGPS() {
  mspCmd(GPS_GET, nullptr, 0);
  while (Serial1.available()) {
    parseMSP(Serial1.read());
  }
}

//reading gyro
void mspReadGyro() {
  mspCmd(GYRO_GET, nullptr, 0);
  while (Serial1.available()) {
    parseMSP(Serial1.read());
  }
}

//reading battery voltage
void mspReadVoltage() {
  mspCmd(BATT_GET, nullptr, 0);
  while (Serial1.available()) {
    parseMSP(Serial1.read());
  }
}

//reading mission status
void mspReadMissionStatus() {
  mspCmd(NAV_STAT, nullptr, 0);
  while (Serial1.available()) {
    parseMSP(Serial1.read());
  }
}

//all modes

void standbyMode(float h, float a) {
  if (h > 50 && a > 3) { //3 m/s^2
      status = 1;
  }
}

void launchedMode(float h, float a) {
  if (h < 300 && a < 0 ) {
    rcValues[4] = 2000; //arm the fc 
    status = 2;
  }
}

void rtwpMode() {
  rcValues[5] = 2000; //ch 6, set to navigate mission
  if (navStat == 0) {
    status = 3;
  }
}
 

//main loop
void loop() {
  //reading values
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0;   //hPa
  float altitude =  44330.0 * (1.0 - pow(pressure / PRESSURE_SEA, 0.1903));
  mspReadGPS();
  mspReadGyro();
  mspReadVoltage();
  mspReadMissionStatus();
  unsigned long now = millis();
  
  //logic for all modes
  switch (status) {
    case 0: standbyMode(altitude, accY); break;
    case 1: launchedMode(altitude, accY); break;
    case 2: rtwpMode(); break;
    case 3: landing = true; break;
    default: 
    while(1) {
      delay(100);
    }
    break;
  }

  //sending msp
  if (now - lastMSP >= 20 && !landing) {
    mspCmd(RC_CMD, (uint8_t*)rcValues, 32);
    lastMSP = now;
  }

  //sending radio
  if (now - lastRadio >= 1000) {
    digitalWrite(LED_BUILTIN, HIGH);
    message pkt = makeMessage(temperature, altitude, pressure);
    rf95.send((uint8_t*)&pkt, sizeof(pkt));
    rf95.waitPacketSent();
    digitalWrite(LED_BUILTIN, LOW);
    lastRadio = now;
  }
}
