
//msp command codes
constexpr uint8_t GPS_GET = 106;
constexpr uint8_t RC_CMD = 200;

//structure of the byte array for the message sent through radio
struct message {
  uint8_t fix;
  uint8_t numSat;
  uint32_t latitude;
  uint32_t longitude;
  int16_t altGPS;
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

unsigned long lastMSP = 0;
unsigned long lastSerial =0;
//values representing all channels for simulating rc input to fc
uint16_t rcValues[16];


//setup function
void setup() {
  //Serial 0 init (only used for the initializing of the arduino)
  Serial.begin(9600);
  //msp init
  Serial1.begin(115200);

  //initializing values for the rc controlling of the fc 
  for (int i = 0; i < 16; i++) {
    rcValues[i] = 1500;
  }
  rcValues[2] = 1000; //throttle needs to be low for arming
}

//assigning all data to byte array, ready to be sent through radio
message makeMessage() {
  message p{};
  p.fix = fix;
  p.numSat = numSat;
  p.latitude = latitude;
  p.longitude = longitude;
  p.altGPS = altGPS;

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

//main loop
void loop() {
  mspReadGPS();

  unsigned long now = millis();
  
  if (now - lastSerial >= 1000) {
  message pkt = makeMessage();
  Serial.print("Fix: ");
  Serial.println(pkt.fix);
  
  Serial.print("Satellites: ");
  Serial.println(pkt.numSat);
  
  // Convert from 1e-7 degrees format to decimal degrees
  double latDeg = pkt.latitude / 10000000.0;
  double lonDeg = pkt.longitude / 10000000.0;
  
  Serial.print("Latitude: ");
  Serial.print(latDeg, 7);
  Serial.print("° (raw: ");
  Serial.print(pkt.latitude);
  Serial.println(")");
  
  Serial.print("Longitude: ");
  Serial.print(lonDeg, 7);
  Serial.print("° (raw: ");
  Serial.print(pkt.longitude);
  Serial.println(")");
  
  Serial.print("Altitude: ");
  Serial.print(pkt.altGPS / 100.0); // Convert cm to meters
  Serial.println(" m");
  Serial.println("=================");
  lastSerial = now;
  }

  //sending msp
  if (now - lastMSP >= 50) {
    mspCmd(RC_CMD, (uint8_t*)rcValues, 32);
    lastMSP = now;
  }

}
