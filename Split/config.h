#include <Arduino.h>

//defined for my sanity
#include <cstdint>
#include <math.h>


//bmp
#define BMP_CS 16
#define PRESSURE_SEA 982.3 //CHANGE BEFORE LAUNCH!!!!!

//imu
#define BNO08X_INT 17
#define BNO08X_RST 18
#define BNO08X_CS 19

#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4
#define RF95_FREQ 433.4 //CHANGE BEFORE LAUNCH!!!!!
