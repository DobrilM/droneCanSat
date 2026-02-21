#include "config.h"
#include "radio.h"
#include <RH_RF95.h>

RH_RF95 rf95(RFM95_CS, RFM95_INT);
void radioSetup(){

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

}

void radioSend(message pkt) {
	
    rf95.send((uint8_t*)&pkt, sizeof(pkt));
    rf95.waitPacketSent();
}
