#include <Arduino.h>
#include "canPowertrain.h"
#include "PowertrainGpio.h"

CANPowertrain canPowertrain(CAN1, DEF);

void setup() {
  Serial.begin(115200);
  PowertrainGPIO::initIO();
  canPowertrain.initializePersistentFault();
  Serial.println("Setup done");
}

void loop() {
  canPowertrain.sendPowertrainData();
  canPowertrain.runQueue(50);
}
