#include <Arduino.h>
#include "canPowertrain.h"
#include "PowertrainGpio.h"
#include "Latcher.h"

CANPowertrain canPowertrain(CAN1, DEF);

void setup() {
  Serial.begin(115200);
  PowertrainGPIO::initIO();
  Latcher::init();
  Serial.println("Setup done");
}

void loop() {
  canPowertrain.sendPowertrainData();
  canPowertrain.runQueue(50);
}
