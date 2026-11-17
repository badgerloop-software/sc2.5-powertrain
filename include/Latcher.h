#pragma once

#include <stdint.h>
#include "powertrainConfig.h"

namespace Latcher {
    constexpr uint8_t FAULT_CAUSE_UNDERVOLTAGE = 0x01;
    constexpr uint8_t FAULT_CAUSE_OVERVOLTAGE = 0x02;
    constexpr uint8_t FAULT_CAUSE_OVERCURRENT = 0x04;
    constexpr uint8_t FAULT_CAUSE_OVERTEMPERATURE = 0x08;
    constexpr uint8_t FAULT_CLEAR_PAYLOAD[FAULT_CLEAR_CAN_DLC] = {
        0x03, 0x7F, 0x20, 0x22, 0x00, 0x00, 0x00, 0x00
    };

    constexpr int FAULT_EEPROM_MAGIC_ADDRESS = 0;
    constexpr int FAULT_EEPROM_STATE_ADDRESS = 1;
    constexpr int FAULT_EEPROM_CAUSES_ADDRESS = 2;
    constexpr int FAULT_EEPROM_UNDERVOLTAGE_VALUE_ADDRESS = 3;
    constexpr int FAULT_EEPROM_OVERVOLTAGE_VALUE_ADDRESS = 5;
    constexpr int FAULT_EEPROM_OVERCURRENT_VALUE_ADDRESS = 7;
    constexpr int FAULT_EEPROM_OVERTEMPERATURE_VALUE_ADDRESS = 9;
    constexpr uint8_t FAULT_EEPROM_MAGIC = 0xB5;
    constexpr uint8_t FAULT_STATE_CLEAR = 0x00;
    constexpr uint8_t FAULT_STATE_LATCHED = 0xA5;
    constexpr uint8_t FAULT_STATE_CLEAR_ON_RESTART = 0x5A;

    // MUST CALL THIS BEFORE ANY OTHER LATCHER FUNCTIONS
    void init();

    void handleFaultClearCommand();
    bool isBpsFaultLatched();
    void latchBpsFault();
};