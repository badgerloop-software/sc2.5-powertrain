#include "Latcher.h"
#include <EEPROM.h>

namespace Latcher {
    bool latcher_initialized = false;

    void requireInitialized() {
        if (!latcher_initialized) {
            Serial.println("Latcher: init() must be called before using Latcher methods");
            for (;;) {}
        }
    }
}

void Latcher::init() {
    const bool storage_initialized = EEPROM.read(FAULT_EEPROM_MAGIC_ADDRESS) == FAULT_EEPROM_MAGIC;

    if (!storage_initialized) {
        EEPROM.update(FAULT_EEPROM_STATE_ADDRESS, FAULT_STATE_CLEAR);
        EEPROM.update(FAULT_EEPROM_MAGIC_ADDRESS, FAULT_EEPROM_MAGIC);
    } else {
        const uint8_t stored_state = EEPROM.read(FAULT_EEPROM_STATE_ADDRESS);
        if (stored_state == FAULT_STATE_CLEAR_ON_RESTART) {
            EEPROM.update(FAULT_EEPROM_STATE_ADDRESS, FAULT_STATE_CLEAR);
        } else {
            const bool bps_fault_latched = stored_state == FAULT_STATE_LATCHED;
            if (!bps_fault_latched && stored_state != FAULT_STATE_CLEAR) {
                EEPROM.update(FAULT_EEPROM_STATE_ADDRESS, FAULT_STATE_CLEAR);
            }
        }
    }

    latcher_initialized = true;
}

void Latcher::handleFaultClearCommand() {
    requireInitialized();
    
    EEPROM.update(
        FAULT_EEPROM_STATE_ADDRESS,
        FAULT_STATE_CLEAR_ON_RESTART
    );
}

uint8_t Latcher::getPersistentFaults() {
    requireInitialized();
    return EEPROM.read(FAULT_EEPROM_STATE_ADDRESS);
}

void Latcher::latchBpsFault(uint8_t causes) {
    requireInitialized();
    EEPROM.update(FAULT_EEPROM_STATE_ADDRESS, FAULT_STATE_LATCHED);
}