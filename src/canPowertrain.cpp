#include "canPowertrain.h"
#include "PowertrainGpio.h"
#include <EEPROM.h>
#include <cstring>
#include <limits>

static_assert(sizeof(float) == sizeof(uint32_t) &&
              std::numeric_limits<float>::is_iec559,
              "Fault CAN readings require IEEE 754 float32");

namespace {
constexpr int FAULT_EEPROM_MAGIC_ADDRESS = 0;
constexpr int FAULT_EEPROM_STATE_ADDRESS = 1;
constexpr uint8_t FAULT_EEPROM_MAGIC = 0xB5;
constexpr uint8_t FAULT_STATE_CLEAR = 0x00;
constexpr uint8_t FAULT_STATE_LATCHED = 0xA5;
constexpr uint8_t FAULT_STATE_CLEAR_ON_RESTART = 0x5A;

constexpr uint8_t FAULT_CLEAR_PAYLOAD[FAULT_CLEAR_CAN_DLC] = {
    0x03, 0x7F, 0x20, 0x22, 0x00, 0x00, 0x00, 0x00
};
}

CANPowertrain::CANPowertrain(CAN_TypeDef* canPort, CAN_PINS pins, int frequency)
    : CANManager(canPort, pins, frequency),
      bps_fault(false),
      bps_fault_latched(false),
      bps_startup_grace_started(false),
      persistent_fault_initialized(false),
      clear_on_restart_armed(false),
      first_bps_message_time(0)
      {};

void CANPowertrain::initializePersistentFault() {
    const bool storage_initialized =
        EEPROM.read(FAULT_EEPROM_MAGIC_ADDRESS) == FAULT_EEPROM_MAGIC;

    if (!storage_initialized) {
        EEPROM.update(FAULT_EEPROM_STATE_ADDRESS, FAULT_STATE_CLEAR);
        EEPROM.update(FAULT_EEPROM_MAGIC_ADDRESS, FAULT_EEPROM_MAGIC);
        bps_fault_latched = false;
    } else {
        const uint8_t stored_state =
            EEPROM.read(FAULT_EEPROM_STATE_ADDRESS);
        if (stored_state == FAULT_STATE_CLEAR_ON_RESTART) {
            EEPROM.update(FAULT_EEPROM_STATE_ADDRESS, FAULT_STATE_CLEAR);
            bps_fault_latched = false;
        } else {
            bps_fault_latched =
                stored_state == FAULT_STATE_LATCHED;
            if (!bps_fault_latched &&
                stored_state != FAULT_STATE_CLEAR) {
                EEPROM.update(
                    FAULT_EEPROM_STATE_ADDRESS,
                    FAULT_STATE_CLEAR
                );
            }
        }
    }

    clear_on_restart_armed = false;
    persistent_fault_initialized = true;
}

bool CANPowertrain::handleTempMsg(const CAN_message_t& msg, FaultEvent& fault_event) {
    uint16_t lowest_temperature = decodeUint16(msg.buf[0], msg.buf[1]);
    uint16_t highest_temperature = decodeUint16(msg.buf[2], msg.buf[3]);

    fault_event.raw = 0;
    fault_event.value = 0.0f;

    if (shouldMonitorBpsFaults()) {
        if (highest_temperature > BPS_TEMPERATURE_MAX_C) {
            fault_event.status = POWERTRAIN_FAULT_PARAMETER_OVERTEMP;
            fault_event.raw = highest_temperature;
            fault_event.value = static_cast<float>(highest_temperature);
            return true;
        } 
    }
    return false;
}

bool CANPowertrain::handleElectricalMsg(const CAN_message_t& msg, FaultEvent& fault_event) {
    uint16_t highest_cell_voltage_raw = decodeUint16(msg.buf[0], msg.buf[1]);
    uint16_t lowest_cell_voltage_raw = decodeUint16(msg.buf[2], msg.buf[3]);
    uint16_t pack_current_raw = decodeUint16(msg.buf[4], msg.buf[5]);

    float highest_cell_voltage = decodeCellVoltage(msg.buf[0], msg.buf[1]);
    float lowest_cell_voltage = decodeCellVoltage(msg.buf[2], msg.buf[3]);
    float pack_current = decodePackCurrent(msg.buf[4], msg.buf[5]);

    fault_event.raw = 0;
    fault_event.value = 0.0f;

    if (shouldMonitorBpsFaults()) {
        if (highest_cell_voltage_raw > BPS_CELL_VOLTAGE_MAX_V) {
            fault_event.raw = highest_cell_voltage_raw;
            fault_event.value = highest_cell_voltage;
            fault_event.status = POWERTRAIN_FAULT_PARAMETER_CELL_OVERVOLT;
            return true;
        } else if (lowest_cell_voltage_raw > BPS_CELL_VOLTAGE_MAX_V) {
            fault_event.raw = lowest_cell_voltage_raw;
            fault_event.value = lowest_cell_voltage;
            fault_event.status = POWERTRAIN_FAULT_PARAMETER_CELL_OVERVOLT;
            return true;
        } else if (lowest_cell_voltage_raw < BPS_CELL_VOLTAGE_MIN_V) {
            fault_event.raw = lowest_cell_voltage_raw;
            fault_event.value = lowest_cell_voltage;
            fault_event.status = POWERTRAIN_FAULT_PARAMETER_CELL_UNDERVOLT;
            return true;
        } else if (highest_cell_voltage_raw < BPS_CELL_VOLTAGE_MIN_V) {
            fault_event.raw = highest_cell_voltage_raw;
            fault_event.value = highest_cell_voltage;
            fault_event.status = POWERTRAIN_FAULT_PARAMETER_CELL_UNDERVOLT;
            return true;
        } else if (pack_current_raw > BPS_PACK_CURRENT_MAX_A) {
            fault_event.raw = pack_current_raw;
            fault_event.value = pack_current;
            fault_event.status = POWERTRAIN_FAULT_PARAMETER_OVERCURRENT;
            return true;
        }
    }
    return false;
}

void CANPowertrain::readHandler(CAN_message_t msg) {
    if (isFaultClearCommand(msg)) {
        handleFaultClearCommand();
        return;
    }

    FaultEvent fault_event;
    bool fault_detected = false;
    if (msg.id == BPS_TEMPERATURE_CAN_ID &&
        msg.len == BPS_TEMPERATURE_CAN_DLC) {
        fault_detected = handleTempMsg(msg, fault_event);
    } else if (msg.id == BPS_ELECTRICAL_CAN_ID &&
               msg.len == BPS_ELECTRICAL_CAN_DLC) {
        fault_detected = handleElectricalMsg(msg, fault_event);
    }

    if (fault_detected) {
        fault_events_queue.push(fault_event);
    }
}

uint16_t CANPowertrain::decodeUint16(uint8_t byte_1, uint8_t byte_2) const {
    return (static_cast<uint16_t>(byte_1) << 8) |
           static_cast<uint16_t>(byte_2);
}

float CANPowertrain::decodeCellVoltage(uint8_t byte_1, uint8_t byte_2) const {
    return static_cast<float>(decodeUint16(byte_1, byte_2)) *
           BPS_CELL_VOLTAGE_SCALE_V;
}

float CANPowertrain::decodePackCurrent(uint8_t byte_1, uint8_t byte_2) const {
    uint16_t pack_current_raw = decodeUint16(byte_1, byte_2);
    const uint16_t current_magnitude =
        pack_current_raw >= BPS_PACK_CURRENT_ZERO_RAW
            ? pack_current_raw - BPS_PACK_CURRENT_ZERO_RAW
            : BPS_PACK_CURRENT_ZERO_RAW - pack_current_raw;
    return static_cast<float>(current_magnitude) * BPS_PACK_CURRENT_SCALE_A;
}

bool CANPowertrain::isFaultClearCommand(const CAN_message_t& msg) const {
    if (msg.id != FAULT_CLEAR_CAN_ID ||
        msg.len != FAULT_CLEAR_CAN_DLC) {
        return false;
    }

    for (uint8_t i = 0; i < FAULT_CLEAR_CAN_DLC; ++i) {
        if (msg.buf[i] != FAULT_CLEAR_PAYLOAD[i]) {
            return false;
        }
    }
    return true;
}

void CANPowertrain::handleFaultClearCommand() {
    if (!persistent_fault_initialized) {
        return;
    }

    EEPROM.update(
        FAULT_EEPROM_STATE_ADDRESS,
        FAULT_STATE_CLEAR_ON_RESTART
    );
    clear_on_restart_armed = true;
}

void CANPowertrain::latchBpsFault() {
    bps_fault_latched = true;
    if (persistent_fault_initialized) {
        EEPROM.update(FAULT_EEPROM_STATE_ADDRESS, FAULT_STATE_LATCHED);
    }
    clear_on_restart_armed = false;
}

bool CANPowertrain::shouldMonitorBpsFaults() {
    const unsigned long current_time = millis();
    if (!bps_startup_grace_started) {
        first_bps_message_time = current_time;
        bps_startup_grace_started = true;
        return false;
    }

    return current_time - first_bps_message_time >= BPS_FAULT_STARTUP_GRACE_MS;
}

void CANPowertrain::sendPowertrainData() {
    const bool estop_pressed = !PowertrainGPIO::get_estop_mcu();
    if (bps_fault && (!bps_fault_latched || clear_on_restart_armed)) {
        latchBpsFault();
    }
    const bool active_fault = estop_pressed || bps_fault_latched;
    PowertrainGPIO::set_mcu_batt_en(!active_fault);
    uint8_t status = fault_events_queue.empty() ? 0 : POWERTRAIN_FAULT_MASK;
    uint8_t fault_payload[POWERTRAIN_FAULT_CAN_DLC]{};
    fault_payload[0] = status;
    if (status & POWERTRAIN_FAULT_MASK) {
        fault_payload[1] = fault_events_queue.front().status;
        fault_payload[2] = fault_events_queue.front().raw >> 8;
        fault_payload[3] = static_cast<uint8_t>(fault_events_queue.front().raw);
        uint32_t value_bits = 0;
        std::memcpy(&value_bits, &fault_events_queue.front().value, sizeof(value_bits));
        for (uint8_t i = 0; i < sizeof(value_bits); ++i) {
            fault_payload[4 + i] = static_cast<uint8_t>(value_bits >> (8 * i));
        }
        fault_events_queue.pop();
    }
    float i_12v = PowertrainGPIO::get_i_12v();
    float v_12v = PowertrainGPIO::get_v_12v();
    float supp_i = PowertrainGPIO::get_supp_i();
    float batt_i = PowertrainGPIO::get_batt_i();
    float supp_v = PowertrainGPIO::get_supp_v();

    // TODO: send messages with their respective CAN IDs
    this->sendMessage(0x500, (void*)&i_12v, sizeof(float));
    this->sendMessage(0x501, (void*)&v_12v, sizeof(float));
    this->sendMessage(0x502, (void*)&supp_i, sizeof(float));
    this->sendMessage(0x503, (void*)&batt_i, sizeof(float));
    this->sendMessage(0x504, (void*)&supp_v, sizeof(float));
    this->sendMessage(
        POWERTRAIN_FAULT_CAN_ID,
        (void*)fault_payload,
        POWERTRAIN_FAULT_CAN_DLC
    );
}