#include "canPowertrain.h"
#include "PowertrainGpio.h"
#include "Latcher.h"
#include <EEPROM.h>
#include <cstring>
#include <limits>

static_assert(sizeof(float) == sizeof(uint32_t) &&
              std::numeric_limits<float>::is_iec559,
              "Fault CAN readings require IEEE 754 float32");

CANPowertrain::CANPowertrain(CAN_TypeDef* canPort, CAN_PINS pins, int frequency)
    : CANManager(canPort, pins, frequency),
      bps_startup_grace_started(false),
      first_bps_message_time(0)
      {};

bool CANPowertrain::handleTempMsg(const CAN_message_t& msg) {
    uint16_t lowest_temperature = decodeUint16(msg.buf[0], msg.buf[1]);
    uint16_t highest_temperature = decodeUint16(msg.buf[2], msg.buf[3]);

    if (shouldMonitorBpsFaults()) {
        if (highest_temperature > BPS_TEMPERATURE_MAX_C) {
            return true;
        } 
    }
    return false;
}

bool CANPowertrain::handleElectricalMsg(const CAN_message_t& msg) {
    uint16_t highest_cell_voltage_raw = decodeUint16(msg.buf[0], msg.buf[1]);
    uint16_t lowest_cell_voltage_raw = decodeUint16(msg.buf[2], msg.buf[3]);
    uint16_t pack_current_raw = decodeUint16(msg.buf[4], msg.buf[5]);

    float highest_cell_voltage = decodeCellVoltage(msg.buf[0], msg.buf[1]);
    float lowest_cell_voltage = decodeCellVoltage(msg.buf[2], msg.buf[3]);
    float pack_current = decodePackCurrent(msg.buf[4], msg.buf[5]);

    if (shouldMonitorBpsFaults()) {
        return highest_cell_voltage > BPS_CELL_VOLTAGE_MAX_V ||
               lowest_cell_voltage > BPS_CELL_VOLTAGE_MAX_V ||
               lowest_cell_voltage < BPS_CELL_VOLTAGE_MIN_V ||
               highest_cell_voltage < BPS_CELL_VOLTAGE_MIN_V ||
               pack_current > BPS_PACK_CURRENT_MAX_A;
    }
    return false;
}

void CANPowertrain::readHandler(CAN_message_t msg) {
    if (isFaultClearCommand(msg)) {
        Latcher::handleFaultClearCommand();
        return;
    }

    if (msg.id == BPS_TEMPERATURE_CAN_ID &&
        msg.len == BPS_TEMPERATURE_CAN_DLC) {
        fault_detected |= handleTempMsg(msg);
    } else if (msg.id == BPS_ELECTRICAL_CAN_ID &&
               msg.len == BPS_ELECTRICAL_CAN_DLC) {
        fault_detected |= handleElectricalMsg(msg);
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
        if (msg.buf[i] != Latcher::FAULT_CLEAR_PAYLOAD[i]) {
            return false;
        }
    }
    return true;
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
    const bool active_fault = estop_pressed || fault_detected;
    if (fault_detected && !Latcher::isBpsFaultLatched()) {
        Latcher::latchBpsFault();
    }

    uint8_t status = fault_detected ? POWERTRAIN_FAULT_MASK : 0;
    PowertrainGPIO::set_mcu_batt_en(!active_fault);
    
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
        (void*)status,
        POWERTRAIN_FAULT_CAN_DLC
    );
}