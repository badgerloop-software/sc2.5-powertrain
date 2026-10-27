#pragma once

#include <queue>
#include "canmanager.h"
#include "powertrainConfig.h"

struct BpsTelemetry {
    float highest_cell_voltage;
    float lowest_cell_voltage;
    float pack_current;
    uint16_t lowest_temperature;
    uint16_t highest_temperature;
};

struct FaultEvent {
    uint8_t status;
    uint8_t raw;
    uint16_t value;
};

class CANPowertrain : public CANManager {
    private:
        bool bps_fault;
        bool bps_fault_latched;
        bool bps_startup_grace_started;
        bool persistent_fault_initialized;
        bool clear_on_restart_armed;
        unsigned long first_bps_message_time;
        std::queue<FaultEvent> fault_events_queue;

        uint16_t decodeUint16(uint8_t byte_1, uint8_t byte_2) const;
        float decodeCellVoltage(uint8_t byte_1, uint8_t byte_2) const;
        float decodePackCurrent(uint8_t byte_1, uint8_t byte_2) const;
        bool isFaultClearCommand(const CAN_message_t& msg) const;
        bool shouldMonitorBpsFaults();
        void handleFaultClearCommand();
        void latchBpsFault();

        bool handleTempMsg(const CAN_message_t& msg, FaultEvent& fault_event);
        bool handleElectricalMsg(const CAN_message_t& msg, FaultEvent& fault_event);

    public:
        CANPowertrain(CAN_TypeDef* canPort, CAN_PINS pins, int frequency = DEFAULT_CAN_FREQ);
        void initializePersistentFault();
        void readHandler(CAN_message_t msg);
        void sendPowertrainData();
};
