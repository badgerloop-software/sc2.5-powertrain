#pragma once

#include <queue>
#include "canmanager.h"
#include "powertrainConfig.h"

struct FaultEvent {
    uint8_t status;
    uint16_t raw;
    float value;
};

class CANPowertrain : public CANManager {
    private:
        bool bps_startup_grace_started;
        unsigned long first_bps_message_time;
        std::queue<FaultEvent> fault_events_queue;

        uint16_t decodeUint16(uint8_t byte_1, uint8_t byte_2) const;
        float decodeCellVoltage(uint8_t byte_1, uint8_t byte_2) const;
        float decodePackCurrent(uint8_t byte_1, uint8_t byte_2) const;
        bool isFaultClearCommand(const CAN_message_t& msg) const;
        bool shouldMonitorBpsFaults();

        /**
         * @param msg The CAN message to check for temp faults.
         * @param fault_event A reference to a FaultEvent struct that will be populated if a fault is detected.
         * @return Returns true if the message contains a fault; otherwise returns false.
         */
        bool handleTempMsg(const CAN_message_t& msg, FaultEvent& fault_event);

        /**
         * @param msg The CAN message to check for electrical faults.
         * @param fault_event A reference to a FaultEvent struct that will be populated if a fault is detected.
         * @return Returns true if the message contains a fault; otherwise returns false.
         */
        bool handleElectricalMsg(const CAN_message_t& msg, FaultEvent& fault_event);

    public:
        CANPowertrain(CAN_TypeDef* canPort, CAN_PINS pins, int frequency = DEFAULT_CAN_FREQ);

        /**
         * @param msg The CAN message to process. This function will check for fault clear commands and handle temperature and electrical messages.
         * If a fault is detected, it will be added to the fault_events_queue.
         */
        void readHandler(CAN_message_t msg);

        /**
         * @brief Sends powertrain data over CAN. This function is responsible for packaging and sending the relevant powertrain data to the CAN bus.
         * It should be called periodically to ensure that the powertrain data is transmitted at the desired frequency.
         */
        void sendPowertrainData();
};
