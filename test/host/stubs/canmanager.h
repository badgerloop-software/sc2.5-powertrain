#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

#define DEFAULT_CAN_FREQ 250000

struct CAN_TypeDef {};
enum CAN_PINS { DEF };

struct CAN_message_t {
    uint32_t id = 0;
    uint8_t len = 0;
    uint8_t buf[8]{};
};

class CANManager {
public:
    std::vector<CAN_message_t> sent_frames;

    CANManager(CAN_TypeDef*, CAN_PINS, int) {}
    virtual ~CANManager() = default;
    virtual void readHandler(CAN_message_t message) = 0;

    bool sendMessage(int id, void* data, int length, int = 10) {
        if (length < 0 || length > 8) {
            throw std::runtime_error("Invalid CAN payload length");
        }
        CAN_message_t frame{};
        frame.id = static_cast<uint32_t>(id);
        frame.len = static_cast<uint8_t>(length);
        std::memcpy(frame.buf, data, static_cast<std::size_t>(length));
        sent_frames.push_back(frame);
        return true;
    }
};
