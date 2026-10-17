// Compile the actual CANPowertrain implementation against transport/MCU fakes.
// Assertions use public behavior and captured wire bytes, not private state.
#include "canPowertrain.h"
#include <EEPROM.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

unsigned long test_millis = 0;
FakeEEPROM EEPROM;
volatile Digital_Data digital_data{};
volatile float i_12v = 0;
volatile float v_12v = 0;
volatile float supp_i = 0;
volatile float batt_i = 0;
volatile float supp_v = 0;

void set_mcu_batt_en(bool enabled) { digital_data.mcu_batt_en = enabled; }

namespace {
constexpr uint8_t EEPROM_MAGIC = 0xB5;
constexpr uint8_t EEPROM_LATCHED = 0xA5;
constexpr uint8_t EEPROM_CLEAR_ON_RESTART = 0x5A;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void reset_environment() {
    test_millis = 0;
    EEPROM.bytes.fill(0xFF);
    digital_data.mcu_batt_en = false;
    digital_data.estop_mcu = true;
}

void set_pair(CAN_message_t& frame, int offset, uint16_t value) {
    frame.buf[offset] = static_cast<uint8_t>(value >> 8);
    frame.buf[offset + 1] = static_cast<uint8_t>(value & 0xFF);
}

CAN_message_t electrical(uint16_t high = 35000, uint16_t low = 34000,
                         uint16_t current = 0x8000) {
    CAN_message_t frame{};
    frame.id = BPS_ELECTRICAL_CAN_ID;
    frame.len = BPS_ELECTRICAL_CAN_DLC;
    set_pair(frame, 0, high);
    set_pair(frame, 2, low);
    set_pair(frame, 4, current);
    return frame;
}

CAN_message_t temperature(uint16_t low = 25, uint16_t high = 30) {
    CAN_message_t frame{};
    frame.id = BPS_TEMPERATURE_CAN_ID;
    frame.len = BPS_TEMPERATURE_CAN_DLC;
    set_pair(frame, 0, low);
    set_pair(frame, 2, high);
    return frame;
}

CAN_message_t clear_command() {
    CAN_message_t frame{};
    frame.id = FAULT_CLEAR_CAN_ID;
    frame.len = FAULT_CLEAR_CAN_DLC;
    const uint8_t payload[8] = {0x03, 0x7F, 0x20, 0x22, 0, 0, 0, 0};
    std::memcpy(frame.buf, payload, sizeof(payload));
    return frame;
}

void finish_grace(CANPowertrain& device) {
    test_millis = 100;
    device.readHandler(electrical());
    test_millis = 2100;
    device.readHandler(electrical());
    device.readHandler(temperature());
    require(!device.hasBpsFault(), "Normal telemetry should be within limits");
}

CAN_message_t fault_frame(CANPowertrain& device) {
    device.sent_frames.clear();
    device.sendPowertrainData();
    for (const auto& frame : device.sent_frames) {
        if (frame.id == 0x505) {
            require(frame.len == 8, "Fault CAN 0x505 must use DLC 8");
            return frame;
        }
    }
    throw std::runtime_error("No CAN 0x505 status frame sent");
}

// Build the IEEE-754 word from little-endian wire bytes before copying to float.
// This does not share the firmware's snapshot serializer.
float snapshot_value(const CAN_message_t& frame) {
    const uint32_t word = static_cast<uint32_t>(frame.buf[4]) |
        (static_cast<uint32_t>(frame.buf[5]) << 8) |
        (static_cast<uint32_t>(frame.buf[6]) << 16) |
        (static_cast<uint32_t>(frame.buf[7]) << 24);
    float value;
    static_assert(sizeof(value) == sizeof(word), "Tests require binary32 float");
    std::memcpy(&value, &word, sizeof(value));
    return value;
}

void check_snapshot(const CAN_message_t& frame, uint8_t reason, uint16_t raw,
                    float value, bool active = true) {
    require(frame.buf[0] == (active ? 1 : 0), "Incorrect fault status byte");
    require(frame.buf[1] == reason, "Incorrect fault reason byte");
    require(frame.buf[2] == static_cast<uint8_t>(raw >> 8) &&
            frame.buf[3] == static_cast<uint8_t>(raw & 0xFF),
            "Snapshot raw code must preserve exact BPS bytes in big-endian order");
    const float decoded = snapshot_value(frame);
    require(std::isfinite(decoded) && std::fabs(decoded - value) < 0.00002f,
            "Snapshot engineering value must be a little-endian float");
}

void check_same_snapshot(const CAN_message_t& actual,
                         const CAN_message_t& expected) {
    require(std::memcmp(actual.buf + 1, expected.buf + 1, 7) == 0,
            "The first fault snapshot changed");
}

void no_fault_and_estop() {
    reset_environment();
    CANPowertrain device(nullptr, DEF);
    device.initializePersistentFault();
    finish_grace(device);
    check_snapshot(fault_frame(device), 0, 0, 0, false);
    require(digital_data.mcu_batt_en, "Healthy BPS should enable battery");
    digital_data.estop_mcu = false;
    check_snapshot(fault_frame(device), 0, 0, 0);
    require(!digital_data.mcu_batt_en, "Estop should disable battery");
    digital_data.estop_mcu = true;
    check_snapshot(fault_frame(device), 0, 0, 0, false);
}

void startup_grace() {
    reset_environment();
    CANPowertrain device(nullptr, DEF);
    device.initializePersistentFault();
    test_millis = 100;
    device.readHandler(electrical(0xFFFF, 0));
    device.readHandler(temperature(0xFFFF, 0xFFFF));
    test_millis = 2099;
    device.readHandler(electrical(0xFFFF, 0));
    require(!device.hasBpsFault(), "Telemetry latched before startup grace elapsed");
    check_snapshot(fault_frame(device), 0, 0, 0, false);
    test_millis = 2100;
    device.readHandler(electrical(41801));
    check_snapshot(fault_frame(device), 1, 41801, 4.1801f);
}

void inclusive_limits() {
    reset_environment();
    CANPowertrain device(nullptr, DEF);
    device.initializePersistentFault();
    finish_grace(device);
    device.readHandler(electrical(41800, 25200, 0x8000 + 350));
    device.readHandler(temperature(0, 56));
    require(!device.hasBpsFault(), "Inclusive electrical/temperature limits tripped");
    check_snapshot(fault_frame(device), 0, 0, 0, false);
    device.readHandler(electrical(41800, 25200, 0x8000 - 350));
    check_snapshot(fault_frame(device), 0, 0, 0, false);
}

void voltage_fields() {
    struct Example { uint16_t high; uint16_t low; uint8_t reason; uint16_t raw; };
    const Example examples[] = {
        {41801, 34000, 1, 41801},
        {35000, 41801, 1, 41801}, // "lowest" can contain the larger garbage value.
        {25199, 34000, 2, 25199}, // "highest" can contain the lower garbage value.
        {35000, 25199, 2, 25199},
        {0xABCD, 34000, 1, 0xABCD},
        {35000, 0x1234, 2, 0x1234},
        {0xFFFF, 34000, 1, 0xFFFF},
        {35000, 0x0000, 2, 0x0000},
        {35000, 0x00FF, 2, 0x00FF},
        {35000, 0x0100, 2, 0x0100},
    };
    for (const auto& example : examples) {
        reset_environment();
        CANPowertrain device(nullptr, DEF);
        device.initializePersistentFault();
        finish_grace(device);
        device.readHandler(electrical(example.high, example.low));
        check_snapshot(fault_frame(device), example.reason, example.raw,
                       static_cast<float>(example.raw) / 10000.0f);
    }
}

void current_encoding() {
    const uint16_t examples[] = {0x815F, 0x7EA1, 0x0000, 0xFFFF};
    for (const uint16_t raw : examples) {
        reset_environment();
        CANPowertrain device(nullptr, DEF);
        device.initializePersistentFault();
        finish_grace(device);
        device.readHandler(electrical(35000, 34000, raw));
        const float magnitude = static_cast<float>(
            raw >= 0x8000 ? raw - 0x8000 : 0x8000 - raw) / 10.0f;
        check_snapshot(fault_frame(device), 3, raw, magnitude);
    }
}

void temperature_fields() {
    const uint16_t examples[][2] = {{25, 57}, {57, 30}, {25, 0x1234}, {0xFFFF, 30}};
    for (const auto& example : examples) {
        reset_environment();
        CANPowertrain device(nullptr, DEF);
        device.initializePersistentFault();
        finish_grace(device);
        device.readHandler(temperature(example[0], example[1]));
        const uint16_t raw = example[0] > 56 ? example[0] : example[1];
        check_snapshot(fault_frame(device), 4, raw, static_cast<float>(raw));
    }
}

void transient_before_send() {
    reset_environment();
    CANPowertrain device(nullptr, DEF);
    device.initializePersistentFault();
    finish_grace(device);
    device.readHandler(electrical(0xFFFF));
    device.readHandler(electrical());
    require(!device.hasBpsFault(), "Live BPS fault should clear after recovery");
    check_snapshot(fault_frame(device), 1, 0xFFFF, 6.5535f);
    require(!digital_data.mcu_batt_en, "Transient BPS violation must remain latched");
    require(EEPROM.read(1) == EEPROM_LATCHED, "Transient was not persisted");
}

void immutable_snapshot() {
    reset_environment();
    CANPowertrain device(nullptr, DEF);
    device.initializePersistentFault();
    finish_grace(device);
    device.readHandler(electrical(35000, 34000, 0x7EA1));
    const CAN_message_t original = fault_frame(device);
    check_snapshot(original, 3, 0x7EA1, 35.1f);
    device.readHandler(electrical(35000, 34000, 0x7D00));
    check_same_snapshot(fault_frame(device), original);
    device.readHandler(electrical());
    require(!device.hasBpsFault(), "Current fault did not recover");
    check_same_snapshot(fault_frame(device), original);
    device.readHandler(temperature(0x1234, 30));
    check_same_snapshot(fault_frame(device), original);
    device.readHandler(electrical(0xFFFF, 0));
    check_same_snapshot(fault_frame(device), original);
}

void restored_latch_has_no_snapshot() {
    reset_environment();
    EEPROM.bytes[0] = EEPROM_MAGIC;
    EEPROM.bytes[1] = EEPROM_LATCHED;
    CANPowertrain device(nullptr, DEF);
    device.initializePersistentFault();
    check_snapshot(fault_frame(device), 0, 0, 0);
    finish_grace(device);
    device.readHandler(electrical(0xFFFF));
    check_snapshot(fault_frame(device), 0, 0, 0);
}

void snapshot_not_persisted() {
    reset_environment();
    {
        CANPowertrain first(nullptr, DEF);
        first.initializePersistentFault();
        finish_grace(first);
        first.readHandler(temperature(25, 57));
        check_snapshot(fault_frame(first), 4, 57, 57.0f);
    }
    test_millis = 0;
    CANPowertrain restarted(nullptr, DEF);
    restarted.initializePersistentFault();
    check_snapshot(fault_frame(restarted), 0, 0, 0);
}

void clear_arms_restart() {
    reset_environment();
    CANPowertrain device(nullptr, DEF);
    device.initializePersistentFault();
    finish_grace(device);
    device.readHandler(electrical(41801));
    const CAN_message_t original = fault_frame(device);
    device.readHandler(electrical());
    device.readHandler(clear_command());
    require(EEPROM.read(1) == EEPROM_CLEAR_ON_RESTART, "Clear command did not arm restart");
    check_same_snapshot(fault_frame(device), original);
    require(EEPROM.read(1) == EEPROM_CLEAR_ON_RESTART, "Recovered fault canceled clear");
    test_millis = 0;
    CANPowertrain restarted(nullptr, DEF);
    restarted.initializePersistentFault();
    check_snapshot(fault_frame(restarted), 0, 0, 0, false);
    require(EEPROM.read(1) == 0, "Restart did not clear EEPROM fault latch");
}

void active_fault_cancels_clear() {
    reset_environment();
    CANPowertrain device(nullptr, DEF);
    device.initializePersistentFault();
    finish_grace(device);
    device.readHandler(electrical(35000, 25199));
    const CAN_message_t original = fault_frame(device);
    device.readHandler(clear_command());
    require(EEPROM.read(1) == EEPROM_CLEAR_ON_RESTART, "Clear did not arm while active");
    check_same_snapshot(fault_frame(device), original);
    require(EEPROM.read(1) == EEPROM_LATCHED, "Active BPS fault must cancel clear");
    device.readHandler(electrical());
    device.readHandler(clear_command());
    device.readHandler(temperature(25, 57));
    device.readHandler(temperature()); // Even a recovered transient must cancel clear.
    check_same_snapshot(fault_frame(device), original);
    require(EEPROM.read(1) == EEPROM_LATCHED, "New fault transient did not cancel clear");
    test_millis = 0;
    CANPowertrain restarted(nullptr, DEF);
    restarted.initializePersistentFault();
    check_snapshot(fault_frame(restarted), 0, 0, 0);
}

void malformed_frames_ignored() {
    reset_environment();
    CANPowertrain device(nullptr, DEF);
    device.initializePersistentFault();
    finish_grace(device);
    CAN_message_t bad = electrical(0xFFFF);
    bad.len = 5;
    device.readHandler(bad);
    bad = temperature(0xFFFF, 0xFFFF);
    bad.len = 3;
    device.readHandler(bad);
    bad = electrical(0xFFFF);
    bad.id = 0x110;
    device.readHandler(bad);
    check_snapshot(fault_frame(device), 0, 0, 0, false);
    bad = clear_command();
    bad.buf[7] = 1;
    device.readHandler(bad);
    require(EEPROM.read(1) == 0, "Malformed clear command altered EEPROM state");
}

struct TestCase { const char* name; void (*run)(); };
const TestCase tests[] = {
    {"no_fault_and_estop", no_fault_and_estop},
    {"startup_grace", startup_grace},
    {"inclusive_limits", inclusive_limits},
    {"voltage_fields", voltage_fields},
    {"current_encoding", current_encoding},
    {"temperature_fields", temperature_fields},
    {"transient_before_send", transient_before_send},
    {"immutable_snapshot", immutable_snapshot},
    {"restored_latch_has_no_snapshot", restored_latch_has_no_snapshot},
    {"snapshot_not_persisted", snapshot_not_persisted},
    {"clear_arms_restart", clear_arms_restart},
    {"active_fault_cancels_clear", active_fault_cancels_clear},
    {"malformed_frames_ignored", malformed_frames_ignored},
};
} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: fault_snapshot_test <case>\n";
        return 2;
    }
    for (const auto& test : tests) {
        if (test.name != std::string(argv[1])) continue;
        try {
            test.run();
            std::cout << test.name << " passed\n";
            return 0;
        } catch (const std::exception& error) {
            std::cerr << test.name << ": " << error.what() << '\n';
            return 1;
        }
    }
    std::cerr << "Unknown case: " << argv[1] << '\n';
    return 2;
}
