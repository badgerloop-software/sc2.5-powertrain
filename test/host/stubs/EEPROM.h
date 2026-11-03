#pragma once

#include <array>
#include <cstdint>

struct FakeEEPROM {
    std::array<uint8_t, 16> bytes{};

    uint8_t read(int address) const { return bytes.at(address); }
    void update(int address, uint8_t value) { bytes.at(address) = value; }
};

extern FakeEEPROM EEPROM;
