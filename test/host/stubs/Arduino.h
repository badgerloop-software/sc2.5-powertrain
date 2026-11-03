#pragma once

#include <cstdint>

extern unsigned long test_millis;
inline unsigned long millis() { return test_millis; }
