#include "PowertrainGpio.h"

namespace {
    bool io_initialized = false;

    void requireInitialized() {
        if (!io_initialized) {
            Serial.println("PowertrainGPIO: initIO() must be called before using GPIO methods");
            for (;;) {}
        }
    }
}

void PowertrainGPIO::initIO() {
    // Initalize digital pins
    pinMode(BATT_NEG_CONT_MCU, INPUT);
    pinMode(ESTOP_MCU, INPUT);
    pinMode(BATT_POS_CONT_MCU, INPUT);
    pinMode(PPC1_SUPP_INVALID, INPUT);
    pinMode(PPC1_DCDC_INVALID, INPUT);

    // Initialize MCU_BATT_EN as push-pull output with explicit mode
    pinMode(MCU_BATT_EN, OUTPUT);
    digitalWrite(MCU_BATT_EN, 1);

    pinMode(MPPT_CONT_MCU, INPUT);
    pinMode(MC_CONT_MCU, INPUT);

    // Initialize analog pins AFTER digital outputs are set
    // This prevents ADC initialization from affecting GPIO state
    initADC(ADC1);
    io_initialized = true;
}

float PowertrainGPIO::get_i_12v() {
    requireInitialized();
    return readADC(I_12V_ADC);        // PA7
}

float PowertrainGPIO::get_v_12v() {
    requireInitialized();
    return readADC(V_12V_ADC);        // PA6
}

float PowertrainGPIO::get_supp_i() {
    requireInitialized();
    return readADC(SUPP_I_ADC);        // PA4
}

float PowertrainGPIO::get_batt_i() {
    requireInitialized();
    return readADC(BATT_I_ADC);        // PA1
}

float PowertrainGPIO::get_supp_v() {
    requireInitialized();
    return readADC(SUPP_V_ADC);        // PA0
}

bool PowertrainGPIO::get_estop_mcu() {
    requireInitialized();
    return digitalRead(ESTOP_MCU);
}

void PowertrainGPIO::set_mcu_batt_en(bool batt_en) {
    requireInitialized();
    digitalWrite(MCU_BATT_EN, batt_en);
}