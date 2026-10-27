#pragma once

#include <Arduino.h>
#include "STM32TimerInterrupt_Generic.h"
#include "adc.h"

// Inputs
#define BATT_NEG_CONT_MCU           PB3
#define ESTOP_MCU                   PA9
#define BATT_POS_CONT_MCU           PA10
#define PPC1_SUPP_INVALID           PB1
#define PPC1_DCDC_INVALID           PB0
#define MPPT_CONT_MCU               PB5
#define MC_CONT_MCU                 PB4
#define I_12V_ADC                   ADC_CHANNEL_12 // PA7
#define V_12V_ADC                   ADC_CHANNEL_11 // PA6
#define SUPP_I_ADC                  ADC_CHANNEL_9  // PA4
#define BATT_I_ADC                  ADC_CHANNEL_6  // PA1
#define SUPP_V_ADC                  ADC_CHANNEL_5  // PA0

// Outputs
#define MCU_BATT_EN                 PA8

namespace PowertrainGPIO {
    // Must be called before any other GPIO function; misuse reports an error and halts.
    void initIO();

    // analog signals
    float get_i_12v();
    float get_v_12v();
    float get_supp_i();
    float get_batt_i();
    float get_supp_v();

    // digital signals
    bool get_estop_mcu();
    void set_mcu_batt_en(bool batt_en);
};
