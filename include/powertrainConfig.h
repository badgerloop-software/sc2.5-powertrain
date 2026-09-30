#ifndef __POWERTRAIN_CONFIG_H__
#define __POWERTRAIN_CONFIG_H__

// CAN message definitions
#define BPS_TEMPERATURE_CAN_ID         0x108
#define BPS_TEMPERATURE_CAN_DLC        4
#define BPS_ELECTRICAL_CAN_ID          0x109
#define BPS_ELECTRICAL_CAN_DLC         6
#define POWERTRAIN_ESTOP_FAULT_CAN_ID  0x505
#define BPS_UNDERVOLTAGE_FAULT_CAN_ID  0x506
#define BPS_OVERVOLTAGE_FAULT_CAN_ID   0x507
#define BPS_OVERCURRENT_FAULT_CAN_ID   0x508
#define BPS_OVERTEMPERATURE_FAULT_CAN_ID 0x509
#define POWERTRAIN_FAULT_STATUS_DLC    1
#define POWERTRAIN_FAULT_STATUS_MASK   0x01

#define FAULT_CLEAR_CAN_ID             0x7EB
#define FAULT_CLEAR_CAN_DLC            8

// Each B1/B2 pair is a big-endian unsigned 16-bit value: B1 is the
// most-significant byte and B2 is the least-significant byte.
// Cell voltages use 0.1 mV/bit (34409 represents 3.4409 V).
#ifndef BPS_CELL_VOLTAGE_SCALE_V
#define BPS_CELL_VOLTAGE_SCALE_V       0.0001f
#endif

#define BPS_PACK_CURRENT_ZERO_RAW      0x8000U
#define BPS_PACK_CURRENT_SCALE_A       0.1f

// Inclusive operating limits
#define BPS_CELL_VOLTAGE_MIN_V         2.52f
#define BPS_CELL_VOLTAGE_MAX_V         4.18f
#define BPS_PACK_CURRENT_MIN_A          0U
#define BPS_PACK_CURRENT_MAX_A         35U
#define BPS_TEMPERATURE_MIN_C           4U
#define BPS_TEMPERATURE_MAX_C          56U

// Ignore BPS telemetry faults while its transmitters finish starting up.
#define BPS_FAULT_STARTUP_GRACE_MS     2000UL

#endif
