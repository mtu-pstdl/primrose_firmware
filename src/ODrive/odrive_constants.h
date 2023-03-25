//
// Created by Jay on 12/10/2022.
//

#ifndef TEENSYCANTRANSCEIVER_ODRIVE_CONSTANTS_H
#define TEENSYCANTRANSCEIVER_ODRIVE_CONSTANTS_H

#include <cstdint>
#include <WString.h>

namespace odrive {

// Define the bitmasks for the error codes
#define INITIALIZING                0x01        // The controller is still initializing
#define SYSTEM_ERROR                0x02        // A firmware error has occurred on the controller
#define TIMING_ERROR                0x04        // A timing error has occurred on the controller
#define MISSING_ESTIMATE            0x08        // The controller has no estimate of the motor position
#define BAD_CONFIG                  0x10        // The controller has a bad configuration
#define DRV_FAULT                   0x20        // The gate driver has likely failed
#define MISSING_INPUT               0x40        // The controller was missing a required input
#define DC_BUS_UNDER_VOLTAGE        0x100       // The DC bus voltage is too low
#define DC_BUS_OVER_VOLTAGE         0x200       // The DC bus voltage is too high
#define DC_BUS_OVER_CURRENT         0x400       // The DC bus current is too high
#define DC_BUS_OVER_REGEN_CURRENT   0x800       // The DC bus regen current is too high
#define CURRENT_LIMIT_VIOLATION     0x1000      // The current limit has been violated
#define MOTOR_OVER_TEMP             0x2000      // The motor temperature is too high
#define INVERTER_OVER_TEMP          0x4000      // The inverter temperature is too high
#define VELOCITY_LIMIT_VIOLATION    0x8000      // The velocity limit has been violated
#define POSITION_LIMIT_VIOLATION    0x10000     // The position limit has been violated
#define WATCHDOG_TIMER_EXPIRED      0x20000     // The watchdog timer has expired
#define ESTOP_REQUESTED             0x40000     // An estop has been requested
#define SPINOUT_DETECTED            0x80000     // A spinout has been detected
#define OTHER_DEVICE_FAILED         0x100000    // Another device has failed
#define CALIBRATION_ERROR           0x200000    // A calibration error has occurred

    enum axis_states : uint8_t {
        UNDEFINED = 0x0,
        IDLE = 0x1,
        STARTUP_SEQUENCE = 0x2,
        FULL_CALIBRATION_SEQUENCE = 0x3,
        MOTOR_CALIBRATION = 0x4,
        ENCODER_INDEX_SEARCH = 0x6,
        ENCODER_OFFSET_CALIBRATION = 0x7,
        CLOSED_LOOP_CONTROL = 0x8,
        LOCKIN_SPIN = 0x9,
        ENCODER_DIR_FIND = 0xA,
        HOMING = 0xB,
        ENCODER_HALL_POLARITY_CALIBRATION = 0xC,
        ENCODER_HALL_PHASE_CALIBRATION = 0xD
    };

    enum procedure_results : uint8_t {
        SUCCESS = 0x0, BUSY = 0x1, CANCELED = 0x2, NO_RESPONSE = 0x3, DISARMED = 0x4,
        POLE_PAIR_CPR_MISMATCH = 0x5, PHASE_RESISTANCE_OUT_OF_RANGE = 0x5, PHASE_INDUCTANCE_OUT_OF_RANGE = 0x6,
        INVALID_CONFIG = 0x7, UNBALANCED_PHASES = 0x8, INVALID_MOTOR_TYPE = 0x9, ILLEGAL_HALL_STATE = 0xA,
        TIMEOUT = 0xB, HOMING_WITHOUT_ENDSTOP = 0xC, INVALID_STATE = 0xD, NOT_CALIBRATED = 0xE
    };

    enum control_modes : uint8_t {
        VOLTAGE_CONTROL = 0x0, TORQUE_CONTROL = 0x1, VELOCITY_CONTROL = 0x2, POSITION_CONTROL = 0x3,
        UNKNOWN_CONTROL_MODE = 0xFF
    };

    enum input_modes : uint8_t {
        MOTOR_INACTIVE = 0x0, PASSTHROUGH = 0x1, VEL_RAMP = 0x2, POS_FILTER = 0x3, MIX_CHANNELS = 0x4,
        TRAP_TRAJ = 0x5, TORQUE_RAMP = 0x6, MIRROR = 0x7, TUNING = 0x8, UNKNOWN_INPUT_MODE = 0xFF
    };

    void sprintf_error_code(char* buffer, uint32_t error_code);

    void sprint_axis_state(char* buffer, axis_states state);

}

#endif //TEENSYCANTRANSCEIVER_ODRIVE_CONSTANTS_H
