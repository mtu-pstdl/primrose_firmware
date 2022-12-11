//
// Created by Jay on 12/10/2022.
//

#ifndef TEENSYCANTRANSCEIVER_ODRIVE_CONSTANTS_H
#define TEENSYCANTRANSCEIVER_ODRIVE_CONSTANTS_H

#include <cstdint>

// Define the bitmasks for the error codes
#define INITIALIZING                0x01        // The controller is still initializing
#define SYSTEM_ERROR                0x02        // A firmware error has occurred on the controller
#define TIMING_ERROR                0x04        // A timing error has occurred on the controller
#define MISSING_ESTIMATE            0x08        // The controller has no estimate of the motor position
#define BAD_CONFIG                  0x10        // The controller has a bad configuration
#define DRV_FAULT                   0x20        // The gate driver has likely failed
#define MISSING_INPUT               0x40        // The controller was missing a required input
#define DC_BUS_OVER_VOLTAGE         0x100       // The DC bus voltage is too high
#define DC_BUS_UNDER_VOLTAGE        0x200       // The DC bus voltage is too low
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

enum axis_states: uint8_t {
    UNDEFINED = 0x0, IDLE = 0x1, STARTUP_SEQUENCE = 0x2, FULL_CALIBRATION_SEQUENCE = 0x3, MOTOR_CALIBRATION = 0x4,
    ENCODER_INDEX_SEARCH = 0x6, ENCODER_OFFSET_CALIBRATION = 0x7, CLOSED_LOOP_CONTROL = 0x8, LOCKIN_SPIN = 0x9,
    ENCODER_DIR_FIND = 0xA, HOMING = 0xB, ENCODER_HALL_POLARITY_CALIBRATION = 0xC, ENCODER_HALL_PHASE_CALIBRATION = 0xD
};

enum procedure_results: uint8_t {
    SUCCESS = 0x0, BUSY = 0x1, CANCELED = 0x2, NO_RESPONSE = 0x3, DISARMED = 0x4,
    POLE_PAIR_CPR_MISMATCH = 0x5, PHASE_RESISTANCE_OUT_OF_RANGE = 0x5, PHASE_INDUCTANCE_OUT_OF_RANGE = 0x6,
    INVALID_CONFIG = 0x7, UNBALANCED_PHASES = 0x8, INVALID_MOTOR_TYPE = 0x9, ILLEGAL_HALL_STATE = 0xA,
    TIMEOUT = 0xB, HOMING_WITHOUT_ENDSTOP = 0xC, INVALID_STATE = 0xD, NOT_CALIBRATED = 0xE
};



/**
 * @brief This function returns a string representation of the error code
 * @param error_code The 32 bit error code that contains the error flags
 * @return A string representation of the error code
 * @note The string must be freed by the caller, it is allocated on the heap
 */
String* get_error_string(uint32_t error_code){
    auto* error_string = new String();
    if(error_code & INITIALIZING) error_string->concat("INITIALIZING, ");
    if(error_code & SYSTEM_ERROR) error_string->concat("SYSTEM_ERROR, ");
    if(error_code & TIMING_ERROR) error_string->concat("TIMING_ERROR, ");
    if(error_code & MISSING_ESTIMATE) error_string->concat("MISSING_ESTIMATE, ");
    if(error_code & BAD_CONFIG) error_string->concat("BAD_CONFIG, ");
    if(error_code & DRV_FAULT) error_string->concat("DRV_FAULT, ");
    if(error_code & MISSING_INPUT) error_string->concat("MISSING_INPUT, ");
    if(error_code & DC_BUS_OVER_VOLTAGE) error_string->concat("DC_BUS_OVER_VOLTAGE, ");
    if(error_code & DC_BUS_UNDER_VOLTAGE) error_string->concat("DC_BUS_UNDER_VOLTAGE, ");
    if(error_code & DC_BUS_OVER_CURRENT) error_string->concat("DC_BUS_OVER_CURRENT, ");
    if(error_code & DC_BUS_OVER_REGEN_CURRENT) error_string->concat("DC_BUS_OVER_REGEN_CURRENT, ");
    if(error_code & CURRENT_LIMIT_VIOLATION) error_string->concat("CURRENT_LIMIT_VIOLATION, ");
    if(error_code & MOTOR_OVER_TEMP) error_string->concat("MOTOR_OVER_TEMP, ");
    if(error_code & INVERTER_OVER_TEMP) error_string->concat("INVERTER_OVER_TEMP, ");
    if(error_code & VELOCITY_LIMIT_VIOLATION) error_string->concat("VELOCITY_LIMIT_VIOLATION, ");
    if(error_code & POSITION_LIMIT_VIOLATION) error_string->concat("POSITION_LIMIT_VIOLATION, ");
    if(error_code & WATCHDOG_TIMER_EXPIRED) error_string->concat("WATCHDOG_TIMER_EXPIRED, ");
    if(error_code & ESTOP_REQUESTED) error_string->concat("ESTOP_REQUESTED, ");
    if(error_code & SPINOUT_DETECTED) error_string->concat("SPINOUT_DETECTED, ");
    if(error_code & OTHER_DEVICE_FAILED) error_string->concat("OTHER_DEVICE_FAILED, ");
    if(error_code & CALIBRATION_ERROR) error_string->concat("CALIBRATION_ERROR, ");
    return error_string;
}

/**
 * @brief This function returns a string representation of the axis state
 * @param axis_state The axis state
 * @return A string representation of the axis state
 */
String* get_axis_state_string(axis_states axis_state) {
    switch (axis_state) {
        case UNDEFINED:
            return new String("UNDEFINED");
        case IDLE:
            return new String("IDLE");
        case STARTUP_SEQUENCE:
            return new String("STARTUP_SEQUENCE");
        case FULL_CALIBRATION_SEQUENCE:
            return new String("FULL_CALIBRATION_SEQUENCE");
        case MOTOR_CALIBRATION:
            return new String("MOTOR_CALIBRATION");
        case ENCODER_INDEX_SEARCH:
            return new String("ENCODER_INDEX_SEARCH");
        case ENCODER_OFFSET_CALIBRATION:
            return new String("ENCODER_OFFSET_CALIBRATION");
        case CLOSED_LOOP_CONTROL:
            return new String("CLOSED_LOOP_CONTROL");
        case LOCKIN_SPIN:
            return new String("LOCKIN_SPIN");
        case ENCODER_DIR_FIND:
            return new String("ENCODER_DIR_FIND");
        case HOMING:
            return new String("HOMING");
        case ENCODER_HALL_POLARITY_CALIBRATION:
            return new String("ENCODER_HALL_POLARITY_CALIBRATION");
        case ENCODER_HALL_PHASE_CALIBRATION:
            return new String("ENCODER_HALL_PHASE_CALIBRATION");
        default:
            return new String("UNKNOWN");
    }
}






#endif //TEENSYCANTRANSCEIVER_ODRIVE_CONSTANTS_H
