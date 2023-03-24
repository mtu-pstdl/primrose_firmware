//
// Created by Jay on 12/11/2022.
//


#include "odrive_constants.h"

namespace odrive {

    /**
     * @brief This function returns a string representation of the error code
     * @param error_code The 32 bit error code that contains the error flags
     * @return A string representation of the error code
     * @note The string must be freed by the caller, it is allocated on the heap
     */
    String* get_error_string(uint32_t error_code) {
        auto *error_string = new String();
        if (error_code & INITIALIZING) error_string->concat("INITIALIZING, ");
        if (error_code & SYSTEM_ERROR) error_string->concat("SYSTEM_ERROR, ");
        if (error_code & TIMING_ERROR) error_string->concat("TIMING_ERROR, ");
        if (error_code & MISSING_ESTIMATE) error_string->concat("MISSING_ESTIMATE, ");
        if (error_code & BAD_CONFIG) error_string->concat("BAD_CONFIG, ");
        if (error_code & DRV_FAULT) error_string->concat("DRV_FAULT, ");
        if (error_code & MISSING_INPUT) error_string->concat("MISSING_INPUT, ");
        if (error_code & DC_BUS_OVER_VOLTAGE) error_string->concat("DC_BUS_OVER_VOLTAGE, ");
        if (error_code & DC_BUS_UNDER_VOLTAGE) error_string->concat("DC_BUS_UNDER_VOLTAGE, ");
        if (error_code & DC_BUS_OVER_CURRENT) error_string->concat("DC_BUS_OVER_CURRENT, ");
        if (error_code & DC_BUS_OVER_REGEN_CURRENT) error_string->concat("DC_BUS_OVER_REGEN_CURRENT, ");
        if (error_code & CURRENT_LIMIT_VIOLATION) error_string->concat("CURRENT_LIMIT_VIOLATION, ");
        if (error_code & MOTOR_OVER_TEMP) error_string->concat("MOTOR_OVER_TEMP, ");
        if (error_code & INVERTER_OVER_TEMP) error_string->concat("INVERTER_OVER_TEMP, ");
        if (error_code & VELOCITY_LIMIT_VIOLATION) error_string->concat("VELOCITY_LIMIT_VIOLATION, ");
        if (error_code & POSITION_LIMIT_VIOLATION) error_string->concat("POSITION_LIMIT_VIOLATION, ");
        if (error_code & WATCHDOG_TIMER_EXPIRED) error_string->concat("WATCHDOG_TIMER_EXPIRED, ");
        if (error_code & ESTOP_REQUESTED) error_string->concat("ESTOP_REQUESTED, ");
        if (error_code & SPINOUT_DETECTED) error_string->concat("SPINOUT_DETECTED, ");
        if (error_code & OTHER_DEVICE_FAILED) error_string->concat("OTHER_DEVICE_FAILED, ");
        if (error_code & CALIBRATION_ERROR) error_string->concat("CALIBRATION_ERROR, ");
        return error_string;
    }

    /**
     * @brief This function returns a string representation of the axis state
     * @param axis_state The axis state
     * @return A string representation of the axis state
     */
    String *get_axis_state_string(axis_states axis_state) {
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
                return new String("UNKNOWN") + axis_state;
        }
    }

}