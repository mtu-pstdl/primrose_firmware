//
// Created by Jay on 12/11/2022.
//


#include <cstdio>
#include "odrive_constants.h"

namespace odrive {

    /**
     * @brief This function returns a string representation of the error code
     * @param error_code The 32 bit error code that contains the error flags
     * @return A string representation of the error code
     * @note The string must be freed by the caller, it is allocated on the heap
     */
    void sprintf_error_code(char* buffer, uint32_t error_code) {
        // For each error code, check if it is set and if so, add it to the string with a comma
        if (error_code & INITIALIZING) sprintf(buffer, "INITIALIZING, ");
        if (error_code & SYSTEM_ERROR) sprintf(buffer, "%sSYSTEM_ERROR, ", buffer);
        if (error_code & TIMING_ERROR) sprintf(buffer, "%sTIMING_ERROR, ", buffer);
        if (error_code & MISSING_ESTIMATE) sprintf(buffer, "%sMISSING_ESTIMATE, ", buffer);
        if (error_code & BAD_CONFIG) sprintf(buffer, "%sBAD_CONFIG, ", buffer);
        if (error_code & DRV_FAULT) sprintf(buffer, "%sDRV_FAULT, ", buffer);
        if (error_code & MISSING_INPUT) sprintf(buffer, "%sMISSING_INPUT, ", buffer);
        if (error_code & DC_BUS_UNDER_VOLTAGE) sprintf(buffer, "%sDC_BUS_UNDER_VOLTAGE, ", buffer);
        if (error_code & DC_BUS_OVER_VOLTAGE) sprintf(buffer, "%sDC_BUS_OVER_VOLTAGE, ", buffer);
        if (error_code & DC_BUS_OVER_CURRENT) sprintf(buffer, "%sDC_BUS_OVER_CURRENT, ", buffer);
        if (error_code & DC_BUS_OVER_REGEN_CURRENT) sprintf(buffer, "%sDC_BUS_OVER_REGEN_CURRENT, ", buffer);
        if (error_code & CURRENT_LIMIT_VIOLATION) sprintf(buffer, "%sCURRENT_LIMIT_VIOLATION, ", buffer);
        if (error_code & MOTOR_OVER_TEMP) sprintf(buffer, "%sMOTOR_OVER_TEMP, ", buffer);
        if (error_code & INVERTER_OVER_TEMP) sprintf(buffer, "%sINVERTER_OVER_TEMP, ", buffer);
        if (error_code & VELOCITY_LIMIT_VIOLATION) sprintf(buffer, "%sVELOCITY_LIMIT_VIOLATION, ", buffer);
        if (error_code & POSITION_LIMIT_VIOLATION) sprintf(buffer, "%sPOSITION_LIMIT_VIOLATION, ", buffer);
        if (error_code & WATCHDOG_TIMER_EXPIRED) sprintf(buffer, "%sWATCHDOG_TIMER_EXPIRED, ", buffer);
        if (error_code & ESTOP_REQUESTED) sprintf(buffer, "%sESTOP_REQUESTED, ", buffer);
        if (error_code & SPINOUT_DETECTED) sprintf(buffer, "%sSPINOUT_DETECTED, ", buffer);
        if (error_code & OTHER_DEVICE_FAILED) sprintf(buffer, "%sOTHER_DEVICE_FAILED, ", buffer);
        if (error_code & CALIBRATION_ERROR) sprintf(buffer, "%sCALIBRATION_ERROR, ", buffer);

        // If the string is empty, then there were no errors
        if (strlen(buffer) == 0) {
            sprintf(buffer, "NO_ERROR");
        }
    }

    /**
     * @brief This function returns a string representation of the axis state
     * @param axis_state The axis state
     * @return A string representation of the axis state
     */
    void sprint_axis_state(char* buffer, axis_states axis_state) {
        switch (axis_state) {
            case UNDEFINED:
                sprintf(buffer, "UNDEFINED");
                break;
            case IDLE:
                sprintf(buffer, "IDLE");
                break;
            case STARTUP_SEQUENCE:
                sprintf(buffer, "STARTUP_SEQUENCE");
                break;
            case FULL_CALIBRATION_SEQUENCE:
                sprintf(buffer, "FULL_CALIBRATION_SEQUENCE");
                break;
            case MOTOR_CALIBRATION:
                sprintf(buffer, "MOTOR_CALIBRATION");
                break;
            case ENCODER_INDEX_SEARCH:
                sprintf(buffer, "ENCODER_INDEX_SEARCH");
                break;
            case ENCODER_OFFSET_CALIBRATION:
                sprintf(buffer, "ENCODER_OFFSET_CALIBRATION");
                break;
            case CLOSED_LOOP_CONTROL:
                sprintf(buffer, "CLOSED_LOOP_CONTROL");
                break;
            case LOCKIN_SPIN:
                sprintf(buffer, "LOCKIN_SPIN");
                break;
           default:
                sprintf(buffer, "UNKNOWN");
                break;
        }
    }

}