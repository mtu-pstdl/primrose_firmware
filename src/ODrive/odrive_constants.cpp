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
        // Clear the buffer
        sprintf(buffer, "");
        // For each error code, check if it is set and if so, add it to the string with a comma

        // Check if there are more than 4 errors present to avoid a buffer overflow in the sprintf
        uint32_t errors_present = 0;
        for (int i = 0; i < 32; i++) {
            if (error_code & (1 << i)) {
                errors_present++;
            }
        }

        // If there are more than 4 errors present, just return the error code
        if (errors_present > 4) {
            sprintf(buffer, "TOO_MANY_ERRORS_0x%08lX", error_code);
            return;
        }

        if (error_code & INITIALIZING) sprintf(buffer, "INITIALIZING ");
        if (error_code & SYSTEM_ERROR) sprintf(buffer, "%sSYSTEM_ERROR ", buffer);
        if (error_code & TIMING_ERROR) sprintf(buffer, "%sTIMING_ERROR ", buffer);
        if (error_code & MISSING_ESTIMATE) sprintf(buffer, "%sMISSING_ESTIMATE ", buffer);
        if (error_code & BAD_CONFIG) sprintf(buffer, "%sBAD_CONFIG ", buffer);
        if (error_code & DRV_FAULT) sprintf(buffer, "%sDRV_FAULT ", buffer);
        if (error_code & MISSING_INPUT) sprintf(buffer, "%sMISSING_INPUT ", buffer);
        if (error_code & DC_BUS_UNDER_VOLTAGE) sprintf(buffer, "%sBUS_UNDER_VOLTAGE ", buffer);
        if (error_code & DC_BUS_OVER_VOLTAGE) sprintf(buffer, "%sBUS_OVER_VOLTAGE ", buffer);
        if (error_code & DC_BUS_OVER_CURRENT) sprintf(buffer, "%sBUS_OVER_CURRENT ", buffer);
        if (error_code & DC_BUS_OVER_REGEN_CURRENT) sprintf(buffer, "%sBUS_OVER_REGEN_CURRENT ", buffer);
        if (error_code & CURRENT_LIMIT_VIOLATION) sprintf(buffer, "%sCURRENT_LIMIT_VIOLATION ", buffer);
        if (error_code & MOTOR_OVER_TEMP) sprintf(buffer, "%sMOTOR_OVER_TEMP ", buffer);
        if (error_code & INVERTER_OVER_TEMP) sprintf(buffer, "%sINVERTER_OVER_TEMP ", buffer);
        if (error_code & VELOCITY_LIMIT_VIOLATION) sprintf(buffer, "%sVEL_LIMIT_VIOLATION ", buffer);
        if (error_code & POSITION_LIMIT_VIOLATION) sprintf(buffer, "%sPOS_LIMIT_VIOLATION ", buffer);
        if (error_code & WATCHDOG_TIMER_EXPIRED) sprintf(buffer, "%sWATCHDOG_EXPIRED ", buffer);
        if (error_code & ESTOP_REQUESTED) sprintf(buffer, "%sESTOP_REQUESTED ", buffer);
        if (error_code & SPINOUT_DETECTED) sprintf(buffer, "%sSPINOUT_DETECTED ", buffer);
        if (error_code & OTHER_DEVICE_FAILED) sprintf(buffer, "%sOTHER_DEVICE_FAILED ", buffer);
        if (error_code & CALIBRATION_ERROR) sprintf(buffer, "%sCALIBRATION_ERROR ", buffer);
        if (error_code & THERMISTOR_DISCONNECTED) sprintf(buffer, "%sTHERMISTOR_DISCONNECTED ", buffer);

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
                sprintf(buffer, "IDLE*");
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
            case ENCODER_HALL_PHASE_CALIBRATION:
                sprintf(buffer, "ENCODER_HALL_PHASE_CALIBRATION");
                break;
            case ENCODER_HALL_POLARITY_CALIBRATION:
                sprintf(buffer, "ENCODER_HALL_POLARITY_CALIBRATION");
                break;
           default:
                sprintf(buffer, "UNKNOWN: 0x%2X", axis_state);
                break;
        }
    }

    void sprint_procedure_result(char* buffer, procedure_results procedure_result) {
        switch (procedure_result) {
            case SUCCESS:
                sprintf(buffer, "SUCCESS");
                break;
            case BUSY:
                sprintf(buffer, "BUSY");
                break;
            case CANCELED:
                sprintf(buffer, "CANCELED");
                break;
            case NO_RESPONSE:
                sprintf(buffer, "NO_RESPONSE");
                break;
            case DISARMED:
                sprintf(buffer, "DISARMED");
                break;
            case POLE_PAIR_CPR_MISMATCH:
                sprintf(buffer, "POLE_PAIR_CPR_MISMATCH");
                break;
            case PHASE_INDUCTANCE_OUT_OF_RANGE:
                sprintf(buffer, "PHASE_INDUCTANCE_OUT_OF_RANGE");
                break;
            case INVALID_CONFIG:
                sprintf(buffer, "INVALID_CONFIG");
                break;
            case UNBALANCED_PHASES:
                sprintf(buffer, "UNBALANCED_PHASES");
                break;
            case INVALID_MOTOR_TYPE:
                sprintf(buffer, "INVALID_MOTOR_TYPE");
                break;
            case ILLEGAL_HALL_STATE:
                sprintf(buffer, "ILLEGAL_HALL_STATE");
                break;
            case TIMEOUT:
                sprintf(buffer, "TIMEOUT");
                break;
            case HOMING_WITHOUT_ENDSTOP:
                sprintf(buffer, "HOMING_WITHOUT_ENDSTOP");
                break;
            case INVALID_STATE:
                sprintf(buffer, "INVALID_STATE");
                break;
            case NOT_CALIBRATED:
                sprintf(buffer, "NOT_CALIBRATED");
                break;
            default:
                sprintf(buffer, "UNKNOWN_PROCEDURE_RESULT");
                break;
        }
    }

    void sprint_control_mode(char* buffer, control_modes control_mode) {
        switch (control_mode) {
            case VOLTAGE_CONTROL:
                sprintf(buffer, "VOLTAGE_CONTROL");
                break;
            case TORQUE_CONTROL:
                sprintf(buffer, "TORQUE_CONTROL");
                break;
            case VELOCITY_CONTROL:
                sprintf(buffer, "VELOCITY_CONTROL");
                break;
            case POSITION_CONTROL:
                sprintf(buffer, "POSITION_CONTROL");
                break;
            default:
                sprintf(buffer, "UNKNOWN_CONTROL_MODE");
                break;
        }
    }

    void sprint_input_mode(char* buffer, input_modes input_mode) {
        switch (input_mode) {
            case MOTOR_INACTIVE:
                sprintf(buffer, "MOTOR_INACTIVE");
                break;
            case PASSTHROUGH:
                sprintf(buffer, "PASSTHROUGH");
                break;
            case VEL_RAMP:
                sprintf(buffer, "VEL_RAMP");
                break;
            case POS_FILTER:
                sprintf(buffer, "POS_FILTER");
                break;
            case MIX_CHANNELS:
                sprintf(buffer, "MIX_CHANNELS");
                break;
            case TRAP_TRAJ:
                sprintf(buffer, "TRAP_TRAJ");
                break;
            case TORQUE_RAMP:
                sprintf(buffer, "TORQUE_RAMP");
                break;
            case MIRROR:
                sprintf(buffer, "MIRROR");
                break;
            case TUNING:
                sprintf(buffer, "TUNING");
                break;
            default:
                sprintf(buffer, "UNKNOWN_INPUT_MODE");
                break;
        }
    }
}