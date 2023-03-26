//
// Created by Jay on 12/10/2022.
//

#include "ODriveS1.h"

//#include <utility>
//#include "odrive_constants.h"

ODriveS1::ODriveS1(uint8_t can_id, String* name, FlexCAN_T4<CAN1, RX_SIZE_64, TX_SIZE_64> *can_bus,
                   void* estop_callback) {
    this->can_id = can_id;
    this->name = name;
    this->can_bus = can_bus;
    this->estop_callback = estop_callback;
    this->allocate_strings();
}

void ODriveS1::init() {
    // Tell the ODrive to begin homing the motor
    this->send_command(ODriveS1::Set_Axis_State, odrive::STARTUP_SEQUENCE);
}

uint8_t ODriveS1::get_can_id() const {
    return this->can_id;
}

/**
 * This method is called any time a message is received from the ODrive
 * @param msg A can message received from the ODrive this object represents
 */
void ODriveS1::on_message(const CAN_message_t &msg) {
    uint8_t msg_type = msg.id & 0x1F; // Use bitmask of 0b00000011111 to get the last 5 bits
    // Bytes are sent little endian
    uint32_t upper_32 = 0;
    uint32_t lower_32 = 0;
    if (msg.len == 8) { // 8 bytes
        upper_32 = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
        lower_32 = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
    } else if (msg.len == 4) { // 4 bytes
        upper_32 = 0;  // Set the upper 32 bits to 0 since we only have 4 bytes
        lower_32 = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
    } else {
        upper_32 = 0;
        lower_32 = 0;
    }
    this->last_message = millis();
    switch (static_cast<ODriveS1::command_ids>(msg_type)){
        case Heartbeat: // Lower 4 bytes are AXIS_ERROR and the upper 4 bytes are AXIS_STATE
            this->AXIS_ERROR       = upper_32;
            // Bitmask the lowest byte of the lower 32 bits to get the axis state
            this->AXIS_STATE       = static_cast<odrive::axis_states> (lower_32 & 0xFF);
            this->PROCEDURE_RESULT = static_cast<odrive::procedure_results>((lower_32 >> 8) & 0xFF);
            this->last_axis_state  = millis();
            this->in_flight_bitmask &= ~AXIS_STATE_FLIGHT_BIT; // Clear the bit
            break;
        case Get_Error:
            this->ACTIVE_ERRORS = lower_32;
            this->DISARM_REASON = upper_32;
            this->last_errors_update = millis();
            this->in_flight_bitmask &= ~ERROR_FLIGHT_BIT; // Clear the bit
            break;
        case Get_Encoder_Estimates:
            this->POS_ESTIMATE = (float_t) lower_32;
            this->VEL_ESTIMATE = (float_t) upper_32;
            this->last_encoder_update = millis();
            this->in_flight_bitmask &= ~ENCODER_FLIGHT_BIT; // Clear the bit
            break;
        case Get_Iq:
            this->IQ_SETPOINT = (float_t) lower_32;
            this->IQ_MEASURED = (float_t) upper_32;
            this->last_iq_update = millis();
            this->in_flight_bitmask &= ~IQ_FLIGHT_BIT; // Clear the bit
            break;
        case Get_Temperature:
            // Temperature is sent as a fix
            this->FET_TEMP =    (float_t) lower_32;
            this->MOTOR_TEMP =  (float_t) upper_32;
            this->last_temp_update = millis();
            this->in_flight_bitmask &= ~TEMP_FLIGHT_BIT; // Clear the bit
            break;
        case Get_Vbus_Voltage_Current:
            // Print the binary representation of the voltage and current for debugging
            this->VBUS_VOLTAGE = (float_t) lower_32;
            this->VBUS_CURRENT = (float_t) upper_32;
            this->last_vbus_update = millis();
            this->in_flight_bitmask &= ~VBUS_FLIGHT_BIT; // Clear the bit
            break;
        default:
            break;
    }
}
/**
 * This method is called to make sure all the data from the ODrive is up to date
 * If not if will send the corresponding data request to the ODrive
 * This method also sends the Heartbeat message to the ODrive to prevent it from E-Stopping
 */
void ODriveS1::refresh_data() {
    // For each refresh bit that is not set, send the corresponding command to the ODrive
    if (this->last_axis_state + AXIS_STATE_UPDATE_RATE < millis() &&
        !(this->in_flight_bitmask & AXIS_STATE_FLIGHT_BIT)){
        if (this->send_command(ODriveS1::Heartbeat))
            this->in_flight_bitmask |= AXIS_STATE_FLIGHT_BIT;  // Set the in flight bit to 1
    }
    if (this->last_errors_update + ERROR_UPDATE_RATE < millis() &&
        !(this->in_flight_bitmask & ERROR_FLIGHT_BIT)){
        if (this->send_command(ODriveS1::Get_Error))
            this->in_flight_bitmask |= ERROR_FLIGHT_BIT;  // Set the in flight bit to 1
    }
    if (this->last_encoder_update + ENCODER_UPDATE_RATE < millis() &&
        !(this->in_flight_bitmask & ENCODER_FLIGHT_BIT)){
        if (this->send_command(ODriveS1::Get_Encoder_Estimates))
            this->in_flight_bitmask |= ENCODER_FLIGHT_BIT;  // Set the in flight bit to 1
    }
    if (this->last_iq_update + IQ_UPDATE_RATE < millis() &&
        !(this->in_flight_bitmask & IQ_FLIGHT_BIT)){
        if (this->send_command(ODriveS1::Get_Iq))
            this->in_flight_bitmask |= IQ_FLIGHT_BIT;  // Set the in flight bit to 1
    }
    if (this->last_temp_update + TEMP_UPDATE_RATE < millis() &&
        !(this->in_flight_bitmask & TEMP_FLIGHT_BIT)){
        if (this->send_command(ODriveS1::Get_Temperature))
            this->in_flight_bitmask |= TEMP_FLIGHT_BIT;  // Set the in flight bit to 1
    }
    if (this->last_vbus_update + VBUS_UPDATE_RATE < millis() &&
        !(this->in_flight_bitmask & VBUS_FLIGHT_BIT)){
        if (this->send_command(ODriveS1::Get_Vbus_Voltage_Current))
            this->in_flight_bitmask |= VBUS_FLIGHT_BIT;  // Set the in flight bit to 1
    }
    if (this->last_heartbeat + HEARTBEAT_UPDATE_RATE < millis() &&
        !(this->in_flight_bitmask & HEARTBEAT_FLIGHT_BIT)){
        if (this->send_command(ODriveS1::Heartbeat))
            this->in_flight_bitmask |= HEARTBEAT_FLIGHT_BIT;  // Set the in flight bit to 1
    }
}


uint8_t ODriveS1::send_command(ODriveS1::command_ids command_id) {
    CAN_message_t msg;
    msg.id = this->can_id << 5 | command_id; // 6 bits for the ID and 5 bits for the command
    msg.flags.remote = true; // Set the remote flag to true (remote transmission request)
    msg.flags.extended = false; // Set the extended flag to false (standard CAN)
    msg.len = 0;
    uint8_t result = this->can_bus->write(msg);
    return result; // Return the result of the write (1 for success, -1 for failure)
}

template <typename T>
uint8_t ODriveS1::send_command(command_ids command_id, T value) {
    CAN_message_t msg;
    msg.id = this->can_id << 5 | command_id; // 6 bits for the ID and 5 bits for the command
    msg.flags.extended = false;
    msg.flags.remote = false;
    msg.len = 4;
    uint32_t lower_32 = *(uint32_t*) &value;
    msg.buf[0] = (uint8_t) (lower_32 & 0xFF);         // Set the first byte to the lower 8 bits of the value
    msg.buf[1] = (uint8_t) ((lower_32 >> 8) & 0xFF);  // Set the second byte to the next 8 bits of the value
    msg.buf[2] = (uint8_t) ((lower_32 >> 16) & 0xFF); // Set the third byte to the next 8 bits of the value
    msg.buf[3] = (uint8_t) ((lower_32 >> 24) & 0xFF); // Set the fourth byte to the upper 8 bits of the value
    uint8_t result = this->can_bus->write(msg);
    return result; // Return the result of the write (1 for success, -1 for failure)
}

template <typename T>
uint8_t ODriveS1::send_command(ODriveS1::command_ids command_id, T lower_data, T upper_data) {
    CAN_message_t msg;
    msg.id = this->can_id << 5 | command_id; // 6 bits for the ID and 5 bits for the command
    msg.flags.remote = false; // Set the remote flag to false (data transmission)
    msg.flags.extended = false; // Set the extended flag to false (standard CAN)
    msg.len = 8;
    uint32_t lower_32 = *(uint32_t*) &lower_data;  // Cast the data to a uint32_t without modifying the bits
    uint32_t upper_32 = *(uint32_t*) &upper_data;  // Cast the data to a uint32_t without modifying the bits
    msg.buf[0] = (uint8_t) (lower_32 & 0xFF);         // Set the first byte to the lower 8 bits of the value
    msg.buf[1] = (uint8_t) ((lower_32 >> 8) & 0xFF);  // Set the second byte to the next 8 bits of the value
    msg.buf[2] = (uint8_t) ((lower_32 >> 16) & 0xFF); // Set the third byte to the next 8 bits of the value
    msg.buf[3] = (uint8_t) ((lower_32 >> 24) & 0xFF); // Set the fourth byte to the upper 8 bits of the value
    msg.buf[4] = (uint8_t) (upper_32 & 0xFF);         // Set the first byte to the lower 8 bits of the value
    msg.buf[5] = (uint8_t) ((upper_32 >> 8) & 0xFF);  // Set the second byte to the next 8 bits of the value
    msg.buf[6] = (uint8_t) ((upper_32 >> 16) & 0xFF); // Set the third byte to the next 8 bits of the value
    msg.buf[7] = (uint8_t) ((upper_32 >> 24) & 0xFF); // Set the fourth byte to the upper 8 bits of the value
    uint8_t result = this->can_bus->write(msg);
    return result; // Return the result of the write (1 for success, -1 for failure)
}

//void ODriveS1::set_config(ODRIVE_MOTOR_CONFIG* config) {
//    this->send_command(command_ids::Set_Limits,
//                          config->velocity_lim, config->current_lim);
//    this->send_command(command_ids::Set_Vel_Gains,
//                          config->velocity_gain, config->velocity_integrator_gain);
//    this->send_command(command_ids::Set_Pos_Gains, config->position_gain);
//    this->send_command(command_ids::Set_Traj_Acc_Limit,
//                          config->acceleration_lim, config->deceleration_lim);
//
//}

//String* ODriveS1::get_state_string() {
//    // Check if there are any errors
//    auto* state_string = new String();
//    state_string->concat("CAN ID: " + String(this->can_id) + " | " + *this->name + "\r\n");
//    if (this->ACTIVE_ERRORS != 0){
//        String* error_string = odrive::get_error_string(this->ACTIVE_ERRORS);
//        state_string->concat("ACTIVE ERRORS: " + *error_string + "\r\n");
//        free(error_string);
//    } else {
//        state_string->concat("ACTIVE ERRORS: None\r\n");
//    }
//    if (this->AXIS_ERROR != 0){
//        String* error_string = odrive::get_error_string(this->AXIS_ERROR);
//        state_string->concat("AXIS ERROR: " + *error_string + "\r\n");
//        free(error_string);
//    } else {
//        state_string->concat("AXIS ERROR: None\r\n");
//    }
//    if (this->DISARM_REASON != 0){
//        String* error_string = odrive::get_error_string(this->DISARM_REASON);
//        state_string->concat("DISARM REASON: " + *error_string + "\r\n");
//        free(error_string);
//    } else {
//        state_string->concat("DISARM REASON: None\r\n");
//    }
//    String* axis_state = odrive::get_axis_state_string(static_cast<odrive::axis_states>(this->AXIS_STATE));
//    state_string->concat("AXIS_STATE: " + *axis_state + "\r\n");
//    free(axis_state);
//
//    state_string->concat("TARGET_IQ: " + String(this->IQ_SETPOINT) + "\r\n");
//    state_string->concat("MEASURED_IQ: " + String(this->IQ_MEASURED) + "\r\n");
//
//    state_string->concat("VBUS: " + String(this->VBUS_VOLTAGE) + "V | "
//    + String(this->VBUS_CURRENT) + "A\r\n");
//
//    state_string->concat("TEMP FET: " + String(this->FET_TEMP) + "C | MOTOR: " +
//    String(this->MOTOR_TEMP) + "C\r\n");
//
//    state_string->concat("ENCODER ESTIMATES: POS: " + String(this->POS_ESTIMATE) + " | VEL: " +
//    String(this->VEL_ESTIMATE) + "\r\n");
//
//    state_string->concat("LAST UPDATE: " + String(millis() - this->last_message) + "ms ago\r\n");
//    state_string->concat("SENT COMMANDS: " + String(this->sent_messages) + "\r\n");
//    return state_string;
//}

float_t ODriveS1::get_fet_temp() const {
    return this->FET_TEMP;
}

float_t ODriveS1::get_motor_temp() const {
    return this->MOTOR_TEMP;
}

float_t ODriveS1::get_vbus_voltage() const {
    return this->VBUS_VOLTAGE;
}

float_t ODriveS1::get_vbus_current() const {
    return this->VBUS_CURRENT;
}

float_t ODriveS1::get_pos_estimate() const {
    return this->POS_ESTIMATE;
}

float_t ODriveS1::get_vel_estimate() const {
    return this->VEL_ESTIMATE;
}

float_t ODriveS1::get_Iq_setpoint() const {
    return this->IQ_SETPOINT;
}

float_t ODriveS1::get_Iq_measured() const {
    return this->IQ_MEASURED;
}

float_t ODriveS1::get_setpoint() const {
    if (this->control_mode == odrive::control_modes::POSITION_CONTROL) {
        return this->position_setpoint;
    } else if (this->control_mode == odrive::control_modes::VELOCITY_CONTROL) {
        return this->velocity_setpoint;
    } else if (this->control_mode == odrive::control_modes::TORQUE_CONTROL) {
        return this->torque_setpoint;
    } else {
        return 0;
    }
}

odrive::axis_states ODriveS1::get_axis_state() const {
    return this->AXIS_STATE;
}

char* ODriveS1::get_axis_state_string() {
    sprintf(this->axis_state_string, ""); // Clear the string
    odrive::sprint_axis_state(this->axis_state_string, static_cast<odrive::axis_states>(this->AXIS_STATE));
    return this->axis_state_string;
}

uint32_t ODriveS1::get_axis_error() const {
    return this->ACTIVE_ERRORS;
}

char* ODriveS1::get_axis_error_string() {
    sprintf(this->axis_error_string, ""); // Clear the string
    odrive::sprintf_error_code(this->axis_error_string, this->AXIS_ERROR);
    return this->axis_error_string;
}

uint32_t ODriveS1::get_active_errors() const {
    return this->ACTIVE_ERRORS;
}

char* ODriveS1::get_active_errors_string() {
    sprintf(this->active_errors_string, ""); // Clear the string
    odrive::sprintf_error_code(this->active_errors_string, this->ACTIVE_ERRORS);
    return this->active_errors_string;
}

uint32_t ODriveS1::get_disarm_reason() const {
    return this->DISARM_REASON;
}

char* ODriveS1::get_disarm_reason_string() {
    sprintf(this->disarm_reason_string, ""); // Clear the string
    odrive::sprintf_error_code(this->disarm_reason_string, this->DISARM_REASON);
    return this->disarm_reason_string;
}

bool ODriveS1::is_connected() const {
    if (millis() - this->last_message > 7500) {
        return false;
    } else {
        return true;
    }
}

void ODriveS1::estop() {
    this->send_command(command_ids::Estop);
}

odrive::procedure_results ODriveS1::get_procedure_results() const {
    return this->PROCEDURE_RESULT;
}

char *ODriveS1::get_procedure_results_string() {
    sprintf(this->procedure_result_string, ""); // Clear the string
    odrive::sprint_procedure_result(this->procedure_result_string, this->PROCEDURE_RESULT);
    return this->procedure_result_string;
}

bool ODriveS1::has_error() const {
    if (this->ACTIVE_ERRORS != 0 || this->AXIS_ERROR != 0) {
        return true;
    } else {
        return false;
    }
}

odrive::control_modes ODriveS1::get_control_mode() const {
    return this->control_mode;
}

char *ODriveS1::get_control_mode_string() {
    sprintf(this->control_mode_string, ""); // Clear the string
    odrive::sprint_control_mode(this->control_mode_string, this->control_mode);
    return this->control_mode_string;
}

void ODriveS1::set_ticks_per_rev(float_t value) {
    this->ticks_per_rev = value;
    this->has_rev_conversion = true;
}

void ODriveS1::set_revs_per_meter(float_t value) {
    this->revs_per_meter = value;
    this->has_meter_conversion = true;
}

void ODriveS1::set_conversion(float_t ticks_value, float_t revs_value) {
    this->ticks_per_rev = ticks_value;
    this->revs_per_meter = revs_value;
    this->has_rev_conversion = true;
    this->has_meter_conversion = true;
}








