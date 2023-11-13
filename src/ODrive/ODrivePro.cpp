//
// Created by Jay on 12/10/2022.
//

#include "ODrivePro.h"

//#include <utility>
//#include "odrive_constants.h"

ODrivePro::ODrivePro(uint8_t can_id, FlexCAN_T4<CAN1, RX_SIZE_64, TX_SIZE_64> *can_bus,
                     void* estop_callback) {
    this->can_id = can_id;
    this->can_bus = can_bus;
    this->estop_callback = estop_callback;
    this->allocate_strings();
}

void ODrivePro::init() {
    // Tell the ODrive to begin homing the motor
    this->send_command(odrive::Clear_Errors);
}

void ODrivePro::set_setpoint(float_t value) {
    switch(this->control_mode){
        case odrive::VOLTAGE_CONTROL:
            break;
        case odrive::TORQUE_CONTROL:
            this->torque_setpoint = value;
            this->send_command(odrive::Set_Input_Torque, value);
            break;
        case odrive::VELOCITY_CONTROL:
            this->velocity_setpoint = value;
            if (this->has_feedforward) {
                this->send_command(odrive::Set_Input_Vel, value,
                                   this->calculate_feedforward(value));
            } else {
                this->send_command(odrive::Set_Input_Vel, value, 0);
            }
            break;
        case odrive::POSITION_CONTROL:
            this->position_setpoint = value;
            this->send_command(odrive::command_ids::Set_Input_Pos, value, 0);
            break;
        case odrive::UNKNOWN_CONTROL_MODE:
            break;
    }
}

uint8_t ODrivePro::get_can_id() const {
    return this->can_id;
}

/**
 * This method is called any time a serial_message is received from the ODrive
 * @param msg A can serial_message received from the ODrive this object represents
 */
void ODrivePro::on_message(const CAN_message_t &msg) {
    uint8_t msg_type = msg.id & 0x1F; // Use bitmask of 0b00000011111 to get the last 5 bits
    // Bytes are sent little endian
    uint32_t upper_32 = 0;
    uint32_t lower_32 = 0;
    if (msg.len == 8) { // 8 bytes
        upper_32 = (msg.buf[3] << 24) | (msg.buf[2] << 16) | (msg.buf[1] << 8) | msg.buf[0];
        lower_32 = (msg.buf[7] << 24) | (msg.buf[6] << 16) | (msg.buf[5] << 8) | msg.buf[4];
    } else if (msg.len == 4) { // 4 bytes
        upper_32 = 0;  // Set the upper 32 bits to 0 since we only have 4 bytes
        lower_32 = (msg.buf[3] << 24) | (msg.buf[2] << 16) | (msg.buf[1] << 8) | msg.buf[0];
    } else {
        upper_32 = 0;
        lower_32 = 0;
    }
    this->last_message = millis();
    switch (static_cast<odrive::command_ids>(msg_type)){
        case odrive::Heartbeat: // Lower 4 bytes are AXIS_ERROR and the
        // upper 4 bytes are AXIS_STATE and PROCEDURE_RESULT
            this->AXIS_ERROR       = upper_32;
            // Bitmask the lowest byte of the lower 32 bits to get the axis state
            this->AXIS_STATE       = static_cast<odrive::axis_states> (lower_32 & 0xFF);
            this->PROCEDURE_RESULT = static_cast<odrive::procedure_results>((lower_32 >> 8) & 0xFF);
            this->last_axis_state  = millis();
            this->in_flight_bitmask &= ~AXIS_STATE_FLIGHT_BIT; // Clear the bit
            break;
        case odrive::Get_Error:
            this->ACTIVE_ERRORS = lower_32;
            this->DISARM_REASON = upper_32;
            this->last_errors_update = millis();
            this->in_flight_bitmask &= ~ERROR_FLIGHT_BIT; // Clear the bit
            break;
        case odrive::Get_Encoder_Estimates:
            this->POS_ESTIMATE = * (float *) &upper_32;
            this->VEL_ESTIMATE = * (float *) &lower_32;
            this->update_odometer();
            this->last_encoder_update = millis();
            this->in_flight_bitmask &= ~ENCODER_FLIGHT_BIT; // Clear the bit
            break;
        case odrive::Get_Iq:
            this->IQ_SETPOINT = * (float *) &upper_32;
            this->IQ_MEASURED = * (float *) &lower_32;
            this->last_iq_update = millis();
            this->in_flight_bitmask &= ~IQ_FLIGHT_BIT; // Clear the bit
            break;
        case odrive::Get_Temperature:
            this->FET_TEMP   = * (float *) &upper_32;
            this->MOTOR_TEMP = * (float *) &lower_32;
            this->last_temp_update = millis();
            this->in_flight_bitmask &= ~TEMP_FLIGHT_BIT; // Clear the bit
            break;
        case odrive::Get_Bus_Voltage_Current:
            this->VBUS_CURRENT = * (float *) &lower_32;
            this->VBUS_VOLTAGE = * (float *) &upper_32;
            this->update_power_consumption();
            this->last_vbus_update = millis();
            this->in_flight_bitmask &= ~VBUS_FLIGHT_BIT; // Clear the bit
            break;
        case odrive::Get_Torques:
            this->TORQUE_TARGET   = * (float *) &lower_32;
            this->TORQUE_ESTIMATE = * (float *) &upper_32;
            this->last_torque_update = millis();
            this->in_flight_bitmask &= ~TORQUE_FLIGHT_BIT; // Clear the bit
            break;
        default:
            break;
    }
}

void ODrivePro::calibration_sequence() {
    switch (this->calibration_step) {
        case 0:
            this->send_command(odrive::command_ids::Set_Axis_State,
                               odrive::MOTOR_CALIBRATION);
            this->calibration_step++;
            break;
        case 1:
            if (this->AXIS_STATE == odrive::axis_states::IDLE) {
                this->send_command(odrive::command_ids::Set_Axis_State,
                                   odrive::ENCODER_HALL_PHASE_CALIBRATION);
                this->calibration_step++;
            }
            break;
        case 2:
            if (this->AXIS_STATE == odrive::axis_states::ENCODER_HALL_PHASE_CALIBRATION) this->calibration_step++;
            break;
//        case 3:
//            if (this->AXIS_STATE == odrive::axis_states::IDLE) {
//                this->send_command(odrive::command_ids::Set_Axis_State,
//                                   odrive::ENCODER_OFFSET_CALIBRATION);
//                this->calibration_step++;
//            }
//            break;
        case 3:
            if (this->AXIS_STATE == odrive::axis_states::IDLE) {
                this->calibrating = false;
                this->calibration_step = 0;
            }
            break;
    }
}

/**
 * This method is called to make sure all the data from the ODrive is up to date
 * If not if will send the corresponding data request to the ODrive
 * This method also sends the Heartbeat serial_message to the ODrive to prevent it from E-Stopping
 */
void ODrivePro::refresh_data() {

    if (this->calibrating) {  // There are 4 steps to calibration
        this->calibration_sequence();
    }

    // For each refresh bit that is not set, send the corresponding command to the ODrive
    if (this->last_axis_state + AXIS_STATE_UPDATE_RATE < millis() &&
        !(this->in_flight_bitmask & AXIS_STATE_FLIGHT_BIT)){
        if (this->send_command(odrive::command_ids::Heartbeat))
            this->in_flight_bitmask |= AXIS_STATE_FLIGHT_BIT;  // Set the in flight bit to 1
    }
    if (this->last_errors_update + ERROR_UPDATE_RATE < millis() &&
        !(this->in_flight_bitmask & ERROR_FLIGHT_BIT)){
        if (this->send_command(odrive::command_ids::Get_Error))
            this->in_flight_bitmask |= ERROR_FLIGHT_BIT;  // Set the in flight bit to 1
    }
    if (this->last_encoder_update + ENCODER_UPDATE_RATE < millis() &&
        !(this->in_flight_bitmask & ENCODER_FLIGHT_BIT)){
        if (this->send_command(odrive::command_ids::Get_Encoder_Estimates))
            this->in_flight_bitmask |= ENCODER_FLIGHT_BIT;  // Set the in flight bit to 1
    }
    if (this->last_iq_update + IQ_UPDATE_RATE < millis() &&
        !(this->in_flight_bitmask & IQ_FLIGHT_BIT)){
        if (this->send_command(odrive::command_ids::Get_Iq))
            this->in_flight_bitmask |= IQ_FLIGHT_BIT;  // Set the in flight bit to 1
    }
    if (this->last_temp_update + TEMP_UPDATE_RATE < millis() &&
        !(this->in_flight_bitmask & TEMP_FLIGHT_BIT)){
        if (this->send_command(odrive::command_ids::Get_Temperature))
            this->in_flight_bitmask |= TEMP_FLIGHT_BIT;  // Set the in flight bit to 1
    }
    if (this->last_vbus_update + VBUS_UPDATE_RATE < millis() &&
        !(this->in_flight_bitmask & VBUS_FLIGHT_BIT)){
        if (this->send_command(odrive::command_ids::Get_Bus_Voltage_Current))
            this->in_flight_bitmask |= VBUS_FLIGHT_BIT;  // Set the in flight bit to 1
    }
    if (this->last_torque_update + TORQUE_UPDATE_RATE < millis() &&
        !(this->in_flight_bitmask & TORQUE_FLIGHT_BIT)){
        if (this->send_command(odrive::command_ids::Get_Torques))
            this->in_flight_bitmask |= TORQUE_FLIGHT_BIT;  // Set the in flight bit to 1
    }
    if (this->last_heartbeat + HEARTBEAT_UPDATE_RATE < millis()) {
        this->send_command(odrive::command_ids::Heartbeat);
        this->last_heartbeat = millis();
    }
}


uint8_t ODrivePro::send_command(odrive::command_ids command_id) {
    CAN_message_t msg;
    msg.id = this->can_id << 5 | command_id; // 6 bits for the ID and 5 bits for the command
    msg.flags.remote = true;    // Set the remote flag to true (remote transmission request)
    msg.flags.extended = false; // Set the extended flag to false (standard CAN)
    msg.len = 0;                // No data to send as this is a remote transmission request
    uint8_t result = this->can_bus->write(msg);
    return result; // Return the result of the write (1 for success, -1 for failure)
}

template <typename T>
uint8_t ODrivePro::send_command(odrive::command_ids command_id, T value) {
    CAN_message_t msg;
    msg.id = this->can_id << 5 | command_id; // 6 bits for the ID and 5 bits for the command
    msg.flags.extended = false;
    msg.flags.remote = false;
    msg.len = 4;
    uint32_t lower_32 = *(uint32_t*) &value;
    msg.buf[3] = (uint8_t) ((lower_32 >> 24) & 0xFF);
    msg.buf[2] = (uint8_t) ((lower_32 >> 16) & 0xFF);
    msg.buf[1] = (uint8_t) ((lower_32 >> 8) & 0xFF);
    msg.buf[0] = (uint8_t)  (lower_32 & 0xFF);
    uint8_t result = this->can_bus->write(msg);
    return result; // Return the result of the write (1 for success, -1 for failure)
}

template <typename T1 , typename T2>
uint8_t ODrivePro::send_command(odrive::command_ids command_id, T1 lower_data, T2 upper_data) {
    CAN_message_t msg;
    msg.id = this->can_id << 5 | command_id; // 6 bits for the ID and 5 bits for the command
    msg.flags.remote = false;   // Set the remote flag to false (remote transmission request)
    msg.flags.extended = false; // Set the extended flag to false (standard CAN)
    msg.len = 8;
    // Some pointer magic to convince the compiler this is a 32 bit value
    uint32_t upper_32 = *(uint32_t*) &upper_data;
    uint32_t lower_32 = *(uint32_t*) &lower_data;
    msg.buf[7] = (uint8_t) ((upper_32 >> 24) & 0xFF);
    msg.buf[6] = (uint8_t) ((upper_32 >> 16) & 0xFF);
    msg.buf[5] = (uint8_t) ((upper_32 >> 8) & 0xFF);
    msg.buf[4] = (uint8_t)  (upper_32 & 0xFF);
    msg.buf[3] = (uint8_t) ((lower_32 >> 24) & 0xFF);
    msg.buf[2] = (uint8_t) ((lower_32 >> 16) & 0xFF);
    msg.buf[1] = (uint8_t) ((lower_32 >> 8) & 0xFF);
    msg.buf[0] = (uint8_t)  (lower_32 & 0xFF);
    uint8_t result = this->can_bus->write(msg); // Send the message to the next available mailbox
    return result; // Return the result of the write (1 for success, -1 for failure)
}

void ODrivePro::clear_errors() {
    this->send_command(odrive::command_ids::Clear_Errors);
}

float_t ODrivePro::get_fet_temp() const {
    return this->FET_TEMP;
}

float_t ODrivePro::get_motor_temp() const {
    return this->MOTOR_TEMP;
}

float_t ODrivePro::get_vbus_voltage() const {
    // Check if the voltage has been updated in the last 100ms
    if (this->last_vbus_update + VBUS_UPDATE_RATE < millis()) return NAN;
    return this->VBUS_VOLTAGE;
}

float_t ODrivePro::get_vbus_current() const {
    return this->VBUS_CURRENT;
}

float_t ODrivePro::unit_conversion(float_t value, bool direction) const {
    if (this->has_rev_conversion && this->has_meter_conversion){
        if (direction){
            return (value / this->meter_per_rev) * this->ticks_per_rev;
        } else {
            return (value / this->ticks_per_rev) * this->meter_per_rev;
        }
    } else if (this->has_rev_conversion) {
        if (direction){
            return value * this->ticks_per_rev;
        } else {
            return value / this->ticks_per_rev;
        }
    } else {
        return value;
    }
}

float_t ODrivePro::get_torque_target() const {
    return this->TORQUE_TARGET;
}

float_t ODrivePro::get_torque_estimate() const {
    return this->TORQUE_ESTIMATE;
}

float_t ODrivePro::get_pos_estimate() const {
   return this->unit_conversion(this->POS_ESTIMATE, false);
}

float_t ODrivePro::get_vel_estimate() const {
    return this->unit_conversion(this->VEL_ESTIMATE, false);
}

float_t ODrivePro::get_Iq_setpoint() const {
    return this->IQ_SETPOINT;
}

float_t ODrivePro::get_Iq_measured() const {
    return this->IQ_MEASURED;
}

float_t ODrivePro::get_setpoint() const {
    if (this->control_mode == odrive::control_modes::POSITION_CONTROL) {
        return this->unit_conversion(this->position_setpoint, false);
    } else if (this->control_mode == odrive::control_modes::VELOCITY_CONTROL) {
        return this->unit_conversion(this->velocity_setpoint, false);
    } else if (this->control_mode == odrive::control_modes::TORQUE_CONTROL) {
        return this->unit_conversion(this->torque_setpoint, false);
    } else {
        return 0;
    }
}

odrive::axis_states ODrivePro::get_axis_state() const {
    return this->AXIS_STATE;
}

char* ODrivePro::get_axis_state_string() {
    sprintf(this->axis_state_string, ""); // Clear the string
    odrive::sprint_axis_state(this->axis_state_string, static_cast<odrive::axis_states>(this->AXIS_STATE));
    return this->axis_state_string;
}

uint32_t ODrivePro::get_axis_error() const {
    return this->ACTIVE_ERRORS;
}

char* ODrivePro::get_axis_error_string() {
    sprintf(this->axis_error_string, ""); // Clear the string
    odrive::sprintf_error_code(this->axis_error_string, this->AXIS_ERROR);
    return this->axis_error_string;
}

uint32_t ODrivePro::get_active_errors() const {
    return this->ACTIVE_ERRORS;
}

char* ODrivePro::get_active_errors_string() {
    sprintf(this->active_errors_string, ""); // Clear the string
    odrive::sprintf_error_code(this->active_errors_string, this->ACTIVE_ERRORS);
    return this->active_errors_string;
}

uint32_t ODrivePro::get_disarm_reason() const {
    return this->DISARM_REASON;
}

char* ODrivePro::get_disarm_reason_string() {
    sprintf(this->disarm_reason_string, ""); // Clear the string
    odrive::sprintf_error_code(this->disarm_reason_string, this->DISARM_REASON);
    return this->disarm_reason_string;
}

bool ODrivePro::is_connected() const {
    if (millis() - this->last_message > 1500) {
        return false;
    } else {
        return true;
    }
}

void ODrivePro::emergency_stop() {
    this->send_command(odrive::Estop);
}

odrive::procedure_results ODrivePro::get_procedure_results() const {
    return this->PROCEDURE_RESULT;
}

char *ODrivePro::get_procedure_results_string() {
    sprintf(this->procedure_result_string, ""); // Clear the string
    odrive::sprint_procedure_result(this->procedure_result_string, this->PROCEDURE_RESULT);
    return this->procedure_result_string;
}

bool ODrivePro::has_error() const {
    if (this->ACTIVE_ERRORS != 0 || this->AXIS_ERROR != 0) {
        return true;
    } else {
        return false;
    }
}

odrive::control_modes ODrivePro::get_control_mode() const {
    return this->control_mode;
}

char *ODrivePro::get_control_mode_string() {
    sprintf(this->control_mode_string, ""); // Clear the string
    odrive::sprint_control_mode(this->control_mode_string, this->control_mode);
    return this->control_mode_string;
}

uint32_t ODrivePro::get_last_update() const {
    return millis() - this->last_message;
}

void ODrivePro::set_control_mode(odrive::control_modes new_control_mode, odrive::input_modes new_input_mode) {
    this->control_mode = new_control_mode;
    this->input_mode = new_input_mode;
    this->send_command(odrive::Set_Controller_Mode, new_control_mode, new_input_mode);
    this->set_axis_state(odrive::axis_states::CLOSED_LOOP_CONTROL);
}

char *ODrivePro::get_setpoint_string() {
    sprintf(this->setpoint_string, ""); // Clear the string
    if (this->control_mode == odrive::control_modes::POSITION_CONTROL) {
        sprintf(this->setpoint_string, "%.3f %s", this->get_setpoint(), this->pos_unit_string);
    } else if (this->control_mode == odrive::control_modes::VELOCITY_CONTROL) {
        sprintf(this->setpoint_string, "%.3f %s", this->get_setpoint(), this->vel_unit_string);
    } else if (this->control_mode == odrive::control_modes::TORQUE_CONTROL) {
        sprintf(this->setpoint_string, "%.3f Nm", this->get_setpoint());
    } else {
        sprintf(this->setpoint_string, "N/A");
    }
    return this->setpoint_string;
}

uint32_t ODrivePro::get_inflight_bitmask() const {
    return this->in_flight_bitmask;
}

void ODrivePro::reboot() {
    this->send_command(odrive::Reboot);
}

void ODrivePro::calibrate() {
    this->calibrating = true;
}

float_t ODrivePro::calculate_feedforward(float_t setpoint){
    // Interpolate the feedforward gain based on the setpoint using a linear interpolation between the two closest points
    // This is done to avoid having to send a feedforward gain for every single setpoint
    // Iterate through the feedforward array to find the two closest points

    // If the feedforward is symmetric, use the absolute value of the setpoint
    if (this->feedforward->symmetric) {
        setpoint = std::abs(setpoint);
    }
    int i = 0;
    while (setpoint > this->feedforward->setpoints[i] && i < this->feedforward->size) {
        i++;
    }
    if (i == this->feedforward->size) {
        // If the setpoint is larger than the largest setpoint in the array, use the last two points
        i = this->feedforward->size - 2;
    } else if (i == 0) {
        // If the setpoint is smaller than the smallest setpoint in the array, use the first two points
        i = 1;
    }
    // Calculate the slope of the line between the two points
    float_t slope = (this->feedforward->ff_gains[i] - this->feedforward->ff_gains[i-1]) /
            (this->feedforward->setpoints[i] - this->feedforward->setpoints[i-1]);
    // Calculate the feedforward gain
    float_t feedforward_gain = slope * (setpoint - this->feedforward->setpoints[i-1])
            + this->feedforward->ff_gains[i-1];

    return feedforward_gain;
}

void ODrivePro::set_feedforward(feedforward_struct* feedforward_struct) {
    this->feedforward = feedforward_struct;
    this->has_feedforward = true;
}

odrive::input_modes ODrivePro::get_input_mode() const {
    return this->input_mode;
}

void ODrivePro::set_axis_state(odrive::axis_states state) {
    this->send_command(odrive::Set_Axis_State, state);
}

void ODrivePro::pass_odometer_data(void *pointer) {
    this->odometer = static_cast<ODrivePro::memory_odometer_value*>(pointer);
}

char *ODrivePro::get_input_mode_string() {
    sprintf(this->input_mode_string, ""); // Clear the string
    odrive::sprint_input_mode(this->input_mode_string, this->input_mode);
    return this->input_mode_string;
}

double_t ODrivePro::get_odometer() const {
    if (this->odometer != nullptr) {
        return this->odometer->odometer / 100.0;
    } else {
        return FP_NAN;
    }
}

void ODrivePro::update_odometer() {
    // Take the absolute value of the difference between the current position and the last position
    if (this->last_pos != this->POS_ESTIMATE && this->AXIS_STATE == odrive::axis_states::CLOSED_LOOP_CONTROL) {
        float_t difference = std::abs(this->POS_ESTIMATE - this->last_pos);
        // Convert the difference to uint32_t
        auto difference_uint = static_cast<uint32_t>(difference * 100);
        this->odometer->odometer += difference_uint;
        this->odometer->changed = true;
    }
    this->last_pos = this->POS_ESTIMATE;
}


void ODrivePro::update_power_consumption() {
    // Calculate the power consumption
    if (this->AXIS_STATE == odrive::axis_states::CLOSED_LOOP_CONTROL) {
        float_t power = this->get_vbus_voltage() * this->get_vbus_current();  // Power in Watts
        float_t power_mW = power * 1000;  // Power in mW
        // Add the power consumption to the rolling average array
        this->power_consumption_samples[this->power_consumption_index] = power_mW;
    } else this->power_consumption_samples[this->power_consumption_index] = 0;
    this->power_consumption_index++;

    if (this->power_consumption_index == VBUS_SAMPLE_SIZE) {
        // Calculate the milliwatt hours consumed using the average power consumption and
        // the time since the last update
        float_t time_since_last_update = (millis() - this->last_power_consumption) / 3600000.0f;  // Time in hours

        // Calculate the average power consumption
        float_t average_power_consumption_mW = 0;
        for (float power_consumption_sample : this->power_consumption_samples) {
            average_power_consumption_mW += power_consumption_sample;
        }
        average_power_consumption_mW /= VBUS_SAMPLE_SIZE;
        float_t power_consumption_mWh = average_power_consumption_mW * time_since_last_update;
        this->odometer->used_power += static_cast<uint32_t>(power_consumption_mWh);  // Power consumption in mWh
        this->odometer->changed = this->AXIS_STATE == odrive::axis_states::CLOSED_LOOP_CONTROL ? true : false;
        this->power_consumption_index = 0;
        this->last_power_consumption = millis();
    }
}

double_t ODrivePro::get_power_consumption() const {
    if (this->odometer != nullptr) {
        return this->odometer->used_power / 1000.0;
    } else {
        return FP_NAN;
    }
}

bool ODrivePro::tripped(char* name, char* reason) {
    // An ODrive will trip an estop if it ever exits closed loop control with an error as indicated in disarm_reason
    // The only trigger state is if we lose communication with the ODrive
    if (!this->is_connected()) {
        sprintf(name, "ODrive: %d", this->can_id);
        sprintf(reason, "Lost Connection");
        return true;
    }
    // If an odrive reports a SPINOUT error, then it should be ignored and closed loop control should be resumed
    if (this->DISARM_REASON == SPINOUT_DETECTED || this->AXIS_ERROR == SPINOUT_DETECTED) {
        this->send_command(odrive::Clear_Errors);
        this->send_command(odrive::Set_Axis_State, odrive::axis_states::CLOSED_LOOP_CONTROL);
        return false;
    }
    if (this->DISARM_REASON != 0x00 && this->AXIS_STATE != odrive::axis_states::CLOSED_LOOP_CONTROL){
        sprintf(name, "ODrive: %d", this->can_id);
        odrive::sprintf_error_code(reason, this->DISARM_REASON);
        return true;
    }
    return false;
}

void ODrivePro::estop() {
    this->send_command(odrive::Estop);
}

void ODrivePro::resume() {
    this->send_command(odrive::Clear_Errors);
}

