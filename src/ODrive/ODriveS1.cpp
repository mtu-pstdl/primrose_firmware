//
// Created by Jay on 12/10/2022.
//

#include "ODriveS1.h"

#include <utility>
#include "odrive_constants.h"

ODriveS1::ODriveS1(uint8_t can_id, String name, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_64> *can_bus){
    this->can_id = can_id;
    this->name = std::move(name);
    this->can_bus = can_bus;
}

uint8_t ODriveS1::get_can_id() const {
    return this->can_id;
}

void ODriveS1::on_message(const CAN_message_t &msg) {
    uint8_t msg_type = msg.id & 0x1F; // Use bitmask of 0b00000011111 to get the last 5 bits
    // Bytes are sent little endian
    uint32_t upper_32 = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
    uint32_t lower_32 = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
    switch (static_cast<ODriveS1::command_ids>(msg_type)){
        case Heartbeat: // Lower 4 bytes are AXIS_ERROR and the upper 4 bytes are AXIS_STATE
            this->AXIS_ERROR = (float) lower_32;
            this->AXIS_STATE = upper_32;
            this->refresh_flags |= AXIS_REFRESH_BIT;
            break;
        case Get_Error:
            this->ACTIVE_ERRORS = lower_32;
            this->DISARM_REASON = upper_32;
            this->refresh_flags |= ERROR_REFRESH_BIT;
            break;
        case Get_Encoder_Estimates:
            this->POS_ESTIMATE = (float_t) lower_32;
            this->VEL_ESTIMATE = (float_t) upper_32;
            this->refresh_flags |= ENCODER_REFRESH_BIT;
            break;
        case Get_Iq:
            this->Iq_Setpoint = (float_t) lower_32;
            this->Iq_Measured = (float_t) upper_32;
            this->refresh_flags |= IQ_REFRESH_BIT;
            break;
        case Get_Temperature:
            this->FET_TEMP =    (float_t) lower_32;
            this->MOTOR_TEMP =  (float_t) upper_32;
            this->refresh_flags |= TEMP_REFRESH_BIT;
            break;
        case Get_Vbus_Voltage_Current:
            this->VBUS_VOLTAGE = (float_t) lower_32;
            this->VBUS_CURRENT = (float_t) upper_32;
            this->refresh_flags |= VBUS_REFRESH_BIT;
            break;
        default:
            break;
    }
}

void ODriveS1::refresh_data() {

    // Check if the last refresh attempt was successful
    if (this->refresh_flags != ODRIVE_REFRESH_FLAG_MASK){
        // If not check if that attempt has been going on for more than 1 second
        if (millis() - this->last_refresh_attempt > 1000){
            // If so, reset the refresh flags and try again
            this->refresh_flags = 0;
            this->last_refresh_attempt = millis();
        }
    } else {
        // If the last refresh attempt was successful, reset the refresh flags and try again
        this->refresh_flags = 0;
        this->last_refresh = millis();
    }

    if (this->last_refresh + 50 < millis()) {
        this->last_refresh_attempt = millis();
        this->refresh_flags = 0;
        this->send_command(ODriveS1::command_ids::Get_Error);
        this->send_command(ODriveS1::command_ids::Get_Encoder_Estimates);
        this->send_command(ODriveS1::command_ids::Get_Iq);
        this->send_command(ODriveS1::command_ids::Get_Temperature);
        this->send_command(ODriveS1::command_ids::Get_Vbus_Voltage_Current);
    }
}



uint8_t ODriveS1::send_command(ODriveS1::command_ids command_id) {
    CAN_message_t msg;
    msg.id = this->can_id << 5 | command_id; // 6 bits for the ID and 5 bits for the command
    msg.flags.remote = true; // Set the remote flag to true (remote transmission request)
    msg.len = 0;
    uint8_t result = this->can_bus->write(msg);
    return result; // Return the result of the write (1 for success, -1 for failure)
}

uint8_t ODriveS1::send_command(ODriveS1::command_ids command_id, uint32_t lower_data, uint32_t upper_data) {
    CAN_message_t msg;
    msg.id = this->can_id << 5 | command_id; // 6 bits for the ID and 5 bits for the command
    msg.flags.remote = false; // Set the remote flag to false (data transmission)
    msg.len = 8;
    msg.buf[0] = (uint8_t) (lower_data >> 24);
    msg.buf[1] = (uint8_t) (lower_data >> 16);
    msg.buf[2] = (uint8_t) (lower_data >> 8);
    msg.buf[3] = (uint8_t) lower_data;
    msg.buf[4] = (uint8_t) (upper_data >> 24);
    msg.buf[5] = (uint8_t) (upper_data >> 16);
    msg.buf[6] = (uint8_t) (upper_data >> 8);
    msg.buf[7] = (uint8_t) upper_data;
    uint8_t result = this->can_bus->write(msg);
    return result; // Return the result of the write (1 for success, -1 for failure)
}

String* ODriveS1::get_state_string() {
    // Check if there are any errors
    auto* state_string = new String();
    state_string->concat("CAN ID: " + String(this->can_id) + " | " + this->name + "\n");
    if (this->ACTIVE_ERRORS != 0){
        String* error_string = get_error_string(this->ACTIVE_ERRORS);
        state_string->concat("ACTIVE ERRORS: " + *error_string + "\n");
        free(error_string);
    } else {
        state_string->concat("ACTIVE ERRORS: None\n");
    }
    if (this->DISARM_REASON != 0){
        String* error_string = get_error_string(this->DISARM_REASON);
        state_string->concat("DISARM REASON: " + *error_string + "\n");
        free(error_string);
    } else {
        state_string->concat("DISARM REASON: None\n");
    }
    String* axis_state = get_axis_state_string(static_cast<axis_states>(this->AXIS_STATE));
    state_string->concat("AXIS_STATE: " + *axis_state + "\n");
    free(axis_state);

    state_string->concat("VBUS STATE: " + String(this->VBUS_VOLTAGE) + "V | "
    + String(this->VBUS_CURRENT) + "A\n");

    state_string->concat("TEMPERATURES: FET: " + String(this->FET_TEMP) + "C | MOTOR: " +
    String(this->MOTOR_TEMP) + "C\n");

    state_string->concat("ENCODER ESTIMATES: POS: " + String(this->POS_ESTIMATE) + " | VEL: " +
    String(this->VEL_ESTIMATE));

    state_string->concat("LAST UPDATE: " + String(millis() - this->last_refresh) + "ms ago\n");
    return state_string;
}


