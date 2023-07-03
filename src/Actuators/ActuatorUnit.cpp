//
// Created by Jay on 3/16/2023.
//

#include <functional>
#include "ActuatorUnit.h"


void ActuatorUnit::build_telemetry_messages() {
    reocurring_messages[0] = *build_message(
            Actuators::serial_commands::read_encoder_count_m1,
            50, 5, &ActuatorUnit::encoder_count_callback);
    reocurring_messages[1] = *build_message(
            Actuators::serial_commands::read_encoder_count_m2,
            50, 5, &ActuatorUnit::encoder_count_callback);
    reocurring_messages[2] = *build_message(
            Actuators::serial_commands::read_encoder_speed_m1,
            60, 5, &ActuatorUnit::encoder_speed_callback);
    reocurring_messages[3] = *build_message(
            Actuators::serial_commands::read_encoder_speed_m2,
            60, 5, &ActuatorUnit::encoder_speed_callback);
    reocurring_messages[4] = *build_message(
            Actuators::serial_commands::read_main_battery_voltage,
            350, 2, &ActuatorUnit::main_battery_voltage_callback);
    reocurring_messages[5] = *build_message(
            Actuators::serial_commands::read_logic_battery_voltage,
            900, 2, &ActuatorUnit::logic_battery_voltage_callback);
    reocurring_messages[6] = *build_message(
            Actuators::serial_commands::read_motor_currents,
            75, 4, &ActuatorUnit::motor_currents_callback);
    reocurring_messages[7] = *build_message(
            Actuators::serial_commands::read_temperature,
            750, 2, &ActuatorUnit::controller_temp_callback);
    reocurring_messages[8] = *build_message(
            Actuators::serial_commands::read_status,
            1000, 2, &ActuatorUnit::controller_status_callback);

}



ActuatorUnit::telemetry_message*
ActuatorUnit::build_message(Actuators::serial_commands command, uint32_t send_interval, uint8_t data_length,
                            void (*callback)(void *, Actuators::message*)) {
    auto* telem = new telemetry_message;
    telem->msg = new Actuators::message;
    telem->msg->command = command;
    telem->msg->data_length = data_length;
    telem->msg->id = this->id;
    telem->msg->expect_response = true;
    telem->msg->protected_action = false;
    telem->msg->object = this;
    telem->msg->callback = callback;
    telem->msg->failure_callback = &ActuatorUnit::message_failure_callback;
    telem->msg->free_after_callback = false;
    telem->send_interval = send_interval;
    telem->last_send_time = 0;
    return telem;
}

void ActuatorUnit::set_duty_cycle(float_t duty_cycle, uint8_t motor) {
    // Convert duty cycle to signed 16 bit integer
    auto duty_cycle_int = (int16_t) (duty_cycle * INT16_MAX);
//    auto duty_cycle_int = INT16_MAX;
    // The controller is big endian but the teensy is little endian
    if (motor == 0) {
        this->command_messages[0].msg->command = Actuators::serial_commands::drive_m1_duty_cycle;
        this->command_messages[0].msg->data[0] = duty_cycle_int >> 8;
        this->command_messages[0].msg->data[1] = duty_cycle_int & 0xFF;
        this->command_messages[0].msg->data_length = 2;
        this->motors[0].control_mode = control_modes::velocity;
    } else {
        this->command_messages[1].msg->command = Actuators::serial_commands::drive_m2_duty_cycle;
        this->command_messages[1].msg->data[0] = duty_cycle_int >> 8;
        this->command_messages[1].msg->data[1] = duty_cycle_int & 0xFF;
        this->command_messages[1].msg->data_length = 2;
        this->motors[1].control_mode = control_modes::velocity;
    }
//    this->command_bus->queue_message(this->command_messages[motor].msg);
}

//void ActuatorUnit::set_pid_gains(float_t p, float_t i, float_t d, uint8_t motor) {
//    if (motor == 0) {
//        this->command_messages[0].msg->command = Actuators::serial_commands::set_velocity_pid_gains_m1;
//    } else {
//        this->command_messages[1].msg->command = Actuators::serial_commands::set_velocity_pid_gains_m2;
//    }
//    int32_t
//    int32_t p_int = (int32_t) (p * 1000000);
//    int32_t i_int = (int32_t) (i * 1000000);
//    int32_t d_int = (int32_t) (d * 1000000);
//    this->command_messages[motor].msg->data_length = 12;
//}

void ActuatorUnit::set_target_position(int32_t position, uint8_t motor) {
    int32_t acceleration = 200;
    int32_t deceleration = 200;
    int32_t max_speed    = 500;
    if (motor == 0) {
        this->command_messages[0].msg->command = Actuators::serial_commands::set_position_m1;
    } else {
        this->command_messages[1].msg->command = Actuators::serial_commands::set_position_m2;
    }
    this->motors[motor].control_mode = control_modes::position;
    memcpy(this->command_messages[motor].msg->data + 0,  &acceleration, 4);
    memcpy(this->command_messages[motor].msg->data + 4,  &max_speed, 4);
    memcpy(this->command_messages[motor].msg->data + 8,  &deceleration, 4);
    memcpy(this->command_messages[motor].msg->data + 12, &position, 4);
    this->command_messages[motor].msg->data[16] = 1;

}

void ActuatorUnit::queue_telemetry_messages() {
    for (int i = 0; i < 2; i++) {  // Queue command messages first so they get priority
        if (millis() - command_messages[i].last_send_time > command_messages[i].send_interval) {
            if (!command_bus->space_available()) return;
            command_bus->queue_message(this->command_messages[i].msg);
            command_messages[i].last_send_time = millis();
        }
    }
    for (int i = 0; i < 9; i++) {  // Queue telemetry messages last so they get sent if there is space
        if (millis() - reocurring_messages[i].last_send_time > reocurring_messages[i].send_interval) {
            if (!command_bus->space_available()) return;
            command_bus->queue_message(this->reocurring_messages[i].msg);
            reocurring_messages[i].last_send_time = millis();
        }
    }
}

void ActuatorUnit::emergency_stop() {
    this->set_control_mode(control_modes::stopped, 0);
    this->set_control_mode(control_modes::stopped, 1);
}


void ActuatorUnit::update() {
    if (this->connected) {
        int16_t max_speed = 0x7FFF;
        int16_t stopped = 0;
        this->queue_telemetry_messages();
        for (int i = 0; i < 2; i++) {
            auto &motor = motors[i];
            switch (motor.control_mode) {
                case control_modes::position:
                case control_modes::velocity:
                    // Queue the target duty cycle message
//                    if (millis() - motor.last_send_time > 50) {
//                        this->send_target_position(i);
//                        motor.last_send_time = millis();
//                    }
                case control_modes::stopped:
                    break;
                case control_modes::homing:
                    auto* msg = new Actuators::message;
                    // Check if the motor has stopped moving
                    if (motor.current_speed == 0 && motor.current_current == 0 && motor.current_position != 0) {
                        if (i == 0) {
                            msg->command = Actuators::serial_commands::set_encoder_count_m1;
                        } else msg->command = Actuators::serial_commands::set_encoder_count_m2;
                        msg->data_length = 4;
                        msg->data[0] = 0;
                        msg->data[1] = 0;
                        msg->data[2] = 0;
                        msg->data[3] = 0;
                        msg->free_after_callback = true;
                        msg->expect_response = false;
                        command_bus->queue_message(msg);
                        motor.homed = true;
                        this->set_control_mode(control_modes::stopped, i);
                    } else {
                        // Command the motor to move towards the home position
                        if (i == 0) {
                            msg->command = Actuators::serial_commands::drive_m1_duty_cycle;
                        } else msg->command = Actuators::serial_commands::drive_m2_duty_cycle;
                        msg->data_length = 2;
                        msg->data[0] = (max_speed >> 8);
                        msg->data[1] = max_speed & 0xFF;
                        msg->free_after_callback = true;
                        msg->expect_response = false;
                        command_bus->queue_message(msg);
                    }
                    break;
            }
        }
    } else {
        this->check_connection();
    }
}


void ActuatorUnit::set_control_mode(control_modes mode, uint8_t motor) {
    auto* msg = new Actuators::message;
    int16_t max_speed = -0x7FFF;
    int16_t stopped = 0;
    switch (mode) {
        case control_modes::position:
            if (this->motors[motor].homed) {
                motors[motor].control_mode = control_modes::position;
            }
            break;
        case control_modes::velocity:
            motors[motor].control_mode = control_modes::velocity;
            break;
        case control_modes::stopped:
            motors[motor].control_mode = control_modes::stopped;
            if (motor == 0) {  // Set the target motor to a duty cycle of 0
                msg->command = Actuators::serial_commands::drive_m1_duty_cycle;
            } else msg->command = Actuators::serial_commands::drive_m2_duty_cycle;
            msg->data_length = 2;
            msg->data[0] = (stopped >> 8) & 0xFF;
            msg->data[1] = stopped & 0xFF;
            msg->free_after_callback = true;
            msg->expect_response = false;
            break;
        case control_modes::homing:
            motors[motor].control_mode = control_modes::homing;
            if (motor == 0) { // Set the target motor to a duty cycle of 0
                msg->command = Actuators::serial_commands::drive_m1_duty_cycle;
            } else msg->command = Actuators::serial_commands::drive_m2_duty_cycle;
            msg->data_length = 2;
            msg->data[0] = (max_speed >> 8);
            msg->data[1] = max_speed & 0xFF;
            msg->free_after_callback = true;
            msg->expect_response = false;
            command_bus->queue_message(msg);
            break;
    }

}

void ActuatorUnit::send_target_position(uint8_t motor) {
    auto* msg = new Actuators::message;
    if (motor == 0) {
        msg->command = Actuators::serial_commands::set_position_m1;
    } else msg->command = Actuators::serial_commands::set_position_m2;
    msg->data_length = 17;
    int32_t accel = ACTUATOR_ACCEL;
    int32_t decel = ACTUATOR_DECEL;
    int32_t speed = ACTUATOR_SPEED;
    int32_t position = this->motors[motor].target_position;
    memcpy(msg->data, &accel, 4);
    memcpy(msg->data + 4, &decel, 4);
    memcpy(msg->data + 8, &speed, 4);
    memcpy(msg->data + 12, &position, 4);
    msg->data[16] = 1;
    msg->free_after_callback = true;
    msg->expect_response = false;
    command_bus->queue_message(msg);
}

void ActuatorUnit::check_connection() {
    // Queue a telemetry message to check if the actuator unit is connected
    if (!this->connected) {
       if (millis() - reocurring_messages[2].last_send_time > 500) {
           this->command_bus->queue_message(reocurring_messages[2].msg);
       }
    }
}

void ActuatorUnit::detailed_encoder_count_callback(void *actuator, Actuators::message *msg) {
    auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
    uint32_t raw_position;
    bool negative;
    switch(msg->command){
        case Actuators::serial_commands::read_encoder_count_m1:
            raw_position = (msg->data[3] << 24) | (msg->data[2] << 16) | (msg->data[1] << 8) | msg->data[0];
            // Trim the position value to a 32 bit signed integer
            negative = msg->data[4] & 0b00000010;
            if (negative) {
                actuator_unit->motors[0].current_position = - (int32_t) raw_position;
            } else actuator_unit->motors[0].current_position = (int32_t) raw_position;
            break;
        case Actuators::serial_commands::read_encoder_count_m2:
            raw_position = (msg->data[3] << 24) | (msg->data[2] << 16) | (msg->data[1] << 8) | msg->data[0];
            // Trim the position value to a 32 bit signed integer
            negative = msg->data[4] & 0b00000010;
            if (negative) {
                actuator_unit->motors[1].current_position = - (int32_t) raw_position;
            } else actuator_unit->motors[1].current_position = (int32_t) raw_position;
            break;
        default:
            break;
    }
    actuator_unit->message_dropped_count = 0;
    actuator_unit->connected = true;
}


int32_t ActuatorUnit::get_position(uint8_t motor) {
    if (!(this->data_flags & M1_POS_MASK) && motor == 0) return INT32_MIN;
    if (!(this->data_flags & M2_POS_MASK) && motor == 1) return INT32_MIN;
    return this->motors[motor].current_position;
}

int32_t ActuatorUnit::get_velocity(uint8_t motor) {
    if (!(this->data_flags & M1_VEL_MASK) && motor == 0) return INT32_MIN;
    if (!(this->data_flags & M2_VEL_MASK) && motor == 1) return INT32_MIN;
    return this->motors[motor].current_speed;
}

float_t ActuatorUnit::get_current(uint8_t motor) {
    if (!(this->data_flags & CURENT_MASK)) return INT32_MIN;
    return (float_t) this->motors[motor].current_current / 100;
}

float_t ActuatorUnit::get_temperature() const {
    return (float_t) this->controller_temperature / 10;
}

float_t ActuatorUnit::get_main_battery_voltage() const {
    if (!(this->data_flags & MN_BAT_MASK)) return INT32_MIN;
    return (float_t) this->main_battery_voltage / 10;
}

float_t ActuatorUnit::get_logic_battery_voltage() const {
    if (!(this->data_flags & LG_BAT_MASK)) return INT32_MIN;
    return (float_t) this->logic_battery_voltage / 10;
}

uint16_t ActuatorUnit::get_status() const {
    return this->status;
}

char* ActuatorUnit::get_motor_fault_string(uint8_t motor) {
    this->motors[motor].fault = false;
    this->motors[motor].warn = false;
    sprintf(this->motors[motor].status_string, "");
    if (!motors[motor].homed && motors[motor].control_mode != homing)
        sprintf(this->motors[motor].status_string, "%s%s_NOT_HOMED ", motors[motor].status_string, motors[motor].name);
    if (motors[motor].current_current > motors[motor].warning_current) {
        sprintf(this->status_string, "%s%s_HIGH_CURRENT ", motors[motor].status_string, motors[motor].name);
        motors[motor].warn = true;
    }

    switch (motor){
        case 0:
            if (status & controller_status_bitmask::m1_over_current) {
                sprintf(motors[motor].status_string, "%sM1_OVER_CURRENT ", this->status_string);
                motors[motor].fault = true;
            }
            if (status & controller_status_bitmask::m1_driver_fault) {
                sprintf(motors[motor].status_string, "%sM1_DRIVER_FAULT ", this->status_string);
                motors[motor].fault = true;
            }
            break;
        case 1:
            if (status & controller_status_bitmask::m2_over_current) {
                sprintf(motors[motor].status_string, "%sM2_OVER_CURRENT ", this->status_string);
                motors[motor].fault = true;
            }
            if (status & controller_status_bitmask::m2_driver_fault) {
                sprintf(motors[motor].status_string, "%sM2_DRIVER_FAULT ", this->status_string);
                motors[motor].fault = true;
            }
            break;
        default:
            break;
    }
    if (strlen(motors[motor].status_string) == 0) {
        switch (motors[motor].control_mode){
            case stopped:
                sprintf(motors[motor].status_string, "%s%s_STOPPED ", motors[motor].status_string, motors[motor].name);
                break;
            case position:
            case velocity:
                sprintf(motors[motor].status_string, "%s%s_ACTIVE", motors[motor].status_string, motors[motor].name);
                break;
            case homing:
                sprintf(motors[motor].status_string, "%s%s_HOMING", motors[motor].status_string, motors[motor].name);
                break;
        }
    }
    return motors[motor].status_string;
}

char* ActuatorUnit::get_status_string() {
    sprintf(this->status_string, "");
    if (status == 0 || status >= controller_status_bitmask::main_battery_high_warn) {
        if (!this->motors[0].fault && !this->motors[1].fault && !this->motors[0].warn && !this->motors[1].warn) {
            sprintf(this->status_string, "OK");
        }
    } else {
        sprintf(this->status_string, "");
        if (this->message_failure_count > 0)
            sprintf(this->status_string, "%sMESSAGE_FAILURE ", this->status_string);
        if (status & controller_status_bitmask::e_stop)
            sprintf(this->status_string, "%sDVR_E_STOP ", this->status_string);
        if (status & controller_status_bitmask::high_temperature_fault)
            sprintf(this->status_string, "%sHI_TEMP_FAULT ", this->status_string);
        if (status & controller_status_bitmask::main_battery_high_fault)
            sprintf(this->status_string, "%sMN_BAT_HI_FAULT ", this->status_string);
        if (status & controller_status_bitmask::logic_battery_high_fault)
            sprintf(this->status_string, "%sLG_BAT_HI_FAULT ", this->status_string);
        if (status & controller_status_bitmask::logic_battery_low_fault)
            sprintf(this->status_string, "%sLG_BAT_LW_FAULT ", this->status_string);
    }
    if (motors[0].fault) {
        sprintf(this->status_string, "%sM1_FLT ", this->status_string);
    } else if (motors[0].warn) {
        sprintf(this->status_string, "%sM1_WRN ", this->status_string);
    }
    if (motors[1].fault) {
        sprintf(this->status_string, "%sM2_FLT ", this->status_string);
    } else if (motors[1].warn) {
        sprintf(this->status_string, "%sM2_WRN ", this->status_string);
    }
    return this->status_string;
}

int32_t ActuatorUnit::get_target_position(uint8_t motor) {
//    if (!(this->data_flags & TGT_POS_MASK)) return INT32_MIN;
    return this->motors[motor].target_position;
}






