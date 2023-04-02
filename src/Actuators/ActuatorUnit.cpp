//
// Created by Jay on 3/16/2023.
//

#include <functional>
#include "ActuatorUnit.h"


void ActuatorUnit::build_telemetry_messages() {
    reocurring_messages[0] = *build_message(
            Actuators::serial_commands::read_encoder_counts,
            100, 8, &ActuatorUnit::encoder_count_callback);
    reocurring_messages[1] = *build_message(
            Actuators::serial_commands::read_encoder_speeds,
            100, 8, &ActuatorUnit::encoder_speed_callback);
    reocurring_messages[2] = *build_message(
            Actuators::serial_commands::read_main_battery_voltage,
            250, 2, &ActuatorUnit::main_battery_voltage_callback);
    reocurring_messages[3] = *build_message(
            Actuators::serial_commands::read_logic_battery_voltage,
            250, 2, &ActuatorUnit::logic_battery_voltage_callback);
    reocurring_messages[4] = *build_message(
            Actuators::serial_commands::read_motor_currents,
            100, 4, &ActuatorUnit::motor_currents_callback);
    reocurring_messages[5] = *build_message(
            Actuators::serial_commands::read_status,
            100, 2, &ActuatorUnit::controller_status_callback);
}



ActuatorUnit::telemetry_message*
ActuatorUnit::build_message(Actuators::serial_commands command, uint32_t send_interval, uint8_t data_length,
                            void (*callback)(void *, Actuators::message*)) {
    auto* telem = new telemetry_message;
    telem->msg = new Actuators::message;
    telem->msg->command = command;
    telem->msg->data_length = data_length;
    telem->msg->id = id;
    telem->msg->expect_response = data_length > 0;
    telem->msg->sent_received = false;
    telem->msg->object = this;
    telem->msg->callback = callback;
    telem->msg->failure_callback = &ActuatorUnit::message_failure_callback;
    telem->send_interval = send_interval;
    telem->last_send_time = 0;
    return telem;
}


void ActuatorUnit::queue_telemetry_messages() {
    for (int i = 0; i < 4; i++) {
        if (millis() - reocurring_messages[i].last_send_time > reocurring_messages[i].send_interval) {
            if (!command_bus->space_available()) return;
            command_bus->queue_message(reocurring_messages[i].msg);
            reocurring_messages[i].last_send_time = millis();
        }
    }
}

void ActuatorUnit::set_target_position(int32_t position_m1, int32_t position_m2) {

}

void ActuatorUnit::emergency_stop() {
    this->set_control_mode(control_modes::stopped, 0);
    this->set_control_mode(control_modes::stopped, 1);
}


void ActuatorUnit::update() {
    if (this->connected) {
        this->queue_telemetry_messages();
        for (int i = 0; i < 2; i++) {
            auto &motor = motors[i];
            switch (motor.control_mode) {
                case control_modes::position:
                case control_modes::speed:
                case control_modes::stopped:
                    break;
                case control_modes::homing:
                    // Check if the motor has stopped moving
                    if (motor.current_speed == 0 && motor.current_current == 0) {
                        auto *msg = new Actuators::message;
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
                    }
                    motor.homed = true;
                    this->set_control_mode(motor.control_mode, i);
                    break;
            }
        }
    } else {
        this->check_connection();
    }
}


void ActuatorUnit::set_control_mode(control_modes mode, uint8_t motor) {
    auto* msg = new Actuators::message;
    int16_t max_speed = 0x7FFF;
    int16_t stopped = 0;
    switch (mode) {
        case control_modes::position:
            if (this->motors[motor].homed) {
                motors[motor].control_mode = control_modes::position;
            }
            break;
        case control_modes::speed:
            motors[motor].control_mode = control_modes::speed;
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
            msg->data[0] = (max_speed >> 8) & 0xFF;
            msg->data[1] = max_speed & 0xFF;
            msg->free_after_callback = true;
            msg->expect_response = false;
            command_bus->queue_message(msg);
            break;
    }

}

void ActuatorUnit::check_connection() {
    auto* msg = new Actuators::message;
    msg->command = Actuators::serial_commands::read_encoder_count_m1;
    msg->data_length = 5;
    msg->failure_callback = ActuatorUnit::message_failure_callback;
    msg->callback = ActuatorUnit::detailed_encoder_count_callback;
    msg->object = this;
    msg->free_after_callback = true;
    msg->expect_response = true;
    command_bus->queue_message(msg);
}


// Welcome to pointer hell

void ActuatorUnit::detailed_encoder_count_callback(void *actuator, Actuators::message *msg) {
    auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
    switch(msg->command){
        case Actuators::serial_commands::read_encoder_count_m1:
            actuator_unit->motors[0].current_position =
                    (msg->data[3] << 24) | (msg->data[2] << 16) | (msg->data[1] << 8) | msg->data[0];
            actuator_unit->motors[0].position_negative = msg->data[4] & 0b00000010;
            break;
        case Actuators::serial_commands::read_encoder_count_m2:
            actuator_unit->motors[1].current_position =
                    (msg->data[3] << 24) | (msg->data[2] << 16) | (msg->data[1] << 8) | msg->data[0];
            actuator_unit->motors[1].position_negative = msg->data[4] & 0b00000010;
            break;
        default:
            break;
    }
    actuator_unit->message_failure_count = 0;
    actuator_unit->connected = true;
}

void ActuatorUnit::encoder_count_callback(void *actuator, Actuators::message *msg) {
    // Cast the void pointer to an ActuatorUnit pointer
    auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
    // Preform all actions on the actuator unit pointer
    actuator_unit->motors[0].current_position =
            (msg->data[3] << 24) | (msg->data[2] << 16) | (msg->data[1] << 8) | msg->data[0];
    actuator_unit->motors[1].current_position =
            (msg->data[7] << 24) | (msg->data[6] << 16) | (msg->data[5] << 8) | msg->data[4];
}

void ActuatorUnit::encoder_speed_callback(void *actuator, Actuators::message *msg) {
    auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
    actuator_unit->motors[0].current_position =
            (msg->data[3] << 24) | (msg->data[2] << 16) | (msg->data[1] << 8) | msg->data[0];
    actuator_unit->motors[1].current_position =
            (msg->data[7] << 24) | (msg->data[6] << 16) | (msg->data[5] << 8) | msg->data[4];
}

void ActuatorUnit::motor_currents_callback(void *actuator, Actuators::message *msg) {
    auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
    actuator_unit->motors[0].current_current =
            (msg->data[1] << 8) | msg->data[0];
    actuator_unit->motors[1].current_current =
            (msg->data[3] << 8) | msg->data[2];
}

void ActuatorUnit::main_battery_voltage_callback(void *actuator, Actuators::message *msg) {
    auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
    actuator_unit->main_battery_voltage = (msg->data[1] << 8) | msg->data[0];
}

void ActuatorUnit::logic_battery_voltage_callback(void *actuator, Actuators::message *msg) {
    auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
    actuator_unit->logic_battery_voltage = (msg->data[1] << 8) | msg->data[0];
}

void ActuatorUnit::controller_status_callback(void *actuator, Actuators::message *msg) {
    auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
    actuator_unit->status = msg->data[1] << 8 | msg->data[0];
}

void ActuatorUnit::message_failure_callback(void *actuator, Actuators::message *msg) {
    auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
    actuator_unit->message_failure_count++;
    if (actuator_unit->message_failure_count > actuator_unit->message_failure_threshold) {
        actuator_unit->connected = false;
    }
}

// Debug serial

//String* ActuatorUnit::get_status_string() {
//    auto* status_string = new String();
//    status_string->concat("------------Actuator Unit Status Report " + String(this->id) + "-------\n\r");
//    status_string->concat("Main Battery Voltage: " + String(this->main_battery_voltage) + "\n\r");
//    status_string->concat("Logic Battery Voltage: " + String(this->logic_battery_voltage) + "\n\r");
//    status_string->concat("Status: " + String(this->status) + "\n\r");
//    status_string->concat("Motor 1 Status: " + String(this->motors[0].control_mode) + "\n\r");
//    status_string->concat("Motor 2 Status: " + String(this->motors[1].control_mode) + "\n\r");
//    status_string->concat("Motor 1 Current: " + String(this->motors[0].current_current) + "\n\r");
//    status_string->concat("Motor 2 Current: " + String(this->motors[1].current_current) + "\n\r");
//    status_string->concat("Message Failure Count: " + String(this->message_failure_count) + "\n\r");
//    return status_string;
//}


int32_t ActuatorUnit::get_position(uint8_t motor) {
    return this->motors[motor].current_position;
}

int32_t ActuatorUnit::get_velocity(uint8_t motor) {
    return this->motors[motor].current_speed;
}

float_t ActuatorUnit::get_current(uint8_t motor) {
    return (float_t) this->motors[motor].current_current / 10;
}

float_t ActuatorUnit::get_temperature() const {
    return (float_t) this->controller_temperature / 10;
}

float_t ActuatorUnit::get_main_battery_voltage() const {
    return (float_t) this->main_battery_voltage / 10;
}

float_t ActuatorUnit::get_logic_battery_voltage() const {
    return (float_t) this->logic_battery_voltage / 10;
}

uint16_t ActuatorUnit::get_status() const {
    return this->status;
}

char* ActuatorUnit::get_motor_fault_string(uint8_t motor) {
    sprintf(this->motors[motor].status_string, "");
    if (!motors[motor].homed) sprintf(this->motors[motor].status_string, "%s%s_NOT_HOMED ",
                                      motors[motor].status_string, motors[motor].name);
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
            case speed:
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
        if (status & controller_status_bitmask::e_stop)
            sprintf(this->status_string, "%s%s", this->status_string, "DRIVER_E_STOP ");
        if (status & controller_status_bitmask::high_temperature_fault)
            sprintf(this->status_string, "%s%s", this->status_string, "HIGH_TEMPERATURE_FAULT ");
        if (status & controller_status_bitmask::main_battery_high_fault)
            sprintf(this->status_string, "%s%s", this->status_string, "MAIN_BATTERY_HIGH_FAULT ");
        if (status & controller_status_bitmask::logic_battery_high_fault)
            sprintf(this->status_string, "%s%s", this->status_string, "LOGIC_BATTERY_HIGH_FAULT ");
        if (status & controller_status_bitmask::logic_battery_low_fault)
            sprintf(this->status_string, "%s%s", this->status_string, "LOGIC_BATTERY_LOW_FAULT ");
    }
    if (motors[0].fault) {
        sprintf(this->status_string, "%sM1_FAULT ", this->status_string);
    } else if (motors[0].warn) {
        sprintf(this->status_string, "%sM1_WARN ", this->status_string);
    }
    if (motors[1].fault) {
        sprintf(this->status_string, "%sM2_FAULT ", this->status_string);
    } else if (motors[1].warn) {
        sprintf(this->status_string, "%sM2_WARN ", this->status_string);
    }
    return this->status_string;
}




