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
            1000, 1, &ActuatorUnit::controller_status_callback);

}



ActuatorUnit::telemetry_message*
ActuatorUnit::build_message(Actuators::serial_commands command, uint32_t send_interval, uint8_t data_length,
                            void (*callback)(void *, Actuators::serial_message*)) {
    auto* telem = new telemetry_message;
    telem->msg = new Actuators::serial_message;
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
    if (this->motors[motor].control_mode == control_modes::E_STOPPED) {
        duty_cycle = 0; // If the motor is e-stopped, all commands are ignored
    } else this->motors[motor].control_mode = control_modes::DUTY_CYCLE;
    this->update_duty_cycle_command(duty_cycle, motor);
}

void ActuatorUnit::update_duty_cycle_command(float_t duty_cycle, uint8_t motor, boolean send_immediately = false) {
    auto duty_cycle_int = (int16_t) (duty_cycle * INT16_MAX);
    if (motor == 0) {
        this->command_messages[0].msg->command = Actuators::serial_commands::drive_m1_duty_cycle;
        this->command_messages[0].msg->data[0] = duty_cycle_int >> 8;
        this->command_messages[0].msg->data[1] = duty_cycle_int & 0xFF;
        this->command_messages[0].msg->data_length = 2;
    } else {
        this->command_messages[1].msg->command = Actuators::serial_commands::drive_m2_duty_cycle;
        this->command_messages[1].msg->data[0] = duty_cycle_int >> 8;
        this->command_messages[1].msg->data[1] = duty_cycle_int & 0xFF;
        this->command_messages[1].msg->data_length = 2;
    }
    if (send_immediately) {
        this->command_bus->queue_message(this->command_messages[motor].msg);
    }
}

void ActuatorUnit::set_target_position(int32_t position, uint8_t motor) {
    if (motor == 0) {
        if (this->motors[0].control_mode != control_modes::E_STOPPED)
            this->motors[0].control_mode = control_modes::POSITION;
        this->motors[0].target_position = position;
        // Check if the position is within the allowable range
        if (position > this->motors[0].max_position) position = this->motors[0].max_position;
        else if (position < this->motors[0].min_position) position = this->motors[0].min_position;
        this->motors[0].target_position = position;
    } else {
        if (this->motors[1].control_mode != control_modes::E_STOPPED)
            this->motors[1].control_mode = control_modes::POSITION;
        // Check if the position is within the allowable range
        if (position > this->motors[1].max_position) position = this->motors[1].max_position;
        else if (position < this->motors[1].min_position) position = this->motors[1].min_position;
        this->motors[1].target_position = position;
    }
}

void ActuatorUnit::position_control_callback(uint8_t motor){

}

void ActuatorUnit::queue_telemetry_messages() {
    for (int i = 0; i < 2; i++) {  // Queue command messages first so they get priority
        if (millis() - command_messages[i].last_send_time > command_messages[i].send_interval) {
            if (!command_bus->space_available()) return;
            if (this->motors[i].control_mode == control_modes::POSITION) continue;
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
    this->set_control_mode(control_modes::E_STOPPED, 0);
    this->set_control_mode(control_modes::E_STOPPED, 1);
    this->set_duty_cycle(0, 0);
    this->set_duty_cycle(0, 1);
}


void ActuatorUnit::update() {
    if (this->connected) {
    } else {
        this->check_connection();
    }
}


void ActuatorUnit::set_control_mode(control_modes mode, uint8_t motor) {

}

void ActuatorUnit::check_connection() {
    // Queue a telemetry serial_message to check if the actuator unit is connected
    if (!this->connected) {
        if (millis() - reocurring_messages[2].last_send_time > 100) {
            this->command_bus->queue_message(reocurring_messages[2].msg);
            reocurring_messages[2].last_send_time = millis();
        }
    }
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
    if (!(this->data_flags & CURENT_MASK)) return FP_NAN;
    return (float_t) this->motors[motor].current_current / 100;
}

float_t ActuatorUnit::get_temperature() const {
    return (float_t) this->controller_temperature / 10;
}

float_t ActuatorUnit::get_main_battery_voltage() const {
    if (!(this->data_flags & MN_BAT_MASK)) return FP_NAN;
    return (float_t) this->main_battery_voltage / 10;
}

float_t ActuatorUnit::get_logic_battery_voltage() const {
    if (!(this->data_flags & LG_BAT_MASK)) return FP_NAN;
    return (float_t) this->logic_battery_voltage / 10;
}

uint16_t ActuatorUnit::get_status() const {
    return this->status;
}

char* ActuatorUnit::get_motor_fault_string(uint8_t motor) {
    this->motors[motor].fault = false;
    this->motors[motor].warn = false;
    sprintf(this->motors[motor].status_string, "");
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
            case E_STOPPED:
                sprintf(motors[motor].status_string, "%s%sESTOPPED ", motors[motor].status_string, motors[motor].name);
                break;
            case POSITION:
                sprintf(motors[motor].status_string, "%s%s_POSITION ", motors[motor].status_string, motors[motor].name);
                break;
            case DUTY_CYCLE:
                sprintf(motors[motor].status_string, "%s%s_MANUAL", motors[motor].status_string, motors[motor].name);
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

void ActuatorUnit::estop() {
    this->motors[0].control_mode = E_STOPPED;
    this->motors[1].control_mode = E_STOPPED;
    this->set_duty_cycle(0, 0);
    this->set_duty_cycle(0, 1);
}

bool ActuatorUnit::tripped(char* device_name, char* device_message) {
    // The conditions that would trip an actuator unit are:
    // 1. Lost communication with the controller
    // 2. Logic battery voltage too low
    // 3. Main battery voltage too low
    // 4. Controller temperature has exceeded 80C
    if (!this->connected) {
        return false;
        sprintf(device_name, "Actuator Unit: %d", this->id);
        sprintf(device_message, "Lost communication");
        return true;
    }
    if (this->logic_battery_voltage < 10) {
        sprintf(device_name, "Actuator Unit: %d", this->id);
        sprintf(device_message, "Logic battery voltage too low");
        return true;
    }
    if (this->main_battery_voltage < 46) {
        sprintf(device_name, "Actuator Unit: %d", this->id);
        sprintf(device_message, "Main battery voltage too low");
        return true;
    }
    if (this->controller_temperature > 800) {
        sprintf(device_name, "Actuator Unit: %d", this->id);
        sprintf(device_message, "Controller temperature too high");
        return true;
    }
    return false;
}




