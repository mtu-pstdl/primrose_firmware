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
            Actuators::serial_commands::read_main_battery_voltage,
            100, 2, &ActuatorUnit::main_battery_voltage_callback);
    reocurring_messages[3] = *build_message(
            Actuators::serial_commands::read_logic_battery_voltage,
            500, 2, &ActuatorUnit::logic_battery_voltage_callback);
    reocurring_messages[4] = *build_message(
            Actuators::serial_commands::read_motor_currents,
            50, 4, &ActuatorUnit::motor_currents_callback);
    reocurring_messages[5] = *build_message(
            Actuators::serial_commands::read_temperature,
            500, 2, &ActuatorUnit::controller_temp_callback);
    reocurring_messages[6] = *build_message(
            Actuators::serial_commands::read_status,
            1000, 2, &ActuatorUnit::controller_status_callback);

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
    this->update_duty_cycle_command(duty_cycle, motor, false);
}

void ActuatorUnit::update_duty_cycle_command(float_t duty_cycle, uint8_t motor,
                                             boolean send_immediately) {
    // Validate that we have not exceeded the position limits in any control mode
    if (this->motors[motor].has_limit) {
        if (this->motors[motor].limit_direction) {
            if (this->motors[motor].current_position > this->motors[motor].max_extension) {
                if (this->motors[motor].limit_action_dir && duty_cycle > 0) duty_cycle = 0;
                else if (!this->motors[motor].limit_action_dir && duty_cycle < 0) duty_cycle = 0;
            }
        } else {
            if (this->motors[motor].current_position < this->motors[motor].max_extension) {
                if (this->motors[motor].limit_action_dir && duty_cycle < 0) duty_cycle = 0;
                else if (!this->motors[motor].limit_action_dir && duty_cycle > 0) duty_cycle = 0;
            }
        }
    }
    // Cap the duty cycle at the maximum allowable value
    if (duty_cycle > 1) duty_cycle = 1;
    else if (duty_cycle < -1) duty_cycle = -1;
    duty_cycle = duty_cycle * this->motors[motor].duty_cycle_limit; // Scale the duty cycle to the maximum duty cycle
    this->motors[motor].current_duty_cycle = duty_cycle;

    auto duty_cycle_int = (int16_t) (duty_cycle * INT16_MAX); // Convert to signed 16 bit integer
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
    if (send_immediately) this->command_bus->queue_message(this->command_messages[motor].msg);
}

void ActuatorUnit::set_target_position(int32_t position, uint8_t motor) {
    if (motor == 0) {
        if (this->motors[0].control_mode != control_modes::E_STOPPED)
            this->motors[0].control_mode = control_modes::POSITION;
        // Check if the position is within the allowable range
        this->motors[0].target_position = position;
    } else {
        if (this->motors[1].control_mode != control_modes::E_STOPPED)
            this->motors[1].control_mode = control_modes::POSITION;
        // Check if the position is within the allowable range
        this->motors[1].target_position = position;
    }
}

void ActuatorUnit::position_control_callback(uint8_t motor){
    // Get the current position
    int32_t current_position = this->motors[motor].current_position;
    int32_t target_position = this->motors[motor].target_position;
    int32_t position_error = target_position - current_position;
    if (this->motors[motor].achieved_position) {
        // Check if the position is outside the activation tolerance
        if (abs(position_error) > this->motors[motor].activation_tolerance) {
            // If it is, set the achieved position flag to false
            this->motors[motor].achieved_position = false;
        }
    } else {
        if (abs(position_error) < this->motors[motor].position_tolerance) {
            // If the position error is within the tolerance, stop the motor
            this->update_duty_cycle_command(0, motor, true);
            // Clear the I term to prevent windup
            this->motors[motor].i_term = 0;
            // Set the achieved position flag to true
            this->motors[motor].achieved_position = true;
        } else {
            // Otherwise, calculate the target duty cycle using a PI controller
            float_t p_term = this->motors[motor].p_gain * position_error;
            this->motors[motor].i_term += this->motors[motor].i_gain * position_error;
            if (abs(this->motors[motor].i_term) > 0.25) {
                // Cap the I term to prevent windup
                this->motors[motor].i_term = 0.25f * this->motors[motor].i_term / abs(this->motors[motor].i_term);
            }
            if (this->motors[motor].reversed) {
                this->update_duty_cycle_command(-(p_term + this->motors[motor].i_term), motor, true);
            } else {
                this->update_duty_cycle_command(p_term + this->motors[motor].i_term, motor, true);
            }

            // Set the duty cycle to the sum of the P and I terms
        }
    }
}

void ActuatorUnit::queue_telemetry_messages() {
    for (int i = 0; i < 2; i++) {  // Queue command messages first so they get priority
        if (millis() - command_messages[i].last_send_time > command_messages[i].send_interval) {
            if (!command_bus->space_available()) return;
            // When in position control mode, the actuator sends its command message immediately after
            // receiving a position control message. This keeps command messages in sync with the current state
            if (this->motors[i].control_mode == control_modes::POSITION) continue;
            command_bus->queue_message(this->command_messages[i].msg);
            command_messages[i].last_send_time = millis();
        }
    }
    for (int i = 0; i < 7; i++) {  // Queue telemetry messages last so they get sent if there is space
        if (millis() - reocurring_messages[i].last_send_time > reocurring_messages[i].send_interval) {
            if (!command_bus->space_available()) return;
            command_bus->queue_message(this->reocurring_messages[i].msg);
            reocurring_messages[i].last_send_time = millis();
        }
    }
}

void ActuatorUnit::update() {
    if (this->connected) {
        this->queue_telemetry_messages();
    } else {
        this->check_connection();
    }
}

void ActuatorUnit::check_connection() {
    // Queue a telemetry serial_message to check if the actuator unit is connected
    if (!this->connected) {
        if (millis() - reocurring_messages[1].last_send_time > 100) {
            this->command_bus->queue_message(reocurring_messages[2].msg);
            reocurring_messages[1].last_send_time = millis();
        }
    }
}

int32_t ActuatorUnit::get_position(uint8_t motor) {
    if (!(this->data_flags & M1_POS_MASK) && motor == 0) return INT32_MIN;
    if (!(this->data_flags & M2_POS_MASK) && motor == 1) return INT32_MIN;
    return this->motors[motor].current_position;
}

boolean ActuatorUnit::on_target(uint8_t motor){
    return this->motors[motor].achieved_position;
}

int32_t ActuatorUnit::get_velocity(uint8_t motor) {
//    if (!(this->data_flags & M1_VEL_MASK) && motor == 0) return INT32_MIN;
    if (!(this->data_flags & M2_VEL_MASK) && motor == 1) return INT32_MIN;
    return this->motors[motor].current_speed;
}

float_t ActuatorUnit::get_current(uint8_t motor) {
    if (!(this->data_flags & CURENT_MASK)) return FP_NAN;
    return (float_t) this->motors[motor].current_current / 100;
}

float_t ActuatorUnit::get_temperature() const {
    if (!(this->data_flags & CTTEMP_MASK)) return FP_NAN;
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
    sprintf(this->motors[motor].status_string, "");
    // Indicate the control mode
    switch (this->motors[motor].control_mode) {
        case control_modes::DUTY_CYCLE:
            sprintf(this->motors[motor].status_string, "DUTY_CYCLE");
            break;
        case control_modes::POSITION:
            sprintf(this->motors[motor].status_string, "POSITION");
            break;
        case control_modes::E_STOPPED:
            sprintf(this->motors[motor].status_string, "E_STOPPED");
            break;
        default:
            sprintf(this->motors[motor].status_string, "UNKNOWN");
            break;
    }
    return motors[motor].status_string;
}

char* ActuatorUnit::get_status_string() {
    sprintf(this->status_string, "");
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
        sprintf(device_name, "Actuator Unit: %d", this->id);
        sprintf(device_message, "Lost communication");
        return true;
    }
//    if (this->motors[0].current_position < -2000) {
//        sprintf(device_name, "Actuator Unit: %d", this->id);
//        sprintf(device_message, "Suspension encoder disconnected");
//        this->estop(); // Make sure this unit stops even if automatic e-stop is disabled
//        return true;
//    }
//    if (this->motors[1].current_position < -2000) {
//        sprintf(device_name, "Actuator Unit: %d", this->id);
//        sprintf(device_message, "Steering encoder disconnected");
//        this->estop(); // Make sure this unit stops even if automatic e-stop is disabled
//        return true;
//    }
//    if (this->motors[0].current_position > 1000) {
//        sprintf(device_name, "Actuator Unit: %d", this->id);
//        sprintf(device_message, "Suspension encoder dead short");
//        this->estop(); // Make sure this unit stops even if automatic e-stop is disabled
//        return true;
//    }
//    if (this->motors[1].current_position > 1000) {
//        sprintf(device_name, "Actuator Unit: %d", this->id);
//        sprintf(device_message, "Steering encoder dead short");
//        this->estop(); // Make sure this unit stops even if automatic e-stop is disabled
//        return true;
//    }
    if (this->get_logic_battery_voltage() < 10) {
        sprintf(device_name, "Actuator Unit: %d", this->id);
        sprintf(device_message, "Logic battery voltage too low: %0.1fV", this->get_logic_battery_voltage());
        return true;
    }
    if (this->get_main_battery_voltage() < 46) {
        sprintf(device_name, "Actuator Unit: %d", this->id);
        sprintf(device_message, "Main battery voltage too low: %0.1fV", this->get_main_battery_voltage());
        return true;
    }
    if (this->get_temperature() > 80) {
        sprintf(device_name, "Actuator Unit: %d", this->id);
        sprintf(device_message, "Controller temperature too high: %0.2fC", this->get_temperature());
        return true;
    }
    return false;
}

float_t ActuatorUnit::get_duty_cycle(uint8_t motor) {
    return this->motors[motor].current_duty_cycle * 2;
}

void ActuatorUnit::resume() {
    this->motors[0].control_mode = DUTY_CYCLE;
    this->motors[1].control_mode = DUTY_CYCLE;
}

void ActuatorUnit::pass_odometer(ActuatorUnit::odometer_value *odometer, uint8_t motor) {
    this->motors[motor].odometer = odometer;
}

void ActuatorUnit::update_odometer(uint8_t motor) {

}

void ActuatorUnit::update_power_consumption(uint8_t motor) {
    // Update the power consumption of the motor
    // Power = Voltage * Current
    float_t power_consumption = this->get_main_battery_voltage() * this->get_current(motor);
}


void ActuatorUnit::current_limit_check(uint8_t motor) {
    // Check if the motor is exceeding the current limit and if slowly reduce duty_cycle_limit
    // If the motor is below the current limit, slowly increase duty_cycle_limit back to max_duty_cycle
    if (this->motors[motor].current_current > this->motors[motor].current_limit) {
        this->motors[motor].duty_cycle_limit -= 0.01;
        if (this->motors[motor].duty_cycle_limit < 0) this->motors[motor].duty_cycle_limit = 0;
    } else {
        this->motors[motor].duty_cycle_limit += 0.01;
        if (this->motors[motor].duty_cycle_limit > this->motors[motor].max_duty_cycle)
            this->motors[motor].duty_cycle_limit = this->motors[motor].max_duty_cycle;
    }
}

void ActuatorUnit::set_inverted(bool inverted, uint8_t motor) {
    this->motors[motor].reversed = inverted;
}

void ActuatorUnit::set_limits(uint8_t motor, int32_t limit, boolean direction, boolean action_dir) {
    this->motors[motor].max_extension = limit;
    this->motors[motor].limit_direction = direction;
    this->motors[motor].limit_action_dir = action_dir;
    this->motors[motor].has_limit = true;
}




