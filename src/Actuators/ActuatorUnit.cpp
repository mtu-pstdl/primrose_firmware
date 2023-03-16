//
// Created by Jay on 3/16/2023.
//

#include <functional>
#include "ActuatorUnit.h"

void ActuatorUnit::queue_messages() {

}

void ActuatorUnit::build_telemetry_messages() {
    reocurring_messages[0] = *build_message(
            Actuators::serial_commands::read_encoder_counts,
            100, 8, &ActuatorUnit::encoder_count_callback);
    reocurring_messages[1] = *build_message(
            Actuators::serial_commands::read_encoder_speeds,
            100, 8, &ActuatorUnit::encoder_speed_callback);
    reocurring_messages[2] = *build_message(
            Actuators::serial_commands::read_main_battery_voltage,
            100, 2, &ActuatorUnit::main_battery_voltage_callback);
    reocurring_messages[3] = *build_message(
            Actuators::serial_commands::read_logic_battery_voltage,
            100, 2, &ActuatorUnit::logic_battery_voltage_callback);
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
    telem->send_interval = send_interval;
    telem->last_send_time = 0;
    return telem;
}

// Welcome to pointer hell

void ActuatorUnit::encoder_count_callback(void *actuator, Actuators::message *msg) {
    // Cast the void pointer to an ActuatorUnit pointer
    auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
    // Preform all actions on the actuator unit pointer
    actuator_unit->encoder_pos_m1 = (msg->data[0] << 24) | (msg->data[1] << 16) | (msg->data[2] << 8) | msg->data[3];
    actuator_unit->encoder_pos_m2 = (msg->data[4] << 24) | (msg->data[5] << 16) | (msg->data[6] << 8) | msg->data[7];
}

void ActuatorUnit::encoder_speed_callback(void *actuator, Actuators::message *msg) {
    auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
    actuator_unit->encoder_speed_m1 = (msg->data[0] << 24) | (msg->data[1] << 16) | (msg->data[2] << 8) | msg->data[3];
    actuator_unit->encoder_speed_m2 = (msg->data[4] << 24) | (msg->data[5] << 16) | (msg->data[6] << 8) | msg->data[7];
}

void ActuatorUnit::main_battery_voltage_callback(void *actuator, Actuators::message *msg) {
    auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
    actuator_unit->main_battery_voltage = (msg->data[0] << 8) | msg->data[1];
}

void ActuatorUnit::logic_battery_voltage_callback(void *actuator, Actuators::message *msg) {
    auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
    actuator_unit->logic_battery_voltage = (msg->data[0] << 8) | msg->data[1];
}

