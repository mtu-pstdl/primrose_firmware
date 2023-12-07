//
// Created by Jay on 7/3/2023.
//

#include "EStopController.h"

void EStopController::check_for_faults() {
    // Copy the value from the estop message buffer to the last estop message buffer
    sprintf(this->estop_message, "");
    sprintf(this->tripped_device_name, "NULL");
    sprintf(this->tripped_device_message, "");
    char tripped_message[132];

    this->should_trigger_estop = false;
    this->number_of_tripped_devices = 0;
    boolean buffer_exceeded = false;

    EStopDeviceList* current = estop_devices;
    EStopDevice* estop_device;
    boolean suppressed;

    for (int i = 0; current != nullptr; i++) {
        current = get_estop_device(i);
        if (current != nullptr) {
            estop_device = current->estop_device;
            suppressed = current->suppressed;
            // still check for faults as this is used by modules to detect internal faults
            sprintf(this->tripped_device_name, "NULL");
            sprintf(this->tripped_device_message, "");
            bool trip = estop_device->tripped(tripped_device_name, tripped_device_message);
            if (!trip) continue;
            if (!(suppressed || (!automatic_estop_enabled || automatic_estop_inhibited))) {
                this->should_trigger_estop = true;
                this->number_of_tripped_devices++;
                snprintf(tripped_message, 132, "[%s] %s\n", tripped_device_name, tripped_device_message);
            } else sprintf(tripped_message, "*[%s] %s\n", tripped_device_name, tripped_device_message);
            if (buffer_exceeded) continue;
            if (strlen(this->estop_message) + strlen(tripped_message) > STATUS_MESSAGE_LENGTH - 42) {
                buffer_exceeded = true;
                snprintf(tripped_message, 132, "[Further Faults Omitted, Buffer Exceeded]\n");
            }
            strlcat(this->estop_message, tripped_message, STATUS_MESSAGE_LENGTH);
        }
    }
    // Remove the trailing newline
    this->estop_message[strlen(this->estop_message) - 1] = '\0';
    if (this->should_trigger_estop) this->trigger_estop(true, false);
}

void EStopController::estop_callback(const std_msgs::Int32 &msg) {
    switch (msg.data) {
        case ESTOP:
            this->trigger_estop(false, true);
            break;
        case RESUME:
            this->resume();
            break;
        case ENABLE_AUTOMATIC:
            this->automatic_estop_enabled = true;
            break;
        case DISABLE_AUTOMATIC:
            this->automatic_estop_enabled = false;
            break;
        case SUPPRESS_CURRENT:
            this->suppress_current();
            break;
        case PI_HEARTBEAT:
            this->pi_heartbeat();
            break;
        case REMOTE_HEARTBEAT:
            this->remote_heartbeat();
            break;

    }
}

bool EStopController::is_high_voltage_enabled() {
    return !digitalReadFast(MAIN_CONTACTOR_PIN);
}

void EStopController::trigger_estop(boolean automatic, boolean remote) {
    digitalWrite(MAIN_CONTACTOR_PIN, HIGH);
    EStopDeviceList* current = estop_devices;
    while (current != nullptr) {
        if (current->estop_device != nullptr) {
            current->estop_device->estop();
        }
        if (current->next == nullptr) break;
        current = current->next;
    }
    if (!automatic && remote) {
        sprintf(this->estop_message, "E-Stop Triggered Remotely\n");
    }
    this->estop_triggered = true;
    this->estop_triggered_time = millis();
}

void EStopController::resume() {
    digitalWrite(MAIN_CONTACTOR_PIN, LOW);
    EStopDeviceList* current = get_estop_device(0);
    while (current != nullptr) {
        if (current->estop_device != nullptr) {
            current->estop_device->resume();
        }
        if (current->next == nullptr) break;
        current = current->next;
    }
    estop_triggered = false;
    estop_triggered_time = 0;
    estop_resume_time = millis();
    this->automatic_estop_inhibited = true;
    sprintf(this->tripped_device_name, "NULL");
    sprintf(this->tripped_device_message, "NULL");
}

void EStopController::update() {
    // Check how many devices are in the linked list
    if (!estop_triggered) {
        if (millis() - estop_resume_time > 3000) this->automatic_estop_inhibited = false;
        this->check_for_faults();
    } else {
        if (millis() - estop_triggered_time > ESTOP_CONTACTOR_DELAY)
        digitalWriteFast(MAIN_CONTACTOR_PIN, HIGH);  // Open the main contactor
    }
}

