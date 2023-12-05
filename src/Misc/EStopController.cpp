//
// Created by Jay on 7/3/2023.
//

#include "EStopController.h"

void EStopController::check_for_faults() {
    sprintf(this->estop_message, "");
    char tripped_message[50];
    if (!automatic_estop_enabled || automatic_estop_inhibited) {
        EStopDevice* estop_device = get_estop_device(0);
        for (int i = 0; estop_device != nullptr; i++) {
            estop_device = get_estop_device(i);
            if (estop_device != nullptr) {
                // still check for faults as this is used by modules to detect internal faults
                estop_device->tripped(tripped_device_name, tripped_device_message);
                sprintf(tripped_message, "*[%s] %s\n", tripped_device_name, tripped_device_message);
                if (strlen(this->estop_message) + strlen(tripped_message) > STATUS_MESSAGE_LENGTH - 42) {
                    sprintf(tripped_message, "[Further Faults Omitted, Buffer Exceeded]\n");
                    strcat(this->estop_message, tripped_message);
                    break;
                }
                strcat(this->estop_message, tripped_message);
                // Reset the tripped device name and message
                sprintf(this->tripped_device_name, "NULL");
                sprintf(this->tripped_device_message, "NULL");
            }
        }
        return;
    }
    this->should_trigger_estop = false;
    this->number_of_tripped_devices = 0;
    EStopDevice* estop_device = get_estop_device(0);
    for (int i = 0; estop_device != nullptr; i++) {
        estop_device = get_estop_device(i);
        if (estop_device != nullptr) {
            if (estop_device->tripped(tripped_device_name, tripped_device_message)) {
                this->should_trigger_estop = true;
                this->number_of_tripped_devices++;
                sprintf(tripped_message, "[%s] %s\n", tripped_device_name, tripped_device_message);
                if (strlen(this->estop_message) + strlen(tripped_message) > STATUS_MESSAGE_LENGTH - 42) {
                    sprintf(tripped_message, "[Further Faults Omitted, Buffer Exceeded]\n");
                    strcat(this->estop_message, tripped_message);
                    this->trigger_estop(true, false);
                    break;
                }
                strcat(this->estop_message, tripped_message);
                // Reset the tripped device name and message
                sprintf(this->tripped_device_name, "NULL");
                sprintf(this->tripped_device_message, "NULL");
            }
        }
    }
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
    EStopDevice* estop_device = get_estop_device(0);
    for (int i = 0; estop_device != nullptr; i++) {
        estop_device = get_estop_device(i);
        if (estop_device != nullptr) {
            estop_device->estop();
        }
    }
    if (!automatic && remote) {
        sprintf(this->estop_message, "E-Stop Triggered Remotely\n");
    }
    this->estop_triggered = true;
    this->estop_triggered_time = millis();
}

void EStopController::resume() {
    digitalWrite(MAIN_CONTACTOR_PIN, LOW);
    EStopDevice* estop_device = get_estop_device(0);
    for (int i = 0; estop_device != nullptr; i++) {
        estop_device = get_estop_device(i);
        if (estop_device != nullptr) {
            estop_device->resume();
        }
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

