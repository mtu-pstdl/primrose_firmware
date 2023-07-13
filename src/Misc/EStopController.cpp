//
// Created by Jay on 7/3/2023.
//

#include "EStopController.h"

void EStopController::estop_callback(const std_msgs::Int32 &msg) {
    switch (msg.data) {
        case ESTOP:
            this->trigger_estop();
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
    }
}

void EStopController::update_strings(){
    if (this->estop_triggered) {
        this->diagnostic_topic->level = 2;
        sprintf(this->message_string, "E-Stop Triggered");
        sprintf(this->diagnostic_topic->values[2].value, "%s", this->tripped_device_name);
        sprintf(this->diagnostic_topic->values[3].value, "%s", this->tripped_device_message);
    } else if (!this->automatic_estop_enabled) {
        this->diagnostic_topic->level = 1;
        sprintf(this->message_string, "Automatic E-Stop Disabled");
        sprintf(this->diagnostic_topic->values[2].value, "Not applicable");
        sprintf(this->diagnostic_topic->values[3].value, "Not applicable");
    } else {
        sprintf(this->diagnostic_topic->values[0].value, "False");
        this->diagnostic_topic->level = 0;
        sprintf(this->message_string, "OK");
        sprintf(this->diagnostic_topic->values[2].value, "Not applicable");
        sprintf(this->diagnostic_topic->values[3].value, "Not applicable");
    }
    sprintf(this->diagnostic_topic->values[0].value, "%s", this->estop_triggered ? "True" : "False");
    sprintf(this->diagnostic_topic->values[1].value, "%s", this->automatic_estop_enabled ? "True" : "False");
    sprintf(this->diagnostic_topic->values[4].value, "%d", this->number_of_tripped_devices);
}
