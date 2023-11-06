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
        case PI_HEARTBEAT:
            this->pi_heartbeat();
            break;
        case REMOTE_HEARTBEAT:
            this->remote_heartbeat();
            break;

    }
}

void EStopController::update_strings(){

}
