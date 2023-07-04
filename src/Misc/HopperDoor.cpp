//
// Created by Jay on 7/3/2023.
//

#include "HopperDoor.h"

void HopperDoor::hopper_door_callback(const std_msgs::Int32 &msg) {
    switch (msg.data) {
        case CLOSE:
            digitalWrite(DIRECTION_PIN, LOW);
            analogWrite(PWM_PIN, 255);
            break;
        case STOP:
            analogWrite(PWM_PIN, 0);
            break;
        case OPEN:
            digitalWrite(DIRECTION_PIN, HIGH);
            analogWrite(PWM_PIN, 255);
            break;
    }
}
