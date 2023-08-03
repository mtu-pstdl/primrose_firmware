//
// Created by Jay on 7/3/2023.
//

#include "HopperDoor.h"

void HopperDoor::hopper_door_callback(const std_msgs::Int32 &msg) {
    switch (msg.data) {
        case CLOSE:
            digitalWrite(IN1_PIN, HIGH);
            digitalWrite(IN2_PIN, LOW);
//            analogWrite(PWM_PIN, 255);
            break;
        case STOP:
            digitalWrite(IN1_PIN, LOW);
            digitalWrite(IN2_PIN, LOW);
//            analogWrite(PWM_PIN, 0);
            break;
        case OPEN:
            digitalWrite(IN1_PIN, LOW);
            digitalWrite(IN2_PIN, HIGH);
//            analogWrite(PWM_PIN, 255);
            break;
    }
}
