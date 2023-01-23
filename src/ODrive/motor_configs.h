//
// Created by Jay on 1/22/2023.
//

#ifndef TEENSYCANTRANSCEIVER_MOTOR_CONFIGS_H
#define TEENSYCANTRANSCEIVER_MOTOR_CONFIGS_H
#include <cstdint>

struct ODRIVE_MOTOR_CONFIG {
    float_t current_lim;
    float_t velocity_lim;
    float_t acceleration_lim;
    float_t deceleration_lim;
    float_t position_gain;
    float_t velocity_gain;
    float_t velocity_integrator_gain;
};



#endif //TEENSYCANTRANSCEIVER_MOTOR_CONFIGS_H
