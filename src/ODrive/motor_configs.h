//
// Created by Jay on 1/22/2023.
//

#ifndef TEENSYCANTRANSCEIVER_MOTOR_CONFIGS_H
#define TEENSYCANTRANSCEIVER_MOTOR_CONFIGS_H
#include <cstdint>

// Feed forward values

struct feedforward_struct{
    bool     symmetric;  // Whether or not the feedforward can be assumed to be mirrored
    uint16_t size;
    float_t* setpoints;  // Unit RPS
    float_t* ff_gains;   // Unit NM
};


#endif //TEENSYCANTRANSCEIVER_MOTOR_CONFIGS_H
