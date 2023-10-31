//
// Created by Jay on 10/31/2023.
//

#ifndef PRIMROSE_MCIU_ADAU_SENSOR_H
#define PRIMROSE_MCIU_ADAU_SENSOR_H

#include "ADAU_Bus_Interface.h"

// Use the bus interface object declared in the
extern ADAU_Bus_Interface ADAU_BUS_INTERFACE;

class ADAU_Sensor {

private:

    // Each sensor instance can have a unique data format which is defined by the user
    // This pointer points to the data structure that the user wants to use
    // There is no way for the bus interface to know what the data structure is so it is void*
    void* data_ptr;
    // The size of the data structure in bytes
    uint8_t data_size;

    boolean valid_data = false;
    uint32_t last_update_time = 0;

public:

    ADAU_Sensor(uint8_t sensor_id, void* data_ptr, uint8_t data_size){
        // Attach pointer of self to the bus interface
        ADAU_BUS_INTERFACE.attachSensor(this);
    }

    uint8_t get_data_size(){
        return this->data_size;
    }

    void* get_data_ptr(){
        return this->data_ptr;
    }

};


#endif //PRIMROSE_MCIU_ADAU_SENSOR_H
