//
// Created by Jay on 10/31/2023.
//

#ifndef PRIMROSE_MCIU_ADAU_SENSOR_H
#define PRIMROSE_MCIU_ADAU_SENSOR_H

#include "ADAU_Bus_Interface.h"

// Use the global bus interface object to prevent conflicts
extern ADAU_Bus_Interface ADAU_BUS_INTERFACE;

class ADAU_Sensor {

private:

    // Each sensor instance can have a unique data format which is defined by the user
    // This pointer points to the data structure that the user wants to use
    // There is no way for the bus interface to know what the data structure is so it is void*
    uint8_t sensor_id = 0;
    void* data_ptr = nullptr;
    // The size of the data structure in bytes
    uint8_t data_size = 0;

    boolean valid_data = false;
    uint32_t last_update_time = 0;

public:

    ADAU_Sensor(uint8_t sensor_id, void* data_ptr, uint8_t data_size){
        // Attach pointer of self to the bus interface
        ADAU_BUS_INTERFACE.attachSensor(this);
        // Set the data pointer and size
        this->sensor_id = sensor_id;
        this->data_ptr = data_ptr;
        this->data_size = data_size;
    }

    uint8_t get_data_size() const{
        return this->data_size;
    }

    void* get_data_ptr(){
        return this->data_ptr;
    }

    void set_last_update_time(uint32_t time){
        this->last_update_time = time;
    }

    void set_valid(boolean valid){
        this->valid_data = valid;
    }

    uint8_t get_sensor_id() const{
        return this->sensor_id;
    }

    uint32_t get_last_update_time() const{
        return this->last_update_time;
    }

    boolean is_valid() const{
        return this->valid_data;
    }

};


#endif //PRIMROSE_MCIU_ADAU_SENSOR_H
