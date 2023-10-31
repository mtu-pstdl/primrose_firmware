//
// Created by Jay on 10/31/2023.
//

#ifndef PRIMROSE_MCIU_SUSPENSIONENCODERS_H
#define PRIMROSE_MCIU_SUSPENSIONENCODERS_H

#include "ADAU_Interfaces/ADAU_Sensor.h"
#include "PositionSensor.h"

class SuspensionEncoders : public PositionSensor {

private:

    struct data {
        uint16_t position = 0;
        boolean  fault = true;
    } data = {};

    ADAU_Sensor* sensor;

public:

    SuspensionEncoders(uint8_t sensor_id){
        this->sensor = new ADAU_Sensor(sensor_id, &data, sizeof(data));
    }

    int32_t get_position() const{
        return data.position;
    }

    uint32_t get_last_update_time(){
        return sensor->get_last_update_time();
    }

    boolean is_valid(){
        return sensor->is_valid();
    }

    boolean fault() const{
        return data.fault;
    }

};


#endif //PRIMROSE_MCIU_SUSPENSIONENCODERS_H
