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
        float_t  velocity = 0;
        boolean  fault = true;
    } data = {};

    ADAU_Sensor* sensor;

public:

    SuspensionEncoders(uint8_t sensor_id){
        this->sensor = new ADAU_Sensor(sensor_id, &data, sizeof(data));
    }

    int32_t get_position() const override{
        return data.position;
    }

    float_t get_velocity() const override{
        return data.velocity;
    }

    uint32_t get_last_update_time() override{
        return sensor->get_last_update_time();
    }

    boolean is_valid() override{
        return sensor->is_valid();
    }

    boolean fault() const{
        return data.fault;
    }

};


#endif //PRIMROSE_MCIU_SUSPENSIONENCODERS_H
