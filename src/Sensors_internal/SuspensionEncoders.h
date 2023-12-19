//
// Created by Jay on 10/31/2023.
//

#ifndef PRIMROSE_MCIU_SUSPENSIONENCODERS_H
#define PRIMROSE_MCIU_SUSPENSIONENCODERS_H

#include "ADAU_Interfaces/ADAU_Sensor.h"
#include "PositionSensor.h"

class SuspensionEncoders : public PositionSensor {

private:

    /**
     * @brief The data structure that a analog linear position sensor sends
     * @note  This is a packed data structure
     */
    #pragma pack(push, 1) // Remove all padding from the data structure to reduce transmission size
    struct data {
        float_t  position = 0;  // The position of the suspension encoder in units of (unit)
        float_t  velocity = 0;  // The velocity of the suspension encoder in units of (unit)/s
        uint32_t sequence = 0;  // The sequence number of the message (always incrementing)
        uint8_t  fault = false; // Indicates if something is wrong with the sensor
    } data = {};
    #pragma pack(pop) // End of data structure

    ADAU_Sensor* sensor;

public:

    /**
     * @brief Construct a new Suspension Encoders object and attach it to the bus interface automatically
     * @param sensor_id The id of the sensor to attach
     */
    explicit SuspensionEncoders(uint8_t sensor_id){
        this->sensor = new ADAU_Sensor(sensor_id, &data, sizeof(data));
    }

    /**
     * @brief Get the position of the suspension encoder in units of (unit)
     * @note  The internal floating point representation is multiplied by 1000 and cast to an int32_t
     * @return The position of the suspension encoder in units of (unit)
     */
    int32_t get_position() const override{
        return (int32_t) (data.position * 1000);
    }

    /**
     * @brief Get the velocity of the suspension encoder in units of (unit)/s
     * @return The velocity of the suspension encoder in units of (unit)/s
     */
    float_t get_velocity() const override{
        return data.velocity;
    }

    /**
     * @brief Get the sequence number of the message (always incrementing)
     * @return The time in microseconds since the last update
     */
    uint32_t get_last_update_time() override{
        return sensor->get_last_update_time();
    }

    boolean is_valid() override{
        // Check if it's been more than 100ms since the last update
        if (sensor->get_last_update_time() > 100000) return false;
        return sensor->is_valid();
    }

    uint8_t fault() override {
//        return true;
        return data.fault;
    }

};


#endif //PRIMROSE_MCIU_SUSPENSIONENCODERS_H
