//
// Created by Jay on 10/31/2023.
//

#ifndef PRIMROSE_MCIU_ADAU_SENSOR_H
#define PRIMROSE_MCIU_ADAU_SENSOR_H

#include "ADAU_Bus_Interface.h"

// Use the global bus interface object to prevent conflicts
extern ADAU_Bus_Interface ADAU_BUS_INTERFACE;

/**
 * @brief This class is used to define a sensor that is attached to the ADAU bus interface
 * @details Sensor data is updated automatically by the bus interface whenever a new message is received
 * @note When using this class the user must define a data structure that will be used to store the sensor data and
 *       pass a pointer to that data structure to the constructor
 * @warning This class preforms no validation on the data structure that is passed to it. It is up to the user to
 *          ensure that the data structure is valid otherwise undefined behavior will occur
 */
class ADAU_Sensor {

private:

    uint8_t sensor_id = 0;
    void* data_ptr = nullptr;
    // The size of the data structure in bytes
    uint8_t data_size = 0;

    boolean attached_properly = false;

    boolean valid_data = false;
    uint32_t last_update_time = 0;

public:

    /**
     * @brief Attaches a ADAU_Sensor object to the bus interface with the given sensor id and data pointer
     * @param sensor_id  A value between 0x00 and 0xFF that is used to identify this sensor
     * @param data_ptr   A pointer to the data structure that will be used to store the sensor data
     * @param data_size  The size of the data structure in bytes (sizeof(data_structure))
     */
    ADAU_Sensor(uint8_t sensor_id, void* data_ptr, const uint8_t data_size) {
        // Set the data pointer and size
        this->sensor_id = sensor_id;
        this->data_ptr = data_ptr;
        this->data_size = data_size;

        // Attach pointer of self to the bus interface
        ADAU_BUS_INTERFACE.attachSensor(this);
    }

    uint8_t get_data_size() const {
        return this->data_size;
    }

    void* get_data_ptr() {
        return this->data_ptr;
    }

    void set_last_update_time(uint32_t time) {
        this->last_update_time = time;
    }

    void set_valid(boolean valid) {
        this->valid_data = valid;
    }

    uint8_t get_sensor_id() const {
        return this->sensor_id;
    }

    uint32_t get_last_update_time() const {
        return micros() - this->last_update_time;
    }

    boolean is_valid() const {
        return this->valid_data;
    }

};


#endif //PRIMROSE_MCIU_ADAU_SENSOR_H
