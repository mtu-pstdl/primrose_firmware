//
// Created by Jay on 12/6/2023.
//

#ifndef PRIMROSE_MCIU_ADAU_TESTER_H
#define PRIMROSE_MCIU_ADAU_TESTER_H

#include "ADAU_Sensor.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/String.h"

#define TIME_BETWEEN_TESTS 6000  // 6 seconds between tests
#define ADAU_SERIAL_BUS    Serial1

/**
 * @brief A class to test the ADAU bus interface without the need for an actual ADAU
 *        This class will send fake data to the serial bus and then read it back
 */
class ADAU_Tester {

private:

    struct Test_Data {
        uint32_t integer_32;
        int64_t  integer_64;
        float    float_32;
    };

    struct data {
        uint16_t position = 0;
        float_t  velocity = 0;
        boolean  fault = true;
    } suspension_data = {};

    enum sensor_test_type {
        valid_data,           // The whole message is added to the buffer with no errors
        incomplete_data,      // The message is sent in fragments but the there are no errors in the data
        invalid_data,         // The message is sent at once but the data is corrupted in some random way
        invalid_sensor_id,    // The sensor id does not match any of the sensors attached to the bus interface
        invalid_data_length,  // The data length is greater than the size of the data struct
        invalid_checksum,     // The checksum is incorrect (the data is not corrupted)
        test_complete,        // All tests have been completed
    };

    ADAU_Sensor* sensors[6] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

    std_msgs::String* output_msg;
    char output_string[100] = {0};

    uint8_t virtual_serial_buffer[255] = {0};  // Make this the serial bus so we can write directly to it without wiring

    uint32_t         last_test    = 0;
    sensor_test_type current_test = valid_data;

    // The test works by running the ADAU through a series of possible

    template<typename T>
    void send_data(uint8_t sensor_id, T* data, uint8_t data_length);

    void test_valid_message();

public:

    explicit ADAU_Tester(std_msgs::String* output_msg) {
        ADAU_SERIAL_BUS.addMemoryForRead(virtual_serial_buffer, 255);
        for (int i = 0; i < 6; i++) {
            auto* data = new Test_Data;
            sensors[i] = new ADAU_Sensor(i + 10, data, sizeof(Test_Data));
        }

        this->output_msg = output_msg;
        this->output_msg->data = output_string;

    }

    void run() {
        if (millis() - last_test < TIME_BETWEEN_TESTS) return;
        last_test = millis();
        switch (current_test) {
            case valid_data:
                // Send a valid message
                test_valid_message();
                break;
            default:
                break;
        }
    }

};


#endif //PRIMROSE_MCIU_ADAU_TESTER_H
