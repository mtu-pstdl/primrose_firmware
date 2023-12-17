//
// Created by Jay on 12/6/2023.
//

#ifndef PRIMROSE_MCIU_ADAU_TESTER_H
#define PRIMROSE_MCIU_ADAU_TESTER_H

#include "ADAU_Sensor.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/String.h"

#define TIME_BETWEEN_TESTS 1000  // 1 second between tests
#define ADAU_SERIAL_BUS    Serial4

/**
 * @brief A class to test the ADAU bus interface without the need for an actual ADAU
 *        This class will send fake data to the serial bus and then read it back
 */
class ADAU_Tester {

private:

    struct Test_Data {
        uint32_t integer_32 = 0;
        int64_t  integer_64 = 0;
        float    float_32   = 0;
    } test_data = {};

    struct data {
        float_t  position = 0;
        float_t  velocity = 0;
        uint32_t sequence = 0;
        boolean  fault = false;
    } suspension_data = {};

    enum corruption_types {
        NO_CORRUPTION,
        CORRUPT_START_BYTE,
        CORRUPT_SENSOR_ID,
        CORRUPT_DATA_LENGTH,
        CORRUPT_CHECKSUM,
        CORRUPT_DATA,
        CORRUPT_END_BYTE,
    };

    ADAU_Sensor* sensors[7] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

    std_msgs::String* output_msg;
    char output_string[1000] = {0};


    uint8_t  virtual_serial_buffer[1024] = {0};  // Make this the serial bus so we can write directly to it without wiring
    uint16_t virtual_serial_buffer_len  = 0;

    uint32_t         last_test    = 0;

    // The test works by running the ADAU through a series of possible
    static uint8_t calculate_checksum(void* data, uint8_t data_length);

    void send_data(uint8_t sensor_id, void* data, uint8_t data_length, corruption_types corrupt = NO_CORRUPTION);

    void check_validation();

public:

    explicit ADAU_Tester(std_msgs::String* output_msg) {
        for (int i = 0; i < 7; i++) {
            auto* data = new Test_Data;
            sensors[i] = new ADAU_Sensor(i + 10, data, sizeof(Test_Data));
        }
        this->output_msg = output_msg;
        this->output_msg->data = ADAU_BUS_INTERFACE.output_string;

    }

    void run() {
//        if (millis() - last_test < TIME_BETWEEN_TESTS) return;
        virtual_serial_buffer_len = 0;

        test_data.integer_32 = random(0, 100000);
        test_data.integer_64 = random(0, 100000);
        test_data.float_32 = random(0, 100000) / 100.0f;
//
        // Test the different types of corruption
        this->send_data(10, &test_data, sizeof(test_data),
                        NO_CORRUPTION);
//        this->send_data(11, &test_data, sizeof(test_data),
//                        CORRUPT_START_BYTE);
//        this->send_data(12, &test_data, sizeof(test_data),
//                        CORRUPT_SENSOR_ID);
        this->send_data(13, &test_data, sizeof(test_data),
                        CORRUPT_DATA_LENGTH);
//        this->send_data(14, &test_data, sizeof(test_data),
//                        CORRUPT_CHECKSUM);
//        this->send_data(15, &test_data, sizeof(test_data),
//                        CORRUPT_DATA);
//        this->send_data(16, &test_data, sizeof(test_data),
//                        CORRUPT_END_BYTE);

        // For now we just send data to suspension encoders 1 2 3 4
        this->send_data(0x01, &suspension_data, sizeof(suspension_data));
        this->send_data(0x02, &suspension_data, sizeof(suspension_data));
        this->send_data(0x03, &suspension_data, sizeof(suspension_data));
        this->send_data(0x04, &suspension_data, sizeof(suspension_data));

        last_test = millis();

        // Change the values of the data
        suspension_data.velocity += 0.5f;
        suspension_data.position += suspension_data.velocity;

        ADAU_SERIAL_BUS.write(virtual_serial_buffer, virtual_serial_buffer_len);
        ADAU_SERIAL_BUS.flush();

    }

    uint8_t calculate_parity(uint8_t sensor_id);
};


#endif //PRIMROSE_MCIU_ADAU_TESTER_H
