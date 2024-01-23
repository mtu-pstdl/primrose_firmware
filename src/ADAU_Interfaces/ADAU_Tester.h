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
 * @note Remove for release version
 */
class ADAU_Tester {

private:

    struct BigData {
        uint8_t  integer_8  = 0;
        uint16_t integer_16 = 0;
        uint32_t integer_32 = 0;
        int64_t  integer_64 = 0;
        float    float_32   = 0;
        double   float_64   = 0;
    } test_data = {};

    #pragma pack(push, 1) // Remove all padding from the data structure to reduce transmission size
    struct susp_data {
        float_t  position = 0;
        float_t  velocity = 0;
        uint32_t sequence = 0;
        boolean  fault = false;
    } suspension_data = {};
    #pragma pack(pop) // End of data structure

#pragma pack(push, 1)
    struct data_struct {
        union {
            struct labeled {
                float_t fl_load;
                float_t fr_load;
                float_t bl_load;
                float_t br_load;
            };
            float_t sensor[4];
        } data;
        union {
            struct {
                uint8_t fl_fault;
                uint8_t fr_fault;
                uint8_t bl_fault;
                uint8_t br_fault;
            } sensors;
            uint8_t raw[4];
        } flags;
        uint32_t seq;
    } loadcell_data = {};
#pragma pack(pop)

    enum corruption_types {
        NO_CORRUPTION,
        CORRUPT_START_BYTE,
        CORRUPT_SENSOR_ID,
        CORRUPT_DATA_LENGTH,
        MORE_DATA_THAN_LENGTH,
        LESS_DATA_THAN_LENGTH,
        CORRUPT_CHECKSUM,
        CORRUPT_DATA,
        CORRUPT_END_BYTE,
    };

    ADAU_Sensor* sensors[7] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

    std_msgs::String* output_msg;
    char output_string[1000] = {0};


    uint8_t  virtual_serial_buffer[2048] = {0};  // Make this the serial bus so we can write directly to it without wiring
    uint16_t virtual_serial_buffer_len  = 0;

    uint32_t         last_test    = 0;

    // The test works by running the ADAU through a series of possible
    static uint8_t calculate_checksum(void* data, uint8_t data_length);

    void send_data(uint8_t sensor_id, void* data, uint8_t data_length);

    void check_validation();

public:

    explicit ADAU_Tester(std_msgs::String* output_msg) {
        this->output_msg = output_msg;
        this->output_msg->data = ADAU_BUS_INTERFACE.output_string;
        sensors[0] = new ADAU_Sensor(0x10, &test_data, sizeof(BigData));
    }

    void run() {
//        if (millis() - last_test < TIME_BETWEEN_TESTS) return;
        virtual_serial_buffer_len = 0;

        this->send_data(0x01, &suspension_data, sizeof(suspension_data));
        this->send_data(0x02, &suspension_data, sizeof(suspension_data));
        this->send_data(0x03, &suspension_data, sizeof(suspension_data));
        this->send_data(0x04, &suspension_data, sizeof(suspension_data));
        this->send_data(0x05, &loadcell_data, sizeof(loadcell_data));
        this->send_data(0x06, &loadcell_data, sizeof(loadcell_data));

        const uint32_t test_data_length =
                (sizeof(suspension_data) + 10) * 4
                + (sizeof(loadcell_data) + 10) * 2;

//        this->send_data(0x10, &test_data, sizeof(BigData), LESS_DATA_THAN_LENGTH);
//        this->send_data(0x10, &test_data, sizeof(BigData), MORE_DATA_THAN_LENGTH);


//        for (int i = 0; i < 34; i++) {
//            this->send_data(0x10, &test_data, sizeof(BigData));
//        }

        last_test = millis();

        // Change the values of the data
        suspension_data.velocity += random(-100, 100) / 100.0f;
        suspension_data.position += suspension_data.velocity;

        ADAU_SERIAL_BUS.write(virtual_serial_buffer, virtual_serial_buffer_len);
//        ADAU_SERIAL_BUS.flush();

    }

    uint8_t calculate_parity(uint8_t sensor_id);
};


#endif //PRIMROSE_MCIU_ADAU_TESTER_H
