//
// Created by Jay on 12/6/2023.
//

#ifndef PRIMROSE_MCIU_ADAU_TESTER_H
#define PRIMROSE_MCIU_ADAU_TESTER_H

#include "ADAU_Sensor.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/String.h"

#define TIME_BETWEEN_TESTS 1000  // 1 second between tests
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
        uint32_t position = 0;
        float_t  velocity = 0;
        boolean  fault = false;
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


    uint8_t  virtual_serial_buffer[255] = {0};  // Make this the serial bus so we can write directly to it without wiring
    uint16_t virtual_serial_buffer_len  = 0;

    uint32_t         last_test    = 0;
    sensor_test_type current_test = valid_data;

    // The test works by running the ADAU through a series of possible
    static uint8_t calculate_checksum(void* data, uint8_t data_length);

    void send_data(uint8_t sensor_id, void* data, uint8_t data_length);

    void test_valid_message();

public:

    explicit ADAU_Tester(std_msgs::String* output_msg) {
        for (int i = 0; i < 6; i++) {
            auto* data = new Test_Data;
            sensors[i] = new ADAU_Sensor(i + 10, data, sizeof(Test_Data));
        }
        ADAU_SERIAL_BUS.addMemoryForRead(virtual_serial_buffer, 255);
        this->output_msg = output_msg;
        this->output_msg->data = output_string;

    }

    void run() {
        if (millis() - last_test < TIME_BETWEEN_TESTS) return;
        __disable_irq();
        virtual_serial_buffer_len = 0;
        // For now we just send data to suspension encoders 1 2 3 4
        this->send_data(0x01, &suspension_data, sizeof(suspension_data));
        this->send_data(0x02, &suspension_data, sizeof(suspension_data));
        this->send_data(0x03, &suspension_data, sizeof(suspension_data));
        this->send_data(0x04, &suspension_data, sizeof(suspension_data));

        last_test = millis();

        // Change the values of the data
        suspension_data.velocity += 0.5f;
        suspension_data.position += suspension_data.velocity;

        sprintf(this->output_string, "");
        // Check if all sensors are attached properly
        ADAU_Bus_Interface::ADAU_Sensor_List* current = ADAU_BUS_INTERFACE.sensor_list;
        while (current != nullptr) {
            if (!current->sensor->is_attached()) {

            }
            current = current->next;
        }

        // Break some rules and update the serial library's head and tail pointers to pretend like we received data
        ADAU_SERIAL_BUS.rx_buffer_head_ = virtual_serial_buffer_len;
        ADAU_SERIAL_BUS.rx_buffer_tail_ = 0;
        // Call the serial event handler
//        ADAU_SERIAL_BUS.serialEventRun();
        __enable_irq();

    }

    uint8_t calculate_parity(uint8_t sensor_id);
};


#endif //PRIMROSE_MCIU_ADAU_TESTER_H
