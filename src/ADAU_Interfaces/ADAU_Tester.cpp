//
// Created by Jay on 12/6/2023.
//

#include "ADAU_Tester.h"



void ADAU_Tester::test_valid_message() {

}

/**
 * @brief  Calculate the checksum for a given sensor id and data
 * @param sensor_id    The sensor id to calculate the checksum for
 * @param data         A pointer to the data to calculate the checksum for
 * @param data_length  The length of the data to calculate the checksum for
 * @return sum of all bytes in the message including the sensor_id
 */
uint8_t ADAU_Tester::calculate_checksum(uint8_t sensor_id, void* data, uint8_t data_length) {
    uint8_t checksum = sensor_id;
    checksum += data_length;
    for (int i = 0; i < data_length; i++) {
        checksum += ((uint8_t *) data)[i];
    }
    return checksum;
}

/**
 * @brief  Send data to the serial bus
 * @note   For the ADAU_tester this writes directly to the serial bus input buffer
 * @param sensor_id   The sensor id to send the data to
 * @param data        A pointer to the data to send
 * @param data_length The length of the data to send
 */
void ADAU_Tester::send_data(uint8_t sensor_id, void* data, uint8_t data_length) {
    virtual_serial_buffer[0] = 0xFF;  // Start byte
    virtual_serial_buffer[1] = sensor_id;
    virtual_serial_buffer[2] = data_length;
    virtual_serial_buffer[3] = calculate_checksum(sensor_id, data, data_length);
    for (int i = 0; i < data_length; i++) {
        virtual_serial_buffer[4 + i] = ((uint8_t *) data)[i];
    }
    memset(&virtual_serial_buffer[4 + data_length], 0, 6);  // End bytes (6 bytes)
}

