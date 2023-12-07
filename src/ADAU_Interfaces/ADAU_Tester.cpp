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
uint8_t ADAU_Tester::calculate_checksum(void* data, uint8_t data_length) {
    uint8_t checksum = 0;
    checksum += data_length;
    for (int i = 0; i < data_length; i++) {
        checksum += ((uint8_t *) data)[i];
    }
    return checksum;
}

/**
 * @brief Calculate the parity bit for a given sensor id and set it (even parity, LSB is parity bit)
 * @param sensor_id The sensor id to calculate the parity for (7 bits)
 * @return The sensor id with the parity bit set (8 bits)
 */
uint8_t ADAU_Tester::calculate_parity(uint8_t sensor_id) {
    sensor_id <<= 1;    // Shift the sensor id left by 1 bit so that the parity bit can be set
    uint8_t parity = 0; // Used to calculate if there are an even number of 1 bits
    for (int i = 0; i < 7; i++) {
        parity += (sensor_id >> i) & 0x01;  // Add the value of the current bit to the parity
    }
    if (parity % 2 == 0) {
        // There are an even number of 1 bits
        sensor_id |= 0x01;  // Set the parity bit to 1
    } else {
        // There are an odd number of 1 bits
        sensor_id &= 0xFE;  // Set the parity bit to 0
    }
    return sensor_id;
}

/**
 * @brief  Send data to the serial bus
 * @note   For the ADAU_tester this writes directly to the serial bus input buffer
 * @param sensor_id   The sensor id to send the data to
 * @param data        A pointer to the data to send
 * @param data_length The length of the data to send
 */
void ADAU_Tester::send_data(uint8_t sensor_id, void* data, uint8_t data_length) {
    uint8_t temp[100] = {0};
    temp[0] = 0xFF;  // Start byte
    temp[1] = calculate_parity(sensor_id);  // Sensor id (1 byte)
    temp[2] = data_length;  // Data length (1 byte)
    temp[3] = calculate_checksum(data, data_length); // Checksum (1 byte)
    for (int i = 0; i < data_length; i++) {
        temp[4 + i] = ((uint8_t *) data)[i];
    }
    memset(&temp[4 + data_length], 0, 6);  // End bytes (6 bytes)

    // Copy the data to the virtual serial buffer
    // This doesn't work
    memcpy(virtual_serial_buffer + virtual_serial_buffer_len, temp, 11 + data_length);
    virtual_serial_buffer_len += 11 + data_length;
}

