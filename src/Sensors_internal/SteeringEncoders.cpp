//
// Created by Jay on 10/24/2023.
//

#include "SteeringEncoders.h"

uint8_t SteeringEncoders::spiWriteRead(uint8_t byte) {
    uint8_t response;
    response = SPI.transfer(byte);
    delayMicroseconds(3);
    return response;
}

void SteeringEncoders::readPosition(){
    // If we read position within the last 10ms return the last position
    if (millis() - last_update_time < 10){
        return;
    }
    this->raw_position = 0;
    begin_transaction();
    bool binaryArray[16] = {false,}; //after receiving the position we will populate this array and use it for calculating the checksum

    //read the first byte of the position
    this->raw_position = spiWriteRead(AMT22_NOP) << 8;

    //this is the time required between bytes as specified in the datasheet.
//    delayMicroseconds(3);

    this->raw_position |= spiWriteRead(AMT22_NOP);

    // Validate the checkbits
    // The checkbits are odd parity over the odd and even bits

    // Populate the binary array

    uint16_t temp_position = this->raw_position;
    for (int i = 0; i < 16; i++) {
        binaryArray[i] = (temp_position >> i) & 0x01;
    }

    // Calculate the odd parity (All odd positions of the first 14 bits)
    bool oddParity = false;
    for (int i = 0; i < 14; i += 2) {
        oddParity ^= binaryArray[i];
    }
    bool evenParity = false;
    for (int i = 1; i < 15; i += 2) {
        evenParity ^= binaryArray[i];
    }

    if (!oddParity == binaryArray[14] && !evenParity == binaryArray[15]) {
        // The parity bits do not match
        this->valid = true;
        this->position = raw_position & 0x3FFF;
    } else {
        // The parity bits match
        this->valid = false;
    }
    end_transaction();
    last_update_time = millis();
}