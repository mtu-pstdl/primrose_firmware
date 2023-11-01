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
    begin_transaction();
    bool binaryArray[16];           //after receiving the position we will populate this array and use it for calculating the checksum

    //read the first byte of the position
    raw_position = spiWriteRead(AMT22_NOP) << 8;

    //this is the time required between bytes as specified in the datasheet.
//    delayMicroseconds(3);

    raw_position = raw_position | spiWriteRead(AMT22_NOP);

    // Validate the checkbits
    // The checkbits are odd parity over the odd and even bits

    // Populate the binary array
    for (int i = 0; i < 16; i++) {
        binaryArray[i] = (raw_position >> i) & 1;
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

    if (oddParity != binaryArray[14] || evenParity != binaryArray[15]) {
        // The parity bits do not match
        this->valid = false;
    } else {
        this->valid = true;
        this->position = raw_position & 0x3FFF;
    }
    end_transaction();
}