//
// Created by Jay on 10/24/2023.
//

#include "SteeringEncoders.h"

void SteeringEncoders::begin_transaction() const {
    // Begin SPI transaction with configuration
    // .5MHz clock, MSB first, SPI mode 0
    SPI_INTERFACE.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_pin, LOW); // Select
    delayMicroseconds(10); //wait for the encoder to be ready (3us as specified in the datasheet)
}

void SteeringEncoders::end_transaction() const {
    delayMicroseconds(5); //wait the required time to release cs (3us as specified in the datasheet)
    digitalWrite(cs_pin, HIGH); // Deselect
    SPI_INTERFACE.endTransaction();
}

uint8_t SteeringEncoders::spiWriteRead(uint8_t byte) {
    uint8_t response;
    response = SPI_INTERFACE.transfer(byte);
    delayMicroseconds(3);
    return response;
}

void SteeringEncoders::zeroEncoder() {
    begin_transaction();
    spiWriteRead(AMT22_NOP);
    //this is the time required between bytes as specified in the datasheet.
    delayMicroseconds(3);
    spiWriteRead(AMT22_ZERO);
    end_transaction();
}

void SteeringEncoders::readPosition(){
    // If we read position within the last 10ms return the last position
//    if (millis() - last_update_time < 10){
//        return;
//    }
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
    end_transaction();
    if (!oddParity == binaryArray[14] && !evenParity == binaryArray[15]) {
        // The parity bits do not match
        this->signal_valid = true;
    } else {
        // The parity bits match
        this->signal_valid = false;
    }
}