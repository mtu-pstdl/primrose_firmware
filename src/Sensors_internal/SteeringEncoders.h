//
// Created by Jay on 10/24/2023.
//

#ifndef PRIMROSE_MCIU_STEERINGENCODERS_H
#define PRIMROSE_MCIU_STEERINGENCODERS_H

#include <cstdint>
#include <Arduino.h>
#include <SPI.h>
#include "PositionSensor.h"

#define AMT22_NOP       0x00
#define AMT22_RESET     0x60
#define AMT22_ZERO      0x70

/*Resolution Parameters*/
#define RES12           12
#define RES14           14

class SteeringEncoders : public PositionSensor {

private:
    /*Object creation*/
    uint8_t cs_pin;
    uint8_t resolution = RES14;

    bool initialized = false;
    bool transaction_in_progress = false;
    bool failure     = false;

    uint16_t position = 0;

    void begin_transaction() {
        this->transaction_in_progress = true;
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        SPI.setClockDivider(SPI_CLOCK_DIV32);
        digitalWriteFast(cs_pin, LOW); // Select
        delayMicroseconds(3); //wait for the encoder to be ready (3us as specified in the datasheet)
    }

    void end_transaction() {
        SPI.endTransaction();
        digitalWriteFast(cs_pin, HIGH); // Deselect
        this->transaction_in_progress = false;
    }

    void reset() {
        spiWriteRead(AMT22_NOP, false);
        //this is the time required between bytes as specified in the datasheet.
        delayMicroseconds(3);
        spiWriteRead(AMT22_RESET, true);
        delay(250); //250 second delay to allow the encoder to start back up
    }

    uint16_t readPosition(){
        uint16_t currentPosition;       //16-bit response from encoder
        bool binaryArray[16];           //after receiving the position we will populate this array and use it for calculating the checksum

        //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
        currentPosition = spiWriteRead(AMT22_NOP, false) << 8;

        //this is the time required between bytes as specified in the datasheet.
        delayMicroseconds(3);

        //OR the low byte with the currentPosition variable. release line after second byte
        currentPosition |= spiWriteRead(AMT22_NOP,true);

        //run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
        for(int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (currentPosition >> (i));

        //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
//        if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^
//        binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^
//        binaryArray[1])) && (binaryArray[14] == !(binaryArray[12] ^
//        binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
//            currentPosition &= 0x3FFF; //we got back a good position, so just mask away the checkbits
//        else
//            currentPosition = 0xFFFF; //bad position

        //If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
        if ((this->resolution == RES12) && (currentPosition != 0xFFFF)) currentPosition = currentPosition >> 2;
        return currentPosition;
    }

    uint8_t spiWriteRead(uint8_t byte, boolean release_line) {
        if (!transaction_in_progress) begin_transaction();
        uint8_t response;
        response = SPI.transfer(byte);
        if (release_line) end_transaction();
        return response;
    }

public:

    /**
     * Constructor for the Steering Encoders
     * @param cs_pin
     * @note This constructor assumes that the SPI bus has already been initialized
     */
    explicit SteeringEncoders(uint8_t cs_pin){
        this->cs_pin = cs_pin;
        pinMode(cs_pin, OUTPUT);
        digitalWriteFast(cs_pin, HIGH); // Deselect
    }

    /**
     * Updates the steering encoders
     * @return true if update was successful
     * @return false if update was unsuccessful
     */
    bool update() {
        if (!this->initialized) return false;
        this->position = this->readPosition();
        if (this->position == 0xFFFF) {
            this->failure = true;
            return false;
        }
        return true;
    }

    /**
     * Gets the position of the steering encoders
     * @return the position of the steering encoders
     */
    uint16_t getPosition() const {
        return this->position;
    }


};


#endif //PRIMROSE_MCIU_STEERINGENCODERS_H
