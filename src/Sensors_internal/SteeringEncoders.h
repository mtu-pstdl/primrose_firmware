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

    bool initialized = false;
    bool failure     = false;
    bool valid       = false;

    uint16_t raw_position = 0;
    uint16_t position = 0;
    float_t  velocity = 0;
    uint16_t last_position = 0;
    uint32_t last_update_time = 0;

    void begin_transaction() const {
        SPI.beginTransaction(SPISettings(10000, LSBFIRST, SPI_MODE0));
        SPI.setClockDivider(SPI_CLOCK_DIV128);
        digitalWrite(cs_pin, LOW); // Select
        delayMicroseconds(10); //wait for the encoder to be ready (3us as specified in the datasheet)
    }

    void end_transaction() const {
        delayMicroseconds(10); //wait for the encoder to be ready (3us as specified in the datasheet
        digitalWrite(cs_pin, HIGH); // Deselect
        delayMicroseconds(5);
        SPI.endTransaction();
    }

    void reset() {
        begin_transaction();
        spiWriteRead(AMT22_NOP);
        //this is the time required between bytes as specified in the datasheet.
        delayMicroseconds(3);
        spiWriteRead(AMT22_RESET);
        end_transaction();
        delay(250); //250 second delay to allow the encoder to start back up
    }

    void readPosition();

    uint8_t spiWriteRead(uint8_t byte);

    void update_velocity() {
        uint16_t delta_position = this->position - this->last_position;
        uint32_t delta_time = millis() - this->last_update_time;
        this->velocity = delta_position / (float_t) delta_time;
        this->last_position = this->position;
        this->last_update_time = millis();
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
        digitalWrite(cs_pin, HIGH);
    }

    /**
     * Updates the steering encoders
     * @return true if update was successful
     * @return false if update was unsuccessful
     */
    void update() override {
        if (!this->initialized) {
//            this->reset();
            this->initialized = true;
        }
        this->readPosition();
        this->update_velocity();
    }

    /**
     * Gets the position of the steering encoders
     * @return the position of the steering encoders
     */
    int32_t get_position() const override {
        return this->position;
    }

    int32_t get_raw_position() const {
        return this->raw_position;
    }

    float_t get_velocity() const override{
        return this->velocity;
    }

    bool data_valid(){
        return this->valid;
    }

};


#endif //PRIMROSE_MCIU_STEERINGENCODERS_H
