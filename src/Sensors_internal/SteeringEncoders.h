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

#define SPI_INTERFACE   SPI

#define MAX_ATTEMPTS_PER_UPDATE 5
#define MAX_INCREMENT_JITTER    2000

/**
 * Interfaces directly with a steering encoder and provides the position and velocity of the steering encoder
 * @details The steering encoders are AMT22 absolute encoders that are connected to SPI bus 1
 * @note This class is a derived class of the PositionSensor class and should be treated as a PositionSensor
 */
class SteeringEncoders : public PositionSensor {

private:
    /*Object creation*/
    uint8_t cs_pin;

    bool initializing       = false;
    bool initialized        = false;
    bool failure            = false;
    bool signal_valid       = false;
    bool valid_position     = false;

    uint16_t raw_position = 0;
    uint16_t position = 0;
    float_t  velocity = 0;
    uint16_t last_position = 0;
    uint32_t last_update_time = 0;

    uint8_t  attempts = 0;
    uint32_t initialization_time = 0;

    void begin_transaction() const;

    void end_transaction() const;

    void reset() {
        begin_transaction();
        spiWriteRead(AMT22_NOP);
        //this is the time required between bytes as specified in the datasheet.
        delayMicroseconds(3);
        spiWriteRead(AMT22_RESET);
        end_transaction();
        this->initialization_time = millis();
    }

    void readPosition();

    static uint8_t spiWriteRead(uint8_t byte);

    void update_velocity() {
        if (!this->signal_valid) return;
        int32_t delta_position = this->position - this->last_position;
        if (delta_position > MAX_INCREMENT_JITTER || delta_position < -MAX_INCREMENT_JITTER) {
            this->valid_position = false;
            return;
        } else {
            this->valid_position = true;
        }
        float_t delta_time =  (float_t) (micros() - this->last_update_time) / 1000000.0f;
        if (delta_time == 0) {
            this->velocity = 0;
            return;
        }
        this->velocity = (float_t) delta_position / (float_t) delta_time;
        this->last_position = this->position;
        this->last_update_time = micros();
    }

public:

    /**
     * @brief Construct a new Steering Encoders object with the given chip select pin
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
     * @note This function must be called once every loop in order to have up to date data
     */
    void update() override {
//        if (this->cs_pin == 0) {
//            this->failure = true;
//            return;
//        }
        if (!this->initialized && !this->initializing) {
            this->reset();
            this->initializing = true;
        }
        if (this->initializing && millis() - this->initialization_time > 1000) {
            this->initializing = false;
            this->initialized = true;
        }
        if (this->initializing) {
            return;
        }
        this->attempts = 0;
        while (this->attempts < MAX_ATTEMPTS_PER_UPDATE) {
            this->readPosition();
            if (this->signal_valid) {
                this->position = this->raw_position & 0x3FFF;
                last_update_time = millis();
                break;
            }
            this->attempts++;
            delayMicroseconds(50);
        }
        this->update_velocity();
    }

    /**
     * Gets the last valid position of the steering encoders,
     * @return The position of the steering encoders in ticks (0 - 16383)
     */
    int32_t get_position() const override {
        return this->position;
    }

    /**
     * Gets the raw position of the steering encoders
     * @return The raw position of the steering encoders in ticks (0 - 16383)
     */
    int32_t get_raw_position() const {
        return this->raw_position;
    }

    float_t get_velocity() const override{
        return this->velocity;
    }

    bool data_valid() const {
        return this->signal_valid;
    }

    bool position_valid() const {
        return this->valid_position;
    }

    boolean is_valid() override {
        return this->initialized && !this->failure && this->signal_valid;
    }

    uint8_t fault() override {
        return this->failure;
    }

    void zeroEncoder();
};


#endif //PRIMROSE_MCIU_STEERINGENCODERS_H
