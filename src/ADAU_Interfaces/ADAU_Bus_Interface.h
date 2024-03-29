//
// Created by Jay on 10/31/2023.
//

#ifndef PRIMROSE_MCIU_ADAU_BUS_INTERFACE_H
#define PRIMROSE_MCIU_ADAU_BUS_INTERFACE_H
#include <Arduino.h>
#include "Misc/EStopDevice.h"

//#define MAKE_CLION_HAPPY

#ifdef MAKE_CLION_HAPPY
#define ADAU_INTERFACE  Serial2
#error Wrong serial port for ADAU interface
#else
#define ADAU_INTERFACE  Serial8
#endif
#define ADAU_BAUD_RATE   1000000  // 1Mbaud
#define ADAU_SERIAL_MODE SERIAL_8N1
#define ADAU_RESET_PIN  3
#define SERIAL_BUFFER_SIZE 4096

// Message parameters
#define MESSAGE_START_BYTE 0xFF
#define MESSAGE_END_BYTE   0x00
#define MESSAGE_MAX_LENGTH 254
#define END_MESSAGE_COUNT  3

#define MAX_PARSE_TIME 2000 // 1ms

// There is only one ADAU on the bus and we want all ADAU_Sensors to share this object
// We want this object to be a singleton and exist only once automatically

// Forward declaration
class ADAU_Sensor;

/**
 * @brief This class is used to interface with the Analog Data Acquisition Unit (ADAU)
 * @details ADAU_Sensors attach themselves to this class automatically and are updated
 * automatically when new data is received from the ADAU. This class is a singleton and
 * only one instance of it should exist.
 * @warning This class is not thread safe and should only be used in the main thread.
 * @warning This class is not interrupt safe and should not be used in an interrupt
 */
class ADAU_Bus_Interface: public EStopDevice {

    // Data sent by the ADAU is formatted as follows:
    // 1 byte: start of message (0xFF)
    // 1 byte: sensor_id
    // 1 byte: data_length (Needs to match the data_size of the sensor otherwise reject the message)
    // 1 byte: checksum (sum of all bytes in the message including data_length, excludes sensor_id)
    // n bytes: data
    // 6 bytes: end of message (0x00 0x00 0x00 0x00 0x00 0x00)
private:


    enum current_state {
        waiting_for_start_byte,
        waiting_for_header,
        waiting_for_data,
        waiting_for_end_byte,
        waiting_for_end_byte_failure,
    } current_state = waiting_for_start_byte;

    uint32_t parse_start_time = 0;
    // Message fragment variables

    uint8_t message_end_count = 0;   // The number of 0x00 bytes received
    struct message_header {
        uint8_t sensor_id = 0;      // The id of the sensor that sent the message
        uint8_t data_length = 0;    // The length of the data in the message
        uint8_t checksum = 0;       // The checksum of the message
    } message_header = {0, 0, 0};   // The header of the message
    uint8_t header_length = 0;           // The length of the header
    uint8_t header_buffer[3] = {0,};     // The buffer that holds the header data
    uint8_t message_buffer[254] = {0,};  // The buffer that holds the message data
    uint8_t message_buffer_len = 0;      // The length of the message data received so far

    boolean  attempting_restart = false;  // True if the bus interface is attempting to restart the ADAU
    uint32_t restart_start_time = 0;      // The time that the restart started
    uint32_t restart_attempts = 0;        // The number of times the bus interface has attempted to restart the ADAU

    // Watchdog variables
    uint32_t failed_message_count = 0;   // The number of messages that failed to parse
    uint32_t failed_checksum_count = 0;  // The number of messages that failed the checksum
    uint32_t failed_not_found_count = 0; // The number of messages that were not found
    uint32_t last_message_time = 0;      // The time that the last message was received

    // Sliding window of the decoding success rate of the last 20 messages
    uint8_t success_rate_window[20] = {0,};

    boolean one_shot = false;

    void reset(){
//        if (millis() - this->restart_start_time < 2000) {
//
//        } else if (!this->attempting_restart) {
//            this->attempting_restart = true;
//            this->restart_start_time = millis();
//            this->restart_attempts++;
////            digitalWriteFast(ADAU_RESET_PIN, LOW);
//        } else if (millis() - this->restart_start_time > 100) {
////            digitalWriteFast(ADAU_RESET_PIN, HIGH);
//            this->attempting_restart = false;
//        }
    }

    void load_header();

    void load_data();

    void finish_message();

    void process_message();

    boolean validate_checksum();

    void cleanup();

    void prevalidate_data_length();

public:

    /**
     * @brief A linked list of ADAU_Sensors so that they can be added at runtime without knowing the number of sensors
     */
    struct ADAU_Sensor_List {
        ADAU_Sensor*      sensor = nullptr;
        ADAU_Sensor_List* next   = nullptr;
    };

    ADAU_Sensor_List* sensor_list = nullptr;

    // Increase the size of the serial buffer to 2kb, so we don't miss any messages when the main thread is busy
    uint8_t serial_buffer[SERIAL_BUFFER_SIZE] = {0};

    char output_string[1000] = {0};

    /**
     * @brief Initializes the serial interface and triggers a reset of the ADAU
     */
    ADAU_Bus_Interface(){
        ADAU_INTERFACE.begin(ADAU_BAUD_RATE, ADAU_SERIAL_MODE);
        ADAU_INTERFACE.addMemoryForRead(serial_buffer, SERIAL_BUFFER_SIZE);
        pinMode(ADAU_RESET_PIN, OUTPUT);
//        reset(); // Put the ADAU in a known state
        // Attach to the serialEvent interrupt for whatever serial port is being used
    }

    void attachSensor(ADAU_Sensor* sensor);

    void parse_buffer();

    // The number of sensors attached to this bus interface
    int num_sensors = 0;
    int parse_count = 0;

    void resume() override {

    }

    void estop() override {

    }

    EStopDevice::TRIP_LEVEL tripped(char* tripped_device_name, char* tripped_device_message) override;

};

#endif //PRIMROSE_MCIU_ADAU_BUS_INTERFACE_H
