//
// Created by Jay on 10/31/2023.
//

#ifndef PRIMROSE_MCIU_ADAU_BUS_INTERFACE_H
#define PRIMROSE_MCIU_ADAU_BUS_INTERFACE_H
#include <Arduino.h>

#define ADAU_INTERFACE  Serial1
#define ADAU_BAUD_RATE  115200
#define ADAU_RESET_PIN  3

// Message parameters
#define MESSAGE_START_BYTE 0xFF
#define MESSAGE_END_BYTE   0x00
#define MESSAGE_MAX_LENGTH 254
#define END_MESSAGE_COUNT  3

// There is only one ADAU on the bus and we want all ADAU_Sensors to share this object
// We want this object to be a singleton and exist only once automatically

// Forward declaration
class ADAU_Sensor;

class ADAU_Bus_Interface {

    // Data sent by the ADAU is formatted as follows:
    // 1 byte: start of message (0xFF)
    // 1 byte: sensor_id (7bits) + parity (1 bit)
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

    // Watchdog variables
    uint32_t failed_message_count = 0;   // The number of messages that failed to parse
    uint32_t last_message_time = 0;      // The time that the last message was received

    static void reset(){
        digitalWriteFast(ADAU_RESET_PIN, LOW);
        delay(10);
        digitalWriteFast(ADAU_RESET_PIN, HIGH);
        delay(10);
    }

    void load_header();

    void load_data();

    void finish_message();

    void process_message();

    boolean validate_checksum();

    void cleanup();

public:

    /**
     * @brief A linked list of ADAU_Sensors so that they can be added at runtime without knowing the number of sensors
     */
    struct ADAU_Sensor_List {
        ADAU_Sensor*      sensor = nullptr;
        ADAU_Sensor_List* next   = nullptr;
    };

    ADAU_Sensor_List* sensor_list = nullptr;

    char output_string[1000] = {0};

    /**
     * This constructor initializes the serial interface to the Analog Data Acquisition Unit (ADAU)
     * and resets the ADAU to a known state. And begins reading data from the ADAU.
     */
    ADAU_Bus_Interface(){
        ADAU_INTERFACE.begin(ADAU_BAUD_RATE);
        pinMode(ADAU_RESET_PIN, OUTPUT);
        reset(); // Put the ADAU in a known state
    }

    void attachSensor(ADAU_Sensor* sensor);

    void parse_buffer();

    boolean validate_parity();

    // The number of sensors attached to this bus interface
    int num_sensors = 0;
    int parse_count = 0;
};

#endif //PRIMROSE_MCIU_ADAU_BUS_INTERFACE_H
