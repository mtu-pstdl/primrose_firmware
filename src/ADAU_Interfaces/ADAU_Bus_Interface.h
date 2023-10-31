//
// Created by Jay on 10/31/2023.
//

#ifndef PRIMROSE_MCIU_ADAU_BUS_INTERFACE_H
#define PRIMROSE_MCIU_ADAU_BUS_INTERFACE_H
#include <Arduino.h>

#define ADAU_INTERFACE Serial1
#define ADAU_RESET_PIN  3

// There is only one ADAU on the bus and we want all ADAU_Sensors to share this object
// We want this object to be a singleton and exist only once automatically

// Forward declaration
class ADAU_Sensor;

class ADAU_Bus_Interface {

    // Data sent by the ADAU is formatted as follows:
    // 1 byte: sensor_id
    // 1 byte: data_length (Needs to match the data_size of the sensor otherwise reject the message)
    // 1 byte: checksum (xor of all bytes in the message including the sensor_id)
    // n bytes: data
    // 5 bytes: end of message (0x00 0x00 0x00 0x00 0x00)

private:

    // A dynamically allocated array of pointers to ADAU_Sensors that attach themselves to this bus interface
    ADAU_Sensor** sensors;
    // The number of sensors attached to this bus interface
    int num_sensors = 0;
    int current_max_sensors = 10;

    // Message fragment variables
    bool   header_received = false; // True if the header of the message has been received
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

    void parse_buffer();

public:

    ADAU_Bus_Interface(){
        ADAU_INTERFACE.begin(115200);
        pinMode(ADAU_RESET_PIN, OUTPUT);
        reset(); // Put the ADAU in a known state

        // Allocate memory for the array of pointers to sensors
        sensors = (ADAU_Sensor**)malloc(sizeof(ADAU_Sensor*) * current_max_sensors);

        // Set all pointers to nullptr
        for(int i = 0; i < current_max_sensors; i++){
            sensors[i] = nullptr;
        }
    }

    void attachSensor(ADAU_Sensor* sensor);

    void bufferCheck() { // Called externally in the main loop
        if(ADAU_INTERFACE.available()){
            parse_buffer();
        }
    }

};

#endif //PRIMROSE_MCIU_ADAU_BUS_INTERFACE_H
