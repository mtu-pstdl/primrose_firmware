//
// Created by Jay on 6/21/2023.
//

#ifndef PRIMROSE_MCIU_ODOMETERS_H
#define PRIMROSE_MCIU_ODOMETERS_H

#include <Arduino.h>
#include <EEPROM.h>

#define ODOMETER_COUNT 15
#define STARTING_ODOMETER_ADDRESS 0
#define SAVE_ODOMETER_INTERVAL 30  // Unit: seconds

class Odometers {

    struct saved_odometer_value {  // stores odometer data in EEPROM
        uint16_t crc16;       // CRC16 checksum (used to detect EEPROM corruption)
        uint16_t sequence_id; // Sequence ID (used to determine which odometer data is most recent)
        uint32_t odometer;    // Unit: 10th of a turn
        uint32_t used_power;  // Unit: watt-hours (scaled by 10)
    };

    struct saved_odometer {
        saved_odometer_value values[4];  // Saves 3 copies of odometer data for redundancy and error correction
    };

    struct memory_odometer_value {  // stores odometer data in RAM
        uint16_t sequence_id; // Sequence ID (used to determine which odometer data is most recent)
        uint32_t odometer;    // Unit: 10th of a turn
        uint32_t used_power;  // Unit: watt-hours (scaled by 10)
    };

    saved_odometer eeprom_odometers[ODOMETER_COUNT];  // stores odometer data in RAM

    memory_odometer_value odometers[ODOMETER_COUNT];  // stores odometer data in RAM

    static uint16_t calc_crc(memory_odometer_value odometer) {
        // Calculates CRC16 checksum of odometer data
        uint16_t crc = 0xFFFF;
        auto* odometer_bytes = (uint8_t*) &odometer;
        for (int i = 0; i < sizeof(memory_odometer_value); i++) {
            crc ^= odometer_bytes[i];

            for (int j = 0; j < 8; j++) {
                if (crc & 0x0001) {
                    crc >>= 1;
                    crc ^= 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

    static memory_odometer_value extract_odometer(saved_odometer_value odometer) {
        // Extracts odometer data from saved_odometer struct
        memory_odometer_value odometer_value{};
        odometer_value.odometer = odometer.odometer;
        odometer_value.used_power = odometer.used_power;
        odometer_value.sequence_id = odometer.sequence_id;
        return odometer_value;
    }

    static void condense_odometer(memory_odometer_value odometer, saved_odometer_value& odometer_value) {
        // Condenses odometer data into saved_odometer_value struct
        odometer_value.odometer = odometer.odometer;
        odometer_value.used_power = odometer.used_power;
        odometer_value.sequence_id = odometer.sequence_id;
        odometer_value.crc16 = calc_crc(odometer);
    }

    void load_odometers() {
        // Each odometer has 4 save slots in EEPROM for redundancy and error correction
        // Each slot has a CRC16 checksum to detect EEPROM corruption
        // and a sequence ID to determine which slot is most recent
        // This function loads odometer data from EEPROM and stores it in RAM

        memory_odometer_value odometer_values[ODOMETER_COUNT][4]; // Temporary array to store odometer data from EEPROM
        for (int i = 0; i < ODOMETER_COUNT; i++) {
            EEPROM.get(STARTING_ODOMETER_ADDRESS + i * sizeof(saved_odometer), eeprom_odometers[i]);
            for (int j = 0; j < 4; j++) {
                odometer_values[i][j] = extract_odometer(eeprom_odometers[i].values[j]);
            }
        }

    }

    void save_odometers() {
        // Each odometer is stored in EEPROM as 3 copies of the same data for redundancy and error correction
        // When saving odometers, the data is written to EEPROM and the 3 copies CRC16 checksums are calculated

    }

public:

    Odometers() {
        load_odometers();
    }

    memory_odometer_value* get_odometer(uint8_t id) {
        // Returns odometer data for a given odometer ID
        return &odometers[id];
    }

    void refresh(){
        // Saves odometer data to EEPROM every SAVE_ODOMETER_INTERVAL seconds
        static unsigned long last_save = 0;
        if (millis() - last_save > SAVE_ODOMETER_INTERVAL * 1000) {
            save_odometers();
            last_save = millis();
        }
    }

};


#endif //PRIMROSE_MCIU_ODOMETERS_H
