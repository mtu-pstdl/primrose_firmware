//
// Created by Jay on 6/21/2023.
//

#ifndef PRIMROSE_MCIU_ODOMETERS_H
#define PRIMROSE_MCIU_ODOMETERS_H

#include <Arduino.h>
#include <EEPROM.h>

#define ODOMETER_COUNT 16
#define STARTING_ODOMETER_ADDRESS 0
#define SAVE_ODOMETER_INTERVAL 30  // Unit: seconds

/**
 * @deprecated This class needs to be rewritten for the new odometer system
 */
class Odometers {

    struct saved_odometer_value {  // stores odometer data in EEPROM
        uint16_t crc16;       // CRC16 checksum (used to detect EEPROM corruption)
        uint32_t sequence_id; // Sequence ID (used to determine which odometer data is most recent)
        uint32_t odometer;    // Unit: 10th of a turn
        uint32_t used_power;  // Unit: watt-hours (scaled by 10)
    };

    struct saved_odometer {
        saved_odometer_value values[4];  // Saves 4 copies of odometer data for redundancy and error correction
    };

    struct memory_odometer_value {  // stores odometer data in RAM
        bool     changed;     // Indicates whether odometer data has changed since last save
        uint32_t sequence_id; // Sequence ID (used to determine which odometer data is most recent)
        uint32_t odometer;    // Unit: 100th of a turn or encoder counts (depending on encoder type)
        uint32_t used_power;  // Unit: milliwatt-hours
    };

    saved_odometer eeprom_odometers[ODOMETER_COUNT]{};  // stores odometer data in RAM

    memory_odometer_value odometers[ODOMETER_COUNT]{};  // stores odometer data in RAM

    static uint16_t calc_crc(memory_odometer_value odometer) {
        // Calculates CRC16 checksum of odometer data
        uint16_t crc = 0xFFFF;
        auto* odometer_bytes = (uint8_t*) &odometer;
        // Skip the first 4 bytes (changed) because it is not saved in EEPROM
        for (int i = 4; i < sizeof(memory_odometer_value); i++) {
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

        memory_odometer_value odometer_values[4]; // Temporary array to store odometer data from EEPROM
        for (int i = 0; i < ODOMETER_COUNT; i++) {
            EEPROM.get(STARTING_ODOMETER_ADDRESS + i * sizeof(saved_odometer), eeprom_odometers[i]);
            for (int j = 0; j < 4; j++) {
                odometer_values[j] = extract_odometer(eeprom_odometers[i].values[j]);
            }
            // Determine which odometer data has the highest sequence ID and a signal_valid CRC16 checksum
            uint16_t highest_sequence_id = 0;
            uint8_t highest_sequence_id_index = 0;

            for (int j = 0; j < 4; j++) {
                if (odometer_values[j].sequence_id > highest_sequence_id &&
                calc_crc(odometer_values[j]) == eeprom_odometers[i].values[j].crc16) {
                    highest_sequence_id = odometer_values[j].sequence_id;
                    highest_sequence_id_index = j;
                }
            }
            // Store the most recent odometer data in RAM
            odometers[i] = odometer_values[highest_sequence_id_index];
        }

    }

    void save_odometers() {
        // Save the odometer data in RAM to EEPROM if it has changed
        for (int i = 0; i < ODOMETER_COUNT; i++) {
            if (odometers[i].changed) {
                // Determine which slot to save the odometer data to
                uint8_t save_index = 0;
                for (int j = 1; j < 4; j++) {
                    if (eeprom_odometers[i].values[j].sequence_id < eeprom_odometers[i].values[save_index].sequence_id) {
                        save_index = j;
                    }
                }
                // Increment the sequence ID
                odometers[i].sequence_id++;
                // Condense the odometer data into a saved_odometer_value struct
                condense_odometer(odometers[i], eeprom_odometers[i].values[save_index]);
                // Save the odometer data to EEPROM at the appropriate address
                EEPROM.put((STARTING_ODOMETER_ADDRESS + i * sizeof(saved_odometer)) +
                                   (save_index * sizeof(saved_odometer_value)),
                                   eeprom_odometers[i].values[save_index]);
                // Mark the odometer data as unchanged
                odometers[i].changed = false;
            }
        }
    }

public:

    Odometers() {
        load_odometers();
    }

    void reset_odometer(uint8_t id) {
        // Resets the odometer data for a given odometer ID
        odometers[id].odometer = 0;
        odometers[id].used_power = 0;
//        odometers[id].sequence_id = 0;
        odometers[id].changed = true;
        save_odometers();
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
