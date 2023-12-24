//
// Created by Jay on 12/22/2023.
//

#ifndef PRIMROSE_MCIU_BREADCRUMBS_H
#define PRIMROSE_MCIU_BREADCRUMBS_H

#include <Arduino.h>
#include "build_info.h"

#define BREADCRUMB_BUFFER_SIZE 24

/**
 * @brief The type of value stored in the breadcrumb
 * @note All values must be exactly 4 bytes long, hence the char4 type
 */
enum breadcrumb_type: uint8_t {
    NO_VALUE, INT, FLOAT, CHAR4, ADDRESS
};

/**
 * @brief A breadcrumb is a piece of information that is dropped at a specific point in the code to help with debugging
 * @details The breadcrumbs are stored in a circular buffer and are printed out when the MCIU crashes (see CrashParser.h)
 * @note The BUILD_DEBUG flag must be set for breadcrumbs to be enabled
 * @warning Don't touch the null_terminator value it's part of the file name and is automatically set
 */
struct breadcrumb {
    char file[36];
    uint8_t null_terminator; // no touchy
    breadcrumb_type type;
    uint16_t line;
    uint32_t time;
    union {
        int32_t  int_value;
        uint32_t address_value;
        float_t  float_value;
        uint8_t  char4_value[4];
    } value;
};

/**
 * @brief Stores the collection of breadcrumbs
 */
struct breadcrumbs {
    breadcrumb crumbs[BREADCRUMB_BUFFER_SIZE];
    uint32_t index; // index of the next breadcrumb to be added
    uint32_t total; // total number of breadcrumbs
    boolean has_crumbs;
};

// Store the breadcrumbs in a section of memory that is not initialized for preservation across resets
static breadcrumbs bread_crumbs __attribute__ ((section(".noinit")));  // NOLINT

// On startup the breadcrumbs stored in the noinit section are copied to this section so that the normal section can be
// cleared and used for new breadcrumbs
static breadcrumbs last_breadcrumbs;  // NOLINT

void save_breadcrumbs();

/**
 * @brief Adds a breadcrumb to the breadcrumb buffer, no value is associated with this breadcrumb
 * @param file The file name where the breadcrumb was dropped (automatically truncated to 32 characters)
 * @param line The line number where the breadcrumb was dropped
 */
void add_breadcrumb(const char *file, uint32_t line);

/**
 * @brief Adds a breadcrumb with a value to the breadcrumb buffer
 * @param file The file name where the breadcrumb was dropped (automatically truncated to 32 characters)
 * @param line The line number where the breadcrumb was dropped
 * @param value The value of the breadcrumb (must be either an int, float, or char4)
 * @param value_type The type of the value, used for printing later
 */
void add_breadcrumb(const char *file, uint32_t line, uint32_t value, breadcrumb_type value_type);

boolean has_breadcrumbs();

uint32_t get_breadcrumb_count();

breadcrumb* get_breadcrumb();

void print_breadcrumb(breadcrumb *crumb, char* buffer);

#ifdef BUILD_DEBUG
#define DROP_CRUMB() add_breadcrumb(__FILE__, __LINE__)
#define DROP_CRUMB_IF(condition) if (condition) add_breadcrumb(__FILE__, __LINE__)
#define DROP_CRUMB_VALUE(value, type) add_breadcrumb(__FILE__, __LINE__, value, type)
#define DROP_CRUMB_VALUE_IF(condition, value, type) if (condition) add_breadcrumb(__FILE__, __LINE__, value, type)
#else
#define DROP_CRUMB()
#define DROP_CRUMB_IF(condition)
#define DROP_CRUMB_VALUE(value, type)
#endif

#endif //PRIMROSE_MCIU_BREADCRUMBS_H
