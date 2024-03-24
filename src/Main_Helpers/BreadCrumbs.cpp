//
// Created by Jay on 12/22/2023.
//


#include "BreadCrumbs.h"

void save_breadcrumbs() {
    memcpy(&last_breadcrumbs, &bread_crumbs, sizeof(breadcrumbs));
    memset(&bread_crumbs, 0, sizeof(breadcrumbs));
    // Clear the current breadcrumbs
    bread_crumbs.index = 0;
    bread_crumbs.total = 0;
    bread_crumbs.has_crumbs = false;
}

void add_breadcrumb(const char *file, uint32_t line) {
    if (bread_crumbs.index >= BREADCRUMB_BUFFER_SIZE) bread_crumbs.index = 0; // Wrap the buffer when we reach the end
    breadcrumb *current = &bread_crumbs.crumbs[bread_crumbs.index];
    // Check if the file name is longer than 32 characters
    if (strlen(file) > 35) {
        // If it is then truncate it removing the beginning first
        sprintf(current->file, "%36s", file + strlen(file) - 36);
    } else {
        sprintf(current->file, "%s", file);
    }
    current->type = NO_VALUE;
    current->line = line;
    current->time = micros();
    bread_crumbs.index++;
    bread_crumbs.total++;
    bread_crumbs.has_crumbs = true;
}

void add_breadcrumb(const char* file, uint32_t line, uint32_t value, breadcrumb_type value_type) {
    if (bread_crumbs.index >= BREADCRUMB_BUFFER_SIZE) bread_crumbs.index = 0; // Wrap the buffer when we reach the end
    breadcrumb *current = &bread_crumbs.crumbs[bread_crumbs.index];
    // Check if the file name is longer than 32 characters
    if (strlen(file) > 35) {
        // If it is then truncate it removing the beginning first
        sprintf(current->file, "%36s", file + strlen(file) - 36);
    } else {
        sprintf(current->file, "%s", file);
    }
    current->line = line;
    current->time = micros();
    current->type = value_type;
    switch (value_type) {
        case INT:
            // Fake cast to int32_t
            current->value.int_value = *((int32_t *) &value);
            break;
        case FLOAT:
            current->value.float_value = *((float_t *) &value);
            break;
        case CHAR4:
            current->value.char4_value[0] = ((uint8_t *) &value)[3]; // Split the 4 char value into 4 bytes
            current->value.char4_value[1] = ((uint8_t *) &value)[2];
            current->value.char4_value[2] = ((uint8_t *) &value)[1];
            current->value.char4_value[3] = ((uint8_t *) &value)[0];
            break;
        default:
            break;
    }
    bread_crumbs.index++;
    bread_crumbs.total++;
    bread_crumbs.has_crumbs = true;
}

boolean has_breadcrumbs() {
    return last_breadcrumbs.total > 0 and last_breadcrumbs.has_crumbs;
}

void print_breadcrumb(breadcrumb *crumb, char* buffer) {
    switch (crumb->type) {
        case NO_VALUE:
            sprintf(buffer, "%lu - %s:%hu", crumb->time, crumb->file, crumb->line);
            break;
        case INT:
            sprintf(buffer, "%lu - %s:%hu - %ld", crumb->time, crumb->file, crumb->line, crumb->value.int_value);
            break;
        case FLOAT:
            sprintf(buffer, "%lu - %s:%hu - %f", crumb->time, crumb->file, crumb->line, crumb->value.float_value);
            break;
        case CHAR4:
            sprintf(buffer, "%lu - %s:%hu - %c%c%c%c", crumb->time, crumb->file, crumb->line,
                    crumb->value.char4_value[0], crumb->value.char4_value[1], crumb->value.char4_value[2],
                    crumb->value.char4_value[3]);
            break;
        case ADDRESS:
            sprintf(buffer, "%lu - %s:%hu - 0x%08lx", crumb->time, crumb->file, crumb->line, crumb->value.address_value);
            break;
    }

}

uint32_t get_breadcrumb_count() {
    return last_breadcrumbs.total;
}

breadcrumb* get_breadcrumb() {
    // The breadcrumbs are stored in a circular buffer with index being the next breadcrumb to be added
    // and total being the total number of breadcrumbs added, if total is less than 16 then the buffer has not wrapped
    // otherwise the buffer has wrapped and the index is the oldest breadcrumb

    // Keep track of where we are in the buffer (this method returns a null pointer once all breadcrumbs have been read)
    if (last_breadcrumbs.total == 0) return nullptr;
    static uint32_t current_index = last_breadcrumbs.index;
    static uint32_t remaining = BREADCRUMB_BUFFER_SIZE;
    if (remaining == 0) return nullptr;
    breadcrumb* current = &last_breadcrumbs.crumbs[current_index];
    if (current_index >= BREADCRUMB_BUFFER_SIZE - 1) {
        current_index = 0;
    } else {
        current_index++;
    }
    remaining--;
    return current;
}
