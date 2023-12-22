//
// Created by Jay on 12/22/2023.
//


#include "BreadCrumbs.h"

void save_breadcrumbs() {
    memcpy(&last_breadcrumbs, &bread_crumbs, sizeof(breadcrumbs));
    // Clear the current breadcrumbs
    bread_crumbs.index = 0;
    bread_crumbs.total = 0;
    bread_crumbs.has_crumbs = false;
}

void add_breadcrumb(const char *pretty_name, const char *file) {
    if (bread_crumbs.index >= 16) {
        bread_crumbs.index = 0;
    }
    breadcrumb *current = &bread_crumbs.crumbs[bread_crumbs.index];
    sprintf(current->pretty_name, "%s", pretty_name);
    sprintf(current->file, "%s", file);
    current->time = millis();
    bread_crumbs.index++;
    bread_crumbs.total++;
    bread_crumbs.has_crumbs = true;
}

boolean has_breadcrumbs() {
    return last_breadcrumbs.total > 0 and last_breadcrumbs.has_crumbs;
}

void print_breadcrumb(breadcrumb *crumb, char* buffer) {
    sprintf(buffer, "%s: %s", crumb->file, crumb->pretty_name);
}

breadcrumb* get_breadcrumb() {
    // The breadcrumbs are stored in a circular buffer with index being the next breadcrumb to be added
    // and total being the total number of breadcrumbs added, if total is less than 16 then the buffer has not wrapped
    // otherwise the buffer has wrapped and the index is the oldest breadcrumb

    // Keep track of where we are in the buffer (this method returns a null pointer once all breadcrumbs have been read)
    if (last_breadcrumbs.total == 0) return nullptr;
    static uint32_t current_index = 0;
    static uint32_t remaining = last_breadcrumbs.total;
    if (remaining == 0) return nullptr;
    breadcrumb* current = &last_breadcrumbs.crumbs[current_index];
    if (current_index >= 15) {
        current_index = 0;
    } else {
        current_index++;
    }
    remaining--;
    return current;
}
