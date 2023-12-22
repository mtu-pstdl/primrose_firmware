//
// Created by Jay on 12/22/2023.
//

#ifndef PRIMROSE_MCIU_BREADCRUMBS_H
#define PRIMROSE_MCIU_BREADCRUMBS_H

#include <Arduino.h>
#include "build_info.h"

struct breadcrumb {
    char pretty_name[100];
    char file[100];
    uint32_t time;
};

struct breadcrumbs {
    breadcrumb crumbs[16];
    uint32_t index; // index of the next breadcrumb to be added
    uint32_t total; // total number of breadcrumbs
    boolean has_crumbs;
};

static breadcrumbs bread_crumbs __attribute__ ((section(".noinit")));

static breadcrumbs last_breadcrumbs __attribute__ ((section(".noinit")));

void save_breadcrumbs();

void add_breadcrumb(const char *pretty_name, const char *file);

boolean has_breadcrumbs();

breadcrumb* get_breadcrumb();

void print_breadcrumb(breadcrumb *crumb, char* buffer);

#ifdef BUILD_DEBUG
#define DROP_CRUMB() add_breadcrumb(__PRETTY_FUNCTION__, __FILE__);
#else
#define DROP_CRUMB()
#endif

#endif //PRIMROSE_MCIU_BREADCRUMBS_H
