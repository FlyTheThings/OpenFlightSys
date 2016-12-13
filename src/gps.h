#ifndef gps_h
#define gps_h

#include "mbed.h"

extern Serial gps;

void init_gps();

void update_gps();

#endif
