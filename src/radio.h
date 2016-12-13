#ifndef radio_h
#define radio_h

#include "nRF24L01P.h"
#define RADIO_PCK_SIZE 24

extern nRF24L01P radio;

void init_radio();

void transmit_radio();

#endif
