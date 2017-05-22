/************************************************************************
 * FILENAME :        radio.h             DESIGN REF:
 *
 * DESCRIPTION :
 *       Radio header file for Drone Control system
 *
 * PUBLIC FUNCTIONS :
 *       void init_radio()       ->   Initialize Radio module NRF24L01
 *       void transmit_radio()   ->   Transmit nav data using Radio
 *
 * NOTES :
 *       This file is a part of the OpenFlightSys Github project
 *       Visit https://github.com/mcprakash/OpenFlightSys for more details
 *
 *       Copyright <Chandra Mangipudi> (2016)  All rights reserved.
 *
 * AUTHOR :    Chandra Mangipudi        START DATE :    01/01/2017
 *
 * LICENSE:
 *       This program is free software: you can redistribute it and/or modify
 *       it under the terms of the GNU General Public License as published by
 *       the Free Software Foundation, either version 3 of the License, or
 *       (at your option) any later version.
 *
 *       This program is distributed in the hope that it will be useful,
 *       but WITHOUT ANY WARRANTY; without even the implied warranty of
 *        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *        GNU General Public License for more details.
 *
 *       You should have received a copy of the GNU General Public License
 *       along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

#ifndef radio_h
#define radio_h

#include <mbed.h>
extern Serial radio;

#define RADIO_PCK_SIZE 64
extern char radio_cmds[RADIO_PCK_SIZE];
void radio_signal_wait();
void transmit_radio();
void radio_callback();

#endif
