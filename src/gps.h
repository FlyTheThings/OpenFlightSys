/************************************************************************
 * FILENAME :        gps.h             DESIGN REF:
 *
 * DESCRIPTION :
 *       GPS header file for Drone Control system
 *
 * PUBLIC FUNCTIONS :
 *       void init_gps()     ->   Initialize GPS module Ublox NEO-8M
 *       void update_gps()   ->   ISR to update GPS upon serial receive
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

#ifndef gps_h
#define gps_h

#include "mbed.h"

extern Serial gps;

void init_gps();

void update_gps();

#endif
