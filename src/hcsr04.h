/************************************************************************
 * FILENAME :        hcsr04.h             DESIGN REF:
 *
 * DESCRIPTION :
 *       Radar header file for Drone Control system
 *
 * PUBLIC FUNCTIONS :
 *       void init_radar()     ->   Initialize Radar module HC-SR04
 *       void update_radar()   ->   ISR to update Radar upon change of distance
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

#include "mbed.h"
#include "ultrasonic.h"

extern ultrasonic radar;

void init_radar();

void update_radar();
