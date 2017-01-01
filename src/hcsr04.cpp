/************************************************************************
* FILENAME :        hcsr04.cpp             DESIGN REF:
*
* DESCRIPTION :
*       Radar source file for Drone Control system
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
#include "data_struct.h"
#include "ultrasonic.h"

void radar_callback(int dist)
{
  nav_data.val.radar_dist =dist;
}
ultrasonic radar(PA_0, PA_1, 0.1, 1, &radar_callback);

void init_radar()
{
  radar.startUpdates();
}

void update_radar()
{
  radar.checkDistance();
}
