/************************************************************************
 * FILENAME :        gps.cpp             DESIGN REF:
 *
 * DESCRIPTION :
 *       GPS source file for Drone Control system
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

#include "gps.h"
#include "data_struct.h"

#define __TinyGPS

#include "TinyGPS.h"
#include "nmea.h"

Serial gps(PA_9, PA_10);

bool flg=true;

#ifdef __NGPS
NMEA ngps(ALL);
#endif

#ifdef __TinyGPS
TinyGPS tgps;
unsigned long age;
#endif

char c;
#ifdef __TinyGPS
double flat, flon, falt;
#endif

void init_gps()
{
  #ifdef _DEBUG_
        radio.printf("<------------------------------------>\r\n");
        radio.printf("            Initializing GPS          \r\n");
  #endif

        gps.baud(38400);
        gps.attach(&update_gps);

  #ifdef _DEBUG_
        radio.printf("            Initialized GPS          \r\n");
        radio.printf("<------------------------------------>\r\n");
  #endif
}

void update_gps()
{
        c=gps.getc();

      #ifdef __NGPS
        ngps.decode(c);
        nav_data.val.flat = ngps.gprmc_latitude();
        nav_data.val.flon = ngps.gprmc_longitude();
      #endif

        //radio.putc(c);
      #ifdef __TinyGPS
        if(tgps.encode(c))
        {
                tgps.f_get_position(&flat, &flon, &age);

                nav_data.val.flat = (float) flat;
                nav_data.val.flon = (float) flon;

                nav_data.val.falt = (float) tgps.f_altitude();
        }
      #endif


      #ifdef _PRINT_GPS_
        radio.printf("flat=%f, flon=%f, falt=%f \r\n", nav_data.val.flat, nav_data.val.flon, nav_data.val.falt);
      #endif
}
