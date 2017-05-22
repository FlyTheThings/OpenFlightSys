/************************************************************************
* FILENAME :        radio.cpp             DESIGN REF:
*
* DESCRIPTION :
*       Radio source file for Drone Control system
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

#include "radio.h"
#include "data_struct.h"

char radio_cmds[RADIO_PCK_SIZE];
bool start_flag = false;

Serial radio(USBTX, USBRX); // tx, rx

void radio_signal_wait()
{
  while(!start_flag)
  {
    wait_ms(100);
  }
}
void transmit_radio()
{
  for(uint i=0; i<sizeof(nav_data.ch); i++)
          radio.putc(nav_data.ch[i]);
}

void radio_callback()
{
        char c = radio.getc();

        // Press 'a' to start program
        if(c=='a')
        {
                start_flag=true;
    #ifdef _DEBUG_
                radio.printf("Starting Program \r\n");
    #endif
        }

        // Press 'b' to end program
        if(c=='b')
        {
    #ifdef _DEBUG_
                radio.printf("Closing program \r\n");
    #endif
                delete &acc;
                delete &gyro;
                delete &mag;
                delete &acc_filt;
                delete &gyro_filt;
                delete &mag_filt;
                exit(0);
        }

  #ifdef IMU_CALIB
        // Press 'i' to get stable reading for 1 iteration of IMU calibration
        if(c=='i')
        {
                flag_calib=true;
                radio.printf("flag_cmd entered \r\n");
        }
  #endif
}
