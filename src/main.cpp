/************************************************************************
 * FILENAME :        main.cpp             DESIGN REF:
 *
 * DESCRIPTION :
 *       Main file for Drone Control system
 *
 * NOTES :
 *       This file is a part of the OpenFlightSys Github project
 *       Visit https://github.com/mcprakash/OpenFlightSys for more details
 *
 *       Copyright <Chandra Mangipudi> (2017)  All rights reserved.
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
#include "nav.h"
#include "gps.h"
#include "imu.h"
#include "radio.h"
#include "hcsr04.h"

int main()
{
        // Initialize Serial Output for Debugging
        radio.baud(9600);
        radio.attach(&radio_callback);

        // Initialize Data Sturctures for AHRS and Radio transmission
        init_data_struct();

        // Initialize and start IMU => MPU6050 + HMC5883L
        init_imu();

        // Initialize Navigation (AHRS) module
        init_nav();

        // Initialize RADAR module => HC-SR04
        init_radar();

        // Initialize GPS module => Ublox Neo 8m
        init_gps();

  #ifdef IMU_CALIB
        t.start();
        //Function for IMU Calibration
        imu_calib();
        t.stop();
        t.reset();
  #endif
        wait(1);

        // Wait for user to start drone program using radio
        radio_signal_wait();

        t.start();
        int icnt=0;
        int t_delay = 0;
        while(1)
        {
                nav_data.val.t = t.read_ms();
                update_imu();
                update_radar();
                run_ekf();

                t_delay = DT - (t.read_ms() - nav_data.val.t);
                if(t_delay>0)
                        wait_ms(t_delay);
                icnt++;

                if(icnt%100==0)
                {
                        transmit_radio();
                }
        }

}
