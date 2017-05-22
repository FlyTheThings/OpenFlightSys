/************************************************************************
 * FILENAME :        imu.h             DESIGN REF:
 *
 * DESCRIPTION :
 *       IMU header file for Drone Control system
 *
 * PUBLIC FUNCTIONS :
 *       void init_imu()      ->   Initialize IMU module MPU6050 + HMC5883L
 *       void update_imu()    ->   Update orientation and position upon new data
 *       void gyro_bias_est() ->   Estimate gyro bias
 *       void imu_calib()     ->   Calibrate IMU using static 3d ellipsoid fit
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

#ifndef imu_h
#define imu_h

//#define IMU_CALIB
//#define _PRINT_IMU_
//#define _PRINT_GPS_

#include "mbed.h"
#include "data_struct.h"
#include "math.h"
#include "radio.h"

extern I2C i2c;
extern bool flag_calib;
void init_imu();
void update_imu();
void gyro_bias_est();
void imu_calib();

#endif
