/************************************************************************
 * FILENAME :        nav.h             DESIGN REF:
 *
 * DESCRIPTION :
 *       Navigation header file for Drone Control system
 *
 * PUBLIC FUNCTIONS :
 *       void init_nav()      ->   Initialize IMU module MPU6050 + HMC5883L
 *       void run_efk()       ->   EKF for IMU sensor data
 *       void getRotMat()     ->   Get current orientation rotation matrix
 *       void euler2quat()    ->   Convert Euler angles to Quaternion
 *       void quat2euler()    ->   Convert Quaternion to Euler angles
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

#ifndef nav_h
#define nav_h

#include "math.h"
#include "Eigen.h"
#include "data_struct.h"
#include "radio.h"

#define RAD2DEG 180.0/M_PI // Radian to degree conversion factor
#define DEG2RAD M_PI/180.0 // Degree to radian conversion factor
#define ANG_TOL M_PI*0.01  // Tolerance of +/-1.8 degrees for pitch = +/- 90 deg
#define MAG_DECLINATION -1*(3+ 51/60+ 46/3600)*M_PI/180 // Value in Chicago from WMM model

void init_nav();
void run_ekf();
void getRotMat(Eigen::Matrix3f &R);
void euler2quat(Eigen::Vector3f &eul, Eigen::Map<Eigen::Vector4f> q);
void quat2euler(Eigen::Map<Eigen::Vector4f> q, Eigen::Vector3f &eul);

#endif
