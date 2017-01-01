/************************************************************************
* FILENAME :        data_struct.h             DESIGN REF:
*
* DESCRIPTION :
*       Data Structure header file for Drone Control system
*
* PUBLIC FUNCTIONS :
*       void init_data_struct()  ->   Initialize all data structures
*       void serial_transmit()   ->   Transmit all nav data serially (DEBUG)
*       void serial_callback()   ->   Function to take callback from user
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

 #ifndef data_struct_h
 #define data_struct_h

 #define _DEBUG_
 #define dt 0.010f
 #define DT 10

 #include "mbed.h"
 #include "Eigen.h"
 #include <Geometry>

 extern bool start_flag;
 extern Serial pc;

 extern Timer t;

 typedef struct
 {
   char ch_enc1[4];
   int t;
   float q[4];

   char ch_enc2[4];
   float pos[3];
   float flat;
   float flon;

   char ch_enc3[4];
   float vel[3];
   float falt;
   int radar_dist;
 } nav_struct;

 typedef struct
 {
   float acc[3];
   float gyro[3];
   float mag[3];
   float acc_filt[3];
   float gyro_filt[3];
   float mag_filt[3];
 }imu_struct;

 typedef struct
 {
   float lat;
   float lon;
   float alt;
   float vel;
   float azimuth;
 }gps_struct;

 union nav_data_bytes
 {
   nav_struct val;
   char ch[sizeof(nav_struct)];
 };

 union imu_bytes
 {
   imu_struct val;
   char ch[sizeof(imu_struct)];
 };

union gps_bytes
{
   gps_struct val;
   char ch[sizeof(gps_struct)];
};

// Declare nav_data struct
extern nav_data_bytes nav_data;

// Declare imu_data struct
extern imu_bytes imu_data;

// Declare gps_data struct
extern gps_bytes gps_data;

// Declare Eigen Variables
//extern Eigen::Vector3f acc, gyro, mag, acc_filt, gyro_filt, mag_filt, vel, pos;
extern Eigen::Map<Eigen::Vector3f> acc, gyro, mag, acc_filt, gyro_filt, mag_filt, vel, pos;
extern Eigen::Map<Eigen::Vector4f> quat;

extern Eigen::Vector3f eul;
extern float phi, theta, psi;

// Declare Identity Matrices
extern Eigen::Matrix3f I3;
extern Eigen::Matrix4f I4;

// Initialization function for data structures
void init_data_struct();

void serial_transmit();

void serial_callback();

#endif
