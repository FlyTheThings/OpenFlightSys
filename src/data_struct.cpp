/************************************************************************
* FILENAME :        data_struct.cpp             DESIGN REF:
*
* DESCRIPTION :
*      Data Structure source file for Drone Control system
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

#include "data_struct.h"
#include "radio.h"
// Define variables for program start
bool start_flag=false;
Serial pc(USBTX, USBRX); // tx, rx

// Define nav_data struct
nav_data_bytes nav_data;

// Define imu_data struct
imu_bytes imu_data;

// Define gps_data struct
gps_bytes gps_data;

// Define Timer
Timer t;

// Define all Eigen vectors
Eigen::Map<Eigen::Vector3f> acc(imu_data.val.acc), gyro(imu_data.val.gyro), mag(imu_data.val.mag), acc_filt(imu_data.val.acc_filt), gyro_filt(imu_data.val.gyro_filt), mag_filt(imu_data.val.mag_filt), vel(nav_data.val.vel), pos(nav_data.val.pos);
Eigen::Map<Eigen::Vector4f> quat(nav_data.val.q);

// Define all Eigen matrices
Eigen::Matrix3f I3;
Eigen::Matrix4f I4;

// Initialize all data variables
void init_data_struct()
{
  // Map all Eigen Variables to data structures
  #ifdef _DEBUG_
    pc.printf("<------------------------------------>\r\n");
    pc.printf("      Initializing Data matrices      \r\n");
  #endif
  I3.setIdentity();
  I4.setIdentity();

  acc.setZero();
  gyro.setZero();
  mag.setZero();
  acc_filt.setZero();
  gyro_filt.setZero();
  mag_filt.setZero();
  quat <<  1.0f, 0.0f, 0.0f, 0.0f;

  nav_data.val.ch_enc1[0] = '1'; nav_data.val.ch_enc1[1] = '9'; nav_data.val.ch_enc1[2] = '9'; nav_data.val.ch_enc1[3] = '1';
  nav_data.val.ch_enc2[0] = '1'; nav_data.val.ch_enc2[1] = '9'; nav_data.val.ch_enc2[2] = '9'; nav_data.val.ch_enc2[3] = '2';
  nav_data.val.ch_enc3[0] = '1'; nav_data.val.ch_enc3[1] = '9'; nav_data.val.ch_enc3[2] = '9'; nav_data.val.ch_enc3[3] = '3';
  //nav_data.val.dummy[0] = 'd'; nav_data.val.dummy[1] = 'u'; nav_data.val.dummy[2] = 'm'; nav_data.val.dummy[3] = 'y';

  #ifdef _DEBUG_
    pc.printf("Initialized data matrices successfully\r\n");
    pc.printf("<------------------------------------>\r\n");
  #endif

}

void serial_transmit()
{
  for(uint i=0; i<sizeof(nav_data.ch); i++)
    pc.putc(nav_data.ch[i]);

  //pc.printf("\r\n");
}


void serial_callback()
{
  char c = pc.getc();

  // Press 'a' to start program
  if(c=='a')
  {
    start_flag=true;
    #ifdef _DEBUG_
      pc.printf("Starting Program \r\n");
    #endif
  }

  // Press 'b' to end program
  if(c=='b')
  {
    #ifdef _DEBUG_
      pc.printf("Closing program \r\n");
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
      pc.printf("flag_cmd entered \r\n");
    }
  #endif
}
