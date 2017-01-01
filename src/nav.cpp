/************************************************************************
* FILENAME :        nav.cpp             DESIGN REF:
*
* DESCRIPTION :
*       Navigation source file for Drone Control system
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

#include "nav.h"

Eigen::Matrix3f K_pos, P_pos, Q_pos, R_pos;
Eigen::Matrix3f K_ang, P_ang, Q_ang, R_ang;

Eigen::Matrix3f T_magno;
float phi, theta, psi;
Eigen::Vector3f eul;

float dummy[4];
Eigen::Map<Eigen::Vector4f> q_m(dummy);

void init_nav()
{
  #ifdef _DEBUG_
    pc.printf("<------------------------------------>\r\n");
    pc.printf("      Initializing Nav Module     \r\n");
  #endif

  K_pos = 0.5*Eigen::Matrix3f::Identity();
  K_ang = 0.5*Eigen::Matrix3f::Identity();

  P_pos = 10*Eigen::Matrix3f::Identity();
  P_ang = 10*Eigen::Matrix3f::Identity();

  Q_pos = 1*Eigen::Matrix3f::Identity();
  Q_ang = 1*Eigen::Matrix3f::Identity();

  R_pos = 0.1*Eigen::Matrix3f::Identity();
  R_ang = 0.1*Eigen::Matrix3f::Identity();

  // Get initial orientation from accelerometer
  theta = asin(acc_filt(0)/acc_filt.norm());

  if(abs(theta-M_PI/2) < ANG_TOL || abs(theta+M_PI/2) < ANG_TOL)
    return;

  phi =asin(-acc_filt(1)/acc_filt.norm()/cos(theta));

  T_magno << cos(theta), sin(theta)*sin(phi), sin(theta)*cos(phi),
		        0, cos(phi), -sin(phi),
		        -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi);

  mag_filt = T_magno*mag;
  psi = MAG_DECLINATION - atan2(mag_filt(1),mag_filt(0));

  eul << phi, theta, psi;
  euler2quat(eul, quat);

  pc.printf("q_init=%f, %f, %f, %f \r\n", quat(0), quat(1), quat(2), quat(3));


  #ifdef _DEBUG_
    pc.printf("      Initialized Nav Moudle      \r\n");
    pc.printf("<------------------------------------>\r\n");
  #endif
}

void run_ekf()
{
  // Time Update of Quaternion (Orientation)
  static Eigen::Matrix<float, 4, 4> Omega;
  static Eigen::Vector4f q_old;
  q_old=quat;

  // Get sensor data
  Omega<< 0, -gyro_filt(0), -gyro_filt(1), -gyro_filt(2),
          gyro_filt(0),  0,  gyro_filt(2), -gyro_filt(1),
          gyro_filt(1), -gyro_filt(2), 0 ,  gyro_filt(0),
          gyro_filt(2),  gyro_filt(1), -gyro_filt(0),  0;

  // Do time update of quaternion
  quat += 0.5*Omega*q_old*dt;
  quat/=quat.norm();

  // Measurement Update of Orientation
  theta = asin(acc_filt(0)/acc_filt.norm());

  if(abs(theta-M_PI/2) < ANG_TOL || abs(theta+M_PI/2) < ANG_TOL)
    return;

  phi =asin(-acc_filt(1)/acc_filt.norm()/cos(theta));

  T_magno << cos(theta), sin(theta)*sin(phi), sin(theta)*cos(phi),
		        0, cos(phi), -sin(phi),
		        -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi);

  mag_filt = T_magno*mag;
  psi = MAG_DECLINATION - atan2(mag_filt(1),mag_filt(0));

  eul<< phi, theta, psi;

  euler2quat(eul, q_m);

  // Fusion of Orientation
  // quat = (quat+q_m)/2;
  // quat/=quat.norm();


  // Time Update of Position
  vel += dt*acc_filt;
  pos += dt*vel;

  // Measurement Update of Position
  #ifdef NAV_DEBUG
    static int cnt=0;
    if(cnt%100==0)
    {
      pc.printf("q_m = %f, %f, %f, %f \r\n", q_m(0), q_m(1), q_m(2), q_m(3));
      pc.printf("eul = %f, %f, %f \r\n", eul(0)*180.0/M_PI, eul(1)*180.0/M_PI, eul(2)*180.0/M_PI);
      quat2euler(quat, eul);
      pc.printf("quat = %f, %f, %f, %f \r\n", quat(0), quat(1), quat(2), quat(3));
      pc.printf("eul = %f, %f, %f \r\n\r\n", eul(0)*180.0/M_PI, eul(1)*180.0/M_PI, eul(2)*180.0/M_PI);
    }
    cnt++;
  #endif
}

// Function to get current Rotation Matrix
void getRotMat(Eigen::Matrix3f &R)
{
  R = Eigen::Quaternionf(quat(0), quat(1), quat(2), quat(3)).toRotationMatrix();
}

// Function to convert Quaternion to Euler angles (roll, pitch, yaw)
void quat2euler(Eigen::Map<Eigen::Vector4f> q, Eigen::Vector3f &eul)
{
  eul << atan2( 2*(q(0)*q(1)+ q(2)*q(3)), 1- 2*q(1)*q(1) -2*q(2)*q(2) ),
         asin(2*(q(0)*q(2)-q(1)*q(3))),
         atan2( 2*(q(0)*q(3)+ q(2)*q(1)), 1- 2*q(3)*q(3) -2*q(2)*q(2) );
}

// Function to convert Euler angles to Quaternion
void euler2quat(Eigen::Vector3f &eul, Eigen::Map<Eigen::Vector4f> q)
{
  q << cos(eul(2)/2)*cos(eul(1)/2)*cos(eul(0)/2) + sin(eul(2)/2)*sin(eul(1)/2)*sin(eul(0)/2),
       cos(eul(2)/2)*cos(eul(1)/2)*sin(eul(0)/2) - sin(eul(2)/2)*sin(eul(1)/2)*cos(eul(0)/2),
       cos(eul(2)/2)*sin(eul(1)/2)*cos(eul(0)/2) + sin(eul(2)/2)*cos(eul(1)/2)*sin(eul(0)/2),
       sin(eul(2)/2)*cos(eul(1)/2)*cos(eul(0)/2) - cos(eul(2)/2)*sin(eul(1)/2)*sin(eul(0)/2);
}
