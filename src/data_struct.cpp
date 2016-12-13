#include "data_struct.h"

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

// Encoding for serial transmission
char ch_encode[4]={'1','9','9','1'};

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
  nav_data.val.dummy[0] = 'd'; nav_data.val.dummy[1] = 'u'; nav_data.val.dummy[2] = 'm'; nav_data.val.dummy[3] = 'y';

  #ifdef _DEBUG_
    pc.printf("Initialized data matrices successfully\r\n");
    pc.printf("<------------------------------------>\r\n");
  #endif

}

void serial_transmit()
{
  for(int i=0; i<4; i++)
   pc.putc(ch_encode[i]);
  for(uint i=0; i<sizeof(nav_data.ch); i++)
   pc.putc(nav_data.ch[i]);

  //pc.printf("\r\n");
}
