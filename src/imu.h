#ifndef imu_h
#define imu_h

//#define IMU_CALIB
//#define _PRINT_IMU_
//#define _PRINT_GPS_

#include "mbed.h"
#include "data_struct.h"
#include "math.h"

extern I2C i2c;
extern bool flag_calib;
void init_imu();
void update_imu();
void get_gyro_bias_est();
void imu_calib();

#endif
