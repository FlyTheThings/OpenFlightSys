/************************************************************************
* FILENAME :        imu.cpp             DESIGN REF:
*
* DESCRIPTION :
*       IMU source file for Drone Control system
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

#include "imu.h"
#include "Eigen.h"
#include "MPU6050.h"
#include "HMC5883L.h"

#define pi 3.14159265359
#define mag_val1 32767.0f*0.92f
#define mag_val2 65535.0f*0.92f


MPU6050 mpu6050;
HMC5883L hmc5883l(I2C_SDA, I2C_SCL);

Eigen::Matrix3f K_acc, K_mag;
Eigen::Vector3f b_acc, b_gyro, b_mag;

int t_mag_cur, t_mag_prev;

bool flag_calib=false;

void init_imu()
{
  //Set Accel Scale factor and bias

  #ifdef IMU_CALIB
    K_acc.setIdentity();
    b_acc << 0,0,0;
    K_mag.setIdentity();
    b_mag << 0,0,0;
  #else
    K_acc << 1.00137,-0.000288811,-0.000140577,
            -0.000288811,1.00014,0.000424818,
            -0.000140577,0.000424818,0.991729;
    b_acc << 0.00308858,-0.00571007,-0.109089;

    K_mag << 1.00171,-0.00472278,-0.0415619,
            -0.00472278,0.994354,-0.00892205,
            -0.0415619,-0.00892205,1.20343;

    b_mag << -23.011,-36.077,-121.388;
  #endif


  //Set up I2C
  //i2c.frequency(400000);  // use fast (400 kHz) I2C

  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t whoami = mpu6050.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
  //pc.printf("I AM 0x%x\n\r", whoami); pc.printf("I SHOULD BE 0x68\n\r");

  if (whoami == 0x68) // WHO_AM_I should always be 0x68
  {
    #ifdef _DEBUG_
      pc.printf("<------------------------------------>\r\n");
      pc.printf("   Initializing MPU6050 Acc + Gyro   \r\n");
    #endif
    //mpu6050.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values

    mpu6050.initMPU6050();

    mpu6050.getAres();
    mpu6050.getGres();

    #ifdef _DEBUG_
      pc.printf("aRes=%f\r\n", aRes);
      pc.printf("gRes=%f\r\n", gRes);
      pc.printf("   Initialized MPU6050 successfully   \r\n");
      pc.printf("<------------------------------------>\r\n");
    #endif

  }
  else
  {
    pc.printf("Could not connect to MPU6050: \r\n");
    pc.printf("%#x \r\n",  whoami);
  }

  wait(1);
  hmc5883l.setDefault();

  t_mag_cur=t.read_ms();
  t_mag_prev=t.read_ms();

  // Get gyro bias estimate from averaging
  gyro_bias_est();

  // Get readings from INU for initial state estimation
  update_imu();
  wait_ms(1000);
  update_imu();

}

void update_imu()
{
  if(mpu6050.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
      mpu6050.readAccelData(accelCount);  // Read the x/y/z adc values
      // Now we'll calculate the accleration value into actual g's
      acc(0) = ((float)accelCount[1])*aRes;  // get actual g value, this depends on scale being set
      acc(1) = ((float)accelCount[0])*aRes;
      acc(2) = -((float)accelCount[2])*aRes;

      mpu6050.readGyroData(gyroCount);  // Read the x/y/z adc values
      // Calculate the gyro value into actual degrees per second
      gyro(0) = ((float)gyroCount[1])*gRes*pi/180.0; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
      gyro(1) = ((float)gyroCount[0])*gRes*pi/180.0; // - gyroBias[1];
      gyro(2) = -((float)gyroCount[2])*gRes*pi/180.0; // - gyroBias[2];

      acc_filt = K_acc*acc + b_acc;
      gyro_filt = gyro - b_gyro;
      //tempCount = mpu6050.readTempData();  // Read the x/y/z adc values
      //temperature = (tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
     }

  t_mag_cur=t.read_ms();

  if(t_mag_cur-t_mag_prev> 0.100f)
  {
    hmc5883l.readData(imu_data.val.mag);
    mag(0) = mag(0) < mag_val1 ?  mag(0) :  mag(0) - mag_val2;
    mag(1) = mag(1) < mag_val1 ? -mag(1) : -mag(1) + mag_val2;
    mag(2) = mag(2) < mag_val1 ? -mag(2) : -mag(2) + mag_val2;
    mag_filt = K_mag*mag+b_mag;
    t_mag_prev=t_mag_cur;
  }

  #ifdef _PRINT_IMU_
     pc.printf("mag=%f,%f,%f \r\n", mag(0), mag(1), mag(2));
     pc.printf("acc=%f,%f,%f \r\n", acc(0), acc(1), acc(2));
     pc.printf("gyro=%f,%f,%f\r\n", gyro(0), gyro(1), gyro(2));
  #endif

}

void gyro_bias_est()
{
  pc.printf("<------------------------------------>\r\n");
  pc.printf("   Bias Estimation Started   \r\n");
  int cnt =0, n_cnt =1000;
  while(cnt<n_cnt)
  {
    if(mpu6050.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01)
    {
        mpu6050.readGyroData(gyroCount);
        gyro(0) = (float)gyroCount[1]*gRes*pi/180.0;
        gyro(1) = (float)gyroCount[0]*gRes*pi/180.0;
        gyro(2) = -(float)gyroCount[2]*gRes*pi/180.0;

        b_gyro += gyro;
        cnt++;
    }
    wait_ms(10);
  }
  b_gyro/=n_cnt;

  pc.printf("b0=%f, b1=%f, b2=%f \r\n", b_gyro(0), b_gyro(1), b_gyro(2));
  pc.printf("   Bias Estimation Completed   \r\n");
  pc.printf("<------------------------------------>\r\n");

}

#ifdef IMU_CALIB
  #define n_calib 50
  #define n_iter 100

  void imu_calib()
  {
    pc.printf("<------------------------------------>\r\n");
    pc.printf("Starting IMU calibration \r\n");
    int cnt =0;

    Eigen::VectorXf ax(n_calib), ay(n_calib), az(n_calib);
    Eigen::VectorXf mx(n_calib), my(n_calib), mz(n_calib);
    ax.setZero();
    ay.setZero();
    az.setZero();
    mx.setZero();
    my.setZero();
    mz.setZero();

    // Store the accel data by averaging and mag data
    while(cnt<n_calib)
    {
      if(flag_calib==true)
      {
        for(int i=0; i<n_iter; i++)
        {
          update_imu();
          ax(cnt)+= acc(0);
          ay(cnt)+= acc(1);
          az(cnt)+= acc(2);
          wait_ms(DT*10);
        }
        mx(cnt)= imu_data.val.mag[0];
        my(cnt)= imu_data.val.mag[1];
        mz(cnt)= imu_data.val.mag[2];
        pc.printf("Calib => cnt = %d, max_cnt=%d \r\n", cnt, n_calib);

        if(cnt==0)
        {
          pc.printf("acc = %f, %f, %f \r\n", ax(0)/n_iter, ay(0)/n_iter, az(0)/n_iter);
          pc.printf("mag = %f, %f, %f \r\n", mx(0), my(0), mz(0));
        }
        flag_calib=false;
        cnt++;
      }
      else
        wait_ms(DT*100);
    }

    //pc.printf("Data acquisition for calibration completed \r\n");
    ax/=n_iter;
    ay/=n_iter;
    az/=n_iter;

    for(int i=0; i<n_calib; i++)
      pc.printf("%f,%f,%f,%f,%f,%f \r\n", ax(i), ay(i), az(i), mx(i), my(i), mz(i));

    exit(0);
  }
#endif
