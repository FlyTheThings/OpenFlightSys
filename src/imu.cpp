
#include "imu.h"
#include "Eigen.h"
#include "MPU6050.h"
#include "HMC5883L.h"

#define pi 3.14159265359
#define mag_val1 32767.0f*0.92f
#define mag_val2 65535.0f*0.92f


MPU6050 mpu6050;
HMC5883L hmc5883l(I2C_SDA, I2C_SCL);

Eigen::Matrix3f K_acc;
Eigen::Vector3f b_acc, b_gyro;

int t_mag_cur, t_mag_prev;

bool flag_calib=false;

void init_imu()
{
  //Set Accel Scale factor and bias
  K_acc.setIdentity();
  b_acc << 0,0,0;

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

  //get_gyro_bias_est();

}

void update_imu()
{
  if(mpu6050.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
      mpu6050.readAccelData(accelCount);  // Read the x/y/z adc values
      // Now we'll calculate the accleration value into actual g's
      acc(0) = ((float)accelCount[0])*aRes;  // get actual g value, this depends on scale being set
      acc(1) = ((float)accelCount[1])*aRes;
      acc(2) = ((float)accelCount[2])*aRes;

      mpu6050.readGyroData(gyroCount);  // Read the x/y/z adc values
      // Calculate the gyro value into actual degrees per second
      gyro(0) = ((float)gyroCount[0])*gRes*pi/180.0; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
      gyro(1) = ((float)gyroCount[1])*gRes*pi/180.0; // - gyroBias[1];
      gyro(2) = ((float)gyroCount[2])*gRes*pi/180.0; // - gyroBias[2];

      acc_filt = K_acc*acc + b_acc;
      gyro_filt = gyro - b_gyro;
      //tempCount = mpu6050.readTempData();  // Read the x/y/z adc values
      //temperature = (tempCount) / 340. + 36.53; // Temperature in degrees Centigrade

      #ifdef _PRINT_GPS_
        pc.printf("flat=%f, flon=%f, falt=%f \r\n", nav_data.val.flat, nav_data.val.flon, nav_data.val.falt);
      #endif
     }

  t_mag_cur=t.read_ms();

  if(t_mag_cur-t_mag_prev> 0.100f)
  {
    hmc5883l.readData(imu_data.val.mag);
    mag(0) = mag(0) < mag_val1 ? mag(0) : mag(0) - mag_val2;
    mag(1) = mag(1) < mag_val1 ? mag(1) : mag(1) - mag_val2;
    mag(2) = mag(2) < mag_val1 ? mag(2) : mag(2) - mag_val2;
    t_mag_prev=t_mag_cur;
  }

  #ifdef _PRINT_IMU_
     pc.printf("mag=%f,%f,%f \r\n", mag(0), mag(1), mag(2));
     pc.printf("acc=%f,%f,%f \r\n", acc(0), acc(1), acc(2));
     pc.printf("gyro=%f,%f,%f\r\n", gyro(0), gyro(1), gyro(2));
  #endif
}

void get_gyro_bias_est()
{
  pc.printf("<------------------------------------>\r\n");
  pc.printf("   Bias Estimation Started   \r\n");
  int cnt =0, n_cnt =100;
  while(cnt<n_cnt)
  {
    if(mpu6050.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01)
    {
        mpu6050.readGyroData(gyroCount);
        gyro(0) = (float)gyroCount[0]*gRes*pi/180.0;
        gyro(1) = (float)gyroCount[1]*gRes*pi/180.0;
        gyro(2) = (float)gyroCount[2]*gRes*pi/180.0;

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

    // Store the mag data
    while(cnt<n_calib)
    {
      if(flag_calib==true)
      {
        update_imu();
        mx(cnt)= imu_data.val.mag[0];
        my(cnt)= imu_data.val.mag[1];
        mz(cnt)= imu_data.val.mag[2];
        wait_ms(DT*100);
        pc.printf("mag => cnt = %d, max_cnt=%d \r\n", cnt, n_calib);
        flag_calib=false;
        cnt++;
      }
      else
        wait_ms(DT*100);
    }

    for(int i=0; i<n_calib; i++)
      pc.printf("%f,%f,%f \r\n", mx(i), my(i), mz(i));

    cnt=0;
    // Store the accel data by averaging
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
        pc.printf("acc => cnt = %d, max_cnt=%d \r\n", cnt, n_calib);
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
    mx/=n_iter;
    my/=n_iter;
    mz/=n_iter;

    for(int i=0; i<n_calib; i++)
      pc.printf("%f,%f,%f,%f,%f,%f \r\n", ax(i), ay(i), az(i), mx(i), my(i), mz(i));

    /*
    pc.printf("1 \r\n");
    // Use a linear ellipsoid fit to get calibration parameters
    // Accelerometer calibration
    Eigen::Matrix<float, n_calib, 9> D;

    D.col(0) = ax.array()*ax.array();
    D.col(1) = ay.array()*ay.array();
    D.col(2) = az.array()*az.array();
    D.col(3) = 2 * ax.array()*ay.array();
    D.col(4) = 2 * ax.array()*az.array();
    D.col(5) = 2 * ay.array()*az.array();
    D.col(6) = 2 * ax;
    D.col(7) = 2 * ay;
    D.col(8) = 2 * az;

    pc.printf("2 \r\n");

    Eigen::VectorXf onerow(n_calib);
    Eigen::MatrixXf v = (D.transpose()*D).inverse() * (D.transpose()*onerow);
    Eigen::Matrix4f A, T, R;
    Eigen::Vector3f cen, val;
    onerow.setOnes();
    T.setIdentity();
    pc.printf("3 \r\n");

    A << v(0), v(3), v(4), v(6),
       v(3), v(1), v(5), v(7),
       v(4), v(5), v(2), v(8),
       v(6), v(7), v(8), -1;

    val << v(6), v(7), v(8);
    cen = -A.block(0, 0, 3, 3).inverse() * val;

    pc.printf("4 \r\n");

    T.row(3) << cen(0), cen(1), cen(2), 1;

    R = T* A * T.transpose();

    Eigen::Matrix3f matA = - R.block(0, 0, 3, 3) / R(3, 3);
    Eigen::Matrix3f Ksqr_est = matA* pow(9.81, 2);
    Eigen::Matrix3f k_star_posdef = (Ksqr_est + Eigen::Matrix3f::Identity()) / 2;
    Eigen::Vector3f bias = -k_star_posdef*cen;
    pc.printf("5 \r\n");

    pc.printf("<------------------------------------>\r\n");
    pc.printf("acc bias = %f, %f, %f \r\n", bias(0), bias(1), bias(2));
    pc.printf("acc scale = %f, %f, %f \r\n %f, %f, %f \r\n %f, %f, %f \r\n \r\n",
               k_star_posdef(0,0),k_star_posdef(0,1),k_star_posdef(0,2),
               k_star_posdef(1,0),k_star_posdef(1,1),k_star_posdef(1,2),
               k_star_posdef(2,0),k_star_posdef(2,1),k_star_posdef(2,2));


    // Use a linear ellipsoid fit to get calibration parameters
    // Magnetometer calibration

    D.col(0) = mx.array()*mx.array();
    D.col(1) = my.array()*my.array();
    D.col(2) = mz.array()*mz.array();
    D.col(3) = 2 * mx.array()*my.array();
    D.col(4) = 2 * mx.array()*mz.array();
    D.col(5) = 2 * my.array()*mz.array();
    D.col(6) = 2 * mx;
    D.col(7) = 2 * my;
    D.col(8) = 2 * mz;

    v = (D.transpose()*D).inverse() * (D.transpose()*onerow);
    T.setIdentity();

    A << v(0), v(3), v(4), v(6),
       v(3), v(1), v(5), v(7),
       v(4), v(5), v(2), v(8),
       v(6), v(7), v(8), -1;

    val << v(6), v(7), v(8);
    cen = -A.block(0, 0, 3, 3).inverse() * val;

    T.row(3) << cen(0), cen(1), cen(2), 1;

    R = T* A * T.transpose();
    matA = - R.block(0, 0, 3, 3) / R(3, 3);
    Ksqr_est = matA* pow(9.81, 2);
    k_star_posdef = (Ksqr_est + Eigen::Matrix3f::Identity()) / 2;
    bias = -k_star_posdef*cen;

    pc.printf("mag bias = %f, %f, %f \r\n", bias(0), bias(1), bias(2));
    pc.printf("mag scale = %f, %f, %f \r\n, %f, %f, %f \r\n, %f, %f, %f \r\n \r\n",
               k_star_posdef(0,0),k_star_posdef(0,1),k_star_posdef(0,2),
               k_star_posdef(1,0),k_star_posdef(1,1),k_star_posdef(1,2),
               k_star_posdef(2,0),k_star_posdef(2,1),k_star_posdef(2,2));

    pc.printf("<------------------------------------>\r\n");
    */
  }
#endif
