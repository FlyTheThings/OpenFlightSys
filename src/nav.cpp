#include "nav.h"

Eigen::Matrix3f K_pos, P_pos, Q_pos, R_pos;
Eigen::Matrix3f K_ang, P_ang, Q_ang, R_ang;

#ifdef _DEBUG_
  float phi, theta, psi;
#endif

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
  quat += Omega*q_old*dt;
  quat/=quat.norm();

  // Time Update of Position
  Eigen::Vector3f vel_old(vel);
  vel = vel_old + dt*acc_filt;

  Eigen::Vector3f pos_old(pos);
  pos = pos_old + dt*pos;

  #ifdef _DEBUG_
    phi = atan2( 2*(-quat(0)*quat(1)+ quat(2)*quat(3)), 1- 2*quat(1)*quat(1) -2*quat(2)*quat(2) )*180.0f/3.142f;
    theta = asin(-2*(quat(0)*quat(2)+quat(1)*quat(3)))*180.0f/3.142f;
    psi = atan2( 2*(-quat(0)*quat(3)+ quat(2)*quat(1)), 1- 2*quat(3)*quat(3) -2*quat(2)*quat(2) )*180.0f/3.142f;
  #endif

  // Measurement Update of Orientation


  // Measurement Update of Position

}

Eigen::Matrix3f getRotMat()
{
  return Eigen::Quaternionf(quat(0), quat(1), quat(2), quat(3)).toRotationMatrix();
}
