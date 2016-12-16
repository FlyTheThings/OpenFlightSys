#ifndef nav_h
#define nav_h

#include "math.h"
#include "Eigen.h"
#include "data_struct.h"

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
