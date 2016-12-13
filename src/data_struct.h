 #ifndef data_struct_h
 #define data_struct_h

 #define _DEBUG_
 #define dt 0.010f
 #define DT 10

 #include "mbed.h"
 #include "Eigen.h"
 #include <Geometry>

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
   char dummy[4];
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

#ifdef _DEBUG_
extern float phi, theta, psi;
#endif

// Declare Identity Matrices
extern Eigen::Matrix3f I3;
extern Eigen::Matrix4f I4;

// Initialization function for data structures
void init_data_struct();

void serial_transmit();

#endif
