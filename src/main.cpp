#include "mbed.h"
#include "rtos.h"
#include "data_struct.h"
#include "nav.h"
#include "gps.h"
#include "imu.h"
#include "radio.h"
#include "hcsr04.h"

int main()
{
  // Initialize Serial Output for Debugging
  pc.baud(115200);
  pc.attach(&serial_callback);

  // Initialize Data Sturctures for AHRS and Radio transmission
  init_data_struct();

  // Initialize and start IMU => MPU6050 + HMC5883L
  init_imu();

  // Initialize Navigation (AHRS) module
  init_nav();

  // Initialize RADAR module => HC-SR04
  init_radar();

  // Initialize GPS module => Ublox Neo 8m
  //init_gps();

  // Initialize Radio module => NRF24L01
  //init_radio();

  #ifdef IMU_CALIB
    t.start();
    //Function for IMU Calibration
    imu_calib();
    t.stop();
    t.reset();
  #endif

   // Wait for user to start program
   while(1)
   {
     if(start_flag == true)
      break;
     wait_ms(100);
   }
   wait(2);

   t.start();

   int icnt=0;
   pc.printf("Outside while loop \r\n");
   while(1)
   {
     nav_data.val.t = t.read_ms();
     update_imu();
     update_radar();
     run_ekf();
     //serial_transmit();
     //transmit_radio();

     wait_ms(DT);
     icnt++;


     if(icnt%100==0)
     {

       pc.printf("radar_dist=%d \r\n", nav_data.val.radar_dist);
      //pc.printf("norm=%f \r\n", acc_filt.norm());
      //pc.printf("phi=%f, theta=%f, psi=%f \r\n", phi, theta, psi);

      //pc.printf("magx=%f, magy=%f, magz=%f \r\n", mag(0), mag(1), mag(2));
      //pc.printf("<------------------------------------>\r\n");
      //pc.printf("lat=%f, lon=%f, alt=%f \r\n", nav_data.val.flat, nav_data.val.flon, nav_data.val.falt);
      //pc.printf("lat=%f, lon=%f, alt=%f \r\n", nav_data.val.flat, nav_data.val.flon, nav_data.val.falt);
      //pc.printf("<------------------------------------>\r\n");

      //pc.printf("g0=%f, g1=%f, g2=%f \r\n", gyro_filt(0), gyro_filt(1), gyro_filta(2));
     }


   }

}
