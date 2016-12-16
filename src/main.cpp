
#include "mbed.h"
#include "rtos.h"
#include "data_struct.h"
#include "nav.h"
#include "gps.h"
#include "imu.h"
#include "servo.h"
#include "radio.h"


Serial pc(USBTX, USBRX); // tx, rx
DigitalOut led_Red(LED1);

bool start_flag=false;

void close_main_callback()
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

int main()
{

  // Initialize Serial Output for Debugging
  pc.baud(115200);
  pc.attach(&close_main_callback);

  // Initialize Data Sturctures for AHRS and Radio transmission
  init_data_struct();

  // Initialize and start IMU => MPU6050 + HMC5883L
  init_imu();

  // Get readings from INU for initial estimation
  update_imu();

  // Get gyro bias estimate from averaging
  get_gyro_bias_est();

  // Initialize Navigation (AHRS) module
  init_nav();

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

  //Thread gps_thread(thread_gps);

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
     run_ekf();
     //serial_transmit();
     //transmit_radio();

     //pc.printf("icnt=%d \r\n", icnt);
     wait_ms(DT);
     icnt++;

     /*
     if(icnt%100==0)
     {
      //pc.printf("phi=%f, theta=%f, psi=%f \r\n", phi, theta, psi);

      pc.printf("magx=%f, magy=%f, magz=%f \r\n", mag(0), mag(1), mag(2));
      //pc.printf("<------------------------------------>\r\n");
      //pc.printf("lat=%f, lon=%f, alt=%f \r\n", nav_data.val.flat, nav_data.val.flon, nav_data.val.falt);
      //pc.printf("lat=%f, lon=%f, alt=%f \r\n", nav_data.val.flat, nav_data.val.flon, nav_data.val.falt);
      //pc.printf("<------------------------------------>\r\n");

      //pc.printf("g0=%f, g1=%f, g2=%f \r\n", gyro_filt(0), gyro_filt(1), gyro_filta(2));
    }
    */

   }

}
