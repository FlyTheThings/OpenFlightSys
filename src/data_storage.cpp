#include "data_storage.h"
#include "data_struct.h"
#include "radio.h"

SDFileSystem sd(SPI_MOSI, SPI_MISO, SPI_SCK, D12, "sd");

FILE *fp;

void init_sd_card_file()
{
  #ifdef _DEBUG_
        radio.printf("<------------------------------------>\r\n");
        radio.printf("          Initializing SD CARD        \r\n");
  #endif

        mkdir("/sd/FlightData", 0777);

        fp = fopen("/sd/FlightData/data.txt", "w");

        if(fp == NULL)
        {
    #ifdef _DEBUG_
                radio.printf("Failed to initialize File on SD card \r\n");
    #endif
        }
  #ifdef _DEBUG_
        radio.printf("          Initialized SD CARD         \r\n");
        radio.printf("<------------------------------------>\r\n");
  #endif
}

char ch[500];
void sd_write_nav_data()
{
        int n = sprintf(ch, "{\"t\": %d, \"q\": [%f, %f, %f, %f], \"flat\": %f, \"flon\": %f, \"falt\": %f, \"pos\": [%f, %f, %f], \"vel\": [%f, %f, %f], \"radar_dist\":%d } \r\n", nav_data.val.t, nav_data.val.q[0], nav_data.val.q[1], nav_data.val.q[2], nav_data.val.q[3], nav_data.val.flat, nav_data.val.flon, nav_data.val.falt, nav_data.val.pos[0], nav_data.val.pos[1], nav_data.val.pos[2], nav_data.val.vel[0], nav_data.val.vel[1], nav_data.val.vel[1], nav_data.val.radar_dist);
        fprintf(fp, ch, n);
}

void sd_write_imu_data()
{
        int n = sprintf(ch, "{\"t\": %d, \"acc\": [%f, %f, %f], \"gyro\": [%f, %f, %f], \"mag\": [%f, %f, %f], \"acc_filt\": [%f, %f, %f], \"gyro_filt\": [%f, %f, %f], \"mag_filt\": [%f, %f, %f]} \r\n", nav_data.val.t, imu_data.val.acc[0], imu_data.val.acc[1],
                        imu_data.val.acc[2], imu_data.val.gyro[0], imu_data.val.gyro[1], imu_data.val.gyro[2], imu_data.val.mag[0], imu_data.val.mag[1], imu_data.val.mag[2],  imu_data.val.acc_filt[0], imu_data.val.acc_filt[1], imu_data.val.acc_filt[2],
                        imu_data.val.gyro_filt[0], imu_data.val.gyro_filt[1], imu_data.val.gyro_filt[2], imu_data.val.mag_filt[0], imu_data.val.mag_filt[1], imu_data.val.mag_filt[2]);
        fprintf(fp, ch, n);
}
