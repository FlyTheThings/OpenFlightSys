#include "gps.h"
#include "data_struct.h"

#define __TinyGPS

#include "TinyGPS.h"
#include "nmea.h"

Serial gps(PA_9, PA_10);

bool flg=true;

#ifdef __NGPS
  NMEA ngps(ALL);
#endif

#ifdef __TinyGPS
  TinyGPS tgps;
  unsigned long age;
#endif

char c;
#ifdef __TinyGPS
  double flat, flon, falt;
#endif

void init_gps()
{
  #ifdef _DEBUG_
    pc.printf("<------------------------------------>\r\n");
    pc.printf("            Initializing GPS          \r\n");
  #endif

  gps.baud(38400);
  gps.attach(&update_gps);

  #ifdef _DEBUG_
    pc.printf("            Initialized GPS          \r\n");
    pc.printf("<------------------------------------>\r\n");
  #endif
}

void update_gps()
{
      c=gps.getc();

      #ifdef __NGPS
        ngps.decode(c);
        nav_data.val.flat = ngps.gprmc_latitude();
        nav_data.val.flon = ngps.gprmc_longitude();
      #endif

      //pc.putc(c);
      #ifdef __TinyGPS
        if(tgps.encode(c))
        {
        tgps.f_get_position(&flat, &flon, &age);

        nav_data.val.flat = (float) flat;
        nav_data.val.flon = (float) flon;

        nav_data.val.falt = (float) tgps.f_altitude();
        }
      #endif
}
