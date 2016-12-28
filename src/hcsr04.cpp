#include "mbed.h"
#include "data_struct.h"
#include "ultrasonic.h"

void radar_callback(int dist)
{
  nav_data.val.radar_dist =dist;
}
ultrasonic radar(PA_0, PA_1, 0.1, 1, &radar_callback);

void init_radar()
{
  radar.startUpdates();
}

void update_radar()
{
  radar.checkDistance();
}
