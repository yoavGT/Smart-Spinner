#ifndef JYRO_H
#define JYRO_H

#include <Arduino.h>

void jyro_init();
void jyro_read_xyz_dps(float &gx_dps, float &gy_dps, float &gz_dps);

float jyro_get_spin_dps();
float jyro_get_wobble_dps();
float jyro_spin_dps_to_rpm(float spin_dps);

#endif