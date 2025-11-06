#ifndef DRONE_SENSORS_H
#define DRONE_SENSORS_H

#include "drone.h"

void accelerometerMeasurement(DRONE_T* );
void gyroscopeMeasurement(DRONE_T* );
void GNSSMeasurement_position(DRONE_T* );
void GNSSMeasurement_velocity(DRONE_T* );

#endif
