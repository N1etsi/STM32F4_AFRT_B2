#ifndef ALTITUDE_SENSOR_ 
#define ALTITUDE_SENSOR_ 

#include <BME280I2C.h>
#include <EnvironmentCalculations.h>

void beginAltitude();
float getAltitude();

#endif