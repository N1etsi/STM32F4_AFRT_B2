#ifndef TOF
#define TOF

#include <Arduino.h>
#include <Wire.h>


#include <fcs.hpp>
#include <VL53L0X.h>

namespace tof
{
    inline VL53L0X vl53l0x;

    //timed -> time between readings in miliseconds
    void setup(int timed);
    void read();

}

#endif
