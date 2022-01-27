#ifndef __BMP__
#define __BMP__

#include <Arduino.h>
#include <Wire.h>
#include <BMP280_DEV.h> 


#include <fcs.hpp>

namespace bmp
{
    void setup();
    void read();

}

#endif