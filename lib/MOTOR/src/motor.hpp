#ifndef MTOUT
#define MTOUT


#include <Arduino.h>
#include <fcs.hpp>

namespace mtout {
    #define escLF PA8
    #define escLB PA9
    #define escRF PA10
    #define escRB PA11

    inline uint32_t channel1, channel2, channel3, channel4;
    HardwareTimer *MyTimLF, *MyTimRF, *MyTimLB, *MyTimRB;

    void setup();
    void output();

}



#endif