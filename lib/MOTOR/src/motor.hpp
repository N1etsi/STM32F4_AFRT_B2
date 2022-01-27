#ifndef __MTOUT__
#define __MTOUT__


#include <Arduino.h>
#include <fcs.hpp>

namespace mto {
    #define escLF PA8
    #define escLB PA9
    #define escRF PA10
    #define escRB PA11

    inline uint32_t channel1, channel2, channel3, channel4;
    inline HardwareTimer *MyTimLF, *MyTimRF, *MyTimLB, *MyTimRB;

    void setup();
    void output();
    void outputKill();

}



#endif