#ifndef RXI
#define RXI

#include <Arduino.h>
#include <fcs.hpp>

namespace rxi{
    // Pin Decl
    #define rx PB4
    // Var decl
    inline HardwareTimer *rx_tim;
    inline uint32_t channel;
    

    void setup();
    void capture();


}



#endif