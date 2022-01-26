#ifndef FCS
#define FCS

#include <Arduino.h>

namespace fcs
{
    //Indexes
    #define X 0
    #define Y 1
    #define Z 2
    #define Ro 0
    #define Pi 1
    #define Ya 2

    //MODES
    #define STDBY -10
    #define GND 0
    #define AIR 1
    #define SHUT -5


    #define nCh 8

    //PID Const
    #define rollP 1.3
    #define pitchP 1.3
    #define yawP 0.2


    #define rollI 0.05
    #define pitchI 0.05
    #define yawI 0

    #define rollD 15.0
    #define pitchD 15.0
    #define yawD 5.0

    typedef struct State {
        float yaw;
        float roll;
        float pitch;
        float dist;
    }State;

    typedef struct RXIN {
        volatile uint32_t chArr[nCh + 1];
        int chCounter = 0;
    }RXIN;

    typedef struct MTOUT {
        uint32_t escLFt, escLBt, escRFt, escRBt;
    } MTOUT;

    inline State state = {0.0, 0.0, 0.0};
    inline RXIN rxin;
    inline MTOUT mtout;

    void pid();



}

#endif