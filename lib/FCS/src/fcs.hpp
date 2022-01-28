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
    #define AIR 0
    #define ALT 1
    #define SHUT 255
    


    #define nCh 8

    //PID Const
    #define rollP 25.0
    #define pitchP rollP
    #define yawP 0
    #define distP 0


    #define rollI 0.00
    #define pitchI rollI
    #define yawI 0.0
    #define distI 0

    #define rollD 55.0
    #define pitchD rollD
    #define yawD 0.0
    #define distD 0


    typedef struct State {
        float yaw;
        float roll;
        float pitch;
        float dist;
        float alti; //from the pressure sensor
    }State;

    typedef struct RXIN {
        volatile uint32_t chArr[nCh + 1];
        int chCounter = 0;
    }RXIN;

    typedef struct MTOUT {
        uint32_t escLFt, escLBt, escRFt, escRBt;
    } MTOUT;

    inline State state = {0.0, 0.0, 0.0, 0.0, 0.0};
    inline RXIN rxin;
    inline MTOUT mtout = {0, 0, 0, 0};
    inline volatile byte mode = SHUT;
    inline volatile byte kill = false;
    inline volatile float read_dist;

    void pid(bool doHeightPID);
    void flight_mode();



}

#endif