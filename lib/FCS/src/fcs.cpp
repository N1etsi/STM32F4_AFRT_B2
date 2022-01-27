#include <fcs.hpp>
#include <Arduino.h>
#include <math.h>

using namespace fcs;

void fcs::pid(bool doHeightPID)
{
    static float spRoll, spPitch, spYaw;
    uint32_t throttle = fcs::rxin.chArr[3];
    static float errRoll = 0, memRoll = 0, lastRoll = 0;
    static float errPitch = 0, memPitch = 0, lastPitch = 0;
    static float errYaw = 0, memYaw = 0, lastYaw = 0;

    spRoll = (float)(fcs::rxin.chArr[1]-1500) / 15.0;
    spPitch = -1.0 * ((float)fcs::rxin.chArr[2]-1500) / 15.0;

    if (fabs(fcs::rxin.chArr[4]-1500) < 20);
    else
    {
        spYaw += ((float)fcs::rxin.chArr[4] - 1500) / 1500.0;
        if (spYaw > 180)
            spYaw -= 360.0;
        else if (spYaw < -180.0)
            spYaw += 360.0;

    }
    float roll = fcs::state.roll;
    float pitch = fcs::state.pitch;
    float yaw = fcs::state.yaw;

    int32_t outRoll = 0, outPitch = 0, outYaw = 0;

    //PID Roll
    errRoll = roll - spRoll;
    memRoll += errRoll;
    outRoll = rollP*errRoll + rollI*memRoll + rollD*(errRoll-lastRoll);
    lastRoll = errRoll;

    outRoll = min(outRoll, (int32_t)400);
    outRoll = max(outRoll, (int32_t)-400);

    //PID Pitch
    errPitch = pitch - spPitch;
    memPitch += errPitch;
    outPitch = pitchP*errPitch + pitchI*memPitch + pitchD*(errPitch-lastPitch);
    lastPitch = errPitch;

    outPitch = min(outPitch, (int32_t)400);
    outPitch = max(outPitch, (int32_t)-400);

    //PID Yaw
    errYaw = yaw - spYaw;
    memYaw += errYaw;
    outYaw = yawP*errYaw + yawI*memYaw + yawD*(errYaw-lastYaw);
    lastYaw = errYaw;

    outYaw = min(outYaw, (int32_t)400);
    outYaw = max(outYaw, (int32_t)-400);

    if(fcs::mode != SHUT)
    {
        fcs::mtout.escLFt = (uint32_t)min(max(throttle + outPitch - outRoll + outYaw, (uint32_t)1200), (uint32_t)1800);
        fcs::mtout.escRFt = (uint32_t)min(max(throttle + outPitch + outRoll - outYaw, (uint32_t)1200), (uint32_t)1800);
        fcs::mtout.escLBt = (uint32_t)min(max(throttle - outPitch - outRoll - outYaw, (uint32_t)1200), (uint32_t)1800);
        fcs::mtout.escRBt = (uint32_t)min(max(throttle - outPitch + outRoll + outYaw, (uint32_t)1200), (uint32_t)1800);
    }
    else{
        fcs::mtout.escRFt = 1000;
        fcs::mtout.escLBt = 1000;
        fcs::mtout.escLFt = 1000;
        fcs::mtout.escRBt = 1000;
    }

    if (doHeightPID) {
        /* TOOD: pid */
    }
}

void fcs::flight_mode()
{
    uint32_t control_channel = fcs::rxin.chArr[8];

    if (control_channel < 1200)
        fcs::mode = SHUT;
    
    else if (control_channel < 1700)
        fcs::mode = AIR;

    else if (control_channel < 2050)
        fcs::mode = ALT;

    else
        fcs::mode = SHUT;

}

