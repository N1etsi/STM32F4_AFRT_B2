#include <tof.hpp>

using namespace tof;

void tof::setup(int timed)
{
    vl53l0x.setTimeout(500);
    vl53l0x.init();
    vl53l0x.startContinuous();

}

void tof::read()
{
    //TODO compensate for drone attitude
    static int count=0;

    fcs::read_dist = vl53l0x.readRangeContinuousMillimeters();

    if (fcs::read_dist!=0){
        fcs::state.dist = fcs::read_dist * sin(sqrt(sq(fcs::state.roll) + sq(fcs::state.pitch)));
        count=0;
    }else
        count++;

    if (count>200)
        fcs::kill = true;
    
}