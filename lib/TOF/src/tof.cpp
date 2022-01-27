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
    
   fcs::state.dist = vl53l0x.readRangeContinuousMillimeters();
    
}