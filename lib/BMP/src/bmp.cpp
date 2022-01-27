#include <bmp.hpp>

using namespace bmp;
void bmp::setup()
{
    delay(100);
}

void bmp::read()
{
    delayMicroseconds(50);
    fcs::state.alti=70;
}