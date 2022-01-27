#include <rxi.hpp>

using namespace rxi;

void rxi::setup()
{
    // RX PIN TIM SETUP
    TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(rx), PinMap_PWM);
    channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(rx), PinMap_PWM));

    rx_tim = new HardwareTimer(Instance);
    rx_tim->setMode(channel, TIMER_INPUT_CAPTURE_RISING, rx);

    uint32_t PrescalerFactor = 84;
    rx_tim->setPrescaleFactor(PrescalerFactor);
    rx_tim->setOverflow(0x10000); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
    rx_tim->attachInterrupt(channel, rxi::capture);
    rx_tim->resume();

}

void rxi::capture(void)
{
    static volatile uint32_t rxCurr, rxPrev = 0, rxCH, chCounter=0;
 
    rxCurr = rx_tim->getCaptureCompare(channel);
    if (rxCurr > rxPrev)
        rxCH = rxCurr - rxPrev;
    else if (rxCurr <= rxPrev)
        rxCH = 0x10000 + rxCurr - rxPrev;
    rxPrev = rxCurr;
    if (rxCH > 3000)
        chCounter = 0;
    fcs::rxin.chArr[chCounter] = rxCH;

    if (chCounter == 8)
        fcs::flight_mode();


    chCounter++;

}