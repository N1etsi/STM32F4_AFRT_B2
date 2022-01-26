#include <motor.hpp>

using namespace mtout;

void mtout::setup()
{
    #define LF PA8 
    #define RF PA9
    #define LB PA10
    #define RB PA11

    TIM_TypeDef *Instance1 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(LF), PinMap_PWM);
    channel1 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(LF), PinMap_PWM));

    TIM_TypeDef *Instance2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(RF), PinMap_PWM);
    channel2 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(RF), PinMap_PWM));

    TIM_TypeDef *Instance3 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(LB), PinMap_PWM);
    channel3 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(LB), PinMap_PWM));

    TIM_TypeDef *Instance4 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(RB), PinMap_PWM);
    channel4 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(RB), PinMap_PWM));

    MyTimLF = new HardwareTimer(Instance1);
    MyTimRF = new HardwareTimer(Instance2);
    MyTimLB = new HardwareTimer(Instance3);
    MyTimRB = new HardwareTimer(Instance4);

    float freq = 4000;
    
    MyTimLF->setPWM(channel1, LF, freq, 45);
    MyTimRF->setPWM(channel2, RF, freq, 45);
    MyTimLB->setPWM(channel3, LB, freq, 45);
    MyTimRB->setPWM(channel4, RB, freq, 45);
}

void mtout::output()
{
    MyTimLF->setCaptureCompare(channel1, fcs::mtout.escLFt);
    MyTimRF->setCaptureCompare(channel2, fcs::mtout.escRFt);
    MyTimLB->setCaptureCompare(channel3, fcs::mtout.escLBt);
    MyTimRB->setCaptureCompare(channel4, fcs::mtout.escRBt);
    
}