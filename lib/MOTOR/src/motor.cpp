#include <motor.hpp>

using namespace mto;

void mto::setup()
{
    #define RB PA5
    #define LF PA8 
    #define RF PA9
    #define LB PA10
    

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
    
    MyTimLF->setPWM(channel1, LF, freq, 49);
    MyTimRF->setPWM(channel2, RF, freq, 49);
    MyTimLB->setPWM(channel3, LB, freq, 49);
    MyTimRB->setPWM(channel4, RB, freq, 49);
}

void mto::output()
{
     Serial.print("  Motor LF: ");
    Serial.print(fcs::mtout.escLFt);
    Serial.print("  Motor RF: ");
    Serial.print(fcs::mtout.escRFt);
    Serial.print("  Motor LB: ");
    Serial.print(fcs::mtout.escLBt); 
    Serial.print("  Motor RB: ");
    Serial.println(fcs::mtout.escRBt);
    
    if(fcs::mode != SHUT)
    {
        MyTimLF->setCaptureCompare(channel1, 160, MICROSEC_COMPARE_FORMAT);
        MyTimRF->setCaptureCompare(channel2, int(fcs::mtout.escRFt/8), MICROSEC_COMPARE_FORMAT);
        MyTimLB->setCaptureCompare(channel3, 160, MICROSEC_COMPARE_FORMAT);
        MyTimRB->setCaptureCompare(channel4, int(fcs::mtout.escRBt/8), MICROSEC_COMPARE_FORMAT);
    }
    else
    {
        MyTimLF->setCaptureCompare(channel1, 49, PERCENT_COMPARE_FORMAT);
        MyTimRF->setCaptureCompare(channel2, 49, PERCENT_COMPARE_FORMAT);
        MyTimLB->setCaptureCompare(channel3, 49, PERCENT_COMPARE_FORMAT);
        MyTimRB->setCaptureCompare(channel4, 49, PERCENT_COMPARE_FORMAT);
    }
    
    
}
