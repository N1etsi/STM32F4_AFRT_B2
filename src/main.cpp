#include <Arduino.h>
#include <STM32TimerInterrupt.h>
#include <Wire.h>

#define DEBUG
#include <fcs.hpp>
#include <imu.hpp>
#include <tof.hpp>
#include <bmp.hpp>
#include <rxi.hpp>
#ifndef DEBUG
  #include <motor.hpp>
#endif

#define INTERRUPT_INTERVAL_US 1*1000 
#define SERIAL_BAUD 128000
#define DO_HEIGHT_PID true
#define DONT_DO_HEIGHT_PID false
#define LED1 2
void printSt();
int fcsSetup();


volatile int syncFlag0 = 0;
STM32Timer ITimer0(TIM4); // se for pra mudar o timer, mudar aqui

void TimerHandler0() {
    syncFlag0 = 1;
}
void sync(void) {
    while(!syncFlag0);
    syncFlag0 = 0;
}

void microCycle1(){
  fcs::pid(DO_HEIGHT_PID);
  #ifndef DEBUG

  mto::output();
#endif
  imu::getAHRS();

}

void microCycle2(){
  fcs::pid(DONT_DO_HEIGHT_PID);
  #ifndef DEBUG

  mto::output();
#endif
  tof::read();
  imu::getAHRS();

}

void microCycle3(){
  fcs::pid(DONT_DO_HEIGHT_PID);
  #ifndef DEBUG
    mto::output();
  #endif
  bmp::read();
  imu::getAHRS();

}

void setup() {
  //init sensors
  delay(3000);
  imu::setup();
  tof::setup(0);
  bmp::setup();
  rxi::setup();
  
  #ifdef DEBUG
    Serial.begin(SERIAL_BAUD);
  #else
    mto::setup();
  #endif

  Wire.setClock(1000000);
  ITimer0.attachInterruptInterval(INTERRUPT_INTERVAL_US, TimerHandler0); 

  delay(3000);

   while(!fcsSetup());
}


void loop() {
  static uint32_t t = 0, t1;

  t = micros();
  microCycle1(); 
  sync();
  microCycle2(); 
  sync();
  microCycle3();   
  printSt();
  sync();
  /*
  t1 = micros();
  Serial.print("T: ");
  Serial.print(t1-t);
  */

}

void printSt() {
  Serial.print(" ,Roll: ");
  Serial.print(fcs::state.roll);
  Serial.print("  ,Pitch: ");
  Serial.print(fcs::state.pitch);
  Serial.print("  ,Yaw: ");
  Serial.print(fcs::state.yaw);
  Serial.print("  ,Dist: ");
  Serial.print(fcs::state.dist);
  Serial.print("  ,Alt: ");
  Serial.print(fcs::state.alti);
  
  
  Serial.print("  ,Motor LF: ");
  Serial.print(fcs::mtout.escLFt);
  Serial.print("  ,Motor RF: ");
  Serial.print(fcs::mtout.escRFt);
  Serial.print(" ,Motor LB: ");
  Serial.print(fcs::mtout.escLBt); 
  Serial.print(" ,Motor RB: ");
  Serial.print(fcs::mtout.escRBt);
  Serial.print(" ,CH3: ");
  Serial.print(fcs::rxin.chArr[3]);
  Serial.print("  ,CH8: ");
  Serial.print(fcs::rxin.chArr[8]);
  Serial.print("  ,mode: ");
  Serial.print(fcs::mode);
  

  Serial.println();
}

int fcsSetup()
{
  for (int i=0; i<100; i++)
  {
    tof::read();
    imu::getAHRS();
    bmp::read();
  }

  return 1;

}