#include <Arduino.h>
#include <STM32TimerInterrupt.h>
#include <Wire.h>

#define DEBUGs
#include <fcs.hpp>
#include <imu.hpp>
#include <tof.hpp>
#include <bmp.hpp>
#include <rxi.hpp>
#ifndef DEBUG
  #include <motor.hpp>
#endif

#define INTERRUPT_INTERVAL_US 1*1000 // half sec
#define SERIAL_BAUD 115200
#define DO_HEIGHT_PID true
#define DONT_DO_HEIGHT_PID false
#define LED1 2
void printSt();


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
  mto::output();
  imu::getAHRS();

}

void microCycle2(){
  fcs::pid(DONT_DO_HEIGHT_PID);
  mto::output();
  tof::read();
  imu::getAHRS();

}

void microCycle3(){
  fcs::pid(DONT_DO_HEIGHT_PID);
  mto::output();
  bmp::read();
  imu::getAHRS();

}

void setup() {
  //init sensors
  imu::setup();
  tof::setup(0);
  bmp::setup();
  
  #ifdef DEBUG
    Serial.begin(SERIAL_BAUD);
  #else
    mto::setup();
  #endif

  rxi::setup();

  Wire.setClock(1000000);
  ITimer0.attachInterruptInterval(INTERRUPT_INTERVAL_US, TimerHandler0); 

  delay(5000);
}

// void loopaaa() {
//   // put your main code here, to run repeatedly:

//   static uint32_t t, t2, t3, t4, t5;
//   t = micros();
//   imu::getAHRS();
//   t2 = micros() - t;
//   tof::read();
//   t3 = micros() - t2 - t;
//   bmp::read();
//   t4 = micros() - t3 - t2 - t;
//   fcs::pid(DO_HEIGHT_PID);
//   t5 =micros() - t4 - t3 - t2 - t; 

//   #ifndef DEBUGS
//     mto::output();
//   #else
//     printSt();
//   #endif
// }

void loop() {
  static uint32_t t, t1, t2, t3, t4, t5;
  t = micros();
  microCycle1(); 
  t1 = micros();
  sync();
  t2 = micros();
  microCycle2(); 
  t3 = micros();
  sync();
  t4 = micros();
  microCycle3(); 
  t5 = micros();
  sync();

  Serial.print(t1-t);
  Serial.print("  ");
  Serial.print(t3-t2);
  Serial.print("  ");
  Serial.print(t5-t4);
  Serial.print("\n");
}

void printSt() {
  Serial.print("Roll: ");
  Serial.print(fcs::state.roll);
  Serial.print("  Pitch: ");
  Serial.print(fcs::state.pitch);
  Serial.print("  Yaw: ");
  Serial.print(fcs::state.yaw);
  Serial.print("  Dist: ");
  Serial.print(fcs::state.dist);
  Serial.print("  Alt: ");
  Serial.print(fcs::state.alti);
  
  Serial.print("  Motor LF: ");
  Serial.print(fcs::mtout.escLFt);
  Serial.print("  Motor RF: ");
  Serial.print(fcs::mtout.escRFt);
  Serial.print("  Motor LB: ");
  Serial.print(fcs::mtout.escLBt); 
  Serial.print("  Motor RB: ");
  Serial.print(fcs::mtout.escRBt);
  Serial.print("  CH3: ");
  Serial.print(fcs::rxin.chArr[3]);
  Serial.print("  CH8: ");
  Serial.print(fcs::rxin.chArr[8]);
  Serial.print("  mode: ");
  Serial.print(fcs::mode);

  Serial.println();
}