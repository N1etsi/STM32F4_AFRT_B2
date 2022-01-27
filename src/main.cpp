#include <Arduino.h>

#define DEBUGs



#include <fcs.hpp>
#include <imu.hpp>
#include <tof.hpp>
#include <bmp.hpp>
#include <rxi.hpp>
#ifndef DEBUG
  #include <motor.hpp>
#endif


void printSt();


void setup() {
  //init sensors
  imu::setup();
  tof::setup(0);
  bmp::setup();
  
  #ifdef DEBUG
    Serial.begin(115200);
  #else
    mto::setup();
  #endif

  rxi::setup();

  Wire.setClock(1000000);
  delay(5000); 

}

void loop() {
  // put your main code here, to run repeatedly:

  static uint32_t t, t2, t3, t4, t5;
  t = micros();
  imu::getAHRS();
  t2 = micros() - t;
  tof::read();
  t3 = micros() - t2 - t;
  bmp::read();
  t4 = micros() - t3 - t2 - t;
  fcs::pid();
  t5 =micros() - t4 - t3 - t2 - t; 

  #ifndef DEBUGS
    mto::output();
  #else
    printSt();
  #endif


  
}

void printSt()
{
  
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