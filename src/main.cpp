#ifndef Arduino
#include <Arduino.h>
#endif

#include <fcs.hpp>
#include <imu.hpp>
#include <tof.hpp>
#include <rxi.hpp>
//#include <motor.hpp>

void printSt();


void setup() {
  //init sensors
  imu::setup();
  tof::setup(20);
  rxi::setup();
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:

  imu::getAHRS();
  tof::read();

  printSt();
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
  Serial.print("  Motor LF: ");
  Serial.print(fcs::mtout.escLFt);
  Serial.print("  Motor RF: ");
  Serial.print(fcs::mtout.escRFt);
  Serial.print("  Motor LB: ");
  Serial.print(fcs::mtout.escLBt); 
  Serial.print("  Motor RB: ");
  Serial.println(fcs::mtout.escRBt);


}