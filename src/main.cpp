#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "fcs_definiton.hpp"
#include "io_definition.hpp"
#include "mpu6050.hpp"

#define usCyc 4000

void setup() {
  //

  //Sensor Start up
  Wire.setClock(400000);
  Wire.begin();

  setupMPU(usCyc);
}

void loop() {
  // put your main code here, to run repeatedly:
}