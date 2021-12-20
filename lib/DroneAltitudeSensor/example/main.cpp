#include <Arduino.h>
#include <DroneAltitudeSensor.hpp>
#include <Wire.h>

#define SERIAL_BAUD 115200

void setup() {
  // put your setup code here, to run once:
  Serial.begin(SERIAL_BAUD);
  while(!Serial); // Wait

  Wire.setClock(400000);
  Wire.begin();

  beginAltitude();
}

void loop() {
  float altitude = getAltitude();
  
  Serial.println(altitude);
  delayMicroseconds(300);
}