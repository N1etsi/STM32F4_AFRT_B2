#include <Arduino.h>
#include <LSM9DS0.hpp>

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(115200);
  Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");
  
  beginLsm();
  displaySensorDetails();
  configureSensor();

  Serial.println("");
}

void loop(void) 
{  
  /* Get a new sensor event */ 
  sensors_event_t accel, mag, gyro, temp;

  getLsmEvents(&accel, &mag, &gyro, &temp); 

  // print out accelleration data
  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(accel.acceleration.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(accel.acceleration.z);     Serial.println("  \tm/s^2");

  // print out magnetometer data
  Serial.print("Magn. X: "); Serial.print(mag.magnetic.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(mag.magnetic.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(mag.magnetic.z);     Serial.println("  \tuT");
  
  // print out gyroscopic data
  Serial.print("Gyro  X: "); Serial.print(gyro.gyro.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(gyro.gyro.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(gyro.gyro.z);     Serial.println("  \trad/s");

  // print out temperature data
  Serial.print("Temp: "); Serial.print(temp.temperature); Serial.println(" *C");

  Serial.println("**********************\n");

  delay(250);
}