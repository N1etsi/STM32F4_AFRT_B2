#include <DroneAltitudeSensor.hpp>

EnvironmentCalculations::AltitudeUnit envAltUnit  =  EnvironmentCalculations::AltitudeUnit_Meters;
EnvironmentCalculations::TempUnit     envTempUnit =  EnvironmentCalculations::TempUnit_Celsius;

float referencePressure = 1013.25;  // NTP (hPa)
float outdoorTemp = 15;             // °C local (avg)

// mess with this
BME280I2C::Settings settings(
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::Mode_Forced,
   BME280::StandbyTime_1000ms,
   BME280::Filter_16,
   BME280::SpiEnable_False,
   BME280I2C::I2CAddr_0x76
);

BME280I2C bme(settings);

float getAltitude() {
   return 
   EnvironmentCalculations::Altitude(
      bme.pres(),
      envAltUnit,
      referencePressure,
      outdoorTemp,
      envTempUnit
      );
}

void beginAltitude() {
   bme.begin();
}