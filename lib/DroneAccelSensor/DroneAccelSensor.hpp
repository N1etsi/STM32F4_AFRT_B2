#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>

void displaySensorDetails(void);
void beginLsm(void);
void configureSensor(void);
void getLsmEvents(sensors_event_t *accel, sensors_event_t *mag, sensors_event_t *gyro, sensors_event_t *temp);

