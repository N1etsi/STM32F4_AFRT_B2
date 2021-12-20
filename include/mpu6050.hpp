#ifndef MPU6050
#define MPU6050

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "fcs_definiton.hpp"
#include "io_definition.hpp"

//i2c address
#define MPU_ADDR 0x68

//data def
struct euler{
    float yaw, pitch, roll;
};

struct imuVal{
    float gyr[3];
    float acc[3];
};

struct imuCal{
    float gyrBias[3] = {0.0, 0.0, 0.0};
    float accBias[3] = {0.0, 0.0, 0.0};
};

extern int usCyc;
extern euler curState;
extern imuVal curIMU;
extern imuCal curCal;

void setupMPU(int usCycle);
euler readMPU();
void calibMPU();
void quaternionFilter();

#endif