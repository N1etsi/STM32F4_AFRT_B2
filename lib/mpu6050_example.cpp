/*
 * MPU6050 Demo
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "fcs_definiton.hpp"
#include "io_definition.hpp"

#define msCyc 5.0f
#define usCyc 5000
uint32_t loopSt;

//mpu6050
float accRes = (float)(4.0f / 32768.0f); //accel resolution calibration aaa
float gyrRes = (float)(500.0f / 32768.0f); //gyro resolution calibration aaa
int16_t rawAG[6];
float gyr[3], gyrBias[3] = {0.0, 0.0, 0.0};
float acc[3], accBias[3] = {0.0, 0.0, 0.0};
uint32_t cycTim;
float yaw, pitch, roll; //Madgwick Quaternion Filter Attitude Results 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float beta = sqrt(3.0f / 4.0f) * PI * (40.0f / 180.0f);
float zeta = sqrt(3.0f / 4.0f) * PI * (2.0f / 180.0f);
float deltat = msCyc / 1000.0f;


//function declaration
void setMPU();
void quaternionFilter();
void readMPU();
void debugPrint();


void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Wire.setClock(4000000);
  Wire.begin();
  delay(500);

  setMPU();
  delay(500);

  loopSt = micros();
}

void loop()
{
  while( (micros()-loopSt) < (usCyc));
  loopSt = micros();
  readMPU();
  quaternionFilter();
  debugPrint();
}


void setMPU()
{
  //Power Management 
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); //power mngmt bit
  Wire.write(0x00); //no sleep mode
  Wire.endTransmission();

  //Sample rate divider and frequency -> set to 1kHz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x19);
  Wire.write(0x04);//0x04 -> 200Hz // 0x07 -> 1kHz
  Wire.endTransmission();

  //set gyro sensitivity
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08); //set to 500deg/s max aaa
  Wire.endTransmission();

  //set accel sensitivity
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x08); //set to 4g max aaa
  Wire.endTransmission();
}

void readMPU()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); //first data address
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,14); //6+2+6(acc, temp, gyr)
  while(Wire.available() < 14);
  rawAG[0] = (int16_t)((Wire.read() << 8) | Wire.read());
  rawAG[1] = (int16_t)((Wire.read() << 8) | Wire.read());
  rawAG[2] = (int16_t)((Wire.read() << 8) | Wire.read());

  rawAG[3] = (int16_t)((Wire.read() << 8) | Wire.read()); //temp value, trash it

  rawAG[3] = (int16_t)((Wire.read() << 8) | Wire.read());
  rawAG[4] = (int16_t)((Wire.read() << 8) | Wire.read());
  rawAG[5] = (int16_t)((Wire.read() << 8) | Wire.read());

  acc[X] = ((float)rawAG[0])*accRes - accBias[X];
  acc[Y] = ((float)rawAG[1])*accRes - accBias[Y];
  acc[Z] = ((float)rawAG[2])*accRes - accBias[Z];

  gyr[X] = ((float)rawAG[3])*gyrRes - gyrBias[X];
  gyr[Y] = ((float)rawAG[4])*gyrRes - gyrBias[Y];
  gyr[Z] = ((float)rawAG[5])*gyrRes - gyrBias[Z];

  while(Wire.available()) Wire.read();
}

void quaternionFilter()
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
  float norm;                                               // vector norm
  float f1, f2, f3;                                         // objetive funcyion elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  float gerrx, gerry, gerrz, gbiasx=0, gbiasy=0, gbiasz=0;        // gyro bias error

  //Degrees to Radians
  gyr[X] *= PI/180.0f; 
  gyr[Y] *= PI/180.0f;
  gyr[Z] *= PI/180.0f;

  // Auxiliary variables to avoid repeated arithmetic
  float _halfq1 = 0.5f * q1;
  float _halfq2 = 0.5f * q2;
  float _halfq3 = 0.5f * q3;
  float _halfq4 = 0.5f * q4;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;

  // Normalise accelerometer measurement
  norm = sqrt(acc[X] * acc[X] + acc[Y] * acc[Y] + acc[Z] * acc[Z]);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  acc[X] *= norm;
  acc[Y] *= norm;
  acc[Z] *= norm;
  
  // Compute the objective function and Jacobian
  f1 = _2q2 * q4 - _2q1 * q3 - acc[X];
  f2 = _2q1 * q2 + _2q3 * q4 - acc[Y];
  f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - acc[Z];
  J_11or24 = _2q3;
  J_12or23 = _2q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;

  // Compute the gradient (matrix multiplication)
  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
  hatDot4 = J_14or21 * f1 + J_11or24 * f2;
  
  // Normalize the gradient
  norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
  hatDot1 /= norm;
  hatDot2 /= norm;
  hatDot3 /= norm;
  hatDot4 /= norm;
  
  // Compute estimated gyroscope biases
  gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
  gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
  gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
  
  // Compute and remove gyroscope biases
  gbiasx += gerrx * deltat * zeta;
  gbiasy += gerry * deltat * zeta;
  gbiasz += gerrz * deltat * zeta;
  gyr[X] -= gbiasx;
  gyr[Y] -= gbiasy;
  gyr[Z] -= gbiasz;
  
  // Compute the quaternion derivative
  qDot1 = -_halfq2 * gyr[X] - _halfq3 * gyr[Y] - _halfq4 * gyr[Z];
  qDot2 =  _halfq1 * gyr[X] + _halfq3 * gyr[Z] - _halfq4 * gyr[Y];
  qDot3 =  _halfq1 * gyr[Y] - _halfq2 * gyr[Z] + _halfq4 * gyr[X];
  qDot4 =  _halfq1 * gyr[Z] + _halfq2 * gyr[Y] - _halfq3 * gyr[X];

  // Compute then integrate estimated quaternion derivative
  q1 += (qDot1 -(beta * hatDot1)) * deltat;
  q2 += (qDot2 -(beta * hatDot2)) * deltat;
  q3 += (qDot3 -(beta * hatDot3)) * deltat;
  q4 += (qDot4 -(beta * hatDot4)) * deltat;

  // Normalize the quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

  // Intrinsic Euler Angles
  yaw = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  
  yaw *= 180.0f / PI;
  pitch *= 180.0f / PI;
  roll *= 180.0f / PI;
}

void debugPrint()
{
  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(yaw);
  Serial.print(",");
  Serial.print(usCyc - micros() + loopSt);
}
