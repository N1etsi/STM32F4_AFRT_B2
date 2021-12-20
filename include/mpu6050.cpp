/*


*/

#include "mpu6050.hpp"


void setupMPU(int usCycle)
{
    usCyc = usCycle;

    extern euler curState;
    extern imuVal curIMU;
    extern imuCal curCal;
    
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

euler readMPU()
{
    static float accRes = (float)(4.0f / 32768.0f); //accel resolution calibration aaa
    static float gyrRes = (float)(500.0f / 32768.0f); //gyro resolution calibration aaa
    static int16_t rawAG[6];

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

    curIMU.acc[X] = ((float)rawAG[0])*accRes - curCal.accBias[X];
    curIMU.acc[Y] = ((float)rawAG[1])*accRes - curCal.accBias[Y];
    curIMU.acc[Z] = ((float)rawAG[2])*accRes - curCal.accBias[Z];

    curIMU.gyr[X] = ((float)rawAG[3])*gyrRes - curCal.gyrBias[X];
    curIMU.gyr[Y] = ((float)rawAG[4])*gyrRes - curCal.gyrBias[Y];
    curIMU.gyr[Z] = ((float)rawAG[5])*gyrRes - curCal.gyrBias[Z];

    while(Wire.available()) Wire.read();  

    quaternionFilter();

    return curState;
}

void calibMPU()
{
    float bias[6];
    int n = 100;
    bias[0] = 0.0;
    bias[1] = 0.0;
    bias[2] = 0.0;
    bias[3] = 0.0;
    bias[4] = 0.0;
    bias[5] = 0.0;

    for(int i=0; i<20; i++) readMPU();
    for(int i=0; i<n; i++)
    {
        readMPU();
        bias[0] += curIMU.acc[X];
        bias[1] += curIMU.acc[Y];
        bias[2] += curIMU.acc[Z];
        bias[3] += curIMU.gyr[X];
        bias[4] += curIMU.gyr[Y];
        bias[5] += curIMU.gyr[Z];
        //Serial.println(i);
    }
    curCal.accBias[X] = bias[0] / n;
    curCal.accBias[Y] = bias[1] / n;
    curCal.accBias[Z] = bias[2] / n;
    curCal.accBias[Z] -= 1.0f;

    curCal.gyrBias[X] = bias[3] / n;
    curCal.gyrBias[Y] = bias[4] / n;
    curCal.gyrBias[Z] = bias[5] / n;
    
}

void quaternionFilter()
{
    static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    static float beta = sqrt(3.0f / 4.0f) * PI * (40.0f / 180.0f);
    static float zeta = sqrt(3.0f / 4.0f) * PI * (2.0f / 180.0f);
    static float deltat = float(usCyc) / 1000.0f / 1000.0f;
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx=0, gbiasy=0, gbiasz=0;        // gyro bias error

    //Degrees to Radians
    curIMU.gyr[X] *= PI/180.0f; 
    curIMU.gyr[Y] *= PI/180.0f;
    curIMU.gyr[Z] *= PI/180.0f;

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
    norm = sqrt(curIMU.acc[X] * curIMU.acc[X] + curIMU.acc[Y] * curIMU.acc[Y] + curIMU.acc[Z] * curIMU.acc[Z]);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    curIMU.acc[X] *= norm;
    curIMU.acc[Y] *= norm;
    curIMU.acc[Z] *= norm;
    
    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - curIMU.acc[X];
    f2 = _2q1 * q2 + _2q3 * q4 - curIMU.acc[Y];
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - curIMU.acc[Z];
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
    curIMU.gyr[X] -= gbiasx;
    curIMU.gyr[Y] -= gbiasy;
    curIMU.gyr[Z] -= gbiasz;
    
    // Compute the quaternion derivative
    qDot1 = -_halfq2 * curIMU.gyr[X] - _halfq3 * curIMU.gyr[Y] - _halfq4 * curIMU.gyr[Z];
    qDot2 =  _halfq1 * curIMU.gyr[X] + _halfq3 * curIMU.gyr[Z] - _halfq4 * curIMU.gyr[Y];
    qDot3 =  _halfq1 * curIMU.gyr[Y] - _halfq2 * curIMU.gyr[Z] + _halfq4 * curIMU.gyr[X];
    qDot4 =  _halfq1 * curIMU.gyr[Z] + _halfq2 * curIMU.gyr[Y] - _halfq3 * curIMU.gyr[X];

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
    curState.yaw = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    curState.pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
    curState.roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    
    curState.yaw *= 180.0f / PI;
    curState.pitch *= 180.0f / PI;
    curState.roll *= 180.0f / PI;
}