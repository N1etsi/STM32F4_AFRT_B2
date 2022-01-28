#ifndef IMU
#define IMU

#include <Arduino.h>
#include <Wire.h>



#include <fcs.hpp>
#include <SFE_LSM9DS0.h>


namespace imu
{
    //Define constants
    #define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
    #define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
    // global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
    #define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
    #define GyroMeasDrift PI * (1.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)

    #define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
    #define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
    #define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
    #define Ki 0.00f
    

    //Init sensor object
    inline LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
    inline uint32_t status = -1;


    //Loop timing TODO: remove this if loop time is stable enough
    inline float deltat = 0.0f; 
    inline uint32_t lastUpdate = 0;    // used to calculate integration interval
    inline uint32_t Now = 0;           // used to calculate integration interval

    //Aux global variables
    inline float abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0};
    inline float ax, ay, az, gx, gy, gz, mx, my, mz;
    inline float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    inline float eInt[3] = {0.0f, 0.0f, 0.0f}; 

    //Function Declaration
    int setup();
    fcs::State getAHRS();
    void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
    void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
    void MadgwickQuaternion(float ax, float ay, float az, float gx, float gy, float gz);
    void calcOffsets(int iter);
    float invSqrt(float x);

}

#endif