#include <Arduino.h>
#include <Wire.h>
#include <SFE_LSM9DS0.h>
#include <Adafruit_VL53L0X.h>
//#include <math.h>

// ===== Value Definition =====
#define nCh 8
#define usCyc 5000
#define msCyc 5.0f

#define Ro 0
#define Pi 1
#define Ya 2
#define Kp 0
#define Ki 1
#define Kd 2
#define X 0
#define Y 1
#define Z 2

#define STDBY -10
#define GND 0
#define AIR 1
#define SHUT -5
// ===== End of Value Def =====

// ===== Pin Declaration =====
#define rx PB4
#define escLF PA8
#define escLB PA9
#define escRF PA10
#define escRB PA11
// #define redL PB14
// #define greenL PB12
#define INTERRUPT_PIN PB5
#define stL PC13
// ===== End of Pin Decl =====

// ===== LSM9DS0 stuff - Accels =====
#define LSM9DS0_XM 0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G 0x6B  // Would be 0x6A if SDO_G is LOW

LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
// ===== End of LSM9DS0 stuff =====

// ===== Control Loop =====
float Kte[3][3];
float setP[3];
float altsetP[3]; // alternate pid settings
float err[3];
float last[3];
float mem[3];
int32_t pidOut[3];
// ===== Control Loop =====

// ===== ppm =====
volatile uint32_t rxPrev = 0, rxCurr, rxCH;
volatile uint32_t rolloverCompareCount = 0;
volatile uint32_t chArr[nCh + 1];
int chCounter = 0;
uint32_t channel;
HardwareTimer *rx_tim;
// ===== ppm =====

// ===== PWM stuff =====
uint32_t escLFt, escLBt, escRFt, escRBt, throttle;
// ===== PWM stuff =====


// ===== Distance Sensor stuff =====
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure; // to read distance use: lox.rangingTest(&measure, false); // a distancia fica guardada na struct measure
// ===== Distance Sensor stuff =====

// ===== Extra global variables =====
byte mode = AIR;
uint32_t loopSt;
float accRes = (float)(4.0f / 32768.0f);   // accel resolution calibration aaa
float gyrRes = (float)(500.0f / 32768.0f); // gyro resolution calibration aaa
int16_t rawAG[6];
float gyr[3], gyrBias[3] = {0.0, 0.0, 0.0};
float acc[3], accBias[3] = {0.0, 0.0, 0.0};
uint32_t cycTim;
float yaw, pitch, roll; // Madgwick Quaternion Filter Attitude Results
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float beta = sqrt(3.0f / 4.0f) * PI * (40.0f / 180.0f);
float zeta = sqrt(3.0f / 4.0f) * PI * (2.0f / 180.0f);
float deltat = msCyc / 1000.0f;
// ===== Extra global variables =====

void pidK();
void pinDecl();
void initRX(void);
void setRX();
void timerS(void);
void escOut();
void escInit();
void calcPID();
void setMPU();
void calibMPU();
void quaternionFilter();
void readMPU();
void varClear();
void debugPrint();
void tiltCheck();
void rxInputCapture(void);

// Setup
void setup()
{
    TIM1->CCR1 = 30;
    // Init, lights off

    // Comms startup
    // Serial.begin(460800);
    Wire.setClock(400000);
    Wire.begin();

    delay(500);

    // FC startup
    pidK();
    pinDecl();
    timerS();
    setRX();
    initRX();
    lox.begin();

    // digitalWrite(redL, HIGH);
    // digitalWrite(greenL, HIGH);
    digitalWrite(stL, HIGH);

    // Sensor start
    setMPU();

    delay(500);
    // Calibrate Sensors (might remove)
    // calibMPU(); - RETIRADO PRA TESTES INICIAIS

    // Blink internal LED when ready
    for (int i = 0; i < 5; i++)
    {
        digitalWrite(stL, !digitalRead(stL));
        delay(250);
    }
    digitalWrite(stL, HIGH);

    loopSt = micros();
}

// Main Loop
void loop()
{
    if ((micros() - loopSt) > 1000.0)
        digitalWrite(stL, LOW);
    while ((micros() - loopSt) < (usCyc))
        ;
    loopSt = micros();

    // MODE INDEPENDENT
    readMPU();
    quaternionFilter();
    tiltCheck();
    // debugPrint();

    // STDBY MODE
    if (mode == AIR)
    {
        calcPID();
        escOut();

        // digitalWrite(redL, LOW);
        // digitalWrite(greenL, HIGH);
    }
    else if (mode == GND)
    {
        escInit();
        varClear();
        // digitalWrite(redL, HIGH);
        // digitalWrite(greenL, LOW);
        setP[Ya] = yaw; // initial position
        // digitalWrite(greenL, LOW);
        // digitalWrite(redL, LOW);
    }
    else if (mode == STDBY)
    {
        escInit();
        varClear();
        // digitalWrite(redL, HIGH);
        // digitalWrite(greenL, LOW);
    }
    else // catch exceptions, same as stdby?
    {
        escInit();
        varClear();
        // digitalWrite(redL, HIGH);
        // digitalWrite(greenL, HIGH);
    }
}

void pidK()
{
    // most stable 4.0 / 0.03 / 150
    // old (had oscillations) most stable until now : 6.0 / 0.08 / 150.0
    // will try baseflight default values 4.0 / 0.03 / 23.0
    setP[Ro] = 0;
    setP[Pi] = 0;
    setP[Ya] = 0;

    Kte[Ro][Kp] = 1.3; // 1.3 seems to low, 10 seems to high
    Kte[Ro][Ki] = 0.05;
    Kte[Ro][Kd] = 15.0;

    Kte[Pi][Kp] = 1.3;
    Kte[Pi][Ki] = 0.05;
    Kte[Pi][Kd] = 15.0;

    Kte[Ya][Kp] = 0.2;
    Kte[Ya][Ki] = 0.00;
    Kte[Ya][Kd] = 5;
}

void pinDecl()
{
    // PPM RX
    // pinMode(rx, INPUT);
    // PWM ESC - Decidir isto
    pinMode(escLF, OUTPUT); // isto tava definido como PWM mas nao havia referencia a nenhum "PWM"
    pinMode(escLB, OUTPUT);
    pinMode(escRB, OUTPUT);
    pinMode(escRF, OUTPUT);
    // LED status
    //   pinMode(redL, OUTPUT);
    //   pinMode(greenL, OUTPUT);
    pinMode(stL, OUTPUT);
    //   digitalWrite(redL, LOW);
    //   digitalWrite(greenL, LOW);
}

void initRX(void)
{
    // RX PIN TIM SETUP
    TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(rx), PinMap_PWM);
    channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(rx), PinMap_PWM));

    rx_tim = new HardwareTimer(Instance);
    rx_tim->setMode(channel, TIMER_INPUT_CAPTURE_RISING, rx);

    uint32_t PrescalerFactor = 84;
    rx_tim->setPrescaleFactor(PrescalerFactor);
    rx_tim->setOverflow(0x10000); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
    rx_tim->attachInterrupt(channel, rxInputCapture);
    rx_tim->resume();
}

void setRX()
{
    chArr[0] = 1000.0;
    chArr[1] = 1500.0;
    chArr[2] = 1500.0;
    chArr[3] = 1000.0;
    chArr[4] = 1500.0;
    chArr[5] = 1000.0;
    chArr[6] = 1000.0;
    chArr[7] = 1000.0;
    chArr[8] = 1000.0;
}

void timerS(void)
{
    // setup timer2-PWM and timer1-ppm
    //   //Timer1.attachCompare1Interrupt(initRX);
    //   TIM1.attachInterrupt(TIMER_CH1, initRX);
    //   TIM1->CR1 = TIM_CR1_CEN;
    //   TIM1->CR2 = 0;
    //   TIM1->SMCR = 0;
    //   TIM1->DIER = TIMER_DIER_CC1IE;
    //   TIM1->EGR = 0;
    //   TIM1->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1;
    //   TIM1->CCMR2 = 0;
    //   TIM1->CCER = TIMER_CCER_CC1E;
    //   TIM1->PSC = 71;
    //   TIM1->ARR = 0xFFFF;
    //   TIM1->DCR = 0;

    TIM1->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
    TIM1->CR2 = 0;
    TIM1->SMCR = 0;
    TIM1->DIER = 0;
    TIM1->EGR = 0;
    TIM1->CCMR1 = (0b110 << 4) | TIM_CCMR1_OC1PE | (0b110 << 12) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = (0b110 << 4) | TIM_CCMR2_OC3PE | (0b110 << 12) | TIM_CCMR2_OC4PE;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM1->PSC = 71; // assumo que isto seja para meter a freq a 1mghz mas o f4 ainda tem 71mghz?
    TIM1->ARR = 5000;
    TIM1->DCR = 0;

    // pwm init default values
    TIM1->CCR1 = 1000;
    TIM1->CCR2 = 1000;
    TIM1->CCR3 = 1000;
    TIM1->CCR4 = 1000;
}

void escOut()
{
    //   Serial.print(escLFt);
    //   Serial.print(" ");
    //   Serial.print(escRFt);
    //   Serial.print(" ");
    //   Serial.print(escLBt);
    //   Serial.print(" ");
    //   Serial.print(escRBt);
    //   Serial.println(" ");

    TIM1->CCR1 = escLFt;
    TIM1->CCR2 = escRFt;
    TIM1->CCR3 = escLBt;
    TIM1->CCR4 = escRBt;
    TIM1->CNT = 5000;
}

void escInit()
{
    TIM1->CCR1 = 1000;
    TIM1->CCR2 = 1000;
    TIM1->CCR3 = 1000;
    TIM1->CCR4 = 1000;
    TIM1->CNT = 5000;
}

void calcPID()
{
    throttle = chArr[3];
    setP[Ro] = ((float)chArr[1] - 1500) / 15.0;
    setP[Pi] = -1.0 * ((float)chArr[2] - 1500) / 15.0;
    if (fabs(chArr[4] - 1500) < 20)
        ;
    else
    {
        setP[Ya] += ((float)chArr[4] - 1500) / 1500.0; // needs testing on rate
        if (setP[Ya] > 180)
            setP[Ya] -= 360.0;
        else if (setP[Ya] < -180)
            setP[Ya] += 360.0;
    }

    // roll pitch
    err[Ro] = roll - setP[Ro];
    mem[Ro] += err[Ro];
    pidOut[Ro] = Kte[Ro][Kp] * err[Ro] + Kte[Ro][Ki] * mem[Ro] + Kte[Ro][Kd] * (err[Ro] - last[Ro]);
    last[Ro] = err[Ro];
    if (pidOut[Ro] > 400)
        pidOut[Ro] = 400;
    else if (pidOut[Ro] < (-400))
        pidOut[Ro] = (-400);

    // pid pitch
    err[Pi] = pitch - setP[Pi];
    mem[Pi] += err[Pi];
    pidOut[Pi] = Kte[Pi][Kp] * err[Pi] + Kte[Pi][Ki] * mem[Pi] + Kte[Pi][Kd] * (err[Pi] - last[Pi]);
    last[Pi] = err[Pi];
    if (pidOut[Pi] > 400)
        pidOut[Pi] = 400;
    else if (pidOut[Pi] < (-400))
        pidOut[Pi] = (-400);

    // pid yaw
    err[Ya] = yaw - setP[Ya];
    if (abs(err[Ya]) > 180)
        err[Ya] *= -1;
    // mem[Ya]+=err[Ya];
    pidOut[Ya] = Kte[Ya][Kp] * err[Ya] + Kte[Ya][Kd] * (err[Ya] - last[Ya]); //+ Kte[Ya][Ki]*mem[Ya] + Kte[Ya][Kd]*(err[Ya]-last[Ya]);
    last[Ya] = err[Ya];
    if (pidOut[Ya] > 400)
        pidOut[Ya] = 400;
    else if (pidOut[Ya] < (-400))
        pidOut[Ya] = (-400);

    // output
    escLFt = (uint32_t)min(max(throttle + pidOut[Pi] - pidOut[Ro] + pidOut[Ya], (uint32_t)1200), (uint32_t)1800);
    escRFt = (uint32_t)min(max(throttle + pidOut[Pi] + pidOut[Ro] - pidOut[Ya], (uint32_t)1200), (uint32_t)1800);
    escLBt = (uint32_t)min(max(throttle - pidOut[Pi] - pidOut[Ro] - pidOut[Ya], (uint32_t)1200), (uint32_t)1800);
    escRBt = (uint32_t)min(max(throttle - pidOut[Pi] + pidOut[Ro] + pidOut[Ya], (uint32_t)1200), (uint32_t)1800);
}

void setMPU()
{
    uint16_t status = dof.begin();
    // Or call it with declarations for sensor scales and data rates:
    // uint16_t status = dof.begin(dof.G_SCALE_2000DPS,
    //                            dof.A_SCALE_6G, dof.M_SCALE_2GS);
    Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
    Serial.println(status, HEX);
    Serial.println("Should be 0x49D4");
    Serial.println();

    dof.setAccelODR(dof.A_ODR_1600);
    dof.setGyroODR(dof.G_ODR_760_BW_30);
    dof.setMagODR(dof.M_ODR_100);
}

void readMPU()
{
    dof.readAccel();
    dof.readGyro();

    acc[X] = dof.calcAccel(dof.ax) - accBias[X];
    acc[Y] = dof.calcAccel(dof.ay) - accBias[Y];
    acc[Z] = dof.calcAccel(dof.az) - accBias[Z];

    gyr[X] = dof.calcGyro(dof.gx) - gyrBias[X];
    gyr[Y] = dof.calcGyro(dof.gy) - gyrBias[Y];
    gyr[Z] = dof.calcGyro(dof.gz) - gyrBias[Z];
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

    for (int i = 0; i < 20; i++)
        readMPU();
    digitalWrite(stL, HIGH);
    for (int i = 0; i < n; i++)
    {
        readMPU();
        bias[0] += acc[X];
        bias[1] += acc[Y];
        bias[2] += acc[Z];
        bias[3] += gyr[X];
        bias[4] += gyr[Y];
        bias[5] += gyr[Z];
        // Serial.println(i);
    }
    digitalWrite(stL, LOW);
    accBias[X] = bias[0] / n;
    accBias[Y] = bias[1] / n;
    accBias[Z] = bias[2] / n;
    accBias[Z] -= 1.0f; // why

    gyrBias[X] = bias[3] / n;
    gyrBias[Y] = bias[4] / n;
    gyrBias[Z] = bias[5] / n;
}

void quaternionFilter()
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx = 0, gbiasy = 0, gbiasz = 0; // gyro bias error

    // Degrees to Radians  -VER SE AINDA É NECESSARIO
    gyr[X] *= PI / 180.0f;
    gyr[Y] *= PI / 180.0f;
    gyr[Z] *= PI / 180.0f;

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
    if (norm == 0.0f)
        return; // handle NaN
    norm = 1.0f / norm;
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
    hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
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
    qDot2 = _halfq1 * gyr[X] + _halfq3 * gyr[Z] - _halfq4 * gyr[Y];
    qDot3 = _halfq1 * gyr[Y] - _halfq2 * gyr[Z] + _halfq4 * gyr[X];
    qDot4 = _halfq1 * gyr[Z] + _halfq2 * gyr[Y] - _halfq3 * gyr[X];

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 - (beta * hatDot1)) * deltat;
    q2 += (qDot2 - (beta * hatDot2)) * deltat;
    q3 += (qDot3 - (beta * hatDot3)) * deltat;
    q4 += (qDot4 - (beta * hatDot4)) * deltat;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // normalise quaternion
    norm = 1.0f / norm;
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
    Serial.print("/");
    Serial.print(pitch);
    Serial.print("/");
    Serial.print(yaw);
    Serial.print("/");
    Serial.print(setP[Ya]);
    Serial.print("/");
    Serial.print(chArr[4]);
    Serial.print("/");
    Serial.print(err[Ya]);
    Serial.print("/");
    Serial.print(pidOut[Ya]);
    Serial.println(" ");
}

void tiltCheck()
{
    if ((roll > 60.0) || (roll < -60.0) || (pitch > 60.0) || (pitch < -60.0))
        mode = SHUT;
}

void varClear()
{
    mem[Ro] = 0;
    mem[Pi] = 0;
    last[Ro] = 0;
    last[Pi] = 0;
}

void rxInputCapture(void)
{
    rxCurr = rx_tim->getCaptureCompare(channel);
    if (rxCurr > rxPrev)
        rxCH = rxCurr - rxPrev;
    else if (rxCurr <= rxPrev)
        rxCH = 0x10000 + rxCurr - rxPrev;
    rxPrev = rxCurr;
    if (rxCH > 3000)
        chCounter = 0;
    chArr[chCounter] = rxCH;
    chCounter++;
}
