// Author: Abdullah Altawaitan
// Description: This research code is part of a long-term goal of
// a fleet of cooperating Flexible Autonomous Machines operating in an uncertain Environment (FAME)
// at Arizona State University

#include "i2c_t3.h"
#include "EEPROM.h"
#include "Arduino.h"
#include "Quaternion.h"
#include "IMU.h"
#include <cmath>

// Teensy 3.2 pins
#define MOTOR1 20
#define MOTOR2 21
#define MOTOR3 6
#define MOTOR4 5
#define CHANNEL1 23
#define CHANNEL2 22
#define CHANNEL3 17
#define CHANNEL4 16
#define CHANNEL5 24
#define CHANNEL6 26
#define CHANNEL7 27
#define CHANNEL8 28
#define LED 13
#define LEDRed 31
#define LEDGreen 33

// Motor parameters
#define MIN_MOTOR_LEVEL 1000
#define MAX_MOTOR_LEVEL 1800
#define ARM_MOTOR_LEVEL 1080
#define OFF_MOTOR_LEVEL 0


// AHRS
#define twoKi (2.0f * 0.25f)
#define twoKp (2.0f * 0.5f)

// Quadrotor modes
#define START_MODE 0
#define TRANS_MODE 1
#define ARMING_MODE 2
#define TEMP_MODE 3
#define DISARMING_MODE 4

// Quadrotor Parameters
#define m 0.558
#define armlength 0.1215
#define k_f 0.000001518 // Thrust coefficient
#define k_m 0.00000001843 // Torque coefficient
#define kappa 0.0127 // Ratio between thrust [N] and torque due to drag [N m] kappa = torque/thrust
#define Ixx 0.002
#define PWM_FACTOR 26.214 // 65535.0 / 2500.0

// Loop
#define FREQ 400
#define dt 0.0025 // Period for 400Hz
#define dtOuter 0.01 // Period for 100Hz
#define dtMicroseconds 2500

// Inner-loop PID parameters
#define kpPQRx 0.1
#define kdPQRx 0.004
#define kiPQRx 0.8
#define kpPQRy 0.1
#define kdPQRy 0.004
#define kiPQRy 0.8
#define kpPQRz 0.1
#define kdPQRz 0.001
#define kiPQRz 0.2
// Outer-loop PID parameters
#define kpx 10
#define kdx 0
#define kix 0
#define kpy 10
#define kdy 0
#define kiy 0
#define kpz 10
#define kdz 0
#define kiz 0

class Quadrotor
{
public:
  struct _STATE {
    float x, y, z, phi, theta, psi, xdot, ydot, zdot, p, q, r;
  };
  struct _XYZ {
    float x, y, z;
  };
  struct _GYRO {
    _XYZ raw, tempOffset, gyroOffset, calibrated, filtered;
  };
  struct _ACCEL {
    _XYZ raw, afterOffset, calibrated, filtered;
  };
  struct _FILTER {
    float b0, b1, b2, a1, a2, element0, element1, element2;
  };
  struct _Q {
    float w, x, y, z;
  };
  struct _TRIG {
    float phi_sin, theta_sin, psi_sin, phi_cos, theta_cos, psi_cos;
  };
  struct _CH {
    short CH1, CH2, CH3, CH4;
  };
  struct _ERROR {
    float p, q, r, p_prev, q_prev, r_prev, p_integral, q_integral,  r_integral;
    float phi, theta, psi, phi_prev, theta_prev, psi_prev, phi_integral, theta_integral,  psi_integral;
  };
  struct _Acc_Cali {
    int16_t accel_raw[6][3], accel_timer = 0;
    int32_t accel_calitmpx, accel_calitmpy, accel_calitmpz;
    float accel_offset[3], a[3][3], T[3][3];
    float g = 8196.72; // for +-4g range
  };

  // Sensor variables
  IMU imu;
  _GYRO gyro;
  _ACCEL accel;
  _FILTER gyroFilterParameterX, gyroFilterParameterY, gyroFilterParameterZ, accelFilterParameterX, accelFilterParameterY, accelFilterParameterZ;
  _Acc_Cali Acc_Cali;
  _Q q = {1.0, 0.0, 0.0, 0.0};
  _XYZ gyroAngle;
  _XYZ accelAngle;
  int8_t gyro_calibration_done = 0;
  int8_t accel_calibration_done = 0;
  short gyro_calibration_counter = 0;
  float GyroCollection[3] = {0, 0, 0};
  float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
  Quaternion quat;
  float temp;


  // Quadrotor variables
  _STATE X;
  _STATE Xdes;
  _ERROR error;
  _TRIG trig;
  int16_t throttle;
  uint8_t QuadrotorState = START_MODE;
  float voltage;
  unsigned long loop_timer;

  // Control variables
  float U1, U2, U3, U4; // Inner loop
  uint8_t outerCounter = 0;

  // Receiver variables
  _CH channel;
  bool lastChannel1, lastChannel2, lastChannel3, lastChannel4;
  unsigned long receiverChannelTimer1, receiverChannelTimer2, receiverChannelTimer3, receiverChannelTimer4;
  float RCYawRate;

  // Motor variables
  float PWM1, PWM2, PWM3, PWM4;
  float volt1, volt2, volt3, volt4;
  float omega1, omega2, omega3, omega4;
  float omega1Squared, omega2Squared, omega3Squared, omega4Squared;

  // Xbee Zigbee variables
  const byte numChars = 32;
  char receivedChars[32];
  char tempChars[32];
  float float1 = 0.0;
  float float2 = 0.0;
  float float3 = 0.0;
  boolean newData = false;
  uint8_t Xbee_couter = 0;
  uint8_t Xbee_couter2 = 0;

  // Methods
  void MotorInit(void);
  void SensorInit(void);
  void IMUread(void);
  void GyroCalibration(void);
  void AccelCalibration(uint8_t point);
  void AccelOffsetRead(void);
  void FilterInit(void);
  void Estimation(int8_t method);
  void NonlinearComplementaryFilter(void);
  void AHRS(void);
  void MahonyAHRS(void);
  void MahonyAHRSUpdate(float gx, float gy, float gz, float ax, float ay, float az);
  void ArmingState(void);
  void BatteryVoltageCheck(void);
  void Receiver(void);
  void AttitudeControl(void);
  void AngularRateControl(void);
  void AltitudeControl(void);
  void GenerateMotorCommands(void);
  void MotorRun(void);
  void XbeeZigbeeSend(void);
  void XbeeZigbeeReceive(void);
  void SecondOrderLowPassFilter(float sample_freq, float cutoff_freq, struct _FILTER *input_IIR);
  float SecondOrderLowPassFilterApply(float cutoff_freq, float sample, struct _FILTER *input_IIR);
  float CONSTRAIN(float x, float min, float max);
  float invSqrt(float number);

private:

};
