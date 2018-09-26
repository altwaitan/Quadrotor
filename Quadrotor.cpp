#include "Quadrotor.h"

// Motor initialization
void Quadrotor::MotorInit(void)
{
  pinMode(MOTOR1, OUTPUT);
  pinMode(MOTOR2, OUTPUT);
  pinMode(MOTOR3, OUTPUT);
  pinMode(MOTOR4, OUTPUT);
  delay(5);
  analogWriteFrequency(MOTOR1, 400);
  analogWriteFrequency(MOTOR2, 400);
  analogWriteFrequency(MOTOR3, 400);
  analogWriteFrequency(MOTOR4, 400);
  analogWriteResolution(16);
  delay(5);

  // Motors initialization
  float PWM = PWM_FACTOR * MIN_MOTOR_LEVEL;
  analogWrite(MOTOR1, PWM);
  analogWrite(MOTOR2, PWM);
  analogWrite(MOTOR3, PWM);
  analogWrite(MOTOR4, PWM);

  // Reset integral variables
  error.p_integral = 0;
  error.q_integral = 0;
  error.r_integral = 0;
  error.phi_integral = 0;
  error.theta_integral = 0;
  error.psi_integral = 0;
}

// Sensor initialization and calibration
// Source: Pololu Robotics and Electronics (https://github.com/pololu/lsm303-arduino)
// Source: Andrea Vitali (DT0053 Design tip - 6-point tumble sensor calibration)
void Quadrotor::SensorInit(void)
{
  Wire.begin();
  Wire.setRate(I2C_RATE_2000);
  delay(200);
  // Gyroscope initialization
  if (!L3GD20.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }
  L3GD20.enableDefault();

  // Accelerometer initialization
  LSM303D.init();
  LSM303D.enableDefault();
  delay(200);

  // Sensor calibration
  Serial.println("Sensor Calibration...");
  Serial.println("Place Quadrotor on level.");
  delay(200);

  // Gyroscope calibration
  while (gyro_calibration_done == 0)
  {
    Quadrotor::L3GD20Calibration();
  }

  // Accelerometer calibration
  Serial.println("If you enter 'a', the calibration will get started.");
  delay(3000);
  char incomingByte = Serial.read();

  if (incomingByte == 'a')
  {
    Serial.println("On Level, and wait for 10 seconds");
    delay(10000);
    while (accel_calibration_done == 0)
    {
      Quadrotor::LSM303DCalibration(4);
    }
    accel_calibration_done = 0;

    Serial.println("On Back, and wait for 10 seconds");
    delay(10000);
    while (accel_calibration_done == 0)
    {
      Quadrotor::LSM303DCalibration(5);
    }
    accel_calibration_done = 0;

    Serial.println("Right Wing Up, and wait for 10 seconds");
    delay(10000);
    while (accel_calibration_done == 0)
    {
      Quadrotor::LSM303DCalibration(0);
    }
    accel_calibration_done = 0;

    Serial.println("Left Wing Up, and wait for 10 seconds");
    delay(10000);
    while (accel_calibration_done == 0)
    {
      Quadrotor::LSM303DCalibration(1);
    }
    accel_calibration_done = 0;

    Serial.println("Nose Up, and wait for 10 seconds");
    delay(10000);
    while (accel_calibration_done == 0)
    {
      Quadrotor::LSM303DCalibration(2);
    }
    accel_calibration_done = 0;

    Serial.println("Nose Down, and wait for 10 seconds");
    delay(10000);
    while (accel_calibration_done == 0)
    {
      Quadrotor::LSM303DCalibration(3);
    }
    accel_calibration_done = 1;
  }
  Quadrotor::LSM303DOffsetRead();
  Serial.println("Sensor Calibration... Done!");
}

// Read Gyroscope Data
// Source: Shi Lu (https://github.com/ragewrath/Mark3-Copter-Pilot)
void Quadrotor::L3GD20read(void)
{
  L3GD20.read();
  float gyro_temp = L3GD20.t * -1.0 + 40;
  if (gyro_temp < - 180 && gyro_temp + 256 >= 40)
    gyro_temp = gyro_temp + 256;
  if (gyro_temp < 15)
    gyro_temp = 15;
  if (gyro_temp > 65)
    gyro_temp = 65;

  float gyro_temp_4, gyro_temp_3, gyro_temp_2;
  gyro_temp_4 = gyro_temp * gyro_temp * gyro_temp * gyro_temp;
  gyro_temp_3 = gyro_temp * gyro_temp * gyro_temp;
  gyro_temp_2 = gyro_temp * gyro_temp;

  gyro.raw.x = L3GD20.g.x;
  gyro.raw.y = L3GD20.g.y;
  gyro.raw.z = L3GD20.g.z;

  // Temperature Compensation Model for L3GD20
  gyro.tempOffset.x = 0.00005023 * gyro_temp_4  - 0.008857 * gyro_temp_3 + 0.4812 * gyro_temp_2 - 6.678 * gyro_temp - 120.4;
  gyro.tempOffset.y = -0.00009043 * gyro_temp_4  + 0.0134 * gyro_temp_3 - 0.6597 * gyro_temp_2 + 6.861 * gyro_temp + 47.37;
  gyro.tempOffset.z = 0.00002922 * gyro_temp_4  - 0.004101 * gyro_temp_3 + 0.1754 * gyro_temp_2 - 4.791 * gyro_temp + 120.7;

  gyro.calibrated.x = (gyro.raw.x - gyro.tempOffset.x) * 1.0 / 65.536;
  gyro.calibrated.y = (gyro.raw.y - gyro.tempOffset.y) * 1.0 / 65.536;
  gyro.calibrated.z = (gyro.raw.z - gyro.tempOffset.z) * 1.0 / 65.536;

  if (gyro_calibration_done == 0 && gyro_calibration_counter <= 1500)
  {
    gyro_calibration_counter++;
    L3GD20Calibration();
  }
  if (gyro_calibration_done == 1)
  {
    gyro.calibrated.x = gyro.calibrated.x - gyro.gyroOffset.x;
    gyro.calibrated.y = gyro.calibrated.y - gyro.gyroOffset.y;
    gyro.calibrated.z = gyro.calibrated.z - gyro.gyroOffset.z;

    gyroAngle.x += gyro.calibrated.x * dt;
    gyroAngle.y += gyro.calibrated.y * dt;
    gyroAngle.z += gyro.calibrated.z * dt;
  }
}

// Gyroscope calibration
// Source: Pololu Robotics and Electronics (https://github.com/pololu/lsm303-arduino)
// Source: Andrea Vitali (DT0053 Design tip - 6-point tumble sensor calibration)
void Quadrotor::L3GD20Calibration(void)
{
  L3GD20.read();
  float gyro_temp = L3GD20.t * -1.0 + 40;
  if (gyro_temp < - 180 && gyro_temp + 256 >= 40) gyro_temp = gyro_temp + 256;
  if (gyro_temp < 15) gyro_temp = 15;
  if (gyro_temp > 65) gyro_temp = 65;

  float gyro_temp_4, gyro_temp_3, gyro_temp_2;
  gyro_temp_4 = gyro_temp * gyro_temp * gyro_temp * gyro_temp;
  gyro_temp_3 = gyro_temp * gyro_temp * gyro_temp;
  gyro_temp_2 = gyro_temp * gyro_temp;

  gyro.raw.x = L3GD20.g.x;
  gyro.raw.y = L3GD20.g.y;
  gyro.raw.z = L3GD20.g.z;

  // Temperature Compensation Model for L3GD20
  gyro.tempOffset.x = 0.00005023 * gyro_temp_4  - 0.008857 * gyro_temp_3 + 0.4812 * gyro_temp_2 - 6.678 * gyro_temp - 120.4;
  gyro.tempOffset.y = -0.00009043 * gyro_temp_4  + 0.0134 * gyro_temp_3 - 0.6597 * gyro_temp_2 + 6.861 * gyro_temp + 47.37;
  gyro.tempOffset.z = 0.00002922 * gyro_temp_4  - 0.004101 * gyro_temp_3 + 0.1754 * gyro_temp_2 - 4.791 * gyro_temp + 120.7;

  gyro.calibrated.x = (gyro.raw.x - gyro.tempOffset.x) * 1.0 / 65.536;
  gyro.calibrated.y = (gyro.raw.y - gyro.tempOffset.y) * 1.0 / 65.536;
  gyro.calibrated.z = (gyro.raw.z - gyro.tempOffset.z) * 1.0 / 65.536;

  if (gyro_calibration_done == 0 && gyro_calibration_counter <= 1500)
  {
    gyro_calibration_counter++;
    if (gyro_calibration_counter > 400 && gyro_calibration_counter < 1001)
    {
      GyroCollection[0] += gyro.calibrated.x;
      GyroCollection[1] += gyro.calibrated.y;
      GyroCollection[2] += gyro.calibrated.z;
    }
    if (gyro_calibration_counter == 1001)
    {
      gyro.gyroOffset.x = GyroCollection[0] / 600;
      gyro.gyroOffset.y = GyroCollection[1] / 600;
      gyro.gyroOffset.z = GyroCollection[2] / 600;
      gyro_calibration_counter = 0;
      GyroCollection[0] = 0;
      GyroCollection[1] = 0;
      GyroCollection[2] = 0;
      gyro_calibration_done = 1;
    }
  }
}

// Read Accelerometer Data
// Source: Andrea Vitali (DT0053 Design tip - 6-point tumble sensor calibration)
// Source: Shi Lu (https://github.com/ragewrath/Mark3-Copter-Pilot)
void Quadrotor::LSM303Dread(void)
{
  LSM303D.read();
  accel.raw.x = LSM303D.a.x;
  accel.raw.y = LSM303D.a.y;
  accel.raw.z = LSM303D.a.z;

  accel.afterOffset.x = (float)accel.raw.x - Acc_Cali.accel_offset[0];
  accel.afterOffset.y = (float)accel.raw.y - Acc_Cali.accel_offset[1];
  accel.afterOffset.z = (float)accel.raw.z - Acc_Cali.accel_offset[2];

  accel.calibrated.x = accel.afterOffset.x *  Acc_Cali.T[0][0] + accel.afterOffset.y *  Acc_Cali.T[1][0] + accel.afterOffset.z *  Acc_Cali.T[2][0];
  accel.calibrated.y = accel.afterOffset.x *  Acc_Cali.T[0][1] + accel.afterOffset.y *  Acc_Cali.T[1][1] + accel.afterOffset.z *  Acc_Cali.T[2][1];
  accel.calibrated.z = accel.afterOffset.x *  Acc_Cali.T[0][2] + accel.afterOffset.y *  Acc_Cali.T[1][2] + accel.afterOffset.z *  Acc_Cali.T[2][2];
}

// Accelerometer Calibration
// Source: Andrea Vitali (DT0053 Design tip - 6-point tumble sensor calibration)
// Source: Shi Lu (https://github.com/ragewrath/Mark3-Copter-Pilot)
void Quadrotor::LSM303DCalibration(uint8_t point)
{
  LSM303D.read();
  accel.raw.x = LSM303D.a.x;
  accel.raw.y = LSM303D.a.y;
  accel.raw.z = LSM303D.a.z;
  /*Point = 0 1 2 3 4 5*/
  if (Acc_Cali.accel_timer == 100)
  {
    Acc_Cali.accel_raw[point][0] = Acc_Cali.accel_calitmpx / 100;
    Acc_Cali.accel_raw[point][1] = Acc_Cali.accel_calitmpy / 100;
    Acc_Cali.accel_raw[point][2] = Acc_Cali.accel_calitmpz / 100;
    Acc_Cali.accel_calitmpx = 0;
    Acc_Cali.accel_calitmpy = 0;
    Acc_Cali.accel_calitmpz = 0;
    Acc_Cali.accel_timer = 0;
    EEPROM.write(100 + 6 * point + 1, Acc_Cali.accel_raw[point][0] & 0b11111111);
    EEPROM.write(100 + 6 * point + 2, Acc_Cali.accel_raw[point][0] >> 8);
    EEPROM.write(100 + 6 * point + 3, Acc_Cali.accel_raw[point][1] & 0b11111111);
    EEPROM.write(100 + 6 * point + 4, Acc_Cali.accel_raw[point][1] >> 8);
    EEPROM.write(100 + 6 * point + 5, Acc_Cali.accel_raw[point][2] & 0b11111111);
    EEPROM.write(100 + 6 * point + 6, Acc_Cali.accel_raw[point][2] >> 8);
    accel_calibration_done = 1;
  }
  else
  {
    Acc_Cali.accel_calitmpx += accel.raw.x;
    Acc_Cali.accel_calitmpy += accel.raw.y;
    Acc_Cali.accel_calitmpz += accel.raw.z;
    Acc_Cali.accel_timer++;
  }
}

// Accelerometer Offset Calculation
// Source: Andrea Vitali (DT0053 Design tip - 6-point tumble sensor calibration)
// Source: Shi Lu (https://github.com/ragewrath/Mark3-Copter-Pilot)
void Quadrotor::LSM303DOffsetRead(void)
{
  uint8_t point;
  for (point = 0; point < 6; point++)
  {
    Acc_Cali.accel_raw[point][0] = (EEPROM.read(100 + 6 * point + 2) << 8) | EEPROM.read(100 + 6 * point + 1);
    Acc_Cali.accel_raw[point][1] = (EEPROM.read(100 + 6 * point + 4) << 8) | EEPROM.read(100 + 6 * point + 3);
    Acc_Cali.accel_raw[point][2] = (EEPROM.read(100 + 6 * point + 6) << 8) | EEPROM.read(100 + 6 * point + 5);
  }

  Acc_Cali.accel_offset[0] = (float)(Acc_Cali.accel_raw[0][0] + Acc_Cali.accel_raw[1][0]) / 2.0;
  Acc_Cali.accel_offset[1] = (float)(Acc_Cali.accel_raw[2][1] + Acc_Cali.accel_raw[3][1]) / 2.0;
  Acc_Cali.accel_offset[2] = (float)(Acc_Cali.accel_raw[4][2] + Acc_Cali.accel_raw[5][2]) / 2.0;

  for (point = 0; point < 3; point++)
    Acc_Cali.a[0][point] = (float)Acc_Cali.accel_raw[0][point] - Acc_Cali.accel_offset[point];
  for (point = 0; point < 3; point++)
    Acc_Cali.a[1][point] = (float)Acc_Cali.accel_raw[2][point] - Acc_Cali.accel_offset[point];
  for (point = 0; point < 3; point++)
    Acc_Cali.a[2][point] = (float)Acc_Cali.accel_raw[4][point] - Acc_Cali.accel_offset[point];

  Acc_Cali.T[0][0] = (Acc_Cali.g * (Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[1][2] * Acc_Cali.a[2][1])) / (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][0] * Acc_Cali.a[1][2] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0] * Acc_Cali.a[2][2] + Acc_Cali.a[0][1] * Acc_Cali.a[1][2] * Acc_Cali.a[2][0] + Acc_Cali.a[0][2] * Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1] * Acc_Cali.a[2][0]);
  Acc_Cali.T[0][1] = -(Acc_Cali.g * (Acc_Cali.a[0][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][2] * Acc_Cali.a[2][1])) / (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][0] * Acc_Cali.a[1][2] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0] * Acc_Cali.a[2][2] + Acc_Cali.a[0][1] * Acc_Cali.a[1][2] * Acc_Cali.a[2][0] + Acc_Cali.a[0][2] * Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1] * Acc_Cali.a[2][0]);
  Acc_Cali.T[0][2] = (Acc_Cali.g * (Acc_Cali.a[0][1] * Acc_Cali.a[1][2] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1])) / (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][0] * Acc_Cali.a[1][2] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0] * Acc_Cali.a[2][2] + Acc_Cali.a[0][1] * Acc_Cali.a[1][2] * Acc_Cali.a[2][0] + Acc_Cali.a[0][2] * Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1] * Acc_Cali.a[2][0]);

  Acc_Cali.T[1][0] = -(Acc_Cali.g * (Acc_Cali.a[1][0] * Acc_Cali.a[2][2] - Acc_Cali.a[1][2] * Acc_Cali.a[2][0])) / (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][0] * Acc_Cali.a[1][2] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0] * Acc_Cali.a[2][2] + Acc_Cali.a[0][1] * Acc_Cali.a[1][2] * Acc_Cali.a[2][0] + Acc_Cali.a[0][2] * Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1] * Acc_Cali.a[2][0]);
  Acc_Cali.T[1][1] = (Acc_Cali.g * (Acc_Cali.a[0][0] * Acc_Cali.a[2][2] - Acc_Cali.a[0][2] * Acc_Cali.a[2][0])) / (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][0] * Acc_Cali.a[1][2] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0] * Acc_Cali.a[2][2] + Acc_Cali.a[0][1] * Acc_Cali.a[1][2] * Acc_Cali.a[2][0] + Acc_Cali.a[0][2] * Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1] * Acc_Cali.a[2][0]);
  Acc_Cali.T[1][2] = -(Acc_Cali.g * (Acc_Cali.a[0][0] * Acc_Cali.a[1][2] - Acc_Cali.a[0][2] * Acc_Cali.a[1][0])) / (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][0] * Acc_Cali.a[1][2] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0] * Acc_Cali.a[2][2] + Acc_Cali.a[0][1] * Acc_Cali.a[1][2] * Acc_Cali.a[2][0] + Acc_Cali.a[0][2] * Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1] * Acc_Cali.a[2][0]);

  Acc_Cali.T[2][0] = (Acc_Cali.g * (Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[1][1] * Acc_Cali.a[2][0])) / (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][0] * Acc_Cali.a[1][2] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0] * Acc_Cali.a[2][2] + Acc_Cali.a[0][1] * Acc_Cali.a[1][2] * Acc_Cali.a[2][0] + Acc_Cali.a[0][2] * Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1] * Acc_Cali.a[2][0]);
  Acc_Cali.T[2][1] = -(Acc_Cali.g * (Acc_Cali.a[0][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[2][0])) / (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][0] * Acc_Cali.a[1][2] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0] * Acc_Cali.a[2][2] + Acc_Cali.a[0][1] * Acc_Cali.a[1][2] * Acc_Cali.a[2][0] + Acc_Cali.a[0][2] * Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1] * Acc_Cali.a[2][0]);
  Acc_Cali.T[2][2] = (Acc_Cali.g * (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0])) / (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][0] * Acc_Cali.a[1][2] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0] * Acc_Cali.a[2][2] + Acc_Cali.a[0][1] * Acc_Cali.a[1][2] * Acc_Cali.a[2][0] + Acc_Cali.a[0][2] * Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1] * Acc_Cali.a[2][0]);
}

// Second Order Low Pass Filter for Gyroscope data
void Quadrotor::FilterInit(void)
{
  Quadrotor::SecondOrderLowPassFilter(400.0, 30.0, &gyroFilterParameterX);
  Quadrotor::SecondOrderLowPassFilter(400.0, 30.0, &gyroFilterParameterY);
  Quadrotor::SecondOrderLowPassFilter(400.0, 30.0, &gyroFilterParameterZ);

  Quadrotor::SecondOrderLowPassFilter(400.0, 30.0, &accelFilterParameterX);
  Quadrotor::SecondOrderLowPassFilter(400.0, 30.0, &accelFilterParameterY);
  Quadrotor::SecondOrderLowPassFilter(400.0, 30.0, &accelFilterParameterZ);
}

// Sensor Estimation Methods
// Method 1: AHRS
// Method 2: Nonlinear Complementary Filter
void Quadrotor::Estimation(int8_t method)
{
  // Choosing one of the methods
  if (method == 1)
  {
    Quadrotor::AHRS();
  }
  else if (method == 2)
  {
    Quadrotor::NonlinearComplementaryFilter();
  }
  else
  {
    Quadrotor::AHRS();
  }
}

// Nonlinear Complementary Filter
// Source: S. Tellex, A. Brown, and S. Lupashin. Estimation for Quadrotors. June 11, 2018.
void Quadrotor::NonlinearComplementaryFilter(void)
{
  Quadrotor::L3GD20read();
  Quadrotor::LSM303Dread();

  accelAngle.x = -atan(accel.calibrated.x / accel.calibrated.z); // Roll [rad]
  accelAngle.y = atan(accel.calibrated.y / accel.calibrated.z); // Pitch [rad]

  _XYZ gyroAngle_rad;
  gyroAngle_rad.x = gyroAngle.x * PI / 180;
  gyroAngle_rad.y = -gyroAngle.y* PI / 180;
  gyroAngle_rad.z = -gyroAngle.z * PI / 180;


  // Nonlinear Complementary Filter using Quaternions
  double tau = (0.90) / (1 - 0.90) * dt;
  quat.q[0] = cos(gyroAngle_rad.x / 2) * cos(gyroAngle_rad.y / 2) * cos(gyroAngle_rad.z / 2)   + sin(gyroAngle_rad.x / 2)  * sin(gyroAngle_rad.y / 2) * sin(gyroAngle_rad.z / 2);
  quat.q[1] = -cos(gyroAngle_rad.x / 2) * sin(gyroAngle_rad.y / 2) * sin(gyroAngle_rad.z / 2)   + cos(gyroAngle_rad.y / 2) * cos(gyroAngle_rad.z / 2)   * sin(gyroAngle_rad.x / 2);
  quat.q[2] = cos(gyroAngle_rad.x / 2) * cos(gyroAngle_rad.z / 2)   * sin(gyroAngle_rad.y / 2) + sin(gyroAngle_rad.x / 2)  * cos(gyroAngle_rad.y / 2) * sin(gyroAngle_rad.z / 2);
  quat.q[3] = cos(gyroAngle_rad.x / 2) * cos(gyroAngle_rad.y / 2) * sin(gyroAngle_rad.z / 2)   - sin(gyroAngle_rad.x / 2)  * cos(gyroAngle_rad.z / 2)   * sin(gyroAngle_rad.y / 2);

  Quaternion dq;

  double omega[3] = {gyro.calibrated.x, gyro.calibrated.y, gyro.calibrated.z};
  double theta = (double)sqrt(omega[0] * omega[0] * dt + omega[1] * omega[1] * dt + omega[2] * omega[2] * dt);
  if (theta < 1e-8) {
    dq = dq;
  } else {
    dq.q[0] = cos(theta / 2.f);
    dq.q[1] = sin(theta / 2.f) * omega[0] * dt / theta;
    dq.q[2] = -sin(theta / 2.f) * omega[1] * dt / theta;
    dq.q[3] = -sin(theta / 2.f) * omega[2] * dt / theta;
  }

  quat = Quaternion().multiply(dq, quat).normalize();

  // Convert to Euler
  double roll_predicted, pitch_predicted, yaw_predicted;
  roll_predicted = atan2(2 * (quat.q[2] * quat.q[3] + quat.q[0] * quat.q[1]),  quat.q[0] * quat.q[0] - quat.q[1] * quat.q[1] - quat.q[2] * quat.q[2] + quat.q[3] * quat.q[3]);
  pitch_predicted = -asin(2 * (quat.q[1] * quat.q[3] - quat.q[0] * quat.q[2]));
  yaw_predicted = atan2(2 * (quat.q[1] * quat.q[2] + quat.q[0] * quat.q[3]),  quat.q[0] * quat.q[0] + quat.q[1] * quat.q[1] - quat.q[2] * quat.q[2] - quat.q[3] * quat.q[3]);


  X.phi = (tau / (tau + dt)) * (roll_predicted) + (dt / (tau + dt)) * accelAngle.x;
  X.theta = (tau / (tau + dt)) * (pitch_predicted) + (dt / (tau + dt)) * accelAngle.y;
  X.psi = yaw_predicted;
}

// Attitude and Heading Reference Systems (AHRS)
// S. Madgwick (http://x-io.co.uk/res/doc/madgwick_internal_report.pdf)
// Source: Shi Lu (https://github.com/ragewrath/Mark3-Copter-Pilot)
void Quadrotor::AHRS(void)
{
  Quadrotor::L3GD20read();
  Quadrotor::LSM303Dread();

  gyro.filtered.x = Quadrotor::SecondOrderLowPassFilterApply(30.0, gyro.calibrated.x, &gyroFilterParameterX);
  gyro.filtered.y = Quadrotor::SecondOrderLowPassFilterApply(30.0, gyro.calibrated.y, &gyroFilterParameterY);
  gyro.filtered.z = Quadrotor::SecondOrderLowPassFilterApply(30.0, gyro.calibrated.z, &gyroFilterParameterZ);

  accel.filtered.x = Quadrotor::SecondOrderLowPassFilterApply(30.0, accel.calibrated.x, &accelFilterParameterX);
  accel.filtered.y = Quadrotor::SecondOrderLowPassFilterApply(30.0, accel.calibrated.y, &accelFilterParameterY);
  accel.filtered.z = Quadrotor::SecondOrderLowPassFilterApply(30.0, accel.calibrated.z, &accelFilterParameterZ);

  Quadrotor::MadgwickMARG();
}

void Quadrotor::MadgwickMARG()
{
  float quat[4], ypr[3], gx, gy, gz, AHRS_val[6];
  AHRS_val[0] = accel.filtered.y / 8192.0;
  AHRS_val[1] = accel.filtered.x / 8192.0;
  AHRS_val[2] = accel.filtered.z / 8192.0;
  AHRS_val[3] = gyro.filtered.x * -1.0;
  AHRS_val[4] = gyro.filtered.y;
  AHRS_val[5] = gyro.filtered.z * -1.0;

  Quadrotor::MahonyAHRS(AHRS_val[3] * PI / 180, AHRS_val[4] * PI / 180, AHRS_val[5] * PI / 180, AHRS_val[0], AHRS_val[1], AHRS_val[2]);

  quat[0] = q.w;
  quat[1] = q.x;
  quat[2] = q.y;
  quat[3] = q.z;

  gx = 2 * (quat[1] * quat[3] - quat[0] * quat[2]);
  gy = 2 * (quat[0] * quat[1] + quat[2] * quat[3]);
  gz = quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2] + quat[3] * quat[3];

  ypr[0] = atan2(2 * quat[1] * quat[2] - 2 * quat[0] * quat[3], 2 * quat[0] * quat[0] + 2 * quat[1] * quat[1] - 1);
  ypr[1] = atan(gx / sqrt(gy * gy + gz * gz));
  ypr[2] = atan(gy / sqrt(gx * gx + gz * gz));

  float p_deg = gyro.filtered.x;
  float q_deg  = -gyro.filtered.y;
  float r_deg  = -gyro.filtered.z;
  X.p = p_deg * PI / 180.0;
  X.q = q_deg * PI / 180.0;
  X.r = r_deg * PI / 180.0;

  X.phi = -ypr[2];
  X.theta = ypr[1];
  X.psi = -ypr[0];

  trig.phi_sin = sin(X.phi);
  trig.theta_sin = sin(X.theta);
  trig.psi_sin = sin(X.psi);
  trig.phi_cos = cos(X.phi);
  trig.theta_cos = cos(X.theta);
  trig.psi_cos = cos(X.psi);
}

void Quadrotor::MahonyAHRS(float gx, float gy, float gz, float ax, float ay, float az) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q.x * q.z - q.w * q.y;
    halfvy = q.w * q.x + q.y * q.z;
    halfvz = q.w * q.w - 0.5f + q.z * q.z;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / FREQ);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / FREQ);
      integralFBz += twoKi * halfez * (1.0f / FREQ);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / FREQ));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / FREQ));
  gz *= (0.5f * (1.0f / FREQ));
  qa = q.w;
  qb = q.x;
  qc = q.y;
  q.w += (-qb * gx - qc * gy - q.z * gz);
  q.x += (qa * gx + qc * gz - q.z * gy);
  q.y += (qa * gy - qb * gz + q.z * gx);
  q.z += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  q.w *= recipNorm;
  q.x *= recipNorm;
  q.y *= recipNorm;
  q.z *= recipNorm;
}

// Quadrotor States of Operation
void Quadrotor::ArmingState(void)
{
  if (QuadrotorState == START_MODE && channel.CH3 < 1020 && channel.CH4 < 1020)
  {
    QuadrotorState = TRANS_MODE;
  }
  if (QuadrotorState == TRANS_MODE && channel.CH4 > 1400 && channel.CH4 < 1800)
  {
    QuadrotorState = ARMING_MODE;
    // Reset variables
    error.p_integral = 0;
    error.q_integral = 0;
    error.r_integral = 0;
    error.phi_integral = 0;
    error.theta_integral = 0;
    error.psi_integral = 0;
  }
  if (QuadrotorState == ARMING_MODE && channel.CH3 < 1020 && channel.CH4 < 1020)
  {
    QuadrotorState = TEMP_MODE;
  }
  if (QuadrotorState == TEMP_MODE && channel.CH4 > 1400 && channel.CH4 < 1800)
  {
    QuadrotorState = START_MODE;
  }
  if (channel.CH3 < 1020 && channel.CH4 > 1800)
  {
    QuadrotorState = DISARMING_MODE;
  }
}

// Checking Battery Voltage
void Quadrotor::BatteryVoltageCheck(void)
{
  float v = (float)analogRead(A14) * 0.020462;
  voltage = v * 0.08 + voltage * 0.92;
  if (voltage < 10.8)
  {
    digitalWrite(LEDRed, HIGH);
    digitalWrite(LEDGreen, LOW);
  }
}

// RC Remote Signal Receiver
void Quadrotor::Receiver(void)
{
  // Remote range from 1000 to 1850
  // positive angles (1433 to 1850), negative angles (1000 to 1417), and zero (1417 to 1433)
  Xdes.phi = 0;
  if (channel.CH1 > 1450) {
    float phi_desired_degree = (channel.CH1 - 1433) / 20;
    Xdes.phi = phi_desired_degree * (PI / 180.0);
  }
  else if (channel.CH1 < 1400) {
    float phi_desired_degree = (channel.CH1 - 1417) / 20;
    Xdes.phi = phi_desired_degree * (PI / 180.0);
  }
  Quadrotor::CONSTRAIN(Xdes.phi, -0.35, 0.35);

  Xdes.theta = 0;
  if (channel.CH2 > 1450) {
    float theta_desired_degree = (channel.CH2 - 1433) / 20;
    Xdes.theta = theta_desired_degree * (PI / 180.0);
  }
  else if (channel.CH2 < 1400) {
    float theta_desired_degree = (channel.CH2 - 1417) / 20;
    Xdes.theta = theta_desired_degree * (PI / 180.0);
  }
  Quadrotor::CONSTRAIN(Xdes.theta, -0.35, 0.35);

  RCYawRate = 0;
  if (channel.CH4 > 1433) {
    float psi_desired_degree = (channel.CH4 - 1433) / 20;
    RCYawRate = psi_desired_degree * (PI / 180.0);
  }
  else if (channel.CH4 < 1417) {
    float psi_desired_degree = (channel.CH4 - 1417) / 20;
    RCYawRate = psi_desired_degree * (PI / 180.0);
  }
  Quadrotor::CONSTRAIN(RCYawRate, -0.35, 0.35);
}

// Attitude Control (Outer-loop at 100Hz)
// P Controller
void Quadrotor::AttitudeControl(void)
{
  static float yaw_target = 0;
  outerCounter++;
  if (outerCounter >= 4)
  {
    outerCounter = 0;
    Quadrotor::Receiver();

    error.phi = Xdes.phi - X.phi;
    Xdes.p = kpx * error.phi;
    Xdes.p = Quadrotor::CONSTRAIN(Xdes.p, -4.4, 4.4);

    error.theta = Xdes.theta - X.theta;
    Xdes.q = kpy * error.theta;
    Xdes.q = Quadrotor::CONSTRAIN(Xdes.q, -4.4, 4.4);

    // Note: Currently not used with Radio Control (RC)
    error.psi = yaw_target - X.psi;
    (error.psi < -PI ? error.psi+(2*PI) : (error.psi > PI ? error.psi - (2*PI): error.psi));
    Xdes.r = kpz * error.psi;
    Xdes.r = Quadrotor::CONSTRAIN(Xdes.r, -2*PI, 2*PI);
    if (RCYawRate >= 0.09 || RCYawRate <= -0.09)
    {
      Xdes.r = RCYawRate;
      yaw_target = X.psi;
    }
    if (channel.CH3 < 1080)
    {
      yaw_target = X.psi;
    }
  }
  Quadrotor::AngularRateControl();
}

// Angular Rate Control or Body Rate Control (Inner-loop at 400Hz)
// PID Controller (PD Controller is sufficient)
void Quadrotor::AngularRateControl(void)
{
  error.p = Xdes.p - X.p;
  error.p_integral += error.p;
  error.p_integral = Quadrotor::CONSTRAIN(error.p_integral, -70, 70);
  U2 = error.p * kpPQRx + error.p_integral * kiPQRx * dt + (error.p - error.p_prev) * kdPQRx / dt;
  error.p_prev = error.p;

  error.q = Xdes.q - X.q;
  error.q_integral += error.q;
  error.q_integral = Quadrotor::CONSTRAIN(error.q_integral, -70, 70);
  U3 = error.q * kpPQRy + error.q_integral * kiPQRy * dt + (error.q - error.q_prev) * kdPQRy / dt;
  error.q_prev = error.q;

  // Note: Using Radio Control (RC) we control the yaw angular rate (NOT yaw angle)
  error.r = Xdes.r - X.r;
  error.r_integral += error.r;
  error.r_integral = Quadrotor::CONSTRAIN(error.r_integral, -70, 70);
  U4 = error.r * kpPQRz + error.r_integral * kiPQRz * dt + (error.r - error.r_prev) * kdPQRz / dt;
  error.r_prev = error.r;

  U2 = Quadrotor::CONSTRAIN(U2, -500, 500);
  U3 = Quadrotor::CONSTRAIN(U3, -500, 500);
  U4 = Quadrotor::CONSTRAIN(U4, -500, 500);
}

// Thrust to PWM Signal Mapping
// Source: M. Faessler, D. Falanga, and D. Scaramuzza, "Thrust Mixing, Saturation, and Body-Rate Control for Accurate Aggressive Quadrotor Flight," IEEE Robot. Autom. Lett. (RA-L), vol. 2, no. 2, pp. 476–482, Apr. 2017.
void Quadrotor::AltitudeControl(void)
{
  float Thrust = (0.000006419 * channel.CH3 * channel.CH3) + (-0.008914 * channel.CH3) + 2.06;
  Thrust = Quadrotor::CONSTRAIN(Thrust, 0, 7);
  if (QuadrotorState == ARMING_MODE)
  {
    U1 = Thrust;
  }
  else
  {
    U1 = 0;
  }
}

// Generate Motor Commands
// Shi Lu (https://github.com/ragewrath/Mark3-Copter-Pilot)
void Quadrotor::GenerateMotorCommands(void)
{
  omega1Squared = (1/(4*k_f)) * U1 + (1/(4*armlength*k_f)) * U2 + (1/(4*armlength*k_f)) * U3 - (1/(4*armlength*k_f)) * U4;
  omega2Squared = (1/(4*k_f)) * U1 - (1/(4*armlength*k_f)) * U2 + (1/(4*armlength*k_f)) * U3 + (1/(4*armlength*k_f)) * U4;
  omega3Squared = (1/(4*k_f)) * U1 - (1/(4*armlength*k_f)) * U2 - (1/(4*armlength*k_f)) * U3 - (1/(4*armlength*k_f)) * U4;
  omega4Squared = (1/(4*k_f)) * U1 + (1/(4*armlength*k_f)) * U2 - (1/(4*armlength*k_f)) * U3 + (1/(4*armlength*k_f)) * U4;
  omega1Squared = Quadrotor::CONSTRAIN(omega1Squared, 0, 9000000);
  omega2Squared = Quadrotor::CONSTRAIN(omega2Squared, 0, 9000000);
  omega3Squared = Quadrotor::CONSTRAIN(omega3Squared, 0, 9000000);
  omega4Squared = Quadrotor::CONSTRAIN(omega4Squared, 0, 9000000);
  omega1 = sqrt(omega1Squared);
  omega2 = sqrt(omega2Squared);
  omega3 = sqrt(omega3Squared);
  omega4 = sqrt(omega4Squared);

  // Motor Model
  if (QuadrotorState == ARMING_MODE && channel.CH3 > 1080)
  {
    float param_a = 888.7, param_b = 3899, param_c = 1120000, param_d = 4718, param_e = 932.6;
    PWM1 = (omega1 * omega1 + param_b * omega1 + param_c) / (param_a * voltage + param_d) + param_e;
    PWM2 = (omega2 * omega2 + param_b * omega2 + param_c) / (param_a * voltage + param_d) + param_e;
    PWM3 = (omega3 * omega3 + param_b * omega3 + param_c) / (param_a * voltage + param_d) + param_e;
    PWM4 = (omega4 * omega4 + param_b * omega4 + param_c) / (param_a * voltage + param_d) + param_e;
    PWM1 = Quadrotor::CONSTRAIN(PWM1, 1080, 1600);
    PWM2 = Quadrotor::CONSTRAIN(PWM2, 1080, 1600);
    PWM3 = Quadrotor::CONSTRAIN(PWM3, 1080, 1600);
    PWM4 = Quadrotor::CONSTRAIN(PWM4, 1080, 1600);
    volt1 = (PWM1 - 1000)/1000 * voltage;
    volt2 = (PWM2 - 1000)/1000 * voltage;
    volt3 = (PWM3 - 1000)/1000 * voltage;
    volt4 = (PWM4 - 1000)/1000 * voltage;
  }
  if (QuadrotorState != ARMING_MODE || channel.CH3 < 1080)
  {
    PWM1 = 1000;
    PWM2 = 1000;
    PWM3 = 1000;
    PWM4 = 1000;
  }
  if (QuadrotorState == DISARMING_MODE)
  {
    PWM1 = 0;
    PWM2 = 0;
    PWM3 = 0;
    PWM4 = 0;
  }

  Quadrotor::MotorRun();
}

// Run Motors
void Quadrotor::MotorRun(void)
{
  float input1 = PWM_FACTOR * PWM1;
  float input2 = PWM_FACTOR * PWM2;
  float input3 = PWM_FACTOR * PWM3;
  float input4 = PWM_FACTOR * PWM4;
  analogWrite(MOTOR1, input1);
  analogWrite(MOTOR2, input2);
  analogWrite(MOTOR3, input3);
  analogWrite(MOTOR4, input4);
}

// XbeeZigbee Send Data Wirelessly
// Source: (https://github.com/mattzzw/Arduino-mpu6050)
void Quadrotor::XbeeZigbeeSend(void)
{
  if (Serial1.available())
  {
    char command_received;
    command_received = Serial1.read();
    if (command_received == '.')
    {
      Serial1.print(U1, 2);
      Serial1.print(", ");
      Serial1.print(PWM1, 2);
      Serial1.print(", ");
      Serial1.println(omega1, 2);
    }
  }
}

// XbeeZigbee Receive Data Wirelessly
// Source: (http://forum.arduino.cc/index.php?topic=396450)
void Quadrotor::XbeeZigbeeReceive(void)
{
  // Update at frequecy of 50 Hz
  Xbee_couter++;
  if (Xbee_couter >= 8)
  {
    Xbee_couter = 0;
    if (newData == true)
    {
      strcpy(tempChars, receivedChars);

      // Parse data
      char * strtokIndx; // this is used by strtok() as an index

      strtokIndx = strtok(tempChars,","); // get the first part - the string
      float1 = atof(strtokIndx); // value 1

      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      float2 = atof(strtokIndx);     // value 2

      strtokIndx = strtok(NULL, ",");
      float3 = atof(strtokIndx);     // value 3

      newData = false;
    }
  }
}

// Second Order Low Pass Filter
// Source: Leonard Hall (https://github.com/PX4/Firmware) PX4/Firmware/src/lib/mathlib/math/filter/LowPassFilter2p.cpp
void Quadrotor::SecondOrderLowPassFilter(float sample_freq, float cutoff_freq, struct _FILTER *input_IIR)
{
  if (cutoff_freq <= 0.0f) {
    // no filtering
    return;
  }
  float fr = sample_freq / cutoff_freq;
  float ohm = tanf(PI / fr);
  float c = 1.0f + 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm;
  input_IIR->b0 = ohm * ohm / c;
  input_IIR->b1 = 2.0f * input_IIR->b0;
  input_IIR->b2 = input_IIR->b0;
  input_IIR->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
  input_IIR->a2 = (1.0f - 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm) / c;
}

// Second Order Low Pass Filter Apply
// Source: Leonard Hall (https://github.com/PX4/Firmware) PX4/Firmware/src/lib/mathlib/math/filter/LowPassFilter2p.cpp
float Quadrotor::SecondOrderLowPassFilterApply(float cutoff_freq, float sample, struct _FILTER *input_IIR)
{
  if (cutoff_freq <= 0.0f) {
    // no filtering
    return sample;
  }
  // do the filtering
  input_IIR->element0 = sample - input_IIR->element1 * input_IIR->a1 - input_IIR->element2 * input_IIR->a2;
  float output = input_IIR->element0 * input_IIR->b0 + input_IIR->element1 * input_IIR->b1 + input_IIR->element2 * input_IIR->b2;

  input_IIR->element2 = input_IIR->element1;
  input_IIR->element1 = input_IIR->element0;

  // return the value.  Should be no need to check limits
  return output;
}

// Constrain Data Between Min and Max Values
float Quadrotor::CONSTRAIN(float x, float min, float max)
{
  if (x < min) x = min;
  if (x > max) x = max;
  return x;
}

// Invert Square Root
float Quadrotor::invSqrt(float number) {
  long i;
  float x2, y;
  const float threehalfs = 1.5F;

  x2 = number * 0.5F;
  y  = number;
  i  = * ( long * ) &y;
  i  = 0x5f3759df - ( i >> 1 );
  y  = * ( float * ) &i;
  y  = y * ( threehalfs - ( x2 * y * y ) );
  return y;
}
