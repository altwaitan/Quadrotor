#include "Quadrotor.h"

// Motor initialization
void Quadrotor::MotorInit(void)
{
  voltage = (float)analogRead(A14) * 0.019586;
  pinMode(MOTOR1, OUTPUT);
  pinMode(MOTOR2, OUTPUT);
  pinMode(MOTOR3, OUTPUT);
  pinMode(MOTOR4, OUTPUT);
  analogWriteFrequency(MOTOR1, 400);
  analogWriteFrequency(MOTOR2, 400);
  analogWriteFrequency(MOTOR3, 400);
  analogWriteFrequency(MOTOR4, 400);
  analogWriteResolution(16);

  // Motors initialization
  float PWM = PWM_FACTOR * OFF_MOTOR_LEVEL;
  analogWrite(MOTOR1, PWM);
  analogWrite(MOTOR2, PWM);
  analogWrite(MOTOR3, PWM);
  analogWrite(MOTOR4, PWM);
}

// Sensor initialization and calibration
// Source: Pololu Robotics and Electronics (https://github.com/pololu/lsm303-arduino)
// Source: Andrea Vitali (DT0053 Design tip - 6-point tumble sensor calibration)
void Quadrotor::SensorInit(void)
{
  Wire.begin();
  Wire.setRate(I2C_RATE_2000);
  Wire1.begin();
  Wire1.setRate(I2C_RATE_2000);

  // Gyroscope initialization
  imu.Init();

  // Sensor calibration
  Serial.println("Sensor Calibration...");
  Serial.println("Place Quadrotor on level.");
  delay(1000);
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
      Quadrotor::AccelCalibration(4);
    }
    accel_calibration_done = 0;

    Serial.println("On Back, and wait for 10 seconds");
    delay(10000);
    while (accel_calibration_done == 0)
    {
      Quadrotor::AccelCalibration(5);
    }
    accel_calibration_done = 0;

    Serial.println("Right Wing Up, and wait for 10 seconds");
    delay(10000);
    while (accel_calibration_done == 0)
    {
      Quadrotor::AccelCalibration(3);
    }
    accel_calibration_done = 0;

    Serial.println("Left Wing Up, and wait for 10 seconds");
    delay(10000);
    while (accel_calibration_done == 0)
    {
      Quadrotor::AccelCalibration(2);
    }
    accel_calibration_done = 0;

    Serial.println("Nose Up, and wait for 10 seconds");
    delay(10000);
    while (accel_calibration_done == 0)
    {
      Quadrotor::AccelCalibration(0);
    }
    accel_calibration_done = 0;

    Serial.println("Nose Down, and wait for 10 seconds");
    delay(10000);
    while (accel_calibration_done == 0)
    {
      Quadrotor::AccelCalibration(1);
    }
    accel_calibration_done = 1;
  }

  Quadrotor::AccelOffsetRead();
  accel_calibration_done = 1;
  while (gyro_calibration_done == 0 && gyro_calibration_counter <= 1500)
  {
    gyro_calibration_counter++;
    GyroCalibration();
  }
  Serial.println("Sensor Calibration... Done!");
}

// Read Gyroscope Data
// Source: Shi Lu (https://github.com/ragewrath/Mark3-Copter-Pilot)
void Quadrotor::IMUread(void)
{
  imu.read();
  accel.raw.x = imu.a.x;
  accel.raw.y = imu.a.y;
  accel.raw.z = imu.a.z;
  gyro.raw.x = imu.g.x;
  gyro.raw.y = imu.g.y;
  gyro.raw.z = imu.g.z;

  // Sensor Thermal Compensation
  float temp, temp2, temp3;
  temperature = (float) imu.temp / 340.0 + 36.53;
  temp = temperature;
  temp = Quadrotor::CONSTRAIN(temp, 22.5, 55.0);
  temp2 = temp * temp;
  temp3 = temp * temp * temp;
  gyro.tempOffset.x = 0.000263 * temp3 - 0.03098 * temp2 + 0.03939 * temp - 29.4;
  gyro.tempOffset.y = -0.0004279 * temp3 + 0.05322 * temp2 - 2.941 * temp + 80.16;
  gyro.tempOffset.z = 0.0004163 * temp3 - 0.0332 * temp2 + 0.6652 * temp + 23.3;
  gyro.calibrated.x = (gyro.raw.x - gyro.tempOffset.x);
  gyro.calibrated.y = (gyro.raw.y - gyro.tempOffset.y);
  gyro.calibrated.z = (gyro.raw.z - gyro.tempOffset.z);

  if (gyro_calibration_done == 1)
  {
    gyro.calibrated.x = gyro.calibrated.x - gyro.gyroOffset.x;
    gyro.calibrated.y = gyro.calibrated.y - gyro.gyroOffset.y;
    gyro.calibrated.z = gyro.calibrated.z - gyro.gyroOffset.z;

    gyro.filtered.x = Quadrotor::SecondOrderLowPassFilterApply(30.0, gyro.calibrated.x, &gyroFilterParameterX);
    gyro.filtered.y = Quadrotor::SecondOrderLowPassFilterApply(30.0, -gyro.calibrated.y, &gyroFilterParameterY);
    gyro.filtered.z = Quadrotor::SecondOrderLowPassFilterApply(30.0, -gyro.calibrated.z, &gyroFilterParameterZ);

    float p_deg = gyro.filtered.x / 32.768;
    float q_deg = gyro.filtered.y / 32.768;
    float r_deg = gyro.filtered.z / 32.768;

    X.p = p_deg * (PI/180);
    X.q = q_deg * (PI/180);
    X.r = r_deg * (PI/180);

    gyroAngle.x += p_deg * dt;
    gyroAngle.y += q_deg * dt;
    gyroAngle.z += r_deg * dt;
  }

  if (accel_calibration_done == 1)
  {
    accel.afterOffset.x = (float)accel.raw.x - Acc_Cali.accel_offset[0];
    accel.afterOffset.y = (float)accel.raw.y - Acc_Cali.accel_offset[1];
    accel.afterOffset.z = (float)accel.raw.z - Acc_Cali.accel_offset[2];

    accel.calibrated.x = accel.afterOffset.x *  Acc_Cali.T[0][0] + accel.afterOffset.y *  Acc_Cali.T[1][0] + accel.afterOffset.z *  Acc_Cali.T[2][0];
    accel.calibrated.y = accel.afterOffset.x *  Acc_Cali.T[0][1] + accel.afterOffset.y *  Acc_Cali.T[1][1] + accel.afterOffset.z *  Acc_Cali.T[2][1];
    accel.calibrated.z = accel.afterOffset.x *  Acc_Cali.T[0][2] + accel.afterOffset.y *  Acc_Cali.T[1][2] + accel.afterOffset.z *  Acc_Cali.T[2][2];

    accel.filtered.x = Quadrotor::SecondOrderLowPassFilterApply(30.0, accel.calibrated.x, &accelFilterParameterX);
    accel.filtered.y = Quadrotor::SecondOrderLowPassFilterApply(30.0, accel.calibrated.y, &accelFilterParameterY);
    accel.filtered.z = Quadrotor::SecondOrderLowPassFilterApply(30.0, accel.calibrated.z, &accelFilterParameterZ);
  }
}

// Gyroscope calibration
// Source: Pololu Robotics and Electronics (https://github.com/pololu/lsm303-arduino)
// Source: Andrea Vitali (DT0053 Design tip - 6-point tumble sensor calibration)
void Quadrotor::GyroCalibration(void)
{
  if (gyro_calibration_counter > 400 && gyro_calibration_counter < 1001)
  {
    Quadrotor::IMUread();
    GyroCollection[0] += gyro.calibrated.x;
    GyroCollection[1] += gyro.calibrated.y;
    GyroCollection[2] += gyro.calibrated.z;
  }
  if (gyro_calibration_counter == 1001)
  {
    gyro.gyroOffset.x = GyroCollection[0] / 600;
    gyro.gyroOffset.y = GyroCollection[1] / 600;
    gyro.gyroOffset.z = GyroCollection[2] / 600;
    GyroCollection[0] = 0;
    GyroCollection[1] = 0;
    GyroCollection[2] = 0;
    gyro_calibration_done = 1;
  }
}

// Accelerometer Calibration
// Source: Andrea Vitali (DT0053 Design tip - 6-point tumble sensor calibration)
// Source: Shi Lu (https://github.com/ragewrath/Mark3-Copter-Pilot)
void Quadrotor::AccelCalibration(uint8_t point)
{
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
    Quadrotor::IMUread();
    Acc_Cali.accel_calitmpx += accel.raw.x;
    Acc_Cali.accel_calitmpy += accel.raw.y;
    Acc_Cali.accel_calitmpz += accel.raw.z;
    Acc_Cali.accel_timer++;
  }
}

// Accelerometer Offset Calculation
// Source: Andrea Vitali (DT0053 Design tip - 6-point tumble sensor calibration)
// Source: Shi Lu (https://github.com/ragewrath/Mark3-Copter-Pilot)
void Quadrotor::AccelOffsetRead(void)
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

  Serial.println("------------------------");
  Serial.println("Accelerometer Offset: ");
  Serial.println("------------------------");
  Serial.print(Acc_Cali.accel_offset[0]);
  Serial.print(", ");
  Serial.print(Acc_Cali.accel_offset[1]);
  Serial.print(", ");
  Serial.println(Acc_Cali.accel_offset[2]);

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


  Serial.println("------------------------");
  Serial.println("Accelerometer Calibration: ");
  Serial.println("------------------------");
  Serial.print(Acc_Cali.T[0][0]);
  Serial.print(", ");
  Serial.print(Acc_Cali.T[0][1]);
  Serial.print(", ");
  Serial.println(Acc_Cali.T[0][2]);
  Serial.print(Acc_Cali.T[1][0]);
  Serial.print(", ");
  Serial.print(Acc_Cali.T[1][1]);
  Serial.print(", ");
  Serial.println(Acc_Cali.T[1][2]);
  Serial.print(Acc_Cali.T[2][0]);
  Serial.print(", ");
  Serial.print(Acc_Cali.T[2][1]);
  Serial.print(", ");
  Serial.println(Acc_Cali.T[2][2]);
  Serial.println("------------------------");

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
  Quadrotor::IMUread();

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
  Quadrotor::IMUread();
  Quadrotor::MahonyAHRS();
}

void Quadrotor::MahonyAHRS()
{
  float quat[4], ypr[3], gx, gy, gz, AHRS_val[9];
  AHRS_val[0] = accel.filtered.x / 8192.0;
  AHRS_val[1] = accel.filtered.y / 8192.0;
  AHRS_val[2] = accel.filtered.z / 8192.0;
  AHRS_val[3] = X.p;
  AHRS_val[4] = -X.q;
  AHRS_val[5] = -X.r;

  Quadrotor::MahonyAHRSUpdate(AHRS_val[3], AHRS_val[4], AHRS_val[5], AHRS_val[0], AHRS_val[1], AHRS_val[2]);

  quat[0] = q.w;
  quat[1] = q.x;
  quat[2] = q.y;
  quat[3] = q.z;

  gx = 2 * (quat[1] * quat[3] - quat[0] * quat[2]);
  gy = 2 * (quat[0] * quat[1] + quat[2] * quat[3]);
  gz = quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2] + quat[3] * quat[3];

  ypr[0] = atan2(2 * quat[1] * quat[2] - 2 * quat[0] * quat[3], 2 * quat[0] * quat[0] + 2 * quat[1] * quat[1] - 1);
  ypr[1] = atan(gx / sqrt(gy * gy + gz * gz));
  ypr[2] = atan(gy / sqrt(gx * gx + gz * gz));;

  X.phi = ypr[2];
  X.theta = ypr[1];
  // When the base stations are off
  if (channel.CH5 < 1600)
  {
    X.psi = ypr[0];
  }
}

void Quadrotor::MahonyAHRSUpdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  float q0, q1, q2, q3;
  q0 = q.w;
  q1 = q.x;
  q2 = q.y;
  q3 = q.z;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

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
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  q.w = q0;
  q.x = q1;
  q.y = q2;
  q.z = q3;
}

// Quadrotor States of Operation
void Quadrotor::ArmingState(void)
{
  if (QuadrotorState == START_MODE && channel.CH3 < 320 && channel.CH4 < 320)
  {
    QuadrotorState = TRANS_MODE;
  }
  if (QuadrotorState == TRANS_MODE && channel.CH3 < 320 && channel.CH4 > 900 && channel.CH4 < 1500)
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
  if (QuadrotorState == ARMING_MODE && channel.CH3 < 320 && channel.CH4 < 320)
  {
    QuadrotorState = TEMP_MODE;
  }
  if (QuadrotorState == TEMP_MODE && channel.CH3 < 320 && channel.CH4 > 900 && channel.CH4 < 1500)
  {
    QuadrotorState = START_MODE;
  }
  if (channel.CH3 < 320 && channel.CH4 > 1600)
  {
    QuadrotorState = DISARMING_MODE;
  }
  if (channel.CH6 > 1000)
  {
    QuadrotorState = DISARMING_MODE;
  }
}

// Checking Battery Voltage
void Quadrotor::BatteryVoltageCheck(void)
{
  float v = (float)analogRead(A14) * 0.019586;
  voltage = v * 0.005 + voltage * 0.995;
  voltage = Quadrotor::CONSTRAIN(voltage, 9.0, 17.0);
  if (voltage < 10.5)
  {
    if (blink == 0)
    {
      if (blinkCounter <= 160)
      {
        blinkCounter += 1;
      }
      else
      {
        blinkCounter = 0;
        blink = 1;
      }
      digitalWrite(LEDRed, HIGH);
      digitalWrite(LEDGreen, LOW);
    }
    else
    {
      if (blinkCounter <= 160)
      {
        blinkCounter += 1;
      }
      else
      {
        blinkCounter = 0;
        blink = 0;
      }
      digitalWrite(LEDRed, LOW);
      digitalWrite(LEDGreen, LOW);
    }
  }
  else
  {
    if (blink == 0)
    {
      if (blinkCounter <= 160)
      {
        blinkCounter += 1;
      }
      else
      {
        blinkCounter = 0;
        blink = 1;
      }
      digitalWrite(LEDRed, LOW);
      digitalWrite(LEDGreen, HIGH);
    }
    else
    {
      if (blinkCounter <= 160)
      {
        blinkCounter += 1;
      }
      else
      {
        blinkCounter = 0;
        blink = 0;
      }
      digitalWrite(LEDRed, LOW);
      digitalWrite(LEDGreen, LOW);
    }
  }
}

// RC Remote Signal Receiver
void Quadrotor::Receiver(void)
{
  if (channel.CH5 < 1600)
  {
    Xdes.phi = 0;
    if (channel.CH1 > 1100)
    {
      float phi_desired_degree = (channel.CH1 - 1100) / 30;
      Xdes.phi = phi_desired_degree * (PI / 180.0);
    }
    else if (channel.CH1 < 900)
    {
      float phi_desired_degree = (channel.CH1 - 900) / 30;
      Xdes.phi = phi_desired_degree * (PI / 180.0);
    }
    Xdes.phi = Quadrotor::CONSTRAIN(Xdes.phi, -0.6, 0.6);

    Xdes.theta = 0;
    if (channel.CH2 > 1100)
    {
      float theta_desired_degree = (channel.CH2 - 1100) / 30;
      Xdes.theta = theta_desired_degree * (PI / 180.0);
    }
    else if (channel.CH2 < 900)
    {
      float theta_desired_degree = (channel.CH2 - 900) / 30;
      Xdes.theta = theta_desired_degree * (PI / 180.0);
    }
    Xdes.theta = Quadrotor::CONSTRAIN(Xdes.theta, -0.6, 0.6);

    RCYawRate = 0;
    if (channel.CH4 > 1100)
    {
      float psi_desired_degree = (channel.CH4 - 1100) / 10;
      RCYawRate = psi_desired_degree * (PI / 180.0);
    }
    else if (channel.CH4 < 900)
    {
      float psi_desired_degree = (channel.CH4 - 900) / 10;
      RCYawRate = psi_desired_degree * (PI / 180.0);
    }
    RCYawRate = Quadrotor::CONSTRAIN(RCYawRate, -1.5, 1.5);
  }
}

// Choosing the control method:
// 1. Classical PID
// 2. Pole-Placement
void Quadrotor::Control(int8_t method)
{
  control_method = method;
}

// Attitude Control (Outer-loop at 100Hz)
// P Controller
void Quadrotor::AttitudeControl(void)
{
  if (outerCounter >= 4)
  {
    Wp.input = 0;
    Wq.input = 0;
    Wr.input = 0;
    Wp.output = 0;
    Wq.output = 0;
    Wr.output = 0;

    if (QuadrotorState == ARMING_MODE)
    {
      if (channel.CH3 > 320 || (channel.CH5 > 1600 && flight_mode == 1) || (channel.CH5 > 1600 && flight_mode == 2) || (channel.CH5 > 1600 && flight_mode == 3))
      {
        error.phi = Xdes.phi - X.phi;
        Xdes.p = kpx * error.phi;

        // Prefilter
        Wp.input = Xdes.p;
        Wp.output = (0.6 * Wp.output_prev1) + (0.2 * Wp.input) + (0.2 * Wp.input_prev1);
        Wp.output = Quadrotor::CONSTRAIN(Wp.output, -6.28, 6.28);

        error.theta = Xdes.theta - X.theta;
        Xdes.q = kpy * error.theta;

        // Prefilter
        Wq.input = Xdes.q;
        Wq.output = (0.6 * Wq.output_prev1) + (0.2 * Wq.input) + (0.2 * Wq.input_prev1);
        Wq.output = Quadrotor::CONSTRAIN(Wq.output, -6.28, 6.28);

        if (channel.CH5 > 1600) // HTC Vive Base Station
        {
          // Already have a desired yaw rate
        }
        else
        {
          error.psi = Xdes.psi - X.psi;
          (error.psi < -PI ? error.psi+(2*PI) : (error.psi > PI ? error.psi - (2*PI): error.psi));
          Xdes.r = kpz * error.psi;
          Xdes.r = Quadrotor::CONSTRAIN(Xdes.r, -1, 1);

          if (RCYawRate >= 0.09 || RCYawRate <= -0.09)
          {
            Xdes.r = RCYawRate;
            Xdes.psi = X.psi;
          }
          if (channel.CH3 < 320)
          {
            Xdes.psi = X.psi;
          }
        }
      }
    }
    Wp.output_prev1 = Wp.output;
    Wp.input_prev1 = Wp.input;
    Wq.output_prev1 = Wq.output;
    Wq.input_prev1 = Wq.input;
  }
  Quadrotor::AngularRateControl();
}

// Angular Rate Control or Body Rate Control (Inner-loop at 400Hz)
// PID Controller (PD Controller is sufficient)
void Quadrotor::AngularRateControl(void)
{
  U2.current = 0;
  U3.current = 0;
  U4.current = 0;
  if (QuadrotorState == ARMING_MODE)
  {
    if (channel.CH3 > 320 || (channel.CH5 > 1600 && flight_mode == 1) || (channel.CH5 > 1600 && flight_mode == 2) || (channel.CH5 > 1600 && flight_mode == 3))
    {
      if (control_method == 1) // PID with No prefilter
      {
        error.p = Xdes.p - X.p;
        error.p_integral += error.p * dt;
        error.p_integral = Quadrotor::CONSTRAIN(error.p_integral, -1, 1);
        U2.current = error.p * kpPQRx + (error.p - error.p_prev1) * kdPQRx / dt + error.p_integral * kiPQRx;
        error.p_prev1 = error.p;

        error.q = Xdes.q - X.q;
        error.q_integral += error.q * dt;
        error.q_integral = Quadrotor::CONSTRAIN(error.q_integral, -1, 1);
        U3.current = error.q * kpPQRy + (error.q - error.q_prev1) * kdPQRy / dt + error.q_integral * kiPQRy;
        error.q_prev1 = error.q;

        // Note: Using Radio Control (RC) we control the yaw angular rate (NOT yaw angle)
        // Base Station is ON
        if (channel.CH5 > 1600)
        {
          error.r = Xdes.r - X.r;
          U4.current = error.r * kpPQRz + (error.r - error.r_prev1) * kdPQRz / dt;
          error.r_prev1 = error.r;
        }
        else // Base Station is OFF
        {
          error.r = Xdes.r - X.r;
          error.r_integral += error.r * dt;
          error.r_integral = Quadrotor::CONSTRAIN(error.r_integral, -1, 1);
          U4.current = error.r * kpPQRz + (error.r - error.r_prev1) * kdPQRz / dt + error.r_integral * kiPQRz;
          error.r_prev1 = error.r;
        }
      }
      else if (control_method == 2) // PID with prefilter (Design for Bandwidth and Robustness)
      {
        error.p = Wp.output - X.p;
        U2.current = (2.2 * U2.prev1) - (1.56 * U2.prev2) + (0.36 * U2.prev3) + (0.0918 * error.p) - (0.08645 * error.p_prev1) - (0.09174 * error.p_prev2) + (0.08652 * error.p_prev3);

        error.q = Wq.output - X.q;
        U3.current = (2.2 * U3.prev1) - (1.56 * U3.prev2) + (0.36 * U3.prev3) + (0.0918 * error.q) - (0.08645 * error.q_prev1) - (0.09174 * error.q_prev2) + (0.08652 * error.q_prev3);

        // Note: Using Radio Control (RC) we control the yaw angular rate (NOT yaw angle)
        // Base Station is ON
        if (channel.CH5 > 1600)
        {
          error.r = Xdes.r - X.r;
          U4.current = error.r * kpPQRz + (error.r - error.r_prev1) * kdPQRz / dt;
          error.r_prev1 = error.r;
        }
        else // Base Station is OFF
        {
          error.r = Xdes.r - X.r;
          error.r_integral += error.r * dt;
          error.r_integral = Quadrotor::CONSTRAIN(error.r_integral, -1, 1);
          U4.current = error.r * kpPQRz + (error.r - error.r_prev1) * kdPQRz / dt + error.r_integral * kiPQRz;
          error.r_prev1 = error.r;
        }
      }
    }
  }
  U2.current = Quadrotor::CONSTRAIN(U2.current, -2, 2);
  U2.prev3 = U2.prev2;
  U2.prev2 = U2.prev1;
  U2.prev1 = U2.current;
  U3.current = Quadrotor::CONSTRAIN(U3.current, -2, 2);
  U3.prev3 = U3.prev2;
  U3.prev2 = U3.prev1;
  U3.prev1 = U3.current;
  error.p_prev3 = error.p_prev2;
  error.p_prev2 = error.p_prev1;
  error.p_prev1 = error.p;
  error.q_prev3 = error.q_prev2;
  error.q_prev2 = error.q_prev1;
  error.q_prev1 = error.q;
  error.r_prev3 = error.r_prev2;
  error.r_prev2 = error.r_prev1;
  error.r_prev1 = error.r;
}

void Quadrotor::ThrottleControl(void)
{
  if (QuadrotorState == ARMING_MODE && channel.CH3 > 320)
  {
    float Thrust = 0.008193 * channel.CH3 - 2.458;
    U1.current = Thrust / (cos(X.phi)*cos(X.theta));
    U1.current = Quadrotor::CONSTRAIN(U1.current, 0, 15);;
  }
  else
  {
    U1.current = 0;
  }
}

void Quadrotor::DifferentialFlatness(void)
{
  if (channel.CH5 > 1600)
  {
    if (outerCounter >= 4)
    {
      if (QuadrotorState == ARMING_MODE)
      {
        if (flight_mode == 1 || flight_mode == 2 || flight_mode == 3)
        {
          U1des.current = Quadrotor::CONSTRAIN(U1des.current, 0, 15);
          Xdes.phi = Quadrotor::CONSTRAIN(Xdes.phi, -0.6, 0.6);
          Xdes.theta = Quadrotor::CONSTRAIN(Xdes.theta, -0.6, 0.6);
          Xdes.r = Quadrotor::CONSTRAIN(Xdes.r, -2, 2);

          // Assign Thrust Here
          U1.current = U1des.current;
        }
        else if (flight_mode == -1)
        {
          QuadrotorState = DISARMING_MODE;
        }
        else
        {
          U1.current = 0;
          Xdes.phi = 0;
          Xdes.theta = 0;
          Xdes.r = 0;
        }
      }
    }
  }
  else
  {
    Quadrotor::ThrottleControl();
  }
}

// Generate Motor Commands
// Shi Lu (https://github.com/ragewrath/Mark3-Copter-Pilot)
// Notes: Please read README file
void Quadrotor::GenerateMotorCommands(void)
{
  omega1Squared = 130958.617 * U1.current - 1480900 * U2.current + 1480900 * U3.current + 1290232.68 * U4.current;
  omega2Squared = 130958.617 * U1.current + 1480900 * U2.current - 1480900 * U3.current + 1290232.68 * U4.current;
  omega3Squared = 130958.617 * U1.current + 1480900 * U2.current + 1480900 * U3.current - 1290232.68 * U4.current;
  omega4Squared = 130958.617 * U1.current - 1480900 * U2.current - 1480900 * U3.current - 1290232.68 * U4.current;
  omega1Squared = Quadrotor::CONSTRAIN(omega1Squared, 0, 900000000);
  omega2Squared = Quadrotor::CONSTRAIN(omega2Squared, 0, 900000000);
  omega3Squared = Quadrotor::CONSTRAIN(omega3Squared, 0, 900000000);
  omega4Squared = Quadrotor::CONSTRAIN(omega4Squared, 0, 900000000);
  omega1 = sqrt(omega1Squared);
  omega2 = sqrt(omega2Squared);
  omega3 = sqrt(omega3Squared);
  omega4 = sqrt(omega4Squared);

  // Motor Model
  if (QuadrotorState == ARMING_MODE)
  {
    float param_a = 1166.0, param_b = 5393, param_c = 299600, param_d = 1544, param_e = 894.5;
    PWM1 = (omega1 * omega1 + param_b * omega1 + param_c) / (param_a * voltage + param_d) + param_e;
    PWM2 = (omega2 * omega2 + param_b * omega2 + param_c) / (param_a * voltage + param_d) + param_e;
    PWM3 = (omega3 * omega3 + param_b * omega3 + param_c) / (param_a * voltage + param_d) + param_e;
    PWM4 = (omega4 * omega4 + param_b * omega4 + param_c) / (param_a * voltage + param_d) + param_e;
    PWM1 = Quadrotor::CONSTRAIN(PWM1, MIN_MOTOR_LEVEL, MAX_MOTOR_LEVEL);
    PWM2 = Quadrotor::CONSTRAIN(PWM2, MIN_MOTOR_LEVEL, MAX_MOTOR_LEVEL);
    PWM3 = Quadrotor::CONSTRAIN(PWM3, MIN_MOTOR_LEVEL, MAX_MOTOR_LEVEL);
    PWM4 = Quadrotor::CONSTRAIN(PWM4, MIN_MOTOR_LEVEL, MAX_MOTOR_LEVEL);
  }
  if (QuadrotorState != ARMING_MODE)
  {
    PWM1 = OFF_MOTOR_LEVEL;
    PWM2 = OFF_MOTOR_LEVEL;
    PWM3 = OFF_MOTOR_LEVEL;
    PWM4 = OFF_MOTOR_LEVEL;
  }
  if (QuadrotorState == DISARMING_MODE)
  {
    PWM1 = OFF_MOTOR_LEVEL;
    PWM2 = OFF_MOTOR_LEVEL;
    PWM3 = OFF_MOTOR_LEVEL;
    PWM4 = OFF_MOTOR_LEVEL;
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

void Quadrotor::LoopCounter(void)
{
  if (outerCounter >= 4)
  {
    outerCounter = 0;
  }
  while (micros() - loop_timer < dtMicroseconds);
  // Reset the zero timer
  loop_timer = micros();
}
