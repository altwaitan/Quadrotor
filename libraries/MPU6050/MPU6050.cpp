#include "MPU6050.h"
#include "i2c_t3.h"

void MPU6050::Init(void)
{
  // Power Management
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPUREG_PWR_MGMT_1);
  Wire.write(MPU_CLK_SEL_PLLGYROZ);
  uint8_t rcode = Wire.endTransmission(0);
  if (rcode) {
    Serial.print(F("Write failed: "));
    Serial.println(rcode);
  }

  // Sample Rate Divider
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPUREG_SMPLRT_DIV);
  Wire.write(0x07);
  rcode = Wire.endTransmission(0);
  if (rcode) {
    Serial.print(F("Write failed: "));
    Serial.println(rcode);
  }

  // Configuration
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPUREG_CONFIG);
  Wire.write(BITS_DLPF_CFG_42HZ);
  rcode = Wire.endTransmission(0);
  if (rcode) {
    Serial.print(F("Write failed: "));
    Serial.println(rcode);
  }

  // Gyroscope Configuration
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPUREG_GYRO_CONFIG);
  Wire.write(BITS_FS_1000DPS);
  rcode = Wire.endTransmission(0);
  if (rcode) {
    Serial.print(F("Write failed: "));
    Serial.println(rcode);
  }

  // Accelerometer Configuration
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPUREG_ACCEL_CONFIG);
  Wire.write(0x08);
  rcode = Wire.endTransmission(0);
  if (rcode) {
    Serial.print(F("Write failed: "));
    Serial.println(rcode);
  }

  // INT Pin / Bypass Enable Configuration
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPUREG_INT_PIN_CFG);
  Wire.write(0x02);
  rcode = Wire.endTransmission(0);
  if (rcode) {
    Serial.print(F("Write failed: "));
    Serial.println(rcode);
  }

  // Who Am I
  uint32_t timeOutTimer;
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPUREG_WHOAMI);
  rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("Write failed: "));
    Serial.println(rcode);
  }
  Wire.requestFrom(MPU6050_ADDRESS, 1, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < 1; i++) {
    if (Wire.available())
      Data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        Data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
      }
    }
  }
}

void MPU6050::read(void)
{
  uint32_t timeOutTimer;
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPUREG_ACCEL_XOUT_H);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial1.print(F("i2cRead failed: "));
    Serial1.println(rcode);
  }
  Wire.requestFrom(MPU6050_ADDRESS, 14, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < 14; i++) {
    if (Wire.available())
      Data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        Data[i] = Wire.read();
      else {
        Serial1.println(F("i2cRead timeout"));
      }
    }
  }
}
