#include "IMU.h"
#include "i2c_t3.h"
#include <math.h>

void IMU::Init(void)
{
  IMU::I2CWrite(MPU6050_ADDRESS, MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ, 0);
  IMU::I2CWrite(MPU6050_ADDRESS, MPUREG_SMPLRT_DIV, 0x07, 0);
  IMU::I2CWrite(MPU6050_ADDRESS, MPUREG_CONFIG, BITS_DLPF_CFG_42HZ, 0);
  IMU::I2CWrite(MPU6050_ADDRESS, MPUREG_GYRO_CONFIG, BITS_FS_1000DPS, 0);
  IMU::I2CWrite(MPU6050_ADDRESS, MPUREG_ACCEL_CONFIG, 0x08, 0);
  IMU::I2CWrite(MPU6050_ADDRESS, MPUREG_INT_PIN_CFG, 0x02, 0);
  IMU::I2CRead(MPU6050_ADDRESS, MPUREG_WHOAMI, 1);

  if (data[0] != 0x68)
  {
    Serial.println("IMU Reading Error!");
  }
}

void IMU::read(void)
{
  IMU::I2CRead(MPU6050_ADDRESS, MPUREG_ACCEL_XOUT_H, 14);
  a.x = ((data[0] << 8) | data[1]);
  a.y = ((data[2] << 8) | data[3]);
  a.z = ((data[4] << 8) | data[5]);
  temp = (data[6] << 8) | data[7];
  g.x = (data[8] << 8) | data[9];
  g.y = ((data[10] << 8) | data[11]);
  g.z = ((data[12] << 8) | data[13]);
}

uint8_t IMU::I2CWrite(uint8_t address, uint8_t reg, uint8_t value, bool stop)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  uint8_t rcode = Wire.endTransmission(stop);
  if (rcode)
  {
    Serial.print(F("Write failed: "));
    Serial.println(rcode);
  }
  return rcode;
}

// http://arduino.cc/en/Reference/WireEndTransmission
uint8_t IMU::I2CRead(uint8_t address, uint8_t reg, uint8_t nbytes)
{
  uint32_t timer;
  Wire.beginTransmission(address);
  Wire.write(reg);
  uint8_t rcode = Wire.endTransmission(false);
  if (rcode)
  {
    Serial.print(F("I2CRead Failed: "));
    Serial.println(rcode);
    return rcode;
  }

  Wire.requestFrom(address, nbytes, (uint8_t)true);
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
    {
      data[i] = Wire.read();
    }
    else
    {
      timer = micros();
      while (((micros() - timer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
      {
        data[i] = Wire.read();
      }
      else
      {
        Serial.println(F("I2CRead Timeout"));
        return 5;
      }
    }
  }
  return 0;
}
