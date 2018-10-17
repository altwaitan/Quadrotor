#include "IMU.h"
#include "i2c_t3.h"
#include <math.h>

void IMU::Init(void)
{
  // L3G4200D Control register 1
  // Output Data Rate = 760 Hz, Cuff-off Freq = 50 Hz
  Wire.beginTransmission(L3G4200D_ADDRESS);
  Wire.write(L3G_CTRL_REG1);
  Wire.write(0xEF);
  uint8_t rcode = Wire.endTransmission(0);
  if (rcode) {
    Serial.print(F("Write failed: "));
    Serial.println("1");
  }

  // L3G4200D Control register 4
  // +-2000 dps
  Wire.beginTransmission(L3G4200D_ADDRESS);
  Wire.write(L3G_CTRL_REG4);
  Wire.write(0x30);
  rcode = Wire.endTransmission(0);
  if (rcode) {
    Serial.print(F("Write failed: "));
    Serial.println("2");
  }

  // LSM303 Control register 1
  // Output Data Rate = 800 Hz
  Wire.beginTransmission(LSM303D_ADDRESS);
  Wire.write(CTRL1);
  Wire.write(0x9F);
  rcode = Wire.endTransmission(0);
  if (rcode) {
    Serial.print(F("Write failed: "));
    Serial.println("3");
  }

  // LSM303 Control register 1
  // Full-scale selection +- 4g, anti-alias filter bandwidth 50 Hz
  Wire.beginTransmission(LSM303D_ADDRESS);
  Wire.write(CTRL2);
  Wire.write(0xC8);
  rcode = Wire.endTransmission(0);
  if (rcode) {
    Serial.print(F("Write failed: "));
    Serial.println("4");
  }
}

void IMU::read(void)
{
  // LSM303D
  Wire.beginTransmission(LSM303D_ADDRESS);
  // assert the MSB of the address to get the accelerometer
  // to do slave-transmit subaddress updating.
  Wire.write(OUT_X_L_A | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(LSM303D_ADDRESS, (byte)6);

  while (Wire.available() < 6);

  byte xla = Wire.read();
  byte xha = Wire.read();
  byte yla = Wire.read();
  byte yha = Wire.read();
  byte zla = Wire.read();
  byte zha = Wire.read();

  // combine high and low bytes
  // This no longer drops the lowest 4 bits of the readings from the DLH/DLM/DLHC, which are always 0
  // (12-bit resolution, left-aligned). The D has 16-bit resolution
  a.x = (int16_t)(xha << 8 | xla);
  a.y = (int16_t)(yha << 8 | yla);
  a.z = (int16_t)(zha << 8 | zla);

  // L3G4D20
  Wire.beginTransmission(L3G4200D_ADDRESS);
  // assert the MSB of the address to get the gyro
  // to do slave-transmit subaddress updating.
  Wire.write(L3G_OUT_X_L | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(L3G4200D_ADDRESS, (byte)6);

  while (Wire.available() < 6);

  uint8_t xlg = Wire.read();
  uint8_t xhg = Wire.read();
  uint8_t ylg = Wire.read();
  uint8_t yhg = Wire.read();
  uint8_t zlg = Wire.read();
  uint8_t zhg = Wire.read();

  // combine high and low bytes
  g.x = (int16_t)(xhg << 8 | xlg);
  g.y = (int16_t)(yhg << 8 | ylg);
  g.z = (int16_t)(zhg << 8 | zlg);

  // Temperature
  Wire.beginTransmission(107);
  Wire.write(L3G_OUT_TEMP | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(107, (byte)1);
  while (Wire.available() < 1);
  temp = Wire.read();

}
