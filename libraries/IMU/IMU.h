#ifndef L3G_h
#define L3G_h

#include <Arduino.h>

#define L3G4200D_ADDRESS  (0xD6 >> 1)
#define L3G_CTRL_REG1     0x20
#define L3G_CTRL_REG2     0x21
#define L3G_CTRL_REG3     0x22
#define L3G_CTRL_REG4     0x23
#define L3G_CTRL_REG5     0x24
#define L3G_REFERENCE     0x25
#define L3G_OUT_TEMP      0x26
#define L3G_STATUS_REG    0x27
#define L3G_OUT_X_L       0x28
#define L3G_OUT_X_H       0x29
#define L3G_OUT_Y_L       0x2A
#define L3G_OUT_Y_H       0x2B
#define L3G_OUT_Z_L       0x2C
#define L3G_OUT_Z_H       0x2D

#define LSM303D_ADDRESS   0b0011101
#define CTRL0             0x1F
#define CTRL1             0x20
#define CTRL2             0x21
#define CTRL3             0x22
#define CTRL4             0x23
#define CTRL5             0x24
#define CTRL6             0x25
#define OUT_X_L_A         0x28
#define OUT_X_H_A         0x29
#define OUT_Y_L_A         0x2A
#define OUT_Y_H_A         0x2B
#define OUT_Z_L_A         0x2C
#define OUT_Z_H_A         0x2D

class IMU
{
  public:
    typedef struct vector
    {
      float x, y, z;
    } vector;
    vector g; // gyro angular velocity readings
    vector a; // accelerometer readings
    float temp;
    void Init(void);
    void read(void);
  private:
};

#endif
