#ifndef L3G_h
#define L3G_h

#include <Arduino.h>
#define MPU6050_ADDRESS         0x68
#define MPUREG_WHOAMI           0x75
#define MPUREG_SMPLRT_DIV       0x19
#define MPUREG_CONFIG           0x1A
#define MPUREG_GYRO_CONFIG      0x1B
#define MPUREG_ACCEL_CONFIG     0x1C
#define MPUREG_FIFO_EN          0x23
#define MPUREG_INT_PIN_CFG      0x37
#define MPUREG_INT_ENABLE       0x38
#define MPUREG_INT_STATUS       0x3A
#define MPUREG_ACCEL_XOUT_H     0x3B
#define MPUREG_ACCEL_XOUT_L     0x3C
#define MPUREG_ACCEL_YOUT_H     0x3D
#define MPUREG_ACCEL_YOUT_L     0x3E
#define MPUREG_ACCEL_ZOUT_H     0x3F
#define MPUREG_ACCEL_ZOUT_L     0x40
#define MPUREG_TEMP_OUT_H       0x41
#define MPUREG_TEMP_OUT_L       0x42
#define MPUREG_GYRO_XOUT_H      0x43
#define MPUREG_GYRO_XOUT_L      0x44
#define MPUREG_GYRO_YOUT_H      0x45
#define MPUREG_GYRO_YOUT_L      0x46
#define MPUREG_GYRO_ZOUT_H      0x47
#define MPUREG_GYRO_ZOUT_L      0x48
#define MPUREG_USER_CTRL        0x6A
#define MPUREG_PWR_MGMT_1       0x6B
#define MPUREG_PWR_MGMT_2       0x6C
#define MPUREG_FIFO_COUNTH      0x72
#define MPUREG_FIFO_COUNTL      0x73
#define MPUREG_FIFO_R_W         0x74
// Configuration bits
#define BIT_SLEEP               0x40
#define BIT_H_RESET             0x80
#define BITS_CLKSEL             0x07
#define MPU_CLK_SEL_PLLGYROX    0x01
#define MPU_CLK_SEL_PLLGYROZ    0x03
#define MPU_EXT_SYNC_GYROX      0x02
#define BITS_FS_250DPS          0x00
#define BITS_FS_500DPS          0x08
#define BITS_FS_1000DPS         0x10
#define BITS_FS_2000DPS         0x18
#define BITS_FS_MASK            0x18
#define BITS_DLPF_CFG_256HZ  0x00//
//Default settings LPF 256Hz/8000Hz sample
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR    0x10
#define BIT_RAW_RDY_EN          0x01
#define BIT_I2C_IF_DIS          0x10
#define BIT_INT_STATUS_DATA     0x01
//GY89
#define L3GD20_ADDRESS    (0xD6 >> 1)
#define L3G_WHO_AM_I      0x0F
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
#define L3G_FIFO_CTRL_REG 0x2E
#define L3G_FIFO_SRC_REG  0x2F
#define L3G_INT1_CFG      0x30
#define L3G_INT1_SRC      0x31
#define L3G_INT1_THS_XH   0x32
#define L3G_INT1_THS_XL   0x33
#define L3G_INT1_THS_YH   0x34
#define L3G_INT1_THS_YL   0x35
#define L3G_INT1_THS_ZH   0x36
#define L3G_INT1_THS_ZL   0x37
#define L3G_INT1_DURATION 0x38
#define LSM303D_ADDRESS   0b0011101
#define LSM303D_CTRL_REG0      0x1F
#define LSM303D_CTRL_REG1      0x20
#define LSM303D_CTRL_REG2      0x21
#define LSM303D_CTRL_REG3      0x22
#define LSM303D_CTRL_REG4      0x23
#define LSM303D_CTRL_REG5      0x24
#define LSM303D_CTRL_REG6      0x25
#define LSM303D_CTRL_REG7      0x26
#define LSM303D_OUT_X_L_A      0x28
#define L3GD20_DEFAULT_FILTER_FREQ    30
#define L3GD20_DEFAULT_RATE 400
#define LSM303D_ACCEL_DEFAULT_RANGE_G      16
#define LSM303D_ACCEL_DEFAULT_RATE      400
#define LSM303D_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ  50
#define LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ  30



class IMU
{
  public:
    typedef struct vector
    {
      short x, y, z;
    } vector;
    vector g; // gyro angular velocity readings
    vector a; // accelerometer readings
    int16_t temp;
    uint8_t data[30];
    void Init(void);
    void read(void);
    uint8_t I2CWrite(uint8_t address, uint8_t reg, uint8_t value, bool stop);
    uint8_t I2CRead(uint8_t address, uint8_t reg, uint8_t nbytes);
  private:
};

#endif
