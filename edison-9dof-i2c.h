#ifndef EDISON_9DOF_I2C_H
#define EDISON_9DOF_I2C_H

#include <stdint.h>


/* I2C device */
#define I2C_DEV_NAME       "/dev/i2c-1"

/* I2C device adresses */
#define XM_ADDRESS         0x1d
#define G_ADDRESS          0x6b

/* G_ADDRESS registers */
#define WHO_AM_I_G         0x0F // r

#define CTRL_REG1_G        0x20 // rw
#define CTRL_REG2_G        0x21 // rw
#define CTRL_REG3_G        0x22 // rw
#define CTRL_REG4_G        0x23 // rw
#define CTRL_REG5_G        0x24 // rw
#define REFERENCE_G        0x25 // rw

#define STATUS_REG_G       0x27 // r
#define OUT_X_L_G          0x28 // r
#define OUT_X_H_G          0x29 // r
#define OUT_Y_L_G          0x2A // r
#define OUT_Y_H_G          0x2B // r
#define OUT_Z_L_G          0x2C // r
#define OUT_Z_H_G          0x2D // r
#define FIFO_CTRL_REG_G    0x2E // rw
#define FIFO_SRC_REG_G     0x2F // r
#define INT1_CFG_G         0x30 // rw
#define INT1_SRC_G         0x31 // r
#define INT1_TSH_XH_G      0x32 // rw
#define INT1_TSH_XL_G      0x33 // rw
#define INT1_TSH_YH_G      0x34 // rw
#define INT1_TSH_YL_G      0x35 // rw
#define INT1_TSH_ZH_G      0x36 // rw
#define INT1_TSH_ZL_G      0x37 // rw
#define INT1_DURATION_G    0x38 // rw

/* XM_ADDRESS registers */
#define OUT_TEMP_L_XM      0x05 // r
#define OUT_TEMP_H_XM      0x06 // r
#define STATUS_REG_M       0x07 // r
#define OUT_X_L_M          0x08 // r
#define OUT_X_H_M          0x09 // r
#define OUT_Y_L_M          0x0A // r
#define OUT_Y_H_M          0x0B // r
#define OUT_Z_L_M          0x0C // r
#define OUT_Z_H_M          0x0D // r

#define WHO_AM_I_XM        0x0F // r

#define INT_CTRL_REG_M     0x12 // rw
#define INT_SRC_REG_M      0x13 // r
#define INT_THS_L_M        0x14 // rw
#define INT_THS_H_M        0x15 // rw
#define OFFSET_X_L_M       0x16 // rw
#define OFFSET_X_H_M       0x17 // rw
#define OFFSET_Y_L_M       0x18 // rw
#define OFFSET_Y_H_M       0x19 // rw
#define OFFSET_Z_L_M       0x1A // rw
#define OFFSET_Z_H_M       0x1B // rw
#define REFERENCE_X        0x1C // rw
#define REFERENCE_Y        0x1D // rw
#define REFERENCE_Z        0x1E // rw
#define CTRL_REG0_XM       0x1F // rw
#define CTRL_REG1_XM       0x20 // rw
#define CTRL_REG2_XM       0x21 // rw
#define CTRL_REG3_XM       0x22 // rw
#define CTRL_REG4_XM       0x23 // rw
#define CTRL_REG5_XM       0x24 // rw
#define CTRL_REG6_XM       0x25 // rw
#define CTRL_REG7_XM       0x26 // rw
#define STATUS_REG_A       0x27 // r
#define OUT_X_L_A          0x28 // r
#define OUT_X_H_A          0x29 // r
#define OUT_Y_L_A          0x2A // r
#define OUT_Y_H_A          0x2B // r
#define OUT_Z_L_A          0x2C // r
#define OUT_Z_H_A          0x2D // r
#define FIFO_CTRL_REG      0x2E // rw
#define FIFO_SRC_REG       0x2F // r
#define INT_GEN_1_REG      0x30 // rw
#define INT_GEN_1_SRC      0x31 // r
#define INT_GEN_1_THS      0x32 // rw
#define INT_GEN_1_DURATION 0x33 // rw
#define INT_GEN_2_REG      0x34 // rw
#define INT_GEN_2_SRC      0x35 // r
#define INT_GEN_2_THS      0x36 // rw
#define INT_GEN_2_DURATION 0x37 // rw
#define CLICK_CFG          0x38 // rw
#define CLICK_SRC          0x39 // r
#define CLICK_THS          0x3A // rw
#define TIME_LIMIT         0x3B // rw
#define TIME_LATENCY       0x3C // rw
#define TIME_WINDOW        0x3D // rw
#define ACT_THS            0x3E // rw
#define ACT_DUR            0x3F // rw

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} Triplet;

typedef struct {
    float x;
    float y;
    float z;
} FTriplet;

typedef enum { // degrees per second
    GYRO_SCALE_245DPS,
    GYRO_SCALE_500DPS,
    GYRO_SCALE_2000DPS,
} GyroScale;
extern const float GyroScaleValue[];

typedef enum {
    ACCEL_SCALE_2G,
    ACCEL_SCALE_4G,
    ACCEL_SCALE_6G,
    ACCEL_SCALE_8G,
    ACCEL_SCALE_16G
} AccelScale;
extern const float AccelScaleValue[];

typedef enum {
    MAG_SCALE_2GS,
    MAG_SCALE_4GS,
    MAG_SCALE_8GS,
    MAG_SCALE_12GS,
} MagScale;
extern const float MagScaleValue[];

int init_device   (const char* device_name);

int write_bytes   (int file, uint8_t address, uint8_t *data, uint8_t count);
int write_byte    (int file, uint8_t address, uint8_t reg, uint8_t data);

int read_bytes    (int file, uint8_t address, uint8_t reg, uint8_t *dest, uint8_t count);
int read_byte (int file, uint8_t address, uint8_t reg, uint8_t *dest);
int read_triplet  (int file, uint8_t address, uint8_t reg, Triplet *coords);

void init_gyro    (int file, GyroScale scale);
void init_mag     (int file, MagScale scale);
void init_acc     (int file, AccelScale scale);

int read_gyro    (int file, Triplet g_bias, GyroScale scale, FTriplet *dps);
int read_mag     (int file, Triplet m_bias, FTriplet m_scale, MagScale scale, FTriplet *gauss);
int read_acc     (int file, Triplet a_bias, AccelScale scale, FTriplet *grav);

#endif // EDISON_9DOF_I2C_H
