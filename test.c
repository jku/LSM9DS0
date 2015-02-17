/* Lot of the ideas and algos are originally from
 * https://github.com/sparkfun/LSM9DS0_Breakout/
 */

#include <getopt.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <inttypes.h>

#include "edison-9dof-i2c.h"

uint8_t read_byte (int file, uint8_t address, uint8_t reg);

#define BYTE2BIN(byte) \
    (byte & 0x80 ? 1 : 0), \
    (byte & 0x40 ? 1 : 0), \
    (byte & 0x20 ? 1 : 0), \
    (byte & 0x10 ? 1 : 0), \
    (byte & 0x08 ? 1 : 0), \
    (byte & 0x04 ? 1 : 0), \
    (byte & 0x02 ? 1 : 0), \
    (byte & 0x01 ? 1 : 0)

uint8_t print_byte;
#define PRINT_REGISTER(file, address, reg) \
  print_byte = read_byte(file, address, reg); \
  printf ("%-18s\t%02x / %d%d%d%d%d%d%d%d\n", \
          #reg":", print_byte, BYTE2BIN(print_byte))

#define ACC_GYRO_BIAS_FILENAME "acc-gyro.bias"


#define GYRO_ERROR M_PI * (40.0f / 180.0f) //rads/s
#define GYRO_DRIFT M_PI * (0.0f / 180.0f)  // rad/s/s
#define MADGWICK_BETA sqrt(3.0f / 4.0f) * GYRO_ERROR


typedef struct {
    float x;
    float y;
    float z;
    float w;
} Quaternion;

static struct option long_options[] = {
  {"dump",  no_argument,       0,  'd' },
  {"help",  no_argument,       0,  'h' },
  {"mode",  required_argument, 0,  'm' },
};

typedef enum {
  OPTION_MODE_SENSOR,
  OPTION_MODE_ANGLES,
} OptionMode;

void dump_config_registers (int file)
{
  printf (" * Non-output registers for %02x:\n", G_ADDRESS);
  PRINT_REGISTER (file, G_ADDRESS, WHO_AM_I_XM);
  PRINT_REGISTER (file, G_ADDRESS, CTRL_REG1_XM);
  PRINT_REGISTER (file, G_ADDRESS, CTRL_REG2_XM);
  PRINT_REGISTER (file, G_ADDRESS, CTRL_REG3_XM);
  PRINT_REGISTER (file, G_ADDRESS, CTRL_REG4_XM);
  PRINT_REGISTER (file, G_ADDRESS, CTRL_REG5_XM);
  PRINT_REGISTER (file, G_ADDRESS, REFERENCE_G);
  PRINT_REGISTER (file, G_ADDRESS, FIFO_CTRL_REG_G);
  PRINT_REGISTER (file, G_ADDRESS, INT1_CFG_G);
  PRINT_REGISTER (file, G_ADDRESS, INT1_TSH_XH_G);
  PRINT_REGISTER (file, G_ADDRESS, INT1_TSH_XL_G);
  PRINT_REGISTER (file, G_ADDRESS, INT1_TSH_YH_G);
  PRINT_REGISTER (file, G_ADDRESS, INT1_TSH_YL_G);
  PRINT_REGISTER (file, G_ADDRESS, INT1_TSH_ZH_G);
  PRINT_REGISTER (file, G_ADDRESS, INT1_TSH_ZL_G);
  PRINT_REGISTER (file, G_ADDRESS, INT1_DURATION_G);

  printf (" * Non-output registers for %02x:\n", XM_ADDRESS);
  PRINT_REGISTER (file, XM_ADDRESS, WHO_AM_I_XM);
  PRINT_REGISTER (file, XM_ADDRESS, INT_CTRL_REG_M);
  PRINT_REGISTER (file, XM_ADDRESS, INT_THS_L_M);
  PRINT_REGISTER (file, XM_ADDRESS, INT_THS_H_M);
  PRINT_REGISTER (file, XM_ADDRESS, OFFSET_X_L_M);
  PRINT_REGISTER (file, XM_ADDRESS, OFFSET_X_H_M);
  PRINT_REGISTER (file, XM_ADDRESS, OFFSET_Y_L_M);
  PRINT_REGISTER (file, XM_ADDRESS, OFFSET_Y_H_M);
  PRINT_REGISTER (file, XM_ADDRESS, OFFSET_Z_L_M);
  PRINT_REGISTER (file, XM_ADDRESS, OFFSET_Z_H_M);
  PRINT_REGISTER (file, XM_ADDRESS, REFERENCE_X);
  PRINT_REGISTER (file, XM_ADDRESS, REFERENCE_Y);
  PRINT_REGISTER (file, XM_ADDRESS, REFERENCE_Z);
  PRINT_REGISTER (file, XM_ADDRESS, CTRL_REG0_XM);
  PRINT_REGISTER (file, XM_ADDRESS, CTRL_REG1_XM);
  PRINT_REGISTER (file, XM_ADDRESS, CTRL_REG2_XM);
  PRINT_REGISTER (file, XM_ADDRESS, CTRL_REG3_XM);
  PRINT_REGISTER (file, XM_ADDRESS, CTRL_REG4_XM);
  PRINT_REGISTER (file, XM_ADDRESS, CTRL_REG5_XM);
  PRINT_REGISTER (file, XM_ADDRESS, CTRL_REG6_XM);
  PRINT_REGISTER (file, XM_ADDRESS, CTRL_REG7_XM);
  PRINT_REGISTER (file, XM_ADDRESS, FIFO_CTRL_REG);
  PRINT_REGISTER (file, XM_ADDRESS, INT_GEN_1_REG);
  PRINT_REGISTER (file, XM_ADDRESS, INT_GEN_1_THS);
  PRINT_REGISTER (file, XM_ADDRESS, INT_GEN_1_DURATION);
  PRINT_REGISTER (file, XM_ADDRESS, INT_GEN_2_REG);
  PRINT_REGISTER (file, XM_ADDRESS, INT_GEN_2_THS);
  PRINT_REGISTER (file, XM_ADDRESS, INT_GEN_2_DURATION);
  PRINT_REGISTER (file, XM_ADDRESS, CLICK_CFG);
  PRINT_REGISTER (file, XM_ADDRESS, CLICK_THS);
  PRINT_REGISTER (file, XM_ADDRESS, TIME_LIMIT);
  PRINT_REGISTER (file, XM_ADDRESS, TIME_LATENCY);
  PRINT_REGISTER (file, XM_ADDRESS, TIME_WINDOW);
  PRINT_REGISTER (file, XM_ADDRESS, ACT_THS);
  PRINT_REGISTER (file, XM_ADDRESS, ACT_DUR);
}

/* Axes for rotations (same as accelerometer and gyro measurements):
 *
 *  +----------------------+
 *  |                      |
 *  |   Intel    Edison    |  X-axis (pitch)
 *  |                      | --->
 *  |                      |
 *  | What  will you make? |
 *  +----------------------+
 *             |
 *             | Y-axis (roll)
 *             v
 *
 * Z-axis (yaw) downwards.
 *
 * The direction of rotation is clockwise when looking along the axis
 * from the origin (aka right hand rule). Roll and pitch are zero when
 * Edison is level, yaw is zero when y-axis points to north.
 * 
 */
void calculate_simple_angles (FTriplet mag, FTriplet acc, float declination, FTriplet *angles)
{
  float zz = acc.z * acc.z;

  angles->x = -atan2(acc.y, sqrt(acc.x * acc.x) + zz) * (180.0 / M_PI);
  angles->y = atan2(acc.x, sqrt(acc.y * acc.y) + zz) * (180.0 / M_PI);

  if (mag.y > 0)
    angles->z = 90 + (atan(mag.x / mag.y) * (180.0 / M_PI));
  else if (mag.y < 0)
    angles->z = -90 + (atan(mag.x / mag.y) * (180.0 / M_PI));
  else if (mag.x < 0)
    angles->z = 0;
  else
    angles->z = 180;
  angles->z -= declination;
}

/* This function originally from 
 * https://github.com/sparkfun/LSM9DS0_Breakout/blob/master/Libraries/Arduino/SFE_LSM9DS0/examples/LSM9DS0_AHRS/LSM9DS0_AHRS.ino 
 * which in turn is an implementation of http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/ */
void madgwick_quaternion(FTriplet acc, FTriplet mag, FTriplet gyro, float deltat, Quaternion *quat)
{
    // short name local variable for readability
    float q1 = quat->x, q2 = quat->y, q3 = quat->z, q4 = quat->w;
    float ax = acc.x, ay = acc.y, az = acc.z;
    float mx = mag.x, my = mag.y, mz = mag.z;
    float gx = gyro.x * M_PI / 180.0,
          gy = gyro.y * M_PI / 180.0,
          gz = gyro.z * M_PI / 180.0;

    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f)
      return; // handle NaN

    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f)
      return; // handle NaN

    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - MADGWICK_BETA * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - MADGWICK_BETA * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - MADGWICK_BETA * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - MADGWICK_BETA * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    quat->x = q1 * norm;
    quat->y = q2 * norm;
    quat->z = q3 * norm;
    quat->w = q4 * norm;
}

void calculate_tait_bryan_angles (Quaternion quat, float declination, FTriplet *angles)
{
    float yaw, pitch, roll;
    float q1 = quat.x, q2 = quat.y, q3 = quat.z, q4 = quat.w;

    yaw   = atan2(2.0f * (q2 * q3 + q1 * q4), q1 * q1 + q2 * q2 - q3 * q3 - q4 * q4);
    pitch = -asin(2.0f * (q2 * q4 - q1 * q3));
    roll  = atan2(2.0f * (q1 * q2 + q3 * q4), -(q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4));

    angles->x = yaw * 180.0f / M_PI - declination; 
    angles->y = pitch * 180.0f / M_PI;
    angles->z = roll * 180.0f / M_PI;
}

int main (int argc, char **argv)
{
  int file;
  FILE *input;
  int16_t temp;
  char g_str[7], a_str[7];
  uint8_t data[2] = {0};
  Triplet a_bias, g_bias;
  MagDistribution m_distribution = {{0},{0},{0}};
  int opt, option_index, help = 0, option_dump = 0;
  OptionMode option_mode = OPTION_MODE_ANGLES;

  while ((opt = getopt_long(argc, argv, "dhm:",
                            long_options, &option_index )) != -1) {
    switch (opt) {
      case 'd' : 
        option_dump = 1;
        break;
      case 'm' :
        if (strcmp (optarg, "sensor") == 0)
          option_mode = OPTION_MODE_SENSOR;
        else if (strcmp (optarg, "angles") == 0)
          option_mode = OPTION_MODE_ANGLES;
        else
          help = 1;
        break;
      default:
        help = 1;
        break;
    }
  }

  if (help || argv[optind] != NULL) {
      printf ("%s [--mode <sensor|angles>] [--dump]\n", argv[0]);
      return 0;
  }

  input = fopen(ACC_GYRO_BIAS_FILENAME, "r");
  if (input) {
    if (fscanf(input, "%s %hd %hd %hd",
               g_str, &g_bias.x, &g_bias.y, &g_bias.z) != 4 ||
        fscanf(input, "%s %hd %hd %hd",
               a_str, &a_bias.x, &a_bias.y, &a_bias.z) != 4 ||
        strcmp (g_str, "g_bias") != 0 ||
        strcmp (a_str, "a_bias") != 0) {
      printf ("Bias file "ACC_GYRO_BIAS_FILENAME" is malformed\n");
      return 1;
    } else {
      printf ("Loaded bias file G: %d %d %d, A: %d %d %d\n",
              g_bias.x, g_bias.y, g_bias.z, a_bias.x, a_bias.y, a_bias.z);
    }
  }

  file = init_device (I2C_DEV_NAME);
  if (file == 0)
    return 1;

  if (option_dump) {
    dump_config_registers(file);
    printf ("\n");
  }

  init_gyro(file, GYRO_SCALE_245DPS);
  init_mag(file, MAG_SCALE_2GS);
  init_acc(file, ACCEL_SCALE_2G);

  // temperature is a 12-bit value: cut out 4 highest bits
  read_bytes (file, XM_ADDRESS, OUT_TEMP_L_XM, &data[0], 2);
  temp = (((data[1] & 0x0f) << 8) | data[0]);
  printf ("Temperature: %d\n", temp);


  if (option_mode == OPTION_MODE_SENSOR)
    printf ("  Gyroscope (deg/s)  | Magnetometer (mGs)  |   Accelerometer (mG)\n");
  else
    printf ("      Rotations (mag + acc):\n");

  while (1) {
    FTriplet gyro, mag, acc, angles1;

    usleep (500000);

    read_gyro (file, g_bias, GYRO_SCALE_245DPS, &gyro);
    read_mag (file, &m_distribution, MAG_SCALE_2GS, &mag);
    read_acc (file, a_bias, ACCEL_SCALE_2G, &acc);

    if (option_mode == OPTION_MODE_SENSOR) {
      printf ("gyro: %4.0f %4.0f %4.0f | ", gyro.x, gyro.y, gyro.z);
      printf ("mag: %4.0f %4.0f %4.0f | ", mag.x*1000, mag.y*1000, mag.z*1000);
      printf ("acc: %4.0f %4.0f %5.0f\n", acc.x*1000, acc.y*1000, acc.z*1000);
    } else {
      calculate_simple_angles (mag, acc, 0.0, &angles1);
      printf ("pitch: %4.0f, roll: %4.0f, yaw: %4.0f\n",
              angles1.x, angles1.y, angles1.z);
    }
  }
  return 0;
}
