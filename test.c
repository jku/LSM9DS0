/* Lot of the ideas and algos are originally from
 * https://github.com/sparkfun/LSM9DS0_Breakout/  */

#include <getopt.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

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
uint16_t GyroScaleValue[] = { 245, 500, 2000 };

typedef enum {
    ACCEL_SCALE_2G = 0,
    ACCEL_SCALE_4G,
    ACCEL_SCALE_6G,
    ACCEL_SCALE_8G,
    ACCEL_SCALE_16G
} AccelScale;
uint16_t AccelScaleValue[] = { 2, 4, 6, 8, 16 };

typedef enum {
    MAG_SCALE_2GS,
    MAG_SCALE_4GS,
    MAG_SCALE_8GS,
    MAG_SCALE_12GS,
} MagScale;
uint16_t MagScaleValue[] = { 2, 4, 6, 8, 12 };

uint8_t accel_scale;
uint8_t gyro_scale;
uint8_t mag_scale;

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

int read_bytes (int file, uint8_t address, uint8_t reg, uint8_t *dest, uint8_t count)
{
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[2];

  /* secret handshake for multibyte read */
  reg = reg | 0x80;

  /* write the register we want to read from */
  messages[0].addr  = address;
  messages[0].flags = 0;
  messages[0].len   = 1;
  messages[0].buf   = &reg;

  /* read */
  messages[1].addr  = address;
  messages[1].flags = I2C_M_RD;
  messages[1].len   = count;
  messages[1].buf   = dest;

  packets.msgs      = messages;
  packets.nmsgs     = 2;

  return ioctl(file, I2C_RDWR, &packets);
}

int write_bytes (int file, uint8_t address, uint8_t *data, uint8_t count)
{
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[1];

  messages[0].addr  = address;
  messages[0].flags = 0;
  messages[0].len   = count;
  messages[0].buf   = data;

  packets.msgs      = messages;
  packets.nmsgs     = 1;

  return ioctl(file, I2C_RDWR, &packets);
}


uint8_t read_byte (int file, uint8_t address, uint8_t reg)
{
  uint8_t buf[1] = {0};
  read_bytes (file, address, reg, buf, 1);
  return buf[0];
}

int write_byte (int file, uint8_t address, uint8_t reg, uint8_t data)
{
  uint8_t buf[2];
  buf[0] = reg;
  buf[1] = data;
  return write_bytes (file, address, buf, 2);
}

int read_triplet (int file, uint8_t address, uint8_t reg, Triplet *coords)
{
  uint8_t data[6] = {0};
  int retval;

  retval = read_bytes (file, address, reg, &data[0], 6);
  coords->x = ((data[1] << 8) | data[0]);
  coords->y = ((data[3] << 8) | data[2]);
  coords->z = ((data[5] << 8) | data[4]);
  return retval;
}

void read_bias(int file, uint8_t address, uint8_t reg, uint8_t count, Triplet *bias)
{
  int i;
  int32_t x, y, z;

  if (count == 0) {
    printf ("No data for calibration\n");
    return;
  }

  x = y = z = 0;
  for (i = 0; i < count; ++i) {
    Triplet data;
    read_triplet (file, address, reg, &data);
    x += data.x;
    y += data.y;
    z += data.z;
  }

  bias->x = x / count;
  bias->y = y / count;
  bias->z = z / count;
}

void read_gyro (int file, Triplet g_bias, FTriplet *dps)
{
  Triplet data = {0};

  read_triplet (file, G_ADDRESS, OUT_X_L_G, &data);
  dps->x = (data.x - g_bias.x) * gyro_scale / 32768.0;
  dps->y = (data.y - g_bias.y) * gyro_scale / 32768.0;
  dps->z = (data.z - g_bias.z) * gyro_scale / 32768.0;
}

void read_mag (int file, FTriplet *gauss)
{
  Triplet data = {0};

  read_triplet (file, XM_ADDRESS, OUT_X_L_M, &data);
  gauss->x = data.x * accel_scale / 32768.0;
  gauss->y = data.y * accel_scale / 32768.0;
  gauss->z = data.z * accel_scale / 32768.0;
}

void read_acc (int file, Triplet a_bias, FTriplet *grav)
{
  Triplet data = {0};

  read_triplet (file, XM_ADDRESS, OUT_X_L_A, &data);
  grav->x = (data.x - a_bias.x) * accel_scale / 32768.0;
  grav->y = (data.y - a_bias.y) * accel_scale / 32768.0;
  grav->z = (data.z - a_bias.z) * accel_scale / 32768.0;
}

/* Axes for rotations:
 *
 *                      ^ pitch (y)
 *                      |
 *           +----------------------+
 *           |                      |
 *  roll (z) |   Intel    Edison    |
 *   <---    |                      |
 *           |                      |
 *           | What  will you make? |
 *           +----------------------+
 *
 * Yaw (x) downwards.
 *
 * The direction of rotation is clockwise when looking along the axis
 * from the origin (aka right hand rule).
 */
void calculate_simple_angles (FTriplet mag, FTriplet acc, float declination, FTriplet *angles)
{
  float zz = acc.z * acc.z;

  if (mag.y > 0)
    angles->x = 90 + (atan(mag.x / mag.y) * (180.0 / M_PI));
  else if (mag.y < 0)
    angles->x = -90 + (atan(mag.x / mag.y) * (180.0 / M_PI));
  else if (mag.x < 0)
    angles->x = 0;
  else
    angles->x = 180;

  angles->x -= declination;
  angles->y = -atan2(acc.x, sqrt(acc.y * acc.y) + zz) * (180.0 / M_PI);
  angles->z = atan2(acc.y, sqrt(acc.x * acc.x) + zz) * (180.0 / M_PI);
}

void calibrate(int file, Triplet *g_bias, Triplet *a_bias)
{
  uint8_t reg5_g, reg0_xm, count;

  // enable FIFOs
  reg5_g = read_byte (file, G_ADDRESS, CTRL_REG5_G);
  write_byte (file, G_ADDRESS, CTRL_REG5_G, reg5_g | 0x40);
  reg0_xm = read_byte (file, XM_ADDRESS, CTRL_REG0_XM);
  write_byte (file, XM_ADDRESS, CTRL_REG0_XM, reg0_xm | 0x40);
  usleep (20000);

  // set to stream mode with 32 samples
  write_byte (file, G_ADDRESS, FIFO_CTRL_REG_G, 0x3F);
  write_byte (file, XM_ADDRESS, FIFO_CTRL_REG, 0x3F);

  // wait for samples
  sleep (1);

  count = read_byte (file, G_ADDRESS, FIFO_SRC_REG_G) & 0x1F;
  read_bias (file, G_ADDRESS, OUT_X_L_G, count, g_bias);

  count = read_byte (file, XM_ADDRESS, FIFO_SRC_REG) & 0x1F;
  read_bias (file, XM_ADDRESS, OUT_X_L_A, count, a_bias);
  // assume device is sensor board down: remove -1g from bias.z value
  a_bias->z += (int)32768.0/accel_scale;

  // disable FIFO
  write_byte (file, G_ADDRESS, CTRL_REG5_G, reg5_g);
  write_byte (file, XM_ADDRESS, CTRL_REG0_XM, reg0_xm);
  usleep (20000);
  // set to bypass mode
  write_byte (file, G_ADDRESS, FIFO_CTRL_REG_G, 0x00);
  write_byte (file, XM_ADDRESS, FIFO_CTRL_REG, 0x00);
}

void init_mag(int file, MagScale scale)
{
  // enable temp sensor
  if (write_byte (file, XM_ADDRESS, CTRL_REG5_XM, 0x98) < 0)
    printf ("Failed to set CTRL_REG5_XM\n");

  // all other bits 0
  write_byte (file, XM_ADDRESS, CTRL_REG6_XM, scale << 5);

  // continuous conversion mode
  if (write_byte (file, XM_ADDRESS, CTRL_REG7_XM, 0x00) < 0)
    printf ("Failed to set CTRL_REG7_XM\n");

  mag_scale = MagScaleValue[scale];
}

void init_gyro(int file, GyroScale scale)
{
  uint8_t reg4_g;

  // normal mode, all axes
  if (write_byte (file, G_ADDRESS, CTRL_REG1_G, 0x0f) < 0)
    printf ("Failed to set CTRL_REG1_G\n");

  // zero the scale bits, then set them
  reg4_g = read_byte (file, G_ADDRESS,  CTRL_REG4_G) & 0xCF;
  write_byte (file, G_ADDRESS, CTRL_REG4_G, reg4_g | scale << 4);

  gyro_scale = GyroScaleValue[scale];
}

void init_accel(int file, AccelScale scale)
{
  uint8_t reg2_xm;

  // 100hz, all axes
  if (write_byte (file, XM_ADDRESS, CTRL_REG1_XM, 0x57) < 0)
    printf ("Failed to set CTRL_REG1_XM\n");

  // zero the scale bits, then set them
  reg2_xm = read_byte (file, XM_ADDRESS, CTRL_REG2_XM) & 0xC7;
  write_byte (file, XM_ADDRESS, CTRL_REG2_XM, reg2_xm | scale << 3);

  accel_scale = AccelScaleValue[scale];
}

int main (int argc, char **argv)
{
  int file;
  int16_t temp;
  uint8_t data[2] = {0};
  Triplet a_bias, g_bias;
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

  if ((file = open(I2C_DEV_NAME, O_RDWR)) < 0) {
    printf("Failed to open the i2c bus\n");
    return 1;
  }

  data[0] = read_byte (file, G_ADDRESS, WHO_AM_I_G);
  data[1] = read_byte (file, XM_ADDRESS, WHO_AM_I_XM);
  if (data[0] != 0xD4 || data[1] != 0x49) {
    printf("Device id mismatch: Got %02x/%02x, expected %02x/%02x\n",
           data[0], data[1], 0xD4, 0x49);
    return 1;
  }

  if (option_dump) {
    dump_config_registers(file);
    printf ("\n");
  }

  init_mag(file, MAG_SCALE_2GS);
  init_gyro(file, GYRO_SCALE_245DPS);
  init_accel(file, ACCEL_SCALE_2G);

  // temperature is a 12-bit value: cut out 4 highest bits
  read_bytes (file, XM_ADDRESS, OUT_TEMP_L_XM, &data[0], 2);
  temp = (((data[1] & 0x0f) << 8) | data[0]);
  printf ("Temperature: %d\n", temp);

  printf ("Scaling: gyro %d | mag %d | acc %d\n", gyro_scale, mag_scale, accel_scale);

  printf ("Calibrating (Edison should be motionless, with logo upwards)...\n");
  calibrate(file, &g_bias, &a_bias);
  printf ("G bias: %d %d %d\n", g_bias.x, g_bias.y, g_bias.z);
  printf ("A bias: %d %d %d\n\n", a_bias.x, a_bias.y, a_bias.z);

  if (option_mode == OPTION_MODE_SENSOR)
    printf ("  Gyroscope (deg/s)  | Magnetometer (mGs)  |   Accelerometer (mG)\n");
  else
    printf ("      Rotations (mag + acc):\n");

  while (1) {
    FTriplet gyro, mag, acc, angles1;

    usleep (500000);

    read_gyro (file, g_bias, &gyro);
    read_mag (file, &mag);
    read_acc (file, a_bias, &acc);

    if (option_mode == OPTION_MODE_SENSOR) {
      printf ("gyro: %4.0f %4.0f %4.0f | ", gyro.x, gyro.y, gyro.z);
      printf ("mag: %4.0f %4.0f %4.0f | ", mag.x*1000, mag.y*1000, mag.z*1000);
      printf ("acc: %4.0f %4.0f %5.0f\n", acc.x*1000, acc.y*1000, acc.z*1000);
    } else {
      calculate_simple_angles (mag, acc, 0.0, &angles1);
      printf ("yaw: %4.0f, pitch: %4.0f, roll: %4.0f\n",
              angles1.x, angles1.y, angles1.z);
    }
  }
  return 0;
}
