#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

#include "edison-9dof-i2c.h"

const float GyroScaleValue[] = {
  245 / 32768.0,
  500 / 32768.0,
  2000 / 32768.0
};

const float AccelScaleValue[] = {
  2 / 32768.0,
  4 / 32768.0,
  6 / 32768.0,
  8 / 32768.0,
  16 / 32768.0
};
const float MagScaleValue[] = {
  2 / 32768.0,
  4 / 32768.0,
  6 / 32768.0,
  8 / 32768.0,
  12 / 32768.0
};

int init_device (const char* device_name)
{
  int file;
  uint8_t g_id, xm_id;

  if ((file = open(device_name, O_RDWR)) < 0) {
    printf("Failed to open the i2c bus '%s'\n", device_name);
    return 0;
  }

  g_id = read_byte (file, G_ADDRESS, WHO_AM_I_G);
  xm_id = read_byte (file, XM_ADDRESS, WHO_AM_I_XM);
  if (g_id != 0xD4 || xm_id != 0x49) {
    printf("Device id mismatch: Got %02x/%02x, expected %02x/%02x\n",
           g_id, xm_id, 0xD4, 0x49);
    close (file);
    return 0;
  }

  return file;
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

int write_byte (int file, uint8_t address, uint8_t reg, uint8_t data)
{
  uint8_t buf[2];
  buf[0] = reg;
  buf[1] = data;
  return write_bytes (file, address, buf, 2);
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

uint8_t read_byte (int file, uint8_t address, uint8_t reg)
{
  uint8_t buf[1] = {0};
  read_bytes (file, address, reg, buf, 1);
  return buf[0];
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

void read_gyro (int file, Triplet g_bias, GyroScale scale, FTriplet *dps)
{
  Triplet data = {0};

  read_triplet (file, G_ADDRESS, OUT_X_L_G, &data);
  dps->x = (data.x - g_bias.x) * GyroScaleValue[scale];
  dps->y = (data.y - g_bias.y) * GyroScaleValue[scale];
  dps->z = (data.z - g_bias.z) * GyroScaleValue[scale];
}

/* todo separate calibration */
void read_mag (int file, Triplet m_bias,  MagScale scale, FTriplet *gauss)
{
  Triplet data = {0};

  read_triplet (file, XM_ADDRESS, OUT_X_L_M, &data);

  gauss->x = (data.x - m_bias.x) * MagScaleValue[scale];
  gauss->y = (data.y - m_bias.y) * MagScaleValue[scale];
  /* invert z axis so it's positive down like other sensors */
  gauss->z = -(data.z - m_bias.z) * MagScaleValue[scale];
}

void read_acc (int file, Triplet a_bias, AccelScale scale, FTriplet *grav)
{
  Triplet data = {0};

  read_triplet (file, XM_ADDRESS, OUT_X_L_A, &data);
  grav->x = (data.x - a_bias.x) * AccelScaleValue[scale];
  grav->y = (data.y - a_bias.y) * AccelScaleValue[scale];
  grav->z = (data.z - a_bias.z) * AccelScaleValue[scale];
}

void init_gyro (int file, GyroScale scale)
{
  uint8_t reg4_g;

  // normal mode, all axes
  if (write_byte (file, G_ADDRESS, CTRL_REG1_G, 0x0f) < 0)
    printf ("Failed to set CTRL_REG1_G\n");

  // zero the scale bits, then set them
  reg4_g = read_byte (file, G_ADDRESS,  CTRL_REG4_G) & 0xCF;
  write_byte (file, G_ADDRESS, CTRL_REG4_G, reg4_g | scale << 4);
}

void init_mag (int file, MagScale scale)
{
  // enable temp sensor
  if (write_byte (file, XM_ADDRESS, CTRL_REG5_XM, 0x98) < 0)
    printf ("Failed to set CTRL_REG5_XM\n");

  // all other bits 0
  write_byte (file, XM_ADDRESS, CTRL_REG6_XM, scale << 5);

  // continuous conversion mode
  if (write_byte (file, XM_ADDRESS, CTRL_REG7_XM, 0x00) < 0)
    printf ("Failed to set CTRL_REG7_XM\n");
}

void init_acc (int file, AccelScale scale)
{
  uint8_t reg2_xm;

  // 100hz, all axes
  if (write_byte (file, XM_ADDRESS, CTRL_REG1_XM, 0x57) < 0)
    printf ("Failed to set CTRL_REG1_XM\n");

  // zero the scale bits, then set them
  reg2_xm = read_byte (file, XM_ADDRESS, CTRL_REG2_XM) & 0xC7;
  write_byte (file, XM_ADDRESS, CTRL_REG2_XM, reg2_xm | scale << 3);
}
