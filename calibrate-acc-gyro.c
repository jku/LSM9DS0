#include <unistd.h>
#include <stdio.h>

#include "edison-9dof-i2c.h"

int read_bias(int file, uint8_t address, uint8_t reg, uint8_t count, Triplet *bias)
{
  int i;
  int32_t x, y, z;

  if (count == 0)
    return 0;

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

  /* TODO fail if any data is too far from bias values
   * (i.e. device was moved) */

  return 1;
}

int calibrate(int file, AccelScale scale, Triplet *g_bias, Triplet *a_bias)
{
  uint8_t reg5_g, reg0_xm, count;
  int a_success, g_success;

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
  g_success = read_bias (file, G_ADDRESS, OUT_X_L_G, count, g_bias);

  count = read_byte (file, XM_ADDRESS, FIFO_SRC_REG) & 0x1F;
  a_success = read_bias (file, XM_ADDRESS, OUT_X_L_A, count, a_bias);
  // assume edison is face up: remove -1g from bias.z value
  a_bias->z += (int)(1.0/AccelScaleValue[scale]);

  // disable FIFO
  write_byte (file, G_ADDRESS, CTRL_REG5_G, reg5_g);
  write_byte (file, XM_ADDRESS, CTRL_REG0_XM, reg0_xm);
  usleep (20000);
  // set to bypass mode
  write_byte (file, G_ADDRESS, FIFO_CTRL_REG_G, 0x00);
  write_byte (file, XM_ADDRESS, FIFO_CTRL_REG, 0x00);

  return (count >= 30 && a_success && g_success);
}


int main (int argc, char **argv)
{
  int file;
  Triplet a_bias, g_bias;

  file = init_device (I2C_DEV_NAME);
  if (file == 0)
    return 1;

  init_gyro (file, GYRO_SCALE_245DPS);
  init_mag (file, MAG_SCALE_2GS);
  init_acc (file, ACCEL_SCALE_2G);
  
  printf ("Calibrating (Edison should be motionless, with logo upwards)...\n");
  while (!calibrate (file, ACCEL_SCALE_2G, &g_bias, &a_bias))
    printf ("Calibration failed, trying again...\n");

  printf ("G bias: %d %d %d\n", g_bias.x, g_bias.y, g_bias.z);
  printf ("A bias: %d %d %d\n\n", a_bias.x, a_bias.y, a_bias.z);

  return 0;
}
