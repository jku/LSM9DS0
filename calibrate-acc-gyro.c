#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

#include "edison-9dof-i2c.h"

#define MAX_GYRO_ERROR 100
#define MAX_ACC_ERROR 400
#define BIAS_FILENAME "acc-gyro.bias"


int read_bias(int file, uint8_t count, Triplet *g_bias, Triplet *a_bias)
{
  int i, div, success = 1;
  int32_t g_x, g_y, g_z, a_x, a_y, a_z;
  Triplet *a_data, *g_data;

  if (count == 0)
    return 0;

  div = count / 33 + 1;

  a_data = malloc (count * sizeof (Triplet));
  g_data = malloc (count * sizeof (Triplet));

  g_x = g_y = g_z = a_x = a_y = a_z = 0;
  for (i = 0; i < count; ++i) {
    read_triplet (file, XM_ADDRESS, OUT_X_L_A, &(a_data[i]));
    a_x += a_data[i].x;
    a_y += a_data[i].y;
    a_z += a_data[i].z;

    read_triplet (file, G_ADDRESS, OUT_X_L_G, &(g_data[i]));
    g_x += g_data[i].x;
    g_y += g_data[i].y;
    g_z += g_data[i].z;

    if (i % div == 0) {
      printf ("*");
      fflush(stdout);
    }
    usleep (30000);
  }

  g_bias->x = g_x / count;
  g_bias->y = g_y / count;
  g_bias->z = g_z / count;

  a_bias->x = a_x / count;
  a_bias->y = a_y / count;
  a_bias->z = a_z / count;

  printf ("                            \r");

  /* fail if it looks like the device moved while calibrating */
  for (i = 0; i < count; ++i) {
    if (abs(g_bias->x - g_data[i].x) > MAX_GYRO_ERROR ||
        abs(g_bias->y - g_data[i].y) > MAX_GYRO_ERROR ||
        abs(g_bias->z - g_data[i].z) > MAX_GYRO_ERROR){
      printf ("Unexpected jitter in gyroscope\n");
      success = 0;
      break;
    }
    if (abs(a_bias->x - a_data[i].x) > MAX_ACC_ERROR ||
        abs(a_bias->y - a_data[i].y) > MAX_ACC_ERROR ||
        abs(a_bias->z - a_data[i].z) > MAX_ACC_ERROR) {
      printf ("Unexpected jitter in acceleration\n");
      success = 0;
      break;
    }
  }

  free (g_data);
  free (a_data);

  return success;
}

int calibrate(int file, AccelScale scale, Triplet *g_bias, Triplet *a_bias)
{
  int success;

  success = read_bias (file, 100, g_bias, a_bias);
  // assume edison is face up: remove -1g from bias.z value
  a_bias->z += (int)(1.0/AccelScaleValue[scale]);

  return (success);
}


int main (int argc, char **argv)
{
  FILE *output;
  int file;
  Triplet a_bias, g_bias;

  file = init_device (I2C_DEV_NAME);
  if (file == 0)
    return 1;

  init_gyro (file, GYRO_SCALE_245DPS);
  init_acc (file, ACCEL_SCALE_2G);

  printf ("Calibrating (Edison should be motionless, with logo upwards)...\n");
  while (!calibrate (file, ACCEL_SCALE_2G, &g_bias, &a_bias))
    printf ("Calibration failed, trying again...\n");

  output = fopen(BIAS_FILENAME, "w");
  if (!output){
    printf("Error opening '"BIAS_FILENAME"' for output.\n");
    return 1;
  }
  fprintf(output, "g_bias %d %d %d\n", g_bias.x, g_bias.y, g_bias.z);
  fprintf(output, "a_bias %d %d %d\n", a_bias.x, a_bias.y, a_bias.z);
  fclose (output);

  printf ("Calibration succesful, wrote '"BIAS_FILENAME"'.\n");  
  return 0;
}
