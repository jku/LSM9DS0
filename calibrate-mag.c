#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

#include "edison-9dof-i2c.h"

#define MAX_MAG_ERROR 100
#define BIAS_FILENAME "mag.bias"

typedef struct {
  Triplet min;
  Triplet max;
  Triplet bias;
} MagDistribution;

void update_mag_bias (Triplet data, MagDistribution *m_dist)
{
  if (data.x < m_dist->min.x) {
    m_dist->min.x = data.x;
    m_dist->bias.x = (m_dist->max.x + m_dist->min.x) / 2;
  } else if (data.x > m_dist->max.x) {
    m_dist->max.x = data.x;
    m_dist->bias.x = (m_dist->max.x + m_dist->min.x) / 2;
  }
  if (data.y < m_dist->min.y) {
    m_dist->min.y = data.y;
    m_dist->bias.y = (m_dist->max.y + m_dist->min.y) / 2;
  } else if (data.y > m_dist->max.y) {
    m_dist->max.y = data.y;
    m_dist->bias.y = (m_dist->max.y + m_dist->min.y) / 2;
  }
  if (data.z < m_dist->min.z) {
    m_dist->min.z = data.z;
    m_dist->bias.z = (m_dist->max.z + m_dist->min.z) / 2;
  } else if (data.z > m_dist->max.z) {
    m_dist->max.z = data.z;
    m_dist->bias.z = (m_dist->max.z + m_dist->min.z) / 2;
  }
}

int calibrate(int file, Triplet *bias)
{
  int i, div;
  MagDistribution m_dist = {{0},{0},{0}};

  div = 33;

  /* TODO Be smarter: stop reading when coverage is good enough */
  for (i = 0; i < 2000; ++i) {
    Triplet data = {0};
    read_triplet (file, XM_ADDRESS, OUT_X_L_M, &data);
    update_mag_bias (data, &m_dist);

    if (i % div == 0) {
      printf ("*");
      fflush(stdout);
    }
    usleep (20000);
  }
  printf ("                            \r");

  bias->x = m_dist.bias.x;
  bias->y = m_dist.bias.y;
  bias->z = m_dist.bias.z;
  return 1;
}

int main (int argc, char **argv)
{
  FILE *output;
  int file;
  Triplet bias;

  file = init_device (I2C_DEV_NAME);
  if (file == 0)
    return 1;

  init_mag (file, MAG_SCALE_2GS);

  output = fopen(BIAS_FILENAME, "w");
  if (!output){
    printf("Error opening '"BIAS_FILENAME"' for output.\n");
    return 1;
  }
  
  printf ("Calibrating (Edison should be turned to all directions until complete...\n");
  while (!calibrate (file, &bias))
    printf ("Calibration failed, trying again...\n");

  fprintf(output, "m_bias %d %d %d\n", bias.x, bias.y, bias.z);
  fclose (output);

  printf ("Calibration succesful, wrote '"BIAS_FILENAME"'.\n");  
  return 0;
}
