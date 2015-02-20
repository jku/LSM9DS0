#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <ncurses.h>

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

int get_col(int width, float f)
{
  /* assume values are in [-10000 , +10000] */
  return width/2 + (int)((width / 2) * f / 10000);
}

void print_distribution (Triplet min, Triplet max, Triplet data, int width, char limit, char sensor)
{
  mvaddch(3, get_col (width, min.x), limit);
  mvaddch(3, get_col (width, data.x), sensor);
  mvaddch(3, get_col (width, max.x), limit);
  mvaddch(4, get_col (width, min.y), limit);
  mvaddch(4, get_col (width, data.y), sensor);
  mvaddch(4, get_col (width, max.y), limit);
  mvaddch(5, get_col (width, min.z), limit);
  mvaddch(5, get_col (width, data.z), sensor);
  mvaddch(5, get_col (width, max.z), limit);
}

int calibrate(int file, int width, Triplet *bias, FTriplet *scale)
{
  Triplet data = {0};
  MagDistribution m_dist = {{0},{0},{0}};
  Triplet diameters;
  int avg_diameter;

  while (1) {
    print_distribution (m_dist.min, m_dist.max, data, width, ' ', ' ');

    read_triplet (file, XM_ADDRESS, OUT_X_L_M, &data);
    update_mag_bias (data, &m_dist);

    print_distribution (m_dist.min, m_dist.max, data, width, '|', '*');

    move (6, 0);
    refresh ();
    usleep (20000);
    if (getch () != ERR)
      break;
  }

  bias->x = m_dist.bias.x;
  bias->y = m_dist.bias.y;
  bias->z = m_dist.bias.z;

  /* Try to scale the axes (crude transform from ellipse to sphere) */
  diameters.x = abs (m_dist.max.x - m_dist.min.x);
  diameters.y = abs (m_dist.max.y - m_dist.min.y);
  diameters.z = abs (m_dist.max.z - m_dist.min.z);
  avg_diameter = (diameters.x + diameters.y + diameters.z) / 3;
  scale->x = avg_diameter / (float)diameters.x;
  scale->y = avg_diameter / (float)diameters.y;
  scale->z = avg_diameter / (float)diameters.z;

  return 1;
}

int main (int argc, char **argv)
{
  FILE *output;
  int file, row, col;
  Triplet bias;
  FTriplet scale;

  initscr ();
  nodelay (stdscr, 1);
  noecho ();
  cbreak ();
  curs_set (0);
  getmaxyx (stdscr, row, col);
  (void)row;

  file = init_device (I2C_DEV_NAME);
  if (file == 0)
    return 1;

  init_mag (file, MAG_SCALE_2GS);

  output = fopen(BIAS_FILENAME, "w");
  if (!output){
    printf("Error opening '"BIAS_FILENAME"' for output.\n");
    return 1;
  }
  mvprintw (0, 0, "Calibrating: Turn the Edison around all axes.");
  mvprintw (1, 0, "Try to find the absolute furthest limits of each axis.");
  mvaddch (3, 0, 'X');
  mvaddch (4, 0, 'Y');
  mvaddch (5, 0, 'Z');

  mvprintw (7, 0, "Press any key when done.");

  calibrate (file, col, &bias, &scale);

  endwin ();

  fprintf(output, "m_bias %d %d %d\n", bias.x, bias.y, bias.z);
  fprintf(output, "m_scale %f %f %f\n", scale.x, scale.y, scale.z);
  fclose (output);

  printf ("Calibration succesful, wrote '"BIAS_FILENAME"'.\n");
  return 0;
}
