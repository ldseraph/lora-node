#ifndef __ADXL355_H__
#define __ADXL355_H__
#include <board.h>
#include <device-tree.h>
#include <rtthread.h>

typedef struct {
  struct rt_device         parent;
  rt_device_t              i2c_bus;
  sensor_adxl355_device_t *device;
  float                    scale;
} sensor_adxl355_t;

typedef struct {
  double xangle;
  double yangle;
  double zangle;
} sensor_adxl355_data_t;

#endif