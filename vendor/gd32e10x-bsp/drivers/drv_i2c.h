#ifndef __DRV_I2C__
#define __DRV_I2C__

#include <device-tree.h>
#include <drivers/i2c.h>
#include <drivers/pin.h>
#include <gd32e10x.h>
#include <rtthread.h>

typedef struct {
  struct rt_i2c_bus_device parent;
  i2c_bus_device_t*        device;
} i2c_bus_t;

// rt_err_t rt_hw_i2c_device_attach(const char *, const char *, const char*);

#endif  // __DRV_I2C__
