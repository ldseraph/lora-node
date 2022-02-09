#ifndef __ADS1232_H__
#define __ADS1232_H__
#include <board.h>
#include <device-tree.h>
#include <rtthread.h>

typedef struct {
  struct rt_device parent;
  rt_base_t        gpio_dout;
  rt_base_t        gpio_sclk;
  rt_base_t        gpio_pdwn;
} sensor_ads1232_t;

#endif