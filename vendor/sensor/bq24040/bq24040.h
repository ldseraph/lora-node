#ifndef __BQ24040_H__
#define __BQ24040_H__
#include <board.h>
#include <device-tree.h>
#include <rtthread.h>

typedef struct {
  struct rt_device         parent;
  sensor_bq24040_device_t* device;
  rt_adc_device_t          adc_dev;
  rt_base_t                gpio_chg;
  rt_base_t                gpio_pg;
} sensor_bq24040_t;

rt_bool_t sensor_bq24040_charge_ok(rt_device_t);
rt_bool_t sensor_bq24040_is_charging(rt_device_t);

#endif