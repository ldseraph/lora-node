#include "bq24040.h"
#define DBG_SECTION_NAME "bq24040"
#include <ulog.h>

rt_bool_t sensor_bq24040_charge_ok(rt_device_t dev) {
  sensor_bq24040_t *sensor_bq24040 = (sensor_bq24040_t *)dev;
  return rt_pin_read(sensor_bq24040->gpio_pg);
}

rt_bool_t sensor_bq24040_is_charging(rt_device_t dev) {
  sensor_bq24040_t *sensor_bq24040 = (sensor_bq24040_t *)dev;
  return !rt_pin_read(sensor_bq24040->gpio_chg);
}

static rt_err_t sensor_bq24040_init(rt_device_t dev) {
  rt_err_t          err            = RT_EOK;
  sensor_bq24040_t *sensor_bq24040 = (sensor_bq24040_t *)dev;

  err = rt_adc_enable(sensor_bq24040->adc_dev, sensor_bq24040->device->channel);
  if (err != RT_EOK) {
    LOG_E("adc enable err");
    return err;
  }

  for (uint32_t i = 0; i < 3; i++) {
    rt_adc_read(sensor_bq24040->adc_dev, sensor_bq24040->device->channel);
  }

  rt_pin_mode(sensor_bq24040->gpio_chg, GPIO_MODE_IN_FLOATING);
  rt_pin_mode(sensor_bq24040->gpio_pg, GPIO_MODE_IN_FLOATING);

  return err;
}

static rt_size_t sensor_bq24040_read(
  rt_device_t dev,
  rt_off_t    pos,
  void *      buffer,
  rt_size_t   max_size) {
  sensor_bq24040_t *sensor_bq24040 = (sensor_bq24040_t *)dev;
  uint32_t *        temp           = buffer;

  uint32_t value = rt_adc_read(sensor_bq24040->adc_dev, sensor_bq24040->device->channel);

  value = value * 330 >> 12;

  temp[0] = value;

  return sizeof(uint32_t);
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops sensor_bq24040_ops = {
  sensor_bq24040_init,
  RT_NULL,
  RT_NULL,
  sensor_bq24040_read,
  RT_NULL,
  RT_NULL
};
#endif

static rt_err_t sensor_bq24040_probe(devices_t *devices) {
  RT_ASSERT(RT_NULL != devices);

  for (uint32_t i = 0; i < devices->num; i++) {
    device_t *d = &(devices->list[i]);

    sensor_bq24040_device_t *sensor_bq24040_device = d->device;
    if (RT_NULL == sensor_bq24040_device) {
      LOG_E("sensor_bq24040_device is null");
      return -RT_EINVAL;
    }

    sensor_bq24040_t *sensor_bq24040 = rt_malloc(sizeof(sensor_bq24040_t));
    if (RT_NULL == sensor_bq24040) {
      LOG_E("OOM sensor_bq24040");
      return -RT_ENOMEM;
    }

    rt_adc_device_t adc_dev = (rt_adc_device_t)rt_device_find(sensor_bq24040_device->adc);
    if (adc_dev == RT_NULL) {
      LOG_E("%s find err ", sensor_bq24040_device->adc);
      return -RT_EINVAL;
    }
    sensor_bq24040->device   = sensor_bq24040_device;
    sensor_bq24040->adc_dev  = adc_dev;
    sensor_bq24040->gpio_chg = rt_pin_get(sensor_bq24040_device->chg);
    sensor_bq24040->gpio_pg  = rt_pin_get(sensor_bq24040_device->pg);

    struct rt_device *parent = &(sensor_bq24040->parent);

    parent->type        = RT_Device_Class_Sensor;
    parent->rx_indicate = RT_NULL;
    parent->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    parent->ops = &sensor_bq24040_ops;
#else
    parent->init    = sensor_bq24040_init;
    parent->open    = RT_NULL;
    parent->close   = RT_NULL;
    parent->read    = sensor_bq24040_read;
    parent->write   = RT_NULL;
    parent->control = RT_NULL;
#endif

    rt_err_t err = rt_device_register(parent, d->name, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
    if (err != RT_EOK) {
      LOG_E("%s register err %d ", d->name, err);
      return err;
    }
  }

  return RT_EOK;
}

// TODO: dts
static device_t init_devices[] = {
  {
    .name   = "bq24040:0",
    .device = &(sensor_bq24040_device_t){
      .adc     = "adc0",
      .channel = 8,
      .chg     = "PA.11",
      .pg      = "PA.12",
    },
  }
};

static int sensor_bq24040_register() {
  devices_t devices = {
    .list = init_devices,
    .num  = (sizeof(init_devices) / sizeof(init_devices[0])),
  };

  rt_err_t err = sensor_bq24040_probe(&devices);
  if (err != RT_EOK) {
    return err;
  }
  return RT_EOK;
}

INIT_DEVICE_EXPORT(sensor_bq24040_register);