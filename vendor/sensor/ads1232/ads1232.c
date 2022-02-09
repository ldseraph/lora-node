#include "ads1232.h"
#define DBG_SECTION_NAME "ads1232"
#include <ulog.h>

static rt_bool_t sensor_ads1232_wait_ready(sensor_ads1232_t *sensor_ads1232) {
  return rt_pin_read(sensor_ads1232->gpio_dout);
}

static void sensor_ads1232_power_down(sensor_ads1232_t *sensor_ads1232) {
  rt_pin_write(sensor_ads1232->gpio_pdwn, PIN_LOW);
  rt_pin_write(sensor_ads1232->gpio_sclk, PIN_LOW);
  rt_pin_mode(sensor_ads1232->gpio_dout, GPIO_MODE_OUT_PP);
  rt_pin_write(sensor_ads1232->gpio_dout, PIN_HIGH);
  rt_pin_mode(sensor_ads1232->gpio_dout, GPIO_MODE_IN_FLOATING);
  rt_thread_mdelay(10);
}

static void sensor_ads1232_power_up(sensor_ads1232_t *sensor_ads1232) {
  rt_pin_write(sensor_ads1232->gpio_pdwn, PIN_HIGH);
}

static int32_t sensor_ads1232_raw_read(sensor_ads1232_t *sensor_ads1232, rt_bool_t calibration) {
  while (sensor_ads1232_wait_ready(sensor_ads1232)) {
  };

  int32_t ad_value = 0;

  for (uint32_t i = 0; i < 24; i++) {
    ad_value = ad_value << 1;
    rt_pin_write(sensor_ads1232->gpio_sclk, PIN_HIGH);

    if (rt_pin_read(sensor_ads1232->gpio_dout)) {
      ad_value |= 0x000001;
    }

    board_delay_us(10);

    rt_pin_write(sensor_ads1232->gpio_sclk, PIN_LOW);
    board_delay_us(10);
  }

  rt_pin_mode(sensor_ads1232->gpio_dout, GPIO_MODE_OUT_PP);
  rt_pin_write(sensor_ads1232->gpio_dout, PIN_HIGH);

  rt_pin_write(sensor_ads1232->gpio_sclk, PIN_HIGH);
  board_delay_us(10);
  rt_pin_write(sensor_ads1232->gpio_sclk, PIN_LOW);
  board_delay_us(10);

  rt_pin_mode(sensor_ads1232->gpio_dout, GPIO_MODE_IN_FLOATING);

  if (calibration) {
    rt_pin_write(sensor_ads1232->gpio_sclk, PIN_HIGH);
    board_delay_us(10);
    rt_pin_write(sensor_ads1232->gpio_sclk, PIN_LOW);
    rt_thread_mdelay(900);
  }

  if (ad_value & 0x800000) {
    ad_value -= 0x1000000;
  }

  return ad_value;
}

static rt_err_t sensor_ads1232_init(rt_device_t dev) {
  sensor_ads1232_t *sensor_ads1232 = (sensor_ads1232_t *)dev;

  rt_pin_mode(sensor_ads1232->gpio_dout, GPIO_MODE_IN_FLOATING);
  rt_pin_mode(sensor_ads1232->gpio_sclk, GPIO_MODE_OUT_PP);
  rt_pin_mode(sensor_ads1232->gpio_pdwn, GPIO_MODE_OUT_PP);

  sensor_ads1232_power_down(sensor_ads1232);
  // sensor_ads1232_power_up(sensor_ads1232);
  // sensor_ads1232_raw_read(sensor_ads1232, RT_TRUE);

  return RT_EOK;
}

static rt_size_t sensor_ads1232_read(
  rt_device_t dev,
  rt_off_t    pos,
  void *      buffer,
  rt_size_t   max_size) {
  sensor_ads1232_t *sensor_ads1232 = (sensor_ads1232_t *)dev;

  sensor_ads1232_power_up(sensor_ads1232);
  sensor_ads1232_raw_read(sensor_ads1232, RT_TRUE);

  int32_t value[20] = { 0 };
  for (uint32_t i = 0; i < 20; i++) {
    value[i] = sensor_ads1232_raw_read(sensor_ads1232, RT_FALSE);
  }

  sensor_ads1232_power_down(sensor_ads1232);

  int32_t temp;
  for (uint32_t i = 0; i < 20 - 1; i++) {
    for (uint32_t j = 0; j < 20 - i - 1; j++) {
      if (value[j] > value[j + 1]) {
        temp         = value[j];
        value[j]     = value[j + 1];
        value[j + 1] = temp;
      }
    }
  }

  int64_t total = 0;
  for (uint32_t i = 6; i < 14; i++) {
    total += value[i];
  }
  total >>= 3;

  int32_t *buf = buffer;
  buf[0]       = (int32_t)total;

  return sizeof(int32_t);
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops sensor_ads1232_ops = {
  sensor_ads1232_init,
  RT_NULL,
  RT_NULL,
  sensor_ads1232_read,
  RT_NULL,
  RT_NULL
};
#endif

static rt_err_t sensor_ads1232_probe(devices_t *devices) {
  RT_ASSERT(RT_NULL != devices);

  for (uint32_t i = 0; i < devices->num; i++) {
    device_t *d = &(devices->list[i]);

    sensor_ads1232_device_t *device = d->device;
    if (RT_NULL == device) {
      LOG_E("sensor_ads1232_device is null");
      return -RT_EINVAL;
    }

    sensor_ads1232_t *sensor_ads1232 = rt_malloc(sizeof(sensor_ads1232_t));
    if (RT_NULL == sensor_ads1232) {
      LOG_E("OOM sensor ads1232");
      return -RT_ENOMEM;
    }

    sensor_ads1232->gpio_dout = rt_pin_get(device->dout);
    sensor_ads1232->gpio_sclk = rt_pin_get(device->sclk);
    sensor_ads1232->gpio_pdwn = rt_pin_get(device->pdwn);

    rt_base_t gpio_avdd = rt_pin_get(device->avdd);

    rt_pin_mode(gpio_avdd, GPIO_MODE_OUT_PP);
    rt_pin_write(gpio_avdd, PIN_HIGH);

    struct rt_device *parent = &(sensor_ads1232->parent);

    parent->type        = RT_Device_Class_Sensor;
    parent->rx_indicate = RT_NULL;
    parent->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    parent->ops = &sensor_ads1232_ops;
#else
    parent->init    = sensor_ads1232_init;
    parent->open    = RT_NULL;
    parent->close   = RT_NULL;
    parent->read    = sensor_ads1232_read;
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
    .name   = "ads1232:0",
    .device = &(sensor_ads1232_device_t){
      .dout = "PB.3",
      .sclk = "PB.4",
      .pdwn = "PB.5",
      .avdd = "PB.6",
    },
  }
};

static int sensor_ads1232_register() {
  devices_t devices = {
    .list = init_devices,
    .num  = (sizeof(init_devices) / sizeof(init_devices[0])),
  };

  rt_err_t err = sensor_ads1232_probe(&devices);
  if (err != RT_EOK) {
    return err;
  }
  return RT_EOK;
}

INIT_DEVICE_EXPORT(sensor_ads1232_register);