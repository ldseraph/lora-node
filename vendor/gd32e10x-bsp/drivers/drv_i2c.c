#include "drv_i2c.h"

static void i2c_bus_init(i2c_bus_t *i2c_bus) {
  rcu_periph_clock_enable(RCU_AF);

  switch (i2c_bus->device->i2c_base) {
  default:
    break;
  case I2C0: {
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_I2C0);
  
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
    i2c_clock_config(I2C0, i2c_bus->device->clkspeed, I2C_DTCY_2);
    i2c_enable(I2C0);
    i2c_ack_config(I2C0,I2C_ACK_ENABLE);
    break;
  }
  }
  return;
}

static rt_err_t i2c_bus_device_probe(devices_t *devices) {
  for (uint32_t i = 0; i < devices->num; i++) {
    device_t *d = &(devices->list[i]);

    i2c_bus_t *i2c_bus = rt_malloc(sizeof(i2c_bus_t));
    if (i2c_bus == RT_NULL) {
      return -RT_ENOMEM;
    }

    i2c_bus_device_t *i2c_bus_device = d->device;
    if (RT_NULL == i2c_bus_device) {
      return -RT_EINVAL;
    }

    i2c_bus->device = i2c_bus_device;

    i2c_bus_init(i2c_bus);

    rt_err_t err = rt_i2c_bus_device_register((struct rt_i2c_bus_device *)i2c_bus, d->name);
    if (err != RT_EOK) {
      return err;
    }
  }

  return RT_EOK;
}

// TODO
static device_t init_devices[] = {
  {
    .name   = "i2c-bus0",
    .device = &(i2c_bus_device_t){
      .i2c_base = I2C0,
      .clkspeed = 100000,
    },
  }
};

int i2c_bus_device_register(void) {
  devices_t devices = {
    .list = init_devices,
    .num  = (sizeof(init_devices) / sizeof(init_devices[0]))
  };

  rt_err_t err = i2c_bus_device_probe(&devices);
  if (err != RT_EOK) {
    return err;
  }

  return RT_EOK;
}

INIT_BOARD_EXPORT(i2c_bus_device_register);