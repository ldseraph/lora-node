#include "drv_i2c.h"
#define DBG_SECTION_NAME "i2c.bus"
#include <ulog.h>

static int i2c_read(rt_uint32_t i2c_periph, rt_uint16_t slave_address, rt_uint8_t *p_buffer, rt_uint16_t data_byte) {
  /* wait until I2C bus is idle */
  while (i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY)) {
  };

  /* send a start condition to I2C bus */
  i2c_start_on_bus(i2c_periph);

  /* wait until SBSEND bit is set */
  while (!i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND)) {
  };

  /* send slave address to I2C bus */
  i2c_master_addressing(i2c_periph, slave_address << 1, I2C_RECEIVER);

  /* wait until ADDSEND bit is set */
  while (!i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND)) {
  };

  /* clear the ADDSEND bit */
  i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);

  if (1 == data_byte) {
    /* disable acknowledge */
    i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(i2c_periph);
  }

  /* while there is data to be read */
  while (data_byte) {
    /* wait until the RBNE bit is set and clear it */
    if (i2c_flag_get(i2c_periph, I2C_FLAG_RBNE)) {
      /* read a byte from the EEPROM */
      *p_buffer = i2c_data_receive(i2c_periph);

      /* point to the next location where the byte read will be saved */
      p_buffer++;

      /* decrement the read bytes counter */
      data_byte--;
      if (1 == data_byte) {
        /* disable acknowledge */
        i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);
        /* send a stop condition to I2C bus */
        i2c_stop_on_bus(i2c_periph);
      }
    }
  }

  /* wait until the stop condition is finished */
  while (I2C_CTL0(i2c_periph) & 0x0200) {
  };

  /* enable acknowledge */
  i2c_ack_config(i2c_periph, I2C_ACK_ENABLE);

  i2c_ackpos_config(i2c_periph, I2C_ACKPOS_CURRENT);

  return 0;
}

static int i2c_write(rt_uint32_t i2c_periph, uint16_t slave_address, uint8_t *p_buffer, uint16_t data_byte) {
  /* wait until I2C bus is idle */
  while (i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY)) {
  };

  /* send a start condition to I2C bus */
  i2c_start_on_bus(i2c_periph);

  /* wait until SBSEND bit is set */
  while (!i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND)) {
  };

  /* send slave address to I2C bus */
  i2c_master_addressing(i2c_periph, slave_address << 1, I2C_TRANSMITTER);

  /* wait until ADDSEND bit is set */
  while (!i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND)) {
  };

  /* clear the ADDSEND bit */
  i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);

  /* wait until the transmit data buffer is empty */
  while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_TBE)) {
  };

  /* while there is data to be read */
  while (data_byte) {
    i2c_data_transmit(i2c_periph, *p_buffer);

    /* point to the next byte to be written */
    p_buffer++;

    /* decrement the write bytes counter */
    data_byte--;

    /* wait until BTC bit is set */
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_BTC)) {
    };
  }

  /* send a stop condition to I2C bus */
  i2c_stop_on_bus(i2c_periph);

  /* wait until the stop condition is finished */
  while (I2C_CTL0(i2c_periph) & 0x0200) {
  };

  return 0;
}

static rt_size_t i2c_xfer(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num) {
  struct rt_i2c_msg *msg;
  rt_uint32_t        i;
  rt_err_t           ret = RT_ERROR;

  i2c_bus_t *i2c_bus = (i2c_bus_t *)bus;

  for (i = 0; i < num; i++) {
    msg = &msgs[i];

    if (msg->flags & RT_I2C_ADDR_10BIT) {
      i2c_mode_addr_config(i2c_bus->device->i2c_base, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_10BITS, 0);
    } else {
      i2c_mode_addr_config(i2c_bus->device->i2c_base, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0);
    }
    if (msg->flags & RT_I2C_RD) {
      if (i2c_read(i2c_bus->device->i2c_base, msg->addr, msg->buf, msg->len) != 0) {
        LOG_E("i2c bus write failed,i2c bus stop!");
        goto out;
      }
    } else {
      if (i2c_write(i2c_bus->device->i2c_base, msg->addr, msg->buf, msg->len) != 0) {
        LOG_E("i2c bus write failed,i2c bus stop!");
        goto out;
      }
    }
  }

  ret = i;

out:
  LOG_D("send stop condition");

  return ret;
}

static const struct rt_i2c_bus_device_ops i2c_ops = {
  i2c_xfer,
  RT_NULL,
  RT_NULL
};

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
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
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
    i2c_bus->parent.ops = &i2c_ops;

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