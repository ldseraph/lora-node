#include "drv_spi.h"

/**
  * Attach the spi device to SPI bus, this function must be used after initialization.
  */
rt_err_t rt_hw_spi_device_attach(const char *bus_name, const char *device_name, const char *cs_gpio_name) {
  RT_ASSERT(bus_name != RT_NULL);
  RT_ASSERT(device_name != RT_NULL);
  RT_ASSERT(cs_gpio_name != RT_NULL);

  rt_base_t cs_gpiox = rt_pin_get(cs_gpio_name);

  rt_pin_mode(cs_gpiox, GPIO_MODE_OUT_PP);

  struct rt_spi_device *spi_device = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));
  if (spi_device == RT_NULL) {
    return -RT_ENOMEM;
  }

  rt_err_t err = rt_spi_bus_attach_device(spi_device, device_name, bus_name, (void *)cs_gpiox);
  if (err != RT_EOK) {
    return err;
  }

  return err;
}

static rt_err_t configure(struct rt_spi_device *       device,
                          struct rt_spi_configuration *configuration) {
  RT_ASSERT(device != RT_NULL);
  RT_ASSERT(configuration != RT_NULL);

  spi_bus_t *spi_bus  = (spi_bus_t *)device->bus;
  uint32_t   spi_base = spi_bus->device->spi_base;

  spi_parameter_struct spi_init_struct;

  if (configuration->data_width <= 8) {
    spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;
  } else if (configuration->data_width <= 16) {
    spi_init_struct.frame_size = SPI_FRAMESIZE_16BIT;
  } else {
    return -RT_EIO;
  }

  rcu_clock_freq_enum spi_src       = (spi_base == SPI1) ? CK_APB1 : CK_APB2;
  uint32_t            spi_apb_clock = rcu_clock_freq_get(spi_src);
  uint32_t            max_hz        = configuration->max_hz;

  if (max_hz >= spi_apb_clock / 2) {
    spi_init_struct.prescale = SPI_PSC_2;
  } else if (max_hz >= spi_apb_clock / 4) {
    spi_init_struct.prescale = SPI_PSC_4;
  } else if (max_hz >= spi_apb_clock / 8) {
    spi_init_struct.prescale = SPI_PSC_8;
  } else if (max_hz >= spi_apb_clock / 16) {
    spi_init_struct.prescale = SPI_PSC_16;
  } else if (max_hz >= spi_apb_clock / 32) {
    spi_init_struct.prescale = SPI_PSC_32;
  } else if (max_hz >= spi_apb_clock / 64) {
    spi_init_struct.prescale = SPI_PSC_64;
  } else if (max_hz >= spi_apb_clock / 128) {
    spi_init_struct.prescale = SPI_PSC_128;
  } else {
    spi_init_struct.prescale = SPI_PSC_256;
  }

  switch (configuration->mode & RT_SPI_MODE_3) {
  case RT_SPI_MODE_0:
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    break;
  case RT_SPI_MODE_1:
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
    break;
  case RT_SPI_MODE_2:
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_1EDGE;
    break;
  case RT_SPI_MODE_3:
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
    break;
  }

  spi_init_struct.endian = (configuration->mode & RT_SPI_MSB) ? SPI_ENDIAN_MSB : SPI_ENDIAN_LSB;

  spi_init_struct.trans_mode  = SPI_TRANSMODE_FULLDUPLEX;
  spi_init_struct.device_mode = SPI_MASTER;
  spi_init_struct.nss         = SPI_NSS_SOFT;

  spi_disable(spi_base);

  spi_init(spi_base, &spi_init_struct);

  spi_crc_off(spi_base);

  spi_enable(spi_base);

  return RT_EOK;
};

static rt_uint32_t xfer(struct rt_spi_device *device, struct rt_spi_message *message) {
  RT_ASSERT(device != NULL);
  RT_ASSERT(message != NULL);

  rt_base_t  gd32_cs_pin = (rt_base_t)device->parent.user_data;
  spi_bus_t *spi_bus     = (spi_bus_t *)device->bus;
  uint32_t   spi_base    = spi_bus->device->spi_base;

  struct rt_spi_configuration *config = &device->config;

  if (message->cs_take) {
    rt_pin_write(gd32_cs_pin, PIN_LOW);
  }

  if (config->data_width <= 8) {
    const rt_uint8_t *send_ptr = message->send_buf;
    rt_uint8_t *      recv_ptr = message->recv_buf;
    rt_uint32_t       size     = message->length;

    while (size--) {
      rt_uint8_t data = 0xFF;

      if (send_ptr != RT_NULL) {
        data = *send_ptr++;
      }

      // Todo: replace register read/write by gd32f3 lib
      //Wait until the transmit buffer is empty
      while (RESET == spi_i2s_flag_get(spi_base, SPI_FLAG_TBE)) {
      };
      // Send the byte
      spi_i2s_data_transmit(spi_base, data);

      //Wait until a data is received
      while (RESET == spi_i2s_flag_get(spi_base, SPI_FLAG_RBNE)) {
      };
      // Get the received data
      data = spi_i2s_data_receive(spi_base);

      if (recv_ptr != RT_NULL) {
        *recv_ptr++ = data;
      }
    }

  } else if (config->data_width <= 16) {
    const rt_uint16_t *send_ptr = message->send_buf;
    rt_uint16_t *      recv_ptr = message->recv_buf;
    rt_uint32_t        size     = message->length;

    while (size--) {
      rt_uint16_t data = 0xFF;

      if (send_ptr != RT_NULL) {
        data = *send_ptr++;
      }

      //Wait until the transmit buffer is empty
      while (RESET == spi_i2s_flag_get(spi_base, SPI_FLAG_TBE)) {
      };
      // Send the byte
      spi_i2s_data_transmit(spi_base, data);

      //Wait until a data is received
      while (RESET == spi_i2s_flag_get(spi_base, SPI_FLAG_RBNE)) {
      };
      // Get the received data
      data = spi_i2s_data_receive(spi_base);

      if (recv_ptr != RT_NULL) {
        *recv_ptr++ = data;
      }
    }
  }

  if (message->cs_release) {
    rt_pin_write(gd32_cs_pin, PIN_HIGH);
  }

  return message->length;
};

static struct rt_spi_ops spi_bus_ops = {
  configure,
  xfer
};

static void spi_bus_init(spi_bus_t *spi_bus) {
  rcu_periph_clock_enable(RCU_AF);

  switch (spi_bus->device->spi_base) {
  default:
    break;
  case SPI0: {
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_SPI0);

    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_7);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    spi_i2s_deinit(SPI0);
    break;
  }
  }
  return;
}

static rt_err_t spi_bus_device_probe(devices_t *devices) {
  for (uint32_t i = 0; i < devices->num; i++) {
    device_t *d = &(devices->list[i]);

    spi_bus_t *spi_bus = rt_malloc(sizeof(spi_bus_t));
    if (spi_bus == RT_NULL) {
      return -RT_ENOMEM;
    }

    spi_bus_device_t *spi_bus_device = d->device;
    if (RT_NULL == spi_bus_device) {
      return -RT_EINVAL;
    }

    spi_bus->device = spi_bus_device;

    spi_bus_init(spi_bus);

    rt_err_t err = rt_spi_bus_register((struct rt_spi_bus *)spi_bus, d->name, &spi_bus_ops);
    if (err != RT_EOK) {
      return err;
    }
  }

  return RT_EOK;
}

static device_t init_devices[] = {
  {
    .name   = "spi-bus0",
    .device = &(spi_bus_device_t){
      .spi_base = SPI0,
    },
  }
};

int spi_bus_device_register(void) {
  devices_t devices = {
    .list = init_devices,
    .num  = (sizeof(init_devices) / sizeof(init_devices[0]))
  };

  rt_err_t err = spi_bus_device_probe(&devices);
  if (err != RT_EOK) {
    return err;
  }

  return RT_EOK;
}

INIT_BOARD_EXPORT(spi_bus_device_register);
