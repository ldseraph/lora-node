#include <lora-radio.h>
#include <math.h>
#define DBG_SECTION_NAME "lora-radio"
#include <ulog.h>

#include "drv_spi.h"

#define EV_LORA_RADIO_DIO_IRQ 0x1
#define EV_LORA_RADIO_TIMEOUT 0x2

// static rt_bool_t lora_radio_is_idle(lora_radio_operating_modes_t mode) {
//   if (mode != MODE_TX && mode != MODE_RX && mode != MODE_CAD) {
//     return RT_TRUE;
//   }
//   return RT_FALSE;
// }

static void lora_radio_timeout_irq(void *parameter) {
  lora_radio_t *lora_radio = (lora_radio_t *)parameter;
  rt_interrupt_enter();
  rt_event_send(&lora_radio->irq_event, EV_LORA_RADIO_TIMEOUT);
  rt_interrupt_leave();
}

static void lora_radio_dio_irq(lora_radio_t *lora_radio) {
  rt_interrupt_enter();
  rt_event_send(&lora_radio->irq_event, EV_LORA_RADIO_DIO_IRQ);
  rt_interrupt_leave();
}

static rt_err_t lora_radio_set_modem(lora_radio_t *lora_radio, lora_radio_modems_t modem) {
  if (lora_radio->modem == modem) {
    return RT_EOK;
  }

  lora_radio_packet_types_t packet_type = LORD_RADIO_PACKET_TYPE_LORA;
  if (modem == LORD_RADIO_MODEM_FSK) {
    packet_type = LORD_RADIO_PACKET_TYPE_GFSK;
  }

  rt_err_t err = lora_radio->ops->set_packet_type(lora_radio, packet_type);
  if (err != RT_EOK) {
    return err;
  }

  err = lora_radio->ops->set_sync_word(lora_radio, modem, lora_radio->lora_sync_word);
  if (err != RT_EOK) {
    return err;
  }

  lora_radio->packet_type = packet_type;
  lora_radio->modem       = modem;
  return RT_EOK;
}

static int8_t lora_radio_compute_tx_power(
  lora_radio_tx_power_t tx_power,
  float                 max_eirp,
  float                 antenna_gain) {
  return (int8_t)floor((max_eirp - (tx_power * 2U)) - antenna_gain);
}

static uint32_t lora_radio_get_lora_bandwidth_hz(lora_radio_lora_bandwidths_t bw) {
  uint32_t bandwidth_hz = 0;

  switch (bw) {
  case LORA_BW_007:
    bandwidth_hz = 7812UL;
    break;
  case LORA_BW_010:
    bandwidth_hz = 10417UL;
    break;
  case LORA_BW_015:
    bandwidth_hz = 15625UL;
    break;
  case LORA_BW_020:
    bandwidth_hz = 20833UL;
    break;
  case LORA_BW_031:
    bandwidth_hz = 31250UL;
    break;
  case LORA_BW_041:
    bandwidth_hz = 41667UL;
    break;
  case LORA_BW_062:
    bandwidth_hz = 62500UL;
    break;
  case LORA_BW_125:
    bandwidth_hz = 125000UL;
    break;
  case LORA_BW_250:
    bandwidth_hz = 250000UL;
    break;
  case LORA_BW_500:
    bandwidth_hz = 500000UL;
    break;
  }

  return bandwidth_hz;
}

static uint32_t lora_radio_get_lora_time_on_air(lora_radio_tx_config_args_t *args) {
  uint16_t                        preamble_length    = args->preamble_length;
  lora_radio_coding_rates_t       coding_rate        = args->modem_args.lora.coding_rate;
  lora_radio_spreading_factors_t  spreading_factors  = args->modem_args.lora.spreading_factors;
  lora_radio_lora_bandwidths_t    bandwidth          = args->modem_args.lora.bandwidth;
  uint32_t                        packet_length      = args->packet_length;
  lora_radio_packet_length_mode_t packet_length_mode = args->packet_length_mode;
  rt_bool_t                       crc_on             = args->crc_on;

  int32_t   cr_denom              = coding_rate + 4;
  rt_bool_t low_datarate_optimize = RT_FALSE;

  if ((spreading_factors == LORA_SF5) || (spreading_factors == LORA_SF6)) {
    if (preamble_length < 12) {
      preamble_length = 12;
    }
  }

  if (((bandwidth == LORA_BW_125) && ((spreading_factors == LORA_SF11) || (spreading_factors == LORA_SF12)))
      || ((bandwidth == LORA_BW_250) && (spreading_factors == LORA_SF12))) {
    low_datarate_optimize = RT_TRUE;
  }

  int32_t ceil_denominator;
  int32_t ceil_numerator = (packet_length << 3) + (crc_on ? 16 : 0) - (4 * spreading_factors) + (packet_length_mode ? 0 : 20);

  if (spreading_factors <= LORA_SF6) {
    ceil_denominator = 4 * spreading_factors;
  } else {
    ceil_numerator += 8;

    if (low_datarate_optimize == RT_TRUE) {
      ceil_denominator = 4 * (spreading_factors - 2);
    } else {
      ceil_denominator = 4 * spreading_factors;
    }
  }

  if (ceil_numerator < 0) {
    ceil_numerator = 0;
  }

  int32_t intermediate = ((ceil_numerator + ceil_denominator - 1) / ceil_denominator) * cr_denom + preamble_length + 12;

  if (spreading_factors <= LORA_SF6) {
    intermediate += 2;
  }

  return (uint32_t)((4 * intermediate + 1) * (1 << (spreading_factors - 2)));
}

static uint32_t lora_radio_get_fsk_time_on_air(lora_radio_tx_config_args_t *args) {
  uint16_t                        preamble_length    = args->preamble_length;
  uint32_t                        packet_length      = args->packet_length;
  lora_radio_packet_length_mode_t packet_length_mode = args->packet_length_mode;
  rt_bool_t                       crc_on             = args->crc_on;
  const uint8_t                   sync_word_length   = 3;

  return (preamble_length << 3) + ((packet_length_mode) ? 0 : 8) + (sync_word_length << 3) + ((packet_length + ((crc_on) ? 2 : 0)) << 3);
}

static rt_time_t lora_radio_tx_time_on_air(lora_radio_tx_config_args_t *args) {
  uint32_t numerator   = 0;
  uint32_t denominator = 1;

  switch (args->modem) {
  case LORD_RADIO_MODEM_FSK: {
    numerator   = lora_radio_get_fsk_time_on_air(args);
    denominator = args->modem_args.fsk.bandwidth;
  } break;
  case LORD_RADIO_MODEM_LORA: {
    numerator   = lora_radio_get_lora_time_on_air(args);
    denominator = lora_radio_get_lora_bandwidth_hz(args->modem_args.lora.bandwidth);
  } break;
  case LORD_RADIO_MODEM_NONE:
  default:
    return 0;
  }

  return (numerator * 1000U + denominator - 1) / denominator;
}

rt_err_t lora_radio_set_channel(rt_device_t dev, uint32_t frequency) {
  lora_radio_t *lora_radio = (lora_radio_t *)dev;
  return lora_radio->ops->set_frequency(lora_radio, frequency);
}

rt_err_t lora_radio_random(rt_device_t dev, uint32_t *random) {
  lora_radio_t *lora_radio = (lora_radio_t *)dev;

  rt_err_t err = lora_radio_set_modem(lora_radio, LORD_RADIO_MODEM_LORA);
  if (err != RT_EOK) {
    return err;
  }

  *random = lora_radio->ops->random(lora_radio);
  if (*random == 0) {
    return -RT_ERROR;
  }
  return RT_EOK;
}

rt_err_t lora_radio_tx_config(rt_device_t dev, lora_radio_tx_config_args_t *args) {
  lora_radio_t *lora_radio = (lora_radio_t *)dev;

  lora_radio_modems_t modem = args->modem;

  rt_err_t err = lora_radio->ops->standby(lora_radio);
  if (err != RT_EOK) {
    return err;
  }

  err = lora_radio_set_modem(lora_radio, modem);
  if (err != RT_EOK) {
    return err;
  }

  err = lora_radio->ops->set_modulation_params(lora_radio, args);
  if (err != RT_EOK) {
    return err;
  }

  err = lora_radio->ops->set_packet_params(lora_radio, args);
  if (err != RT_EOK) {
    return err;
  }

  int8_t phy_tx_power = lora_radio_compute_tx_power(
    args->tx_power,
    lora_radio->max_eirp,
    lora_radio->antenna_gain);

  err = lora_radio->ops->set_tx_power(lora_radio, phy_tx_power);
  if (err != RT_EOK) {
    return err;
  }

  err = lora_radio->ops->after_tx_config(lora_radio, args);
  if (err != RT_EOK) {
    return err;
  }

  lora_radio->packet_length_mode = args->packet_length_mode;
  lora_radio->tx_time_on_air     = lora_radio_tx_time_on_air(args);

  return RT_EOK;
}

rt_err_t lora_radio_rx_config(rt_device_t dev, lora_radio_rx_config_args_t *args) {
  lora_radio_t *lora_radio = (lora_radio_t *)dev;

  lora_radio_modems_t modem = args->modem;

  lora_radio->rx_continuous = args->rx_continuous;

  if (args->rx_continuous == RT_TRUE) {
    args->symbol_timeout = 0;
  }

  rt_err_t err = lora_radio->ops->before_rx_config(lora_radio, args);
  if (err != RT_EOK) {
    return err;
  }

  err = lora_radio->ops->standby(lora_radio);
  if (err != RT_EOK) {
    return err;
  }

  err = lora_radio_set_modem(lora_radio, modem);
  if (err != RT_EOK) {
    return err;
  }

  err = lora_radio->ops->set_modulation_params(lora_radio, args);
  if (err != RT_EOK) {
    return err;
  }

  err = lora_radio->ops->set_packet_params(lora_radio, args);
  if (err != RT_EOK) {
    return err;
  }

  err = lora_radio->ops->after_rx_config(lora_radio, args);
  if (err != RT_EOK) {
    return err;
  }

  return RT_EOK;
}

rt_err_t lora_radio_sleep(rt_device_t dev) {
  lora_radio_t *lora_radio = (lora_radio_t *)dev;

  lora_radio_sleep_args_t args = {
    .wakeup_rtc = RT_FALSE,
    .reset      = RT_FALSE,
    .warm_start = RT_TRUE,
  };

  rt_err_t err = lora_radio->ops->sleep(lora_radio, &args);
  if (err != RT_EOK) {
    return err;
  }
  rt_thread_mdelay(2);
  return RT_EOK;
}

lora_radio_lora_rx_status_t lora_radio_get_lora_rx_status(rt_device_t dev) {
  lora_radio_t *lora_radio = (lora_radio_t *)dev;
  return lora_radio->lora_rx_status;
}

lora_radio_fsk_rx_status_t lora_radio_get_fsk_rx_status(rt_device_t dev) {
  lora_radio_t *lora_radio = (lora_radio_t *)dev;
  return lora_radio->fsk_rx_status;
}

uint32_t lora_radio_wakeup_time(rt_device_t dev) {
  lora_radio_t *lora_radio = (lora_radio_t *)dev;
  return lora_radio->ops->wakeup_time(lora_radio);
}

static rt_err_t timeout_init(lora_radio_t *lora_radio) {
  lora_radio->timeout_timer = rt_timer_create(
    "lora-radio:tot",
    lora_radio_timeout_irq,
    lora_radio,
    RT_TICK_MAX,
    RT_TIMER_FLAG_ONE_SHOT);
  if (lora_radio->timeout_timer == RT_NULL) {
    return -RT_EEMPTY;
  }

  return RT_EOK;
}

static rt_err_t lora_radio_dio_irq_process(lora_radio_t *lora_radio, lora_radio_event_t *ev) {
  uint32_t irq;
  rt_err_t err = lora_radio->ops->get_irq(lora_radio, &irq);
  if (err != RT_EOK) {
    LOG_E("dio irq process get irq err: %d", err);
    return err;
  }

  err = lora_radio->ops->clean_irq(lora_radio, irq);
  if (err != RT_EOK) {
    LOG_E("dio irq process clean irq err: %d", err);
    return err;
  }

  if ((irq & IRQ_TX_DONE) == IRQ_TX_DONE) {
    err = rt_timer_stop(lora_radio->timeout_timer);
    if (err != RT_EOK) {
      LOG_E("dio irq process timer stop err: %d", err);
      return err;
    }

    lora_radio->mode = MODE_STDBY;
    *ev |= LORA_RADIO_EVENT_TX_DONE;
  }

  if ((irq & IRQ_RX_DONE) == IRQ_RX_DONE) {
    if ((irq & IRQ_CRC_ERROR) == IRQ_CRC_ERROR) {
      LOG_E("crc err");
    }
    if (lora_radio->rx_continuous == RT_FALSE) {
      err = rt_timer_stop(lora_radio->timeout_timer);
      if (err != RT_EOK) {
        LOG_E("dio irq process timer stop err: %d", err);
        return err;
      }
      lora_radio->mode = MODE_STDBY;

      rt_err_t err = lora_radio->ops->rx_done(lora_radio);
      if (err != RT_EOK) {
        LOG_E("rx done err: %d", err);
        return 0;
      }
    }
    *ev |= LORA_RADIO_EVENT_RX_DONE;
  }

  // if ((irqRegs & IRQ_CAD_DONE) == IRQ_CAD_DONE) {
  //   //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
  //   SX126xSetOperatingMode(MODE_STDBY_RC);
  //   if ((RadioEvents != NULL) && (RadioEvents->CadDone != NULL)) {
  //     RadioEvents->CadDone(((irqRegs & IRQ_CAD_ACTIVITY_DETECTED) == IRQ_CAD_ACTIVITY_DETECTED));
  //   }
  // }

  if ((irq & IRQ_RX_TX_TIMEOUT) == IRQ_RX_TX_TIMEOUT) {
    err = rt_timer_stop(lora_radio->timeout_timer);
    if (err != RT_EOK) {
      LOG_E("dio irq process timer stop err: %d", err);
      return err;
    }

    if (lora_radio->mode == MODE_TX) {
      *ev |= LORA_RADIO_EVENT_TX_TIMEOUT;
    } else if (lora_radio->mode == MODE_RX) {
      *ev |= LORA_RADIO_EVENT_RX_TIMEOUT;
    }

    lora_radio->mode = MODE_STDBY;
  }

  if ((irq & IRQ_PREAMBLE_DETECTED) == IRQ_PREAMBLE_DETECTED) {
    __NOP();
  }

  if ((irq & IRQ_SYNCWORD_VALID) == IRQ_SYNCWORD_VALID) {
    *ev |= LORA_RADIO_EVENT_RX_HEAD_DETECTION;
  }

  if ((irq & IRQ_HEADER_VALID) == IRQ_HEADER_VALID) {
    *ev |= LORA_RADIO_EVENT_RX_HEAD_DETECTION;
  }

  if ((irq & IRQ_HEADER_ERROR) == IRQ_HEADER_ERROR) {
    if (lora_radio->rx_continuous == RT_FALSE) {
      err = rt_timer_stop(lora_radio->timeout_timer);
      if (err != RT_EOK) {
        LOG_E("dio irq process timer stop err: %d", err);
        return err;
      }
      lora_radio->mode = MODE_STDBY;
    }
    *ev |= LORA_RADIO_EVENT_RX_ERROR;
  }

  return RT_EOK;
}

rt_err_t lora_radio_event_recv(rt_device_t dev, lora_radio_event_t *ev) {
  lora_radio_t *lora_radio = (lora_radio_t *)dev;

  rt_uint32_t radio_event;

  rt_err_t err = rt_event_recv(
    &lora_radio->irq_event,
    EV_LORA_RADIO_DIO_IRQ | EV_LORA_RADIO_TIMEOUT,
    RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
    RT_WAITING_FOREVER,
    &radio_event);
  if (err != RT_EOK) {
    LOG_E("lora radio event recv err: %d", err);
    return err;
  }

  switch (radio_event) {
  case EV_LORA_RADIO_DIO_IRQ:
    err = lora_radio_dio_irq_process(lora_radio, ev);
    if (err != RT_EOK) {
      LOG_E("lora radio dio process err: %d", err);
      return err;
    }
    break;
  case EV_LORA_RADIO_TIMEOUT:
    if (lora_radio->mode == MODE_TX) {
      *ev |= LORA_RADIO_EVENT_TX_TIMEOUT;
    } else if (lora_radio->mode == MODE_RX) {
      *ev |= LORA_RADIO_EVENT_RX_TIMEOUT;
    }

    lora_radio->mode = MODE_STDBY;
    break;
  default:
    LOG_E("thread ev:invalid ev(%d)", radio_event);
    return -RT_EINVAL;
  }

  return err;
}

static rt_err_t lora_radio_spi_init(
  const char *bus_name,
  const char *device_name,
  const char *gpio_name) {
  rt_err_t err = RT_EOK;

  RT_ASSERT(bus_name != RT_NULL);
  RT_ASSERT(device_name != RT_NULL);
  RT_ASSERT(gpio_name != RT_NULL);

  err = rt_hw_spi_device_attach(bus_name, device_name, gpio_name);
  if (err != RT_EOK) {
    LOG_E("rt_spi_bus_attach_device failed: %d", err);
    return err;
  }

  struct rt_spi_device *spi_device = (struct rt_spi_device *)rt_device_find(device_name);
  if (!spi_device) {
    LOG_E("spi sample run failed! cant't find %s device!", device_name);
    return -RT_EEMPTY;
  }

  struct rt_spi_configuration cfg;
  cfg.data_width = 8;
  cfg.mode       = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0. */
  cfg.max_hz     = 8 * 1000000;                                /* max 10M */

  err = rt_spi_configure(spi_device, &cfg);
  if (err != RT_EOK) {
    LOG_E("rt_spi_configure failed: %d", err);
    return err;
  }

  err = rt_spi_take_bus(spi_device);
  if (err != RT_EOK) {
    LOG_E("rt_spi_take_bus failed: %d", err);
    return err;
  }

  err = rt_spi_release_bus(spi_device);
  if (err != RT_EOK) {
    LOG_E("rt_spi_release_bus failed: %d", err);
    return err;
  }

  return err;
}

static rt_err_t lora_radio_init(rt_device_t dev) {
  rt_err_t      err        = RT_EOK;
  lora_radio_t *lora_radio = (lora_radio_t *)dev;

  err = rt_event_init(&lora_radio->irq_event, lora_radio->lora_radio_name, RT_IPC_FLAG_FIFO);  //RT_IPC_FLAG_PRIO
  if (err != RT_EOK) {
    LOG_E("rt_event_init err: %d", err);
    return err;
  }

  lora_radio->dio_irq = lora_radio_dio_irq;

  err = timeout_init(lora_radio);
  if (err != RT_EOK) {
    LOG_E("timeout_init err: %d", err);
    return err;
  }

  lora_radio->modem          = LORD_RADIO_MODEM_NONE;
  lora_radio->lora_sync_word = LORA_RADIO_PUBLIC_SYNCWORD;
  lora_radio->antenna_gain   = LORA_RADIO_DEFAULT_ANTENNA_GAIN;
  lora_radio->max_eirp       = LORA_RADIO_DEFAULT_MAX_EIRP;

  err = lora_radio->ops->init(lora_radio);
  if (err != RT_EOK) {
    LOG_E("ops init err: %d", err);
    return err;
  }

  return err;
}

static rt_size_t lora_radio_read(
  rt_device_t dev,
  rt_off_t    pos,
  void *      buffer,
  rt_size_t   max_size) {
  lora_radio_t *lora_radio = (lora_radio_t *)dev;

  rt_size_t size;

  rt_err_t err = lora_radio->ops->read(lora_radio, (uint8_t *)buffer, &size, max_size);
  if (err != RT_EOK) {
    LOG_E("ops read err: %d", err);
    return 0;
  }

  return size;
}

static rt_size_t lora_radio_write(
  rt_device_t dev,
  rt_off_t    pos,
  const void *buffer,
  rt_size_t   size) {
  lora_radio_t *lora_radio = (lora_radio_t *)dev;

  rt_err_t err = lora_radio->ops->send(lora_radio, pos, (const uint8_t *)buffer, size);
  if (err != RT_EOK) {
    LOG_E("write send err: %d", err);
    return 0;
  }

  uint32_t tick = rt_tick_from_millisecond(lora_radio->tx_time_on_air);

  tick = tick * 3 / 2;
  err  = rt_timer_control(lora_radio->timeout_timer, RT_TIMER_CTRL_SET_TIME, &tick);
  if (err != RT_EOK) {
    LOG_E("write set tx time err: %d", err);
    return 0;
  }

  err = rt_timer_start(lora_radio->timeout_timer);
  if (err != RT_EOK) {
    LOG_E("write time start err: %d", err);
    return 0;
  }

  return lora_radio->tx_time_on_air;
}

rt_err_t lora_radio_rx(rt_device_t dev, uint32_t timeout) {
  lora_radio_t *lora_radio = (lora_radio_t *)dev;

  if (lora_radio->rx_continuous == RT_FALSE) {
    uint32_t tick = rt_tick_from_millisecond(timeout);

    rt_err_t err = rt_timer_control(lora_radio->timeout_timer, RT_TIMER_CTRL_SET_TIME, &tick);
    if (err != RT_EOK) {
      LOG_E("rx set time err: %d", err);
      return 0;
    }

    err = rt_timer_start(lora_radio->timeout_timer);
    if (err != RT_EOK) {
      LOG_E("rx time start err: %d", err);
      return 0;
    }
  }

  rt_err_t err = lora_radio->ops->rx(lora_radio);
  if (err != RT_EOK) {
    LOG_E("ops rx err: %d", err);
    return err;
  }

  return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops lora_radio_ops = {
  lora_radio_init,
  RT_NULL,
  RT_NULL,
  lora_radio_read,
  lora_radio_write,
  RT_NULL
};
#endif

rt_err_t lora_radio_device_register(lora_radio_t *lora_radio, const char *name, void *user_data) {
  RT_ASSERT(lora_radio != RT_NULL);
  RT_ASSERT(lora_radio->ops != RT_NULL);
  RT_ASSERT(lora_radio->device != RT_NULL);

  for (uint32_t i = 0; i < sizeof(*lora_radio->ops) / sizeof(lora_radio->ops->init); i++) {
    uint32_t *ops = (uint32_t *)lora_radio->ops;
    RT_ASSERT(&ops[i] != RT_NULL);
  }

  struct rt_device *parent = &(lora_radio->parent);

  parent->type        = RT_Device_Class_PHY;
  parent->rx_indicate = RT_NULL;
  parent->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
  parent->ops = &lora_radio_ops;
#else
  parent->init    = lora_radio_init;
  parent->open    = RT_NULL;
  parent->close   = RT_NULL;
  parent->read    = lora_radio_read;
  parent->write   = lora_radio_write;
  parent->control = RT_NULL;
#endif

  parent->user_data = user_data;

  rt_err_t err = RT_EOK;
  if ((err = lora_radio_spi_init(lora_radio->device->spi_bus,
                                 name,
                                 lora_radio->device->spi_cs))
      != RT_EOK) {
    return err;
  }

  struct rt_spi_device *spi_device = (struct rt_spi_device *)rt_device_find(name);
  if (!spi_device) {
    LOG_E("spi sample run failed! cant't find %s device!", name);
    return -RT_EEMPTY;
  }

  lora_radio->spi_device = spi_device;

  char prefix[] = "lora-radio.";

  lora_radio->lora_radio_name = (char *)rt_malloc(sizeof(prefix) + rt_strlen(name));
  if (lora_radio->lora_radio_name == RT_NULL) {
    LOG_E("OOM lora_radio_name");
    return -RT_ENOMEM;
  }

  rt_strncpy(lora_radio->lora_radio_name, prefix, RT_NAME_MAX);
  rt_strncpy(lora_radio->lora_radio_name + rt_strlen(prefix), name, RT_NAME_MAX);

  return rt_device_register(parent, lora_radio->lora_radio_name, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
}
