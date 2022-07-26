#include "sx126x.h"
#define DBG_SECTION_NAME "sx126x"
#include <ulog.h>

#define SX126X_RADIO_WAKEUP_TIME 3
#define SX126X_XTAL_FREQ 32000000UL
#define SX126X_PLL_STEP_SHIFT_AMOUNT (14)
#define SX126X_PLL_STEP_SCALED (SX126X_XTAL_FREQ >> (25 - SX126X_PLL_STEP_SHIFT_AMOUNT))
#define SX126X_MAX_LORA_SYMB_NUM_TIMEOUT 248

static rt_err_t sx126x_lora_radio_ops_wakeup(lora_radio_t *);

static inline rt_err_t sx126x_wait_on_busy(lora_radio_t *lora_radio) {
  sx126x_t *sx126x  = lora_radio->parent.user_data;
  uint32_t  timeout = 10;
  while (rt_pin_read(sx126x->gpio_busy) == PIN_HIGH) {
    timeout--;
    rt_thread_delay(1);
    if (!timeout) {
      LOG_E("sx126x_wait_on_busy timeout");
      return -RT_ETIMEOUT;
    }
  };
  return RT_EOK;
}

static rt_err_t sx126x_check_device_ready(lora_radio_t *lora_radio) {
  if ((lora_radio->mode == MODE_SLEEP) || (lora_radio->mode == MODE_RX_DC)) {
    return sx126x_lora_radio_ops_wakeup(lora_radio);
    // Switch is turned off when device is in sleep mode and turned on is all other modes
    // SX126xAntSwOn();
  }

  return sx126x_wait_on_busy(lora_radio);
}

static rt_err_t sx126x_lora_radio_write_command(
  lora_radio_t *          lora_radio,
  sx126x_radio_commands_t command,
  uint8_t *               buffer,
  uint16_t                size) {
  // LOG_I("%s %x", __FUNCTION__, command);
  // for (uint32_t i = 0; i < size; i++) {
  //   LOG_I("%x ", buffer[i]);
  // }

  rt_err_t err = sx126x_check_device_ready(lora_radio);
  if (err != RT_EOK) {
    LOG_E("%s sx126x_check_device_ready:%d err:%d",__FUNCTION__,command,err);
    return err;
  }

  err = rt_spi_send_then_send(lora_radio->spi_device, &command, 1, buffer, size);
  if (err != RT_EOK) {
    LOG_E("%s rt_spi_send_then_send:%d err:%d",__FUNCTION__,command,err);
    return err;
  }

  if (command != RADIO_SET_SLEEP) {
    err = sx126x_wait_on_busy(lora_radio);
    if (err != RT_EOK) {
      LOG_E("%s sx126x_wait_on_busy:%d err:%d",__FUNCTION__,command,err);
      return err;
    }
  }

  return err;
}

static rt_err_t sx126x_lora_radio_read_command(
  lora_radio_t *          lora_radio,
  sx126x_radio_commands_t command,
  uint8_t *               buffer,
  uint16_t                size) {
  rt_err_t err = sx126x_check_device_ready(lora_radio);
  if (err != RT_EOK) {
    LOG_E("%s sx126x_check_device_ready:%d err:%d",__FUNCTION__,command,err);
    return err;
  }

  uint8_t tmp[] = {
    [0] = command,
    [1] = 0,
  };

  err = rt_spi_send_then_recv(lora_radio->spi_device, tmp, sizeof(tmp), buffer, size);
  if (err != RT_EOK) {
    LOG_E("%s rt_spi_send_then_recv:%d err:%d",__FUNCTION__,command,err);
    return err;
  }

  if (command != RADIO_SET_SLEEP) {
    err = sx126x_wait_on_busy(lora_radio);
      if (err != RT_EOK) {
        LOG_E("%s sx126x_wait_on_busy:%d err:%d",__FUNCTION__,command,err);
        return err;
      }
  }

  return err;
}

static rt_err_t sx126x_write_buffer(
  lora_radio_t * lora_radio,
  uint8_t        offset,
  const uint8_t *buffer,
  rt_size_t      size) {
  rt_err_t err = sx126x_check_device_ready(lora_radio);
  if (err != RT_EOK) {
    LOG_E("%s sx126x_check_device_ready err:%d",__FUNCTION__,err);
    return err;
  }

  uint8_t msg[] = { RADIO_WRITE_BUFFER, offset };

  err = rt_spi_send_then_send(lora_radio->spi_device, msg, sizeof(msg), buffer, size);
  if (err != RT_EOK) {
    LOG_E("%s rt_spi_send_then_send err:%d",__FUNCTION__,err);
    return err;
  }

  return sx126x_wait_on_busy(lora_radio);
}

static rt_err_t sx126x_read_buffer(
  lora_radio_t *lora_radio,
  uint8_t       offset,
  uint8_t *     buffer,
  rt_size_t     size) {
  rt_err_t err = sx126x_check_device_ready(lora_radio);
  if (err != RT_EOK) {
    LOG_E("%s sx126x_check_device_ready err:%d",__FUNCTION__,err);
    return err;
  }

  uint8_t msg[] = { RADIO_READ_BUFFER, offset, 0 };

  err = rt_spi_send_then_recv(lora_radio->spi_device, msg, sizeof(msg), buffer, size);
  if (err != RT_EOK) {
    LOG_E("%s rt_spi_send_then_recv err:%d",__FUNCTION__,err);
    return err;
  }

  return sx126x_wait_on_busy(lora_radio);
}

static rt_err_t sx126x_read_registers(
  lora_radio_t *lora_radio,
  uint16_t      address,
  uint8_t *     buffer,
  uint16_t      size) {
  rt_err_t err = sx126x_check_device_ready(lora_radio);
  if (err != RT_EOK) {
    LOG_E("%s sx126x_check_device_ready:%d err:%d",__FUNCTION__,address,err);
    return err;
  }

  uint8_t msg[] = { RADIO_READ_REGISTER, (address & 0xFF00) >> 8, address & 0x00FF, 0 };

  err = rt_spi_send_then_recv(lora_radio->spi_device, msg, sizeof(msg), buffer, size);
  if (err != RT_EOK) {
    LOG_E("%s rt_spi_send_then_recv:%d err:%d",__FUNCTION__,address,err);
    return err;
  }

  return sx126x_wait_on_busy(lora_radio);
}

static rt_err_t sx126x_read_register(
  lora_radio_t *lora_radio,
  uint16_t      address,
  uint8_t *     data) {
  return sx126x_read_registers(lora_radio, address, data, 1);
}

static rt_err_t sx126x_write_registers(
  lora_radio_t *lora_radio,
  uint16_t      address,
  uint8_t *     buffer,
  uint16_t      size) {
  // LOG_I("%s %x", __FUNCTION__, address);
  // for (uint32_t i = 0; i < size; i++) {
  //   LOG_I("%x ", buffer[i]);
  // }

  rt_err_t err = sx126x_check_device_ready(lora_radio);
  if (err != RT_EOK) {
    LOG_E("%s sx126x_check_device_ready%d err:%d",__FUNCTION__,address,err);
    return err;
  }

  uint8_t msg[] = { RADIO_WRITE_REGISTER, (address & 0xFF00) >> 8, address & 0x00FF };

  err = rt_spi_send_then_send(lora_radio->spi_device, msg, sizeof(msg), buffer, size);
  if (err != RT_EOK) {
    LOG_E("%s rt_spi_send_then_send%d err:%d",__FUNCTION__,address,err);
    return err;
  }

  return sx126x_wait_on_busy(lora_radio);
}

static rt_err_t sx126x_write_register(
  lora_radio_t *lora_radio,
  uint16_t      address,
  uint8_t       value) {
  return sx126x_write_registers(lora_radio, address, &value, 1);
}

static rt_err_t sx126x_set_dio_irq_params(
  lora_radio_t *lora_radio,
  uint16_t      irqMask,
  uint16_t      dio1Mask,
  uint16_t      dio2Mask,
  uint16_t      dio3Mask) {
  uint8_t buf[] = {
    [0] = (uint8_t)((irqMask >> 8) & 0x00FF),
    [1] = (uint8_t)(irqMask & 0x00FF),
    [2] = (uint8_t)((dio1Mask >> 8) & 0x00FF),
    [3] = (uint8_t)(dio1Mask & 0x00FF),
    [4] = (uint8_t)((dio2Mask >> 8) & 0x00FF),
    [5] = (uint8_t)(dio2Mask & 0x00FF),
    [6] = (uint8_t)((dio3Mask >> 8) & 0x00FF),
    [7] = (uint8_t)(dio3Mask & 0x00FF),
  };
  return sx126x_lora_radio_write_command(lora_radio,
                                         RADIO_CFG_DIOIRQ,
                                         (uint8_t *)buf,
                                         sizeof(buf));
}

static rt_err_t sx126x_set_pa_config(
  lora_radio_t *lora_radio,
  uint8_t       paDutyCycle,
  uint8_t       hpMax,
  uint8_t       deviceSel,
  uint8_t       paLut) {
  uint8_t buf[] = { paDutyCycle, hpMax, deviceSel, paLut };
  return sx126x_lora_radio_write_command(lora_radio,
                                         RADIO_SET_PACONFIG,
                                         (uint8_t *)buf,
                                         sizeof(buf));
}

static rt_err_t sx126x_set_tx_params(
  lora_radio_t *      lora_radio,
  int8_t              power,
  sx126x_ramp_times_t rampTime) {
  sx126x_t *        sx126x    = lora_radio->parent.user_data;
  sx126x_deviceID_t device_id = sx126x->device->device_id;
  rt_err_t          err;

  if (device_id == SX1261) {
    uint8_t duty_cycle = 0x04;

    if (power == 15) {
      duty_cycle = 0x06;
    }

    err = sx126x_set_pa_config(lora_radio, duty_cycle, 0x00, 0x01, 0x01);
    if (err != RT_EOK) {
      return err;
    }

    if (power >= 14) {
      power = 14;
    } else if (power < -17) {
      power = -17;
    }

    err = sx126x_write_register(lora_radio, REG_OCP, 0x18);
    if (err != RT_EOK) {
      return err;
    }

  } else {
    // WORKAROUND - Better Resistance of the SX1262 Tx to Antenna Mismatch, see DS_SX1261-2_V1.2 datasheet chapter 15.2
    // RegTxClampConfig = @address 0x08D8
    uint8_t data = 0;

    err = sx126x_read_register(lora_radio, REG_TX_CLAMP_CONFIG, &data);
    if (err != RT_EOK) {
      return err;
    }

    data |= 0x1E;

    err = sx126x_write_register(lora_radio, REG_TX_CLAMP_CONFIG, data);
    if (err != RT_EOK) {
      return err;
    }

    err = sx126x_set_pa_config(lora_radio, 0x04, 0x07, 0x00, 0x01);
    if (err != RT_EOK) {
      return err;
    }

    if (power > 22) {
      power = 22;
    } else if (power < -9) {
      power = -9;
    }

    err = sx126x_write_register(lora_radio, REG_OCP, 0x38);
    if (err != RT_EOK) {
      return err;
    }
  }

  uint8_t buf[] = { power, rampTime };
  return sx126x_lora_radio_write_command(lora_radio,
                                         RADIO_SET_TXPARAMS,
                                         (uint8_t *)buf,
                                         sizeof(buf));
}

static rt_err_t sx126x_set_buffer_base_address(
  lora_radio_t *lora_radio,
  uint8_t       txBaseAddress,
  uint8_t       rxBaseAddress) {
  uint8_t buf[] = { txBaseAddress, rxBaseAddress };
  return sx126x_lora_radio_write_command(lora_radio,
                                         RADIO_SET_BUFFERBASEADDRESS,
                                         (uint8_t *)buf,
                                         sizeof(buf));
}

static rt_err_t sx126x_set_regulator_mode(lora_radio_t *lora_radio) {
  sx126x_t *               sx126x         = lora_radio->parent.user_data;
  sx126x_regulator_modes_t regulator_mode = sx126x->device->regulator_mode;

  return sx126x_lora_radio_write_command(lora_radio,
                                         RADIO_SET_REGULATORMODE,
                                         (uint8_t *)&regulator_mode,
                                         sizeof(sx126x_regulator_modes_t));
}

static rt_err_t sx126x_set_io_tcxo(lora_radio_t *lora_radio) {
  sx126x_t *                 sx126x     = lora_radio->parent.user_data;
  sx126x_tcxo_ctrl_voltage_t tcxo_ctrl  = sx126x->device->tcxo_ctrl;
  uint32_t                   tcxo_delay = sx126x->device->tcxo_delay;

  uint8_t buf[4] = {
    tcxo_ctrl & 0x07,
    (tcxo_delay >> 16) & 0xFF,
    (tcxo_delay >> 8) & 0xFF,
    tcxo_delay & 0xFF,
  };

  rt_err_t err = sx126x_lora_radio_write_command(
    lora_radio,
    RADIO_SET_TCXOMODE,
    (uint8_t *)buf,
    sizeof(buf));
  if (err != RT_EOK) {
    return err;
  }

  uint8_t calibParam = 0x7f;
  return sx126x_lora_radio_write_command(
    lora_radio,
    RADIO_CALIBRATE,
    (uint8_t *)&calibParam,
    sizeof(calibParam));
}

static rt_err_t sx126x_set_rf_switch(lora_radio_t *lora_radio) {
  sx126x_t *sx126x            = lora_radio->parent.user_data;
  uint8_t   dio2_as_rf_switch = sx126x->device->dio2_as_rf_switch;

  return sx126x_lora_radio_write_command(
    lora_radio,
    RADIO_SET_RFSWITCHMODE,
    (uint8_t *)&dio2_as_rf_switch,
    sizeof(dio2_as_rf_switch));
}

static rt_err_t sx126x_lora_radio_ops_standby(lora_radio_t *lora_radio) {
  sx126x_t *sx126x = lora_radio->parent.user_data;

  sx126x_standby_modes_t standby_mode = sx126x->device->standby_mode;

  rt_err_t err = sx126x_lora_radio_write_command(
    lora_radio,
    RADIO_SET_STANDBY,
    &standby_mode,
    sizeof(sx126x_standby_modes_t));
  if (err != RT_EOK) {
    return err;
  }

  lora_radio->mode = MODE_STDBY;

  return RT_EOK;
}

static rt_err_t sx126x_lora_radio_ops_sleep(
  lora_radio_t *           lora_radio,
  lora_radio_sleep_args_t *args) {
  // SX126xAntSwOff( );

  uint8_t value = (args->warm_start << 2) | (args->reset << 1) | (args->wakeup_rtc);

  rt_err_t err = sx126x_lora_radio_write_command(
    lora_radio,
    RADIO_SET_SLEEP,
    &value,
    sizeof(value));
  if (err != RT_EOK) {
    return err;
  }

  lora_radio->mode = MODE_SLEEP;

  LOG_I("lora radio sleep");
  return RT_EOK;
}

static rt_err_t sx126x_set_rx(lora_radio_t *lora_radio, uint32_t timeout) {
  rt_err_t err = sx126x_write_register(lora_radio, REG_RX_GAIN, 0x94);
  if (err != RT_EOK) {
    return err;
  }

  uint8_t buf[3] = {
    [0] = (uint8_t)((timeout >> 16) & 0xFF),
    [1] = (uint8_t)((timeout >> 8) & 0xFF),
    [2] = (uint8_t)(timeout & 0xFF),
  };

  err = sx126x_lora_radio_write_command(lora_radio,
                                        RADIO_SET_RX,
                                        (uint8_t *)buf,
                                        sizeof(buf));
  if (err != RT_EOK) {
    return err;
  }

  lora_radio->mode = MODE_RX;

  return RT_EOK;
}

static rt_err_t sx126x_set_tx(lora_radio_t *lora_radio, uint32_t timeout) {
  uint8_t buf[3] = {
    [0] = (uint8_t)((timeout >> 16) & 0xFF),
    [1] = (uint8_t)((timeout >> 8) & 0xFF),
    [2] = (uint8_t)(timeout & 0xFF),
  };

  rt_err_t err = sx126x_lora_radio_write_command(lora_radio,
                                                 RADIO_SET_TX,
                                                 (uint8_t *)buf,
                                                 sizeof(buf));
  if (err != RT_EOK) {
    return err;
  }

  lora_radio->mode = MODE_TX;

  return RT_EOK;
}

static uint32_t sx126x_lora_radio_ops_random(lora_radio_t *lora_radio) {
  uint8_t  reg_ana_lna;
  rt_err_t err = sx126x_read_register(lora_radio, REG_ANA_LNA, &reg_ana_lna);
  if (err != RT_EOK) {
    return 0;
  }

  err = sx126x_write_register(lora_radio, REG_ANA_LNA, reg_ana_lna & ~(1 << 0));
  if (err != RT_EOK) {
    return 0;
  }

  uint8_t reg_ana_mixer;
  err = sx126x_read_register(lora_radio, REG_ANA_MIXER, &reg_ana_mixer);
  if (err != RT_EOK) {
    return 0;
  }

  err = sx126x_write_register(lora_radio, REG_ANA_MIXER, reg_ana_mixer & ~(1 << 7));
  if (err != RT_EOK) {
    return 0;
  }

  err = sx126x_set_rx(lora_radio, 0xFFFFFF);
  if (err != RT_EOK) {
    return 0;
  }

  uint32_t number = 0;

  err = sx126x_read_registers(lora_radio, RANDOM_NUMBER_GENERATORBASEADDR, (uint8_t *)&number, sizeof(number));
  if (err != RT_EOK) {
    return 0;
  }

  err = sx126x_lora_radio_ops_standby(lora_radio);
  if (err != RT_EOK) {
    return 0;
  }

  err = sx126x_write_register(lora_radio, REG_ANA_LNA, reg_ana_lna);
  if (err != RT_EOK) {
    return 0;
  }

  err = sx126x_write_register(lora_radio, REG_ANA_MIXER, reg_ana_mixer);
  if (err != RT_EOK) {
    return 0;
  }

  return number;
}

static rt_err_t sx126x_lora_radio_ops_wakeup(lora_radio_t *lora_radio) {
  uint8_t msg[] = { RADIO_GET_STATUS, 0x00 };

  rt_size_t size = rt_spi_send(lora_radio->spi_device, msg, sizeof(msg));
  if (size == 0) {
    LOG_E("sx126x_lora_radio_ops_wakeup rt_spi_send err");
    return -RT_ERROR;
  }

  rt_err_t err = sx126x_wait_on_busy(lora_radio);
  if (err != RT_EOK) {
    LOG_E("sx126x_lora_radio_ops_wakeup busy err %d",err);
    return err;
  }

  lora_radio->mode = MODE_STDBY;

  return RT_EOK;
}

static rt_err_t sx126x_lora_radio_ops_reset(lora_radio_t *lora_radio) {
  sx126x_t *sx126x = lora_radio->parent.user_data;

  rt_pin_mode(sx126x->gpio_reset, GPIO_MODE_OUT_PP);
  rt_pin_write(sx126x->gpio_reset, PIN_LOW);
  rt_thread_mdelay(20);

  rt_pin_mode(sx126x->gpio_reset, GPIO_MODE_AIN);
  rt_thread_mdelay(10);

  return RT_EOK;
}

static rt_err_t sx126x_calibrate_image(lora_radio_t *lora_radio, uint32_t freq) {
  uint8_t cal_freq[2];

  if (freq > 900000000) {
    cal_freq[0] = 0xE1;
    cal_freq[1] = 0xE9;
  } else if (freq > 850000000) {
    cal_freq[0] = 0xD7;
    cal_freq[1] = 0xDB;
  } else if (freq > 770000000) {
    cal_freq[0] = 0xC1;
    cal_freq[1] = 0xC5;
  } else if (freq > 460000000) {
    cal_freq[0] = 0x75;
    cal_freq[1] = 0x81;
  } else if (freq > 425000000) {
    cal_freq[0] = 0x6B;
    cal_freq[1] = 0x6F;
  }

  return sx126x_lora_radio_write_command(lora_radio,
                                         RADIO_CALIBRATEIMAGE,
                                         (uint8_t *)cal_freq,
                                         sizeof(cal_freq));
}

static uint32_t sx126x_convert_freq_in_hz_to_pll_step(uint32_t freq_in) {
  uint32_t steps_int  = freq_in / SX126X_PLL_STEP_SCALED;
  uint32_t steps_frac = freq_in - (steps_int * SX126X_PLL_STEP_SCALED);
  return (steps_int << SX126X_PLL_STEP_SHIFT_AMOUNT) + (((steps_frac << SX126X_PLL_STEP_SHIFT_AMOUNT) + (SX126X_PLL_STEP_SCALED >> 1)) / SX126X_PLL_STEP_SCALED);
}

static rt_err_t sx126x_lora_radio_ops_set_frequency(
  lora_radio_t *lora_radio,
  uint32_t      frequency) {
  sx126x_t *sx126x = lora_radio->parent.user_data;

  if (sx126x->image_calibrated == RT_FALSE) {
    rt_err_t err = sx126x_calibrate_image(lora_radio, frequency);
    if (err != RT_EOK) {
      return err;
    }
    sx126x->image_calibrated = RT_TRUE;
  }

  uint32_t freq_in_pll_steps = sx126x_convert_freq_in_hz_to_pll_step(frequency);

  uint8_t buf[4] = {
    [0] = (uint8_t)((freq_in_pll_steps >> 24) & 0xFF),
    [1] = (uint8_t)((freq_in_pll_steps >> 16) & 0xFF),
    [2] = (uint8_t)((freq_in_pll_steps >> 8) & 0xFF),
    [3] = (uint8_t)(freq_in_pll_steps & 0xFF),
  };

  rt_err_t err = sx126x_lora_radio_write_command(lora_radio, RADIO_SET_RFFREQUENCY, (uint8_t *)buf, sizeof(buf));
  if (err != RT_EOK) {
    return err;
  }

  return RT_EOK;
}

static rt_err_t sx126x_lora_radio_ops_set_sync_word(
  lora_radio_t *      lora_radio,
  lora_radio_modems_t modem,
  uint32_t            sync_word) {
  rt_err_t err = RT_EOK;

  switch (modem) {
  case LORD_RADIO_MODEM_LORA: {
    rt_err_t err = sx126x_write_register(lora_radio, REG_LR_SYNCWORD, (sync_word >> 8) & 0xFF);
    if (err != RT_EOK) {
      return err;
    }

    err = sx126x_write_register(lora_radio, REG_LR_SYNCWORD + 1, sync_word & 0xFF);
    if (err != RT_EOK) {
      return err;
    }

  } break;
  case LORD_RADIO_MODEM_FSK: {
    uint8_t syncword[] = LORD_RADIO_FSK_SYNCWORD;

    err = sx126x_write_registers(lora_radio, REG_LR_SYNCWORDBASEADDRESS, syncword, sizeof(syncword));
    if (err != RT_EOK) {
      return err;
    }

  } break;
  default:
    err = -RT_EINVAL;
    break;
  }

  return err;
}

typedef struct {
  uint32_t bandwidth;
  uint8_t  reg_value;
} sx126x_lora_radio_fsk_bandwidth_t;

const sx126x_lora_radio_fsk_bandwidth_t fsk_bandwidths[] = {
  { 4800, 0x1F },
  { 5800, 0x17 },
  { 7300, 0x0F },
  { 9700, 0x1E },
  { 11700, 0x16 },
  { 14600, 0x0E },
  { 19500, 0x1D },
  { 23400, 0x15 },
  { 29300, 0x0D },
  { 39000, 0x1C },
  { 46900, 0x14 },
  { 58600, 0x0C },
  { 78200, 0x1B },
  { 93800, 0x13 },
  { 117300, 0x0B },
  { 156200, 0x1A },
  { 187200, 0x12 },
  { 234300, 0x0A },
  { 312000, 0x19 },
  { 373600, 0x11 },
  { 467000, 0x09 },
  { 500000, 0x00 },
};

static uint8_t sx126x_lora_radio_fsk_bandwidth(uint32_t bandwidth) {
  if (bandwidth == 0) {
    return 0x1F;
  }

  for (uint8_t i = 0; i < (sizeof(fsk_bandwidths) / sizeof(sx126x_lora_radio_fsk_bandwidth_t)) - 1; i++) {
    if ((bandwidth >= fsk_bandwidths[i].bandwidth) && (bandwidth < fsk_bandwidths[i + 1].bandwidth)) {
      return fsk_bandwidths[i + 1].reg_value;
    }
  }

  return 0x1f;
}

static rt_err_t sx126x_lora_radio_ops_set_modulation_params(
  lora_radio_t *               lora_radio,
  lora_radio_tx_config_args_t *args) {
  rt_err_t            err   = RT_EOK;
  lora_radio_modems_t modem = lora_radio->modem;

  switch (modem) {
  case LORD_RADIO_MODEM_FSK: {
    uint32_t br        = (uint32_t)(32 * SX126X_XTAL_FREQ / args->modem_args.fsk.datarate);
    uint32_t bandwidth = sx126x_lora_radio_fsk_bandwidth(args->modem_args.fsk.bandwidth << 1);
    uint32_t fdev      = sx126x_convert_freq_in_hz_to_pll_step(args->modem_args.fsk.fdev);

    uint8_t buf[] = {
      [0] = (br >> 16) & 0xFF,
      [1] = (br >> 8) & 0xFF,
      [2] = br & 0xFF,
      [3] = MOD_SHAPING_G_BT_1,
      [4] = bandwidth,
      [5] = (fdev >> 16) & 0xFF,
      [6] = (fdev >> 8) & 0xFF,
      [7] = (fdev & 0xFF),
    };

    err = sx126x_lora_radio_write_command(lora_radio,
                                          RADIO_SET_MODULATIONPARAMS,
                                          (uint8_t *)buf,
                                          sizeof(buf));
  } break;
  case LORD_RADIO_MODEM_LORA: {
    rt_bool_t                      low_datarate_optimize = RT_FALSE;
    lora_radio_lora_bandwidths_t   bandwidth             = args->modem_args.lora.bandwidth;
    lora_radio_spreading_factors_t spreading_factors     = args->modem_args.lora.spreading_factors;

    if (((bandwidth == LORA_BW_125) && ((spreading_factors == LORA_SF11) || (spreading_factors == LORA_SF12)))
        || ((bandwidth == LORA_BW_250) && (spreading_factors == LORA_SF12))) {
      low_datarate_optimize = RT_TRUE;
    }

    uint8_t buf[] = {
      [0] = spreading_factors,
      [1] = bandwidth,
      [2] = args->modem_args.lora.coding_rate,
      [3] = low_datarate_optimize,
    };

    err = sx126x_lora_radio_write_command(lora_radio,
                                          RADIO_SET_MODULATIONPARAMS,
                                          (uint8_t *)buf,
                                          sizeof(buf));
  } break;
  default:
    err = -RT_EINVAL;
    break;
  }

  return err;
}

static rt_err_t sx126x_lora_radio_ops_set_packet_params(
  lora_radio_t *               lora_radio,
  lora_radio_tx_config_args_t *args) {
  rt_err_t            err   = RT_EOK;
  lora_radio_modems_t modem = lora_radio->modem;

  switch (modem) {
  case LORD_RADIO_MODEM_FSK: {
    uint32_t                        crc_length         = args->crc_on ? RADIO_CRC_2_BYTES_IBM : RADIO_CRC_OFF;
    lora_radio_packet_length_mode_t packet_length_mode = args->packet_length_mode;
    uint16_t                        preamble_length    = args->preamble_length;
    if (crc_length == RADIO_CRC_2_BYTES_IBM) {
      // TODO:
      // SX126xSetCrcSeed(CRC_IBM_SEED);
      // SX126xSetCrcPolynomial(CRC_POLYNOMIAL_IBM);
      crc_length = RADIO_CRC_2_BYTES;
    }

    uint8_t buf[] = {
      [0] = (preamble_length >> 8) & 0xFF,
      [1] = (preamble_length)&0xFF,
      [5] = !packet_length_mode,
      [7] = crc_length,
    };

    // buf[2] = packetParams->Params.Gfsk.PreambleMinDetect;
    // buf[3] = packetParams->Params.Gfsk.SyncWordLength;
    // buf[4] = packetParams->Params.Gfsk.AddrComp;
    // buf[6] = packetParams->Params.Gfsk.PayloadLength;
    // buf[8] = packetParams->Params.Gfsk.DcFree;
    err = sx126x_lora_radio_write_command(lora_radio,
                                          RADIO_SET_PACKETPARAMS,
                                          (uint8_t *)buf,
                                          sizeof(buf));
  } break;
  case LORD_RADIO_MODEM_LORA: {
    uint16_t                        preamble_length    = args->preamble_length;
    lora_radio_spreading_factors_t  spreading_factors  = args->modem_args.lora.spreading_factors;
    lora_radio_packet_length_mode_t packet_length_mode = args->packet_length_mode;

    if (((spreading_factors == LORA_SF5) || (spreading_factors == LORA_SF6)) && (preamble_length < 12)) {
      preamble_length = 12;
    }

    uint8_t buf[] = {
      [0] = (preamble_length >> 8) & 0xFF,
      [1] = (preamble_length)&0xFF,
      [2] = packet_length_mode,
      [3] = args->packet_length,
      [4] = args->crc_on,
      [5] = args->modem_args.lora.invert_iq,
    };
    err = sx126x_lora_radio_write_command(lora_radio,
                                          RADIO_SET_PACKETPARAMS,
                                          (uint8_t *)buf,
                                          sizeof(buf));
  } break;
  default:
    err = -RT_EINVAL;
    break;
  }

  return err;
}

static rt_err_t sx126x_lora_radio_ops_set_packet_type(
  lora_radio_t *            lora_radio,
  lora_radio_packet_types_t packet_type) {
  uint8_t buf[] = {
    [0] = (uint8_t)(packet_type & 0xFF),
  };

  rt_err_t err = sx126x_lora_radio_write_command(lora_radio,
                                                 RADIO_SET_PACKETTYPE,
                                                 (uint8_t *)buf,
                                                 sizeof(buf));
  if (err != RT_EOK) {
    return err;
  }

  return RT_EOK;
}

static rt_err_t sx126x_lora_radio_ops_after_tx_config(
  lora_radio_t *               lora_radio,
  lora_radio_tx_config_args_t *args) {
  rt_err_t err = RT_EOK;

  uint8_t data = 0;

  err = sx126x_read_register(lora_radio, 0x0889, &data);
  if (err != RT_EOK) {
    return err;
  }

  if ((args->modem == LORD_RADIO_MODEM_LORA) && (args->modem_args.lora.bandwidth == LORA_BW_500)) {
    data = data & 0xfb;
  } else {
    data = data | 0x04;
  }

  err = sx126x_write_register(lora_radio, 0x0889, data);
  if (err != RT_EOK) {
    return err;
  }

  return err;
}

static rt_err_t sx126x_lora_radio_ops_before_rx_config(
  lora_radio_t *               lora_radio,
  lora_radio_rx_config_args_t *args) {
  uint8_t buf[] = {
    [0] = RT_FALSE,
  };
  return sx126x_lora_radio_write_command(lora_radio,
                                         RADIO_SET_STOPRXTIMERONPREAMBLE,
                                         (uint8_t *)buf,
                                         sizeof(buf));
}

static rt_err_t sx126x_set_lora_symbol_timeout(
  lora_radio_t *lora_radio,
  uint32_t      symbol_timeout) {
  uint8_t mant = (((symbol_timeout > SX126X_MAX_LORA_SYMB_NUM_TIMEOUT) ? SX126X_MAX_LORA_SYMB_NUM_TIMEOUT : symbol_timeout) + 1) >> 1;

  uint8_t exp = 0;

  while (mant > 31) {
    mant = (mant + 3) >> 2;
    exp++;
  }

  uint8_t buf[] = {
    [0] = (mant << (2 * exp + 1)),
  };

  rt_err_t err = sx126x_lora_radio_write_command(lora_radio,
                                                 RADIO_SET_LORASYMBTIMEOUT,
                                                 (uint8_t *)buf,
                                                 sizeof(buf));
  if (err != RT_EOK) {
    return err;
  }

  if (symbol_timeout != 0) {
    uint8_t buf[] = {
      [0] = (exp + (mant << 3)),
    };
    err = sx126x_write_registers(lora_radio, REG_LR_SYNCH_TIMEOUT, buf, sizeof(buf));
    if (err != RT_EOK) {
      return err;
    }
  }

  return RT_EOK;
}

static rt_err_t sx126x_lora_radio_ops_after_rx_config(
  lora_radio_t *               lora_radio,
  lora_radio_rx_config_args_t *args) {
  rt_err_t  err    = RT_EOK;
  sx126x_t *sx126x = lora_radio->parent.user_data;

  switch (args->modem) {
  case LORD_RADIO_MODEM_FSK: {
    // SX126xSetSyncWord( ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
    // SX126xSetWhiteningSeed( 0x01FF );
    sx126x->rx_timeout = (uint32_t)args->symbol_timeout * 8000UL / args->modem_args.fsk.datarate;
  } break;
  case LORD_RADIO_MODEM_LORA: {
    err = sx126x_set_lora_symbol_timeout(lora_radio, args->symbol_timeout);
    if (err != RT_EOK) {
      return err;
    }

    uint8_t data = 0;

    err = sx126x_read_register(lora_radio, 0x0736, &data);
    if (err != RT_EOK) {
      return err;
    }

    if (args->modem_args.lora.invert_iq) {
      data = data & ~(1 << 2);
    } else {
      data = data | (1 << 2);
    }

    err = sx126x_write_register(lora_radio, 0x0736, data);
    if (err != RT_EOK) {
      return err;
    }

    sx126x->rx_timeout = 0xFFFF;

  } break;
  default:
    break;
  }

  return err;
}

static rt_err_t sx126x_lora_radio_ops_set_tx_power(lora_radio_t *lora_radio, int8_t power) {
  sx126x_t *sx126x = lora_radio->parent.user_data;
  return sx126x_set_tx_params(lora_radio, power, sx126x->ramp_times);
}

static void dio_irq(void *args) {
  lora_radio_t *lora_radio = (lora_radio_t *)args;
  lora_radio->dio_irq(lora_radio);
}

static rt_err_t sx126x_gpio_init(lora_radio_t *lora_radio) {
  sx126x_t *sx126x = lora_radio->parent.user_data;

  sx126x->gpio_busy = rt_pin_get(sx126x->device->gpio_busy);
  rt_pin_mode(sx126x->gpio_busy, GPIO_MODE_IN_FLOATING);

  sx126x->gpio_dio1 = rt_pin_get(sx126x->device->gpio_dio1);
  rt_pin_mode(sx126x->gpio_dio1, GPIO_MODE_IN_FLOATING);

  sx126x->gpio_reset = rt_pin_get(sx126x->device->gpio_reset);
  rt_pin_mode(sx126x->gpio_reset, GPIO_MODE_AIN);

  // #if defined( LORA_RADIO_DIO2_PIN )
  //     rt_pin_mode(LORA_RADIO_DIO2_PIN, PIN_MODE_INPUT_PULLDOWN);
  // #endif

  // #if defined( LORA_RADIO_RFSW1_PIN ) && defined ( LORA_RADIO_RFSW2_PIN )
  //     rt_pin_mode(LORA_RADIO_RFSW1_PIN, PIN_MODE_OUTPUT);
  //     rt_pin_mode(LORA_RADIO_RFSW2_PIN, PIN_MODE_OUTPUT);
  // #endif

  rt_err_t err = rt_pin_attach_irq(sx126x->gpio_dio1, PIN_IRQ_MODE_RISING, dio_irq, lora_radio);
  if (err != RT_EOK) {
    LOG_E("rt_pin_attach_irq Failed: %d", err);
    return err;
  }

  err = rt_pin_irq_enable(sx126x->gpio_dio1, PIN_IRQ_ENABLE);
  if (err != RT_EOK) {
    LOG_E("rt_pin_irq_enable Failed: %d", err);
    return err;
  }

  return RT_EOK;
}

static rt_err_t sx126x_lora_radio_ops_init(lora_radio_t *lora_radio) {
  rt_err_t err = sx126x_gpio_init(lora_radio);
  if (err != RT_EOK) {
    return err;
  }

  err = sx126x_lora_radio_ops_reset(lora_radio);
  if (err != RT_EOK) {
    return err;
  }

  err = sx126x_lora_radio_ops_wakeup(lora_radio);
  if (err != RT_EOK) {
    LOG_E("wakeup err:%d", err);
    return err;
  }

  err = sx126x_lora_radio_ops_standby(lora_radio);
  if (err != RT_EOK) {
    LOG_E("standby err:%d", err);
    return err;
  }

  err = sx126x_set_io_tcxo(lora_radio);
  if (err != RT_EOK) {
    LOG_E("set tcxo init err:%d", err);
    return err;
  }

  err = sx126x_set_rf_switch(lora_radio);
  if (err != RT_EOK) {
    LOG_E("set rf switch err:%d", err);
    return err;
  }

  err = sx126x_set_regulator_mode(lora_radio);
  if (err != RT_EOK) {
    LOG_E("regulator err:%d", err);
    return err;
  }

  err = sx126x_set_buffer_base_address(lora_radio, 0x00, 0x00);
  if (err != RT_EOK) {
    LOG_E("set base addr err:%d", err);
    return err;
  }

  err = sx126x_set_tx_params(lora_radio, 0, RADIO_RAMP_200_US);
  if (err != RT_EOK) {
    LOG_E("set tx params err:%d", err);
    return err;
  }

  err = sx126x_set_dio_irq_params(lora_radio, IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
  if (err != RT_EOK) {
    LOG_E("set dio irq params err:%d", err);
    return err;
  }

  // RadioAddRegisterToRetentionList( REG_RX_GAIN );
  // RadioAddRegisterToRetentionList( REG_TX_MODULATION );

  sx126x_t *sx126x     = lora_radio->parent.user_data;
  uint32_t  ramp_times = sx126x->device->ramp_times;

  if (ramp_times <= 10) {
    sx126x->ramp_times = RADIO_RAMP_10_US;
  } else if (ramp_times <= 20) {
    sx126x->ramp_times = RADIO_RAMP_20_US;
  } else if (ramp_times <= 40) {
    sx126x->ramp_times = RADIO_RAMP_40_US;
  } else if (ramp_times <= 80) {
    sx126x->ramp_times = RADIO_RAMP_80_US;
  } else if (ramp_times <= 200) {
    sx126x->ramp_times = RADIO_RAMP_200_US;
  } else if (ramp_times <= 800) {
    sx126x->ramp_times = RADIO_RAMP_800_US;
  } else if (ramp_times <= 1700) {
    sx126x->ramp_times = RADIO_RAMP_1700_US;
  } else if (ramp_times <= 3400) {
    sx126x->ramp_times = RADIO_RAMP_3400_US;
  } else {
    sx126x->ramp_times = RADIO_RAMP_40_US;
  }

  float antenna_gain = sx126x->device->antenna_gain;
  float max_eirp     = sx126x->device->max_eirp;

  if (antenna_gain != 0) {
    lora_radio->antenna_gain = antenna_gain;
  }

  if (max_eirp != 0) {
    lora_radio->max_eirp = max_eirp;
  }

  return err;
}

static rt_err_t sx126x_lora_radio_ops_send(
  lora_radio_t * lora_radio,
  rt_off_t       pos,
  const uint8_t *buf,
  rt_size_t      size) {
  rt_err_t err = sx126x_set_dio_irq_params(
    lora_radio,
    IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
    IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
    IRQ_RADIO_NONE,
    IRQ_RADIO_NONE);
  if (err != RT_EOK) {
    LOG_E("set dio irq params err:%d", err);
    return err;
  }

  err = sx126x_write_buffer(lora_radio, pos, buf, size);
  if (err != RT_EOK) {
    LOG_E("write buffer err:%d", err);
    return err;
  }

  err = sx126x_set_tx(lora_radio, 0);
  if (err != RT_EOK) {
    LOG_E("set tx err:%d", err);
    return err;
  }

  return RT_EOK;
}

static rt_err_t sx126x_lora_radio_ops_rx(lora_radio_t *lora_radio) {
  rt_err_t err = sx126x_set_dio_irq_params(
    lora_radio,
    IRQ_RADIO_ALL,
    IRQ_RADIO_ALL,
    IRQ_RADIO_NONE,
    IRQ_RADIO_NONE);
  if (err != RT_EOK) {
    LOG_E("rx set dio irq params err:%d", err);
    return err;
  }

  sx126x_t *sx126x     = lora_radio->parent.user_data;
  uint32_t  rx_timeout = 0xFFFFFF;

  if (lora_radio->rx_continuous == RT_FALSE) {
    rx_timeout = sx126x->rx_timeout << 6;
  }

  err = sx126x_set_rx(lora_radio, rx_timeout);
  if (err != RT_EOK) {
    return 0;
  }

  return RT_EOK;
}

static rt_err_t sx126x_lora_radio_ops_rx_done(lora_radio_t *lora_radio) {
  rt_err_t err = sx126x_write_register(lora_radio, 0x0902, 0x00);
  if (err != RT_EOK) {
    return err;
  }

  uint8_t data = 0;

  err = sx126x_read_register(lora_radio, 0x0944, &data);
  if (err != RT_EOK) {
    return err;
  }

  data |= (1 << 1);

  err = sx126x_write_register(lora_radio, 0x0944, data);
  if (err != RT_EOK) {
    return err;
  }

  return RT_EOK;
}

static rt_err_t sx126x_lora_radio_ops_read(
  lora_radio_t *lora_radio,
  uint8_t *     buffer,
  rt_size_t *   size,
  rt_size_t     max_size) {
  uint8_t status[2];

  rt_err_t err = sx126x_lora_radio_read_command(
    lora_radio,
    RADIO_GET_RXBUFFERSTATUS,
    (uint8_t *)status,
    sizeof(status));
  if (err != RT_EOK) {
    LOG_E("lora redio read command err");
    return err;
  }

  if (lora_radio->packet_type == LORD_RADIO_PACKET_TYPE_LORA
      && lora_radio->packet_length_mode == LORA_PACKET_FIXED_LENGTH) {
    uint8_t temp;
    err = sx126x_read_register(lora_radio, REG_LR_PAYLOADLENGTH, &temp);
    if (err != RT_EOK) {
      return err;
    }
    *size = temp;
  } else {
    *size = status[0];
  }

  if (*size > max_size) {
    return -RT_EINVAL;
  }

  uint8_t offset = status[1];

  err = sx126x_read_buffer(lora_radio, offset, buffer, *size);
  if (err != RT_EOK) {
    LOG_E("read buffer err:%d", err);
    return err;
  }

  uint8_t pack_status[3];
  err = sx126x_lora_radio_read_command(lora_radio,
                                       RADIO_GET_PACKETSTATUS,
                                       (uint8_t *)pack_status,
                                       sizeof(pack_status));
  if (err != RT_EOK) {
    return err;
  }

  switch (lora_radio->packet_type) {
  case LORD_RADIO_PACKET_TYPE_GFSK: {
    lora_radio->fsk_rx_status.rx_status = pack_status[0];
    lora_radio->fsk_rx_status.rssi_avg  = -pack_status[2] >> 1;
    lora_radio->fsk_rx_status.rssi_sync = -pack_status[1] >> 1;
  } break;
  case LORD_RADIO_PACKET_TYPE_LORA: {
    lora_radio->lora_rx_status.rssi        = -pack_status[0] >> 1;
    lora_radio->lora_rx_status.snr         = (((int8_t)pack_status[1]) + 2) >> 2;
    lora_radio->lora_rx_status.signal_rssi = -pack_status[2] >> 1;
    break;
  }
  default:
    return -RT_ERROR;
  }

  return RT_EOK;
}

static rt_err_t sx126x_lora_radio_ops_get_irq(lora_radio_t *lora_radio, uint32_t *irq) {
  uint8_t irq_status[2] = { 0 };

  rt_err_t err = sx126x_lora_radio_read_command(lora_radio,
                                                RADIO_GET_IRQSTATUS,
                                                (uint8_t *)irq_status,
                                                sizeof(irq_status));
  if (err != RT_EOK) {
    return err;
  }

  *irq = (irq_status[0] << 8) | irq_status[1];

  return RT_EOK;
}

static rt_err_t sx126x_lora_radio_ops_clean_irq(lora_radio_t *lora_radio, uint32_t irq) {
  uint8_t buf[] = {
    [0] = (uint8_t)(((uint16_t)irq >> 8) & 0x00FF),
    [1] = (uint8_t)((uint16_t)irq & 0x00FF),
  };

  return sx126x_lora_radio_write_command(lora_radio,
                                         RADIO_CLR_IRQSTATUS,
                                         (uint8_t *)buf,
                                         sizeof(buf));
}

static uint32_t sx126x_lora_radio_ops_wakeup_time(lora_radio_t *lora_radio) {
  sx126x_t *sx126x = lora_radio->parent.user_data;
  return sx126x->device->tcxo_wakeup_time + SX126X_RADIO_WAKEUP_TIME;
}

static const lora_radio_ops_t sx126x_lora_radio_ops = {
  .init                  = sx126x_lora_radio_ops_init,
  .reset                 = sx126x_lora_radio_ops_reset,
  .wakeup                = sx126x_lora_radio_ops_wakeup,
  .standby               = sx126x_lora_radio_ops_standby,
  .sleep                 = sx126x_lora_radio_ops_sleep,
  .random                = sx126x_lora_radio_ops_random,
  .wakeup_time           = sx126x_lora_radio_ops_wakeup_time,
  .set_frequency         = sx126x_lora_radio_ops_set_frequency,
  .set_packet_type       = sx126x_lora_radio_ops_set_packet_type,
  .set_modulation_params = sx126x_lora_radio_ops_set_modulation_params,
  .set_packet_params     = sx126x_lora_radio_ops_set_packet_params,
  .set_sync_word         = sx126x_lora_radio_ops_set_sync_word,
  .set_tx_power          = sx126x_lora_radio_ops_set_tx_power,
  .after_tx_config       = sx126x_lora_radio_ops_after_tx_config,
  .before_rx_config      = sx126x_lora_radio_ops_before_rx_config,
  .after_rx_config       = sx126x_lora_radio_ops_after_rx_config,
  .send                  = sx126x_lora_radio_ops_send,
  .read                  = sx126x_lora_radio_ops_read,
  .rx                    = sx126x_lora_radio_ops_rx,
  .rx_done               = sx126x_lora_radio_ops_rx_done,
  .get_irq               = sx126x_lora_radio_ops_get_irq,
  .clean_irq             = sx126x_lora_radio_ops_clean_irq,
};

static rt_err_t sx126x_device_probe(devices_t *devices) {
  RT_ASSERT(RT_NULL != devices);

  for (uint32_t i = 0; i < devices->num; i++) {
    device_t *d = &(devices->list[i]);

    sx126x_t *sx126x = rt_malloc(sizeof(sx126x_t));
    if (RT_NULL == sx126x) {
      LOG_E("OOM sx126x");
      return -RT_ENOMEM;
    }

    sx126x_device_t *sx126x_device = d->device;
    if (RT_NULL == sx126x_device) {
      LOG_E("OOM sx126x_device");
      return -RT_EINVAL;
    }

    sx126x->device           = sx126x_device;
    sx126x->image_calibrated = RT_FALSE;

    // lora radio init
    lora_radio_t *lora_radio = rt_malloc(sizeof(lora_radio_t));
    if (RT_NULL == lora_radio) {
      LOG_E("OOM lora_radio");
      return -RT_ENOMEM;
    }

    rt_memset(lora_radio, 0, sizeof(lora_radio_t));

    lora_radio_device_t *lora_radio_device = &(sx126x_device->lora_radio_device);

    lora_radio->device = lora_radio_device;
    lora_radio->ops    = &sx126x_lora_radio_ops;

    rt_err_t err = lora_radio_device_register(lora_radio, d->name, sx126x);
    if (err != RT_EOK) {
      return err;
    }
  }

  return RT_EOK;
}

// TODO: dts
static device_t init_devices[] = {
  {
    .name   = "sx126x0",
    .device = &(sx126x_device_t){
      .lora_radio_device = {
        .spi_bus = "spi-bus0",
        .spi_cs  = "PA.4",
      },
      .gpio_reset        = "PA.8",
      .gpio_busy         = "PA.9",
      .gpio_dio1         = "PA.10",
      .standby_mode      = STDBY_RC,
      .regulator_mode    = REGULATOR_DCDC,
      .device_id         = SX1268,
      .tcxo_ctrl         = TCXO_CTRL_1_7V,
      .tcxo_delay        = 0xa0,
      .dio2_as_rf_switch = RT_TRUE,
      .ramp_times        = 40,
      .tcxo_wakeup_time  = 0,
      .antenna_gain      = LORA_RADIO_DEFAULT_ANTENNA_GAIN,
      .max_eirp          = LORA_RADIO_DEFAULT_MAX_EIRP,
    },
  }
};

static int sx126x_device_register() {
  devices_t devices = {
    .list = init_devices,
    .num  = (sizeof(init_devices) / sizeof(init_devices[0])),
  };

  rt_err_t err = sx126x_device_probe(&devices);
  if (err != RT_EOK) {
    return err;
  }
  return RT_EOK;
}

INIT_DEVICE_EXPORT(sx126x_device_register);