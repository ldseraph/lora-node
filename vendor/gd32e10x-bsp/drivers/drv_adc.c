/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-01-03     iysheng           first version
 */

#include <board.h>
#include <drv_comm.h>
#include <drv_gpio.h>
// #include <drivers/adc.h>

#define DBG_TAG "drv.adc"
#define DBG_LVL DBG_INFO

#include <rtdbg.h>

#ifdef RT_USING_ADC

#define MAX_EXTERN_ADC_CHANNEL 16

typedef struct {
  struct rt_adc_device adc_dev;
  char                 name[8];
  rt_base_t            adc_pins[16];
  void *               private_data;
} gd32_adc_device;

static gd32_adc_device g_gd32_devs[] = {
#ifdef BSP_USING_ADC0
  {
    {},
    "adc0",
    {
      PIN_NUM(0, 0),
      PIN_NUM(0, 1),
      PIN_NUM(0, 2),
      PIN_NUM(0, 3),
      PIN_NUM(0, 4),
      PIN_NUM(0, 5),
      PIN_NUM(0, 6),
      PIN_NUM(0, 7),
      PIN_NUM(1, 0),
      PIN_NUM(1, 1),
      PIN_NUM(2, 0),
      PIN_NUM(2, 1),
      PIN_NUM(2, 2),
      PIN_NUM(2, 3),
      PIN_NUM(2, 4),
      PIN_NUM(2, 5),
    },
    ADC0,
  },
#endif

#ifdef BSP_USING_ADC1
  {
    {},
    "adc1",
    {
      GET_PIN(A, 0),
      GET_PIN(A, 1),
      GET_PIN(A, 2),
      GET_PIN(A, 3),
      GET_PIN(A, 4),
      GET_PIN(A, 5),
      GET_PIN(A, 6),
      GET_PIN(A, 7),
      GET_PIN(B, 0),
      GET_PIN(B, 1),
      GET_PIN(C, 0),
      GET_PIN(C, 1),
      GET_PIN(C, 2),
      GET_PIN(C, 3),
      GET_PIN(C, 4),
      GET_PIN(C, 5),
    },
    ADC1,
  },
#endif
};

/*
 * static void init_pin4adc
 *
 * 初始化指定的管腳爲 analog 模式
 * @ rt_uint32_t pin: pin information
 * return: N/A
 */
static void init_pin4adc(rt_base_t pin) {
  gpio_init(PIN_PORT(pin), GPIO_MODE_AIN, GPIO_OSPEED_MAX, PIN_NO(pin));
}

static rt_err_t gd32_adc_enabled(struct rt_adc_device *device, rt_uint32_t channel, rt_bool_t enabled) {
  gd32_adc_device *gd32_adc = (gd32_adc_device *)device;

  if (channel >= MAX_EXTERN_ADC_CHANNEL) {
    LOG_E("invalid channel");
    return -RT_EINVAL;
  }

  uint32_t ADCx = (uint32_t *)(device->parent.user_data);

  if (enabled == ENABLE) {
    init_pin4adc(gd32_adc->adc_pins[channel]);
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADCx, ADC_DATAALIGN_RIGHT);
    /* ADC channel length config */
    adc_channel_length_config(ADCx, ADC_REGULAR_CHANNEL, 1U);

    /* ADC trigger config */
    adc_external_trigger_source_config(ADCx, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_NONE);
    /* ADC external trigger config */
    adc_external_trigger_config(ADCx, ADC_REGULAR_CHANNEL, ENABLE);

    /* enable ADC interface */
    adc_enable(ADCx);
    rt_thread_delay(1U);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADCx);
  } else {
    adc_deinit(ADCx);
  }

  return 0;
}

static uint16_t adc_channel_sample(uint32_t ADCx, uint8_t channel) {
  /* ADC regular channel config */
  adc_regular_channel_config(ADCx, 0U, channel, ADC_SAMPLETIME_55POINT5);
  /* ADC software trigger enable */
  adc_software_trigger_enable(ADCx, ADC_REGULAR_CHANNEL);

  /* wait the end of conversion flag */
  while (!adc_flag_get(ADCx, ADC_FLAG_EOC))
    ;
  /* clear the end of conversion flag */
  adc_flag_clear(ADCx, ADC_FLAG_EOC);
  /* return regular channel sample value */
  return (adc_regular_data_read(ADCx));
}

static rt_err_t gd32_adc_convert(struct rt_adc_device *device, rt_uint32_t channel, rt_uint32_t *value) {
  if (!value) {
    LOG_E("invalid param");
    return -RT_EINVAL;
  }

  uint32_t ADCx = (uint32_t *)(device->parent.user_data);

  *value = adc_channel_sample(ADCx, channel);

  return 0;
}

static struct rt_adc_ops g_gd32_adc_ops = {
  gd32_adc_enabled,
  gd32_adc_convert,
};

static int rt_hw_adc_init(void) {
  int ret, i = 0;

#ifdef BSP_USING_ADC0
  rcu_periph_clock_enable(RCU_ADC0);
  rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);
#endif

#ifdef BSP_USING_ADC1
  rcu_periph_clock_enable(RCU_ADC1);
  rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);
#endif

  for (; i < ARRAY_SIZE(g_gd32_devs); i++) {
    ret = rt_hw_adc_register(&g_gd32_devs[i].adc_dev,
                             (const char *)g_gd32_devs[i].name,
                             &g_gd32_adc_ops,
                             g_gd32_devs[i].private_data);
    if (ret != RT_EOK) {
      /* TODO err handler */
      LOG_E("failed register %s, err=%d", g_gd32_devs[i].name, ret);
    }
  }

  return ret;
}
INIT_BOARD_EXPORT(rt_hw_adc_init);
#endif
