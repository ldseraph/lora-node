#include "lorawan.h"
#define DBG_SECTION_NAME "lorawan"
#include <ulog.h>

#define LORAMAC_APP_TX_MQ_MAX_SIZE 1
#define LORAWAN_EVENT_THREAD_STACK_SIZE 2048
#define EV_LORAWAN_RX_WINDOW1 (1 << 1)
#define EV_LORAWAN_RX_WINDOW2 (1 << 2)
#define EV_LORAWAN_JOIN (1 << 3)
#define EV_LORAWAN_TX_APP (1 << 4)

typedef struct {
  lorawan_region_type_t type;
  lorawan_region_t*     region;
} region_map_t;

region_map_t region_map[] = { {
  .type   = LORAMAC_REGION_CN470,
  .region = &region_cn470,
} };

static rt_err_t lorawan_proc_tx_done(lorawan_t* lorawan) {
  rt_err_t err;

  LOG_I("tx done");
  if (lorawan->current_class != CLASS_C) {
    err = lora_radio_sleep(lorawan->lora_radio_device);
    if (err != RT_EOK) {
      LOG_E("sleep err(%d)", err);
      return RT_NULL;
    }
  }

  int32_t rx_window1_delay;
  int32_t rx_window2_delay;

  lorawan->region->compute_tx_window(lorawan);

  if (lorawan->joined) {
    rx_window1_delay = lorawan->receive_delay1;
    rx_window2_delay = lorawan->receive_delay2;
  } else {
    rx_window1_delay = lorawan->join_accept_delay1;
    rx_window2_delay = lorawan->join_accept_delay2;
  }

  rx_window1_delay += lorawan->window_offset;
  rx_window2_delay += lorawan->window_offset;

  if (rx_window1_delay <= 0 || rx_window2_delay <= 0) {
    LOG_E("window delay err:%d", rx_window1_delay);
    LOG_E("window offset:%d", lorawan->window_offset);
    return -RT_EINVAL;
  }

  uint32_t tx_window1_tick = rt_tick_from_millisecond(rx_window1_delay);
  uint32_t tx_window2_tick = rt_tick_from_millisecond(rx_window2_delay);

  err = rt_timer_control(
    lorawan->rx_window_timer1,
    RT_TIMER_CTRL_SET_TIME,
    &tx_window1_tick);
  if (err != RT_EOK) {
    LOG_E("tx done set rx window1 err: %d", err);
    return err;
  }

  err = rt_timer_start(lorawan->rx_window_timer1);
  if (err != RT_EOK) {
    LOG_E("tx done rx window1 start err: %d", err);
    return err;
  }

  err = rt_timer_control(
    lorawan->rx_window_timer2,
    RT_TIMER_CTRL_SET_TIME,
    &tx_window2_tick);
  if (err != RT_EOK) {
    LOG_E("tx done set rx window2 err: %d", err);
    return err;
  }

  err = rt_timer_start(lorawan->rx_window_timer2);
  if (err != RT_EOK) {
    LOG_E("tx done rx window2 start err: %d", err);
    return err;
  }

  //   if( ( Nvm.MacGroup2.DeviceClass == CLASS_C ) || ( MacCtx.NodeAckRequested
  //   == true ) )
  //   {
  //       getPhy.Attribute = PHY_ACK_TIMEOUT;
  //       phyParam = RegionGetPhyParam( Nvm.MacGroup2.Region, &getPhy );
  //       TimerSetValue( &MacCtx.AckTimeoutTimer, MacCtx.RxWindow2Delay +
  //       phyParam.Value ); TimerStart( &MacCtx.AckTimeoutTimer );
  //   }

  lorawan->tx_done_tick = rt_tick_get();
  // TODO: cycle update time credits
  // LOG_I("lorawan->joined %d",lorawan->joined);
  if (lorawan->joined) {
    lorawan->fcnt_list.fcnt_up++;
  }

  return RT_EOK;
}

static void lorawan_update_rx_slot(lorawan_t* lorawan) {
  if (lorawan->current_class != CLASS_C) {
    lorawan->rx_slot = RX_SLOT_NONE;
  } else {
    lorawan->rx_slot = RX_SLOT_WIN_CLASS_C;
  }
}

static rt_err_t lorawan_proc_rx_done(lorawan_t* lorawan, lorawan_rx_msg_t* msg) {
  rt_size_t size   = msg->phy_msg.size;
  uint8_t*  buffer = msg->phy_msg.buffer;

  rt_err_t err = lora_radio_sleep(lorawan->lora_radio_device);
  if (err != RT_EOK) {
    LOG_E("sleep err(%d)", err);
    return err;
  }

  lorawan_ftype_t ftype = lorawan_serializer_get_ftype(buffer[0]);
  switch (ftype) {
  case LORAWAN_FTYPE_JOIN_ACCEPT: {
    if (size < LORAMAC_JOIN_ACCEPT_FRAME_MIN_SIZE) {
      LOG_E("size(%d) < LORAMAC_JOIN_ACCEPT_FRAME_MIN_SIZE(%d)", size, LORAMAC_JOIN_ACCEPT_FRAME_MIN_SIZE);
      err = -RT_ERROR;
      goto err_handle;
    }

    err = lorawan_serializer_join_accept(lorawan, buffer, size);
    if (err != RT_EOK) {
      goto err_handle;
    }

    lorawan->joined     = RT_TRUE;
    lorawan->first_boot = RT_FALSE;

    LOG_I("joined");
  } break;
  case LORAWAN_FTYPE_DATA_CONFIRMED_DOWN:
  case LORAWAN_FTYPE_DATA_UNCONFIRMED_DOWN: {
    uint32_t max_phy_payload = lorawan->region->get_max_phy_payload(lorawan);

    if (((int16_t)((int16_t)size - (int16_t)LORAMAC_FRAME_PAYLOAD_OVERHEAD_SIZE) > (int16_t)max_phy_payload)
        || (size < LORAMAC_FRAME_PAYLOAD_MIN_SIZE)) {
      LOG_E("rx done fype(%d) size(%d) err", ftype, size);
      err = -RT_EINVAL;
      goto err_handle;
    }

    err = lorawan_deserialize_data(lorawan, msg);
    if (err != RT_EOK) {
      goto err_handle;
    }

    if (lorawan->joined) {
      lorawan->confirmed_count = 0;
    }

    if (msg->fport != 0) {
      // sub fport size
      msg->mac_payload.size -= 1;
    }

  } break;
  case LORAWAN_FTYPE_TYPE_PROPRIETARY: {
    LOG_W("not implemented");
  } break;
  default:
    break;
  }

  lorawan_update_rx_slot(lorawan);
  return RT_EOK;
err_handle:
  // PrepareRxDoneAbort
  lorawan_update_rx_slot(lorawan);
  return err;
}

static void lorawan_event_thread(void* param) {
  lorawan_t*         lorawan = (lorawan_t*)param;
  lora_radio_event_t ev;
  rt_err_t           err;

  for (;;) {
    ev  = LORA_RADIO_EVENT_NONE;
    err = lora_radio_event_recv(lorawan->lora_radio_device, &ev);
    if (err != RT_EOK) {
      LOG_E("lora radio event recv err %d", err);
      continue;
    }

    if ((ev & LORA_RADIO_EVENT_TX_DONE) == LORA_RADIO_EVENT_TX_DONE) {
      LOG_I("lora radio event tx done");
      err = lorawan_proc_tx_done(lorawan);
      if (err != RT_EOK) {
        LOG_E("process tx done err(%d).", err);
      }
    }

    if ((ev & LORA_RADIO_EVENT_TX_TIMEOUT) == LORA_RADIO_EVENT_TX_TIMEOUT) {
      LOG_I("lora radio event tx timeout");
      if (lorawan->current_class != CLASS_C) {
        err = lora_radio_sleep(lorawan->lora_radio_device);
        if (err != RT_EOK) {
          LOG_E("sleep err(%d)", err);
          continue;
        }
      }
      lorawan_update_rx_slot(lorawan);
    }

    if ((ev & LORA_RADIO_EVENT_RX_DONE) == LORA_RADIO_EVENT_RX_DONE) {
      LOG_I("lora radio event rx done");
      lorawan->rx_done_tick = rt_tick_get();

      lorawan_rx_msg_t* rx_msg = rt_malloc(sizeof(lorawan_rx_msg_t));
      if (RT_NULL == rx_msg) {
        LOG_E("OOM: lora radio rx done");
        continue;
      }

      rx_msg->phy_msg.size = rt_device_read(
        lorawan->lora_radio_device,
        0,
        rx_msg->phy_msg.buffer,
        LORAMAC_PHY_MAXPAYLOAD);

      if (rx_msg->phy_msg.size == 0) {
        LOG_E("lora radio read err.");
        rt_free(rx_msg);
        continue;
      }

      err = lorawan_proc_rx_done(lorawan, rx_msg);
      rt_free(rx_msg);
      if (err != RT_EOK) {
        LOG_E("proc rx done err(%d).", err);
        // return;
      }

      lorawan_update_rx_slot(lorawan);
    }

    if (((ev & LORA_RADIO_EVENT_RX_TIMEOUT) == LORA_RADIO_EVENT_RX_TIMEOUT)
        || ((ev & LORA_RADIO_EVENT_RX_ERROR) == LORA_RADIO_EVENT_RX_ERROR)) {
      LOG_I("lora radio event rx timeout or error");
      // TODO: config dummy
      if (!lorawan->joined) {
        LOG_E("joined:%d rx window%d timeout", lorawan->joined, lorawan->rx_slot + 1);
      }
      if (lorawan->current_class != CLASS_C) {
        err = lora_radio_sleep(lorawan->lora_radio_device);
        if (err != RT_EOK) {
          LOG_E("sleep err(%d)", err);
          continue;
        }
      }
      lorawan_update_rx_slot(lorawan);
    }

    if ((ev & LORA_RADIO_EVENT_RX_HEAD_DETECTION) == LORA_RADIO_EVENT_RX_HEAD_DETECTION) {
      LOG_I("lora radio event rx head detection");
      if (lorawan->rx_slot == RX_SLOT_WIN_1) {
        rt_timer_stop(lorawan->rx_window_timer2);
      }
    }

    if ((ev & LORA_RADIO_EVENT_RADIO_TIMEOUT) == LORA_RADIO_EVENT_RADIO_TIMEOUT) {
      LOG_E("lora radio event rx radio timeout: panic");
      if (lorawan->exception_handle) {
        lorawan->exception_handle();
      }
    }
  }

  return;
}

static void lorwan_app_tx(void* param) {
  lorawan_t* lorawan = (lorawan_t*)param;
  if (lorawan->confirmed_count > 10) {
    lorawan->joined          = RT_FALSE;
    lorawan->confirmed_count = 0;
  }

  uint32_t tx_duty_cycle = lorawan->tx_duty_cycle;
  if (lorawan->joined) {
    LOG_I("tx fcont:%d", lorawan->fcnt_list.fcnt_up);
    rt_event_send(&lorawan->irq_event, EV_LORAWAN_TX_APP);
  } else {
    LOG_I("join");
    rt_event_send(&lorawan->irq_event, EV_LORAWAN_JOIN);
    if (!lorawan->first_boot) {
      LOG_E("rejoin");
      tx_duty_cycle *= 3;
    }
  }

  int      random_cycle     = lorawan_rand(-2000, 2000);
  uint32_t tx_duty_cycle_ms = rt_tick_from_millisecond(tx_duty_cycle + random_cycle);

  rt_err_t err = rt_timer_control(
    lorawan->app_tx_timer,
    RT_TIMER_CTRL_SET_TIME,
    &tx_duty_cycle_ms);
  if (err != RT_EOK) {
    LOG_E("app tx control err: %d", err);
    return;
  }

  err = rt_timer_start(lorawan->app_tx_timer);
  if (err != RT_EOK) {
    LOG_E("app tx start err: %d", err);
    return;
  }
}

static void lorawan_rx_window1(void* param) {
  lorawan_t* lorawan = (lorawan_t*)param;
  rt_interrupt_enter();
  rt_event_send(&lorawan->irq_event, EV_LORAWAN_RX_WINDOW1);
  rt_interrupt_leave();
}

static void lorawan_rx_window2(void* param) {
  lorawan_t* lorawan = (lorawan_t*)param;
  rt_interrupt_enter();
  rt_event_send(&lorawan->irq_event, EV_LORAWAN_RX_WINDOW2);
  rt_interrupt_leave();
}

static void lorawan_tx_delay(void* param) {
  // TODO tx delay
  rt_interrupt_enter();
  rt_interrupt_leave();
}

static void lorawan_ack_timer_done(void* param) {
  rt_interrupt_enter();
  rt_interrupt_leave();
}

lorawan_t* lorawan_new(
  rt_device_t           lora_radio_device,
  lorawan_region_type_t region_type) {
  lorawan_t* lorawan = rt_malloc(sizeof(lorawan_t));
  if (RT_NULL == lorawan) {
    LOG_E("OOM lorawan");
    return RT_NULL;
  }

  rt_memset(lorawan, 0, sizeof(lorawan_t));

  lorawan->default_class     = CLASS_A;
  lorawan->current_class     = CLASS_A;
  lorawan->activation_type   = ACTIVATION_TYPE_OTAA;
  lorawan->lora_radio_device = lora_radio_device;
  lorawan->version.value     = LORAMAC_VERSION;
  lorawan->joined            = RT_FALSE;
  lorawan->rx_slot           = RX_SLOT_NONE;
  lorawan->system_max_rx_err = 20;
  lorawan->min_rx_symbols    = 14;
  lorawan->tx_duty_cycle     = LORAWAN_APP_DEFAULT_TX_DUTYCYCLE;
  lorawan->adr_on            = RT_TRUE;
  lorawan->ack_requested     = RT_FALSE;
  lorawan->confirmed_count   = 0;
  lorawan->first_boot        = RT_TRUE;
  lorawan->exception_handle  = RT_NULL;

  rt_bool_t find = RT_FALSE;
  for (uint32_t i = 0; i < sizeof(region_map); i++) {
    if (region_map[i].type == region_type) {
      lorawan->region = region_map[i].region;

      rt_err_t err = lorawan->region->init(lorawan);
      if (err != RT_EOK) {
        LOG_E("region(%d) init err:%d", region_map[i].type, err);
        return RT_NULL;
      }
      find = RT_TRUE;
      break;
    }
  }

  if (find != RT_TRUE) {
    LOG_E("no find region(%d).", region_type);
    return RT_NULL;
  }

  uint8_t join_eui[] = LORAWAN_DEFAULT_JOIN_EUI;
  rt_memcpy(lorawan->secure_element.join_eui, join_eui, sizeof(join_eui));

  uint8_t dev_eui[] = LORAWAN_DEFAULT_DEVICE_EUI;
  rt_memcpy(lorawan->secure_element.dev_eui, dev_eui, sizeof(dev_eui));

  uint8_t app_key[] = LORAWAN_DEFAULT_APP_KEY;
  rt_memcpy(lorawan->secure_element.app_key, app_key, sizeof(app_key));

  uint32_t seed;
  rt_err_t err = lora_radio_random(lorawan->lora_radio_device, &seed);
  if (err != RT_EOK) {
    LOG_E("lorawan init seed err: %d.", err);
    return RT_NULL;
  }

  lorawan_srand(seed);

  lorawan->event_thread = rt_thread_create("lorawan",
                                           lorawan_event_thread,
                                           lorawan,
                                           LORAWAN_EVENT_THREAD_STACK_SIZE,
                                           5,
                                           20);
  if (lorawan->event_thread == RT_NULL) {
    LOG_E("rt_thread_init err: %d", err);
    return RT_NULL;
  }

  err = rt_thread_startup(lorawan->event_thread);
  if (err != RT_EOK) {
    LOG_E("rt_thread_startup err: %d", err);
    return RT_NULL;
  }

  // TX done

  // radio public
  // Radio.SetPublicNetwork( Nvm.MacGroup2.PublicNetwork );

  // reset mac param

  lorawan->rx_window_timer1 = rt_timer_create("lorawan:rx-timer1",
                                              lorawan_rx_window1,
                                              lorawan,
                                              RT_TICK_MAX,
                                              RT_TIMER_FLAG_ONE_SHOT);
  if (lorawan->rx_window_timer1 == RT_NULL) {
    return RT_NULL;
  }

  lorawan->rx_window_timer2 = rt_timer_create("lorawan:rx-timer2",
                                              lorawan_rx_window2,
                                              lorawan,
                                              RT_TICK_MAX,
                                              RT_TIMER_FLAG_ONE_SHOT);
  if (lorawan->rx_window_timer2 == RT_NULL) {
    return RT_NULL;
  }

  lorawan->tx_delay_timer = rt_timer_create("lorawan:tx-delay",
                                            lorawan_tx_delay,
                                            lorawan,
                                            RT_TICK_MAX,
                                            RT_TIMER_FLAG_ONE_SHOT);
  if (lorawan->tx_delay_timer == RT_NULL) {
    return RT_NULL;
  }

  lorawan->ack_timer = rt_timer_create("lorawan:ack-timer",
                                       lorawan_ack_timer_done,
                                       lorawan,
                                       RT_TICK_MAX,
                                       RT_TIMER_FLAG_ONE_SHOT);
  if (lorawan->ack_timer == RT_NULL) {
    return RT_NULL;
  }

  lorawan->app_tx_timer = rt_timer_create("lorawan:app-tx",
                                          lorwan_app_tx,
                                          lorawan,
                                          100,
                                          RT_TIMER_FLAG_ONE_SHOT);
  if (lorawan->app_tx_timer == RT_NULL) {
    return RT_NULL;
  }

  err = rt_event_init(&lorawan->irq_event,
                      "lorawan:irq-event",
                      RT_IPC_FLAG_FIFO);  // RT_IPC_FLAG_PRIO
  if (err != RT_EOK) {
    LOG_E("rt_event_init err: %d", err);
    return RT_NULL;
  }

  lorawan->tx_mb = rt_mb_create(
    "lorawan:app-tx",
    LORAMAC_APP_TX_MQ_MAX_SIZE,
    RT_IPC_FLAG_FIFO);
  if (lorawan->tx_mb == RT_NULL) {
    return RT_NULL;
  }

  lorawan->fcnt_list.fcnt_up     = 0;
  lorawan->fcnt_list.fcnt_down   = LORAMAC_FCNT_DOWN_INITAL_VALUE;
  lorawan->fcnt_list.n_fcnt_down = LORAMAC_FCNT_DOWN_INITAL_VALUE;
  lorawan->fcnt_list.a_fcnt_down = LORAMAC_FCNT_DOWN_INITAL_VALUE;

  for (int32_t i = 0; i < LORAMAC_MAX_MC_CTX; i++) {
    lorawan->fcnt_list.mc_fcnt_down[i] = LORAMAC_FCNT_DOWN_INITAL_VALUE;
  }

  // CryptoNvm->LastDownFCnt = CryptoNvm->FCntList.FCntDown;

  // secure init

  // mac command init
  // Set multicast downlink counter

  err = lora_radio_sleep(lorawan->lora_radio_device);
  if (err != RT_EOK) {
    LOG_E("init sleep err(%d)", err);
    return RT_NULL;
  }

  // classB init

  return lorawan;
}

#if 0
static lorawan_error_t lorawan_switch_class(lorawan_t* lorawan, lorawan_class_t class) {
  switch (lorawan->current_class) {
  case CLASS_A:
    switch (class) {
    case CLASS_A:
      //  Nvm.MacGroup2.MacParams.RxCChannel = Nvm.MacGroup2.MacParams.Rx2Channel;
      break;
    case CLASS_B:
      LOG_E("not yet implemented!");
      goto ERR_PARAMETER_INVALID;
      break;
    case CLASS_C:
      LOG_E("not yet implemented!");
      goto ERR_PARAMETER_INVALID;
      break;
    default:
      LOG_E("switch class(%d) err.", class);
      goto ERR_PARAMETER_INVALID;
    }
    break;
  case CLASS_B:
    LOG_E("not yet implemented!");
    goto ERR_PARAMETER_INVALID;
    break;
  case CLASS_C:
    LOG_E("not yet implemented!");
    goto ERR_PARAMETER_INVALID;
    break;
  default:
    goto ERR_PARAMETER_INVALID;
  }
  return LORAMAC_STATUS_OK;

ERR_PARAMETER_INVALID:
  return -LORAMAC_STATUS_PARAMETER_INVALID;
}

#endif

static rt_base_t lorawan_check_dr_range(
  int8_t value,
  int8_t min,
  int8_t max) {
  if ((value >= max) && (value <= min)) {
    return RT_TRUE;
  }
  return RT_FALSE;
}

static rt_err_t lorawan_next_channel(
  lorawan_t* lorawan,
  uint32_t*  channel,
  rt_time_t* delay) {
  const lorawan_region_t* region = lorawan->region;
  // TODO:update Band Time Off
  // get delay

  uint8_t enable_channel_count = 0;
  uint8_t enabled_channels[region->channels_nb];

  for (uint8_t i = 0, k = 0; i < region->channels_nb; i += 16, k++) {
    for (uint8_t j = 0; j < 16; j++) {
      if ((region->channels_mask[k] & (1 << j)) != 0) {
        if (region->channels[i + j].frequency == 0) {
          continue;
        }

        if ((lorawan->joined == RT_FALSE)
            && (region->join_channels_mask != NULL)) {
          if ((region->join_channels_mask[k] & (1 << j)) == 0) {
            continue;
          }
        }

        if (lorawan_check_dr_range(lorawan->dr,
                                   region->channels[i + j].dr_range.min,
                                   region->channels[i + j].dr_range.max)
            == RT_FALSE) {
          continue;
        }

        if (region->channels[i + j].band->ready_for_ransmission == RT_FALSE) {
          // nbRestrictedChannelsCount++;
          continue;
        }

        enabled_channels[enable_channel_count++] = i + j;
      }
    }
  }

  if (enable_channel_count == 0) {
    return -RT_EINVAL;
  }

  uint32_t random = lorawan_rand(0, enable_channel_count - 1);
  if (lorawan->dr == DR_0) {
    random = lorawan_rand(0, enable_channel_count - 2);
  }

  if (lorawan->confirmed_count > 5) {
    *channel = enabled_channels[enable_channel_count - 1];
  } else {
    *channel = enabled_channels[random];
  }

  return RT_EOK;
}

static rt_err_t lorawan_delay_tx(
  lorawan_t* lorawan,
  uint8_t*   buffer,
  rt_size_t  size) {
  uint32_t  channel;
  rt_time_t delay = 0;

  rt_err_t err = lorawan_next_channel(lorawan, &channel, &delay);
  if (err != RT_EOK) {
    LOG_E("lorawan next channel err(%d)!", err);
    return err;
  }

  lorawan->channel = channel;

  if (delay != 0) {
    // TODO
    LOG_I("lorawan delay(%d) tx!", delay);
    return err;
  }

  lorawan_region_tx_config_args_t params = {
    .channel = channel,
    .pkt_len = size,
  };

  err = lorawan->region->tx_config(lorawan, &params);
  if (err != RT_EOK) {
    LOG_E("lorawan tx config err(%d)!", err);
    return -RT_EINVAL;
  }

  rt_time_t tx_time = (rt_time_t)rt_device_write(lorawan->lora_radio_device, 0, buffer, size);
  if (tx_time == 0) {
    LOG_E("lorawan radio send err: %d.", err);
    return RT_NULL;
  }

  // TODO:update time credits
  lorawan->tx_time_on_air = tx_time;
  lorawan->ack_requested  = RT_FALSE;

  return RT_EOK;
}

rt_err_t lorawan_set_channels_mask(
  lorawan_t* lorawan,
  uint16_t*  mask,
  rt_size_t  size) {
  if (size > lorawan->region->channels_mask_nb) {
    return -RT_EINVAL;
  }

  if ((lorawan->region->channels_mask != NULL) && (mask != NULL)) {
    for (uint8_t i = 0; i < size; i++) {
      lorawan->region->channels_mask[i] = mask[i];
    }
  }

  return RT_EOK;
}

static rt_err_t lorawan_join(lorawan_t* lorawan) {
  rt_err_t err = RT_EOK;
  LOG_I("join start");
  switch (lorawan->activation_type) {
  case ACTIVATION_TYPE_OTAA: {
    // TODO: memory pool
    uint8_t* send_buffer = rt_malloc(LORAWAN_JOIN_REQUEST_SIZE);
    if (RT_NULL == send_buffer) {
      LOG_E("OOM: OTAA lorawan_join");
      return -RT_ENOMEM;
    }

    // switch_class

    err = lorawan_serializer_join_request(lorawan, send_buffer);
    if (err != RT_EOK) {
      rt_free(send_buffer);
      LOG_E("lorawan serializer join requese err(%d)!", err);
      return err;
    }

    lorawan->dr = DR_0;

    err = lorawan_delay_tx(lorawan, send_buffer, LORAWAN_JOIN_REQUEST_SIZE);
    rt_free(send_buffer);
    if (err != RT_EOK) {
      LOG_E("lorawan tx delay err(%d)!", err);
      return err;
    }

  } break;
  case ACTIVATION_TYPE_ABP:
  default:
    LOG_E("activation_type(%d) not yet implemented!", lorawan->activation_type);
    break;
  }
  LOG_I("join end");

  return err;
}

static rt_bool_t lorawan_adr_calc_next(lorawan_t* lorawan) {
  // TODO change
  // dr
  // tx power
  // adrAckCounter
  return RT_FALSE;
}

static void lorawan_msg_fctrl(lorawan_t* lorawan, lorawan_frame_ctrl_t* fctrl) {
  fctrl->bits.fopts_len = 0;
  fctrl->bits.fpending  = 0;
  fctrl->bits.adr       = lorawan->adr_on;
  fctrl->bits.ack       = lorawan->ack_requested;

  if (lorawan->current_class == CLASS_B) {
    fctrl->bits.fpending = 1;
  }

  fctrl->bits.adr_ack = lorawan_adr_calc_next(lorawan);

  return;
}

static rt_err_t lorawan_send_msg(
  lorawan_t*        lorawan,
  lorawan_tx_msg_t* msg) {
  if (!lorawan->joined) {
    return -RT_EEMPTY;
  }
  LOG_I("send msg start");
  lorawan_msg_fctrl(lorawan, &msg->fctrl);

  // lorawan_serializer_cmds(lorawan,msg)
  // return -RT_EBUSY;

  // VerifyTxFrame

  uint32_t fcnt_up = lorawan->fcnt_list.fcnt_up;
  // if ((MacCtx.ChannelsNbTransCounter >= 1) || (MacCtx.AckTimeoutRetriesCounter > 1)) {
  //   fcnt_up -= 1;
  // }
  msg->fcnt_up = fcnt_up;

  rt_err_t err = lorawan_serializer_data(lorawan, msg);
  if (err != RT_EOK) {
    LOG_E("lorawan send msg serializer err(%d)!", err);
    return err;
  }

  lora_radio_lora_rx_status_t lora_rx_status = lora_radio_get_lora_rx_status(lorawan->lora_radio_device);
  if (lora_rx_status.rssi < -100) {
    lorawan->dr = DR_0;
  } else if (lorawan->confirmed_count > 5) {
    lorawan->dr = DR_0;
  } else {
    lorawan->dr = (lorawan_region_dr_t)lorawan_rand(DR_0, DR_5);
  }

  err = lorawan_delay_tx(lorawan, msg->phy_msg.buffer, msg->phy_msg.size);
  if (err != RT_EOK) {
    LOG_E("lorawan tx delay err(%d)!", err);
    return err;
  }

  if (lorawan->joined && (msg->ftype == LORAWAN_FTYPE_DATA_CONFIRMED_UP)) {
    lorawan->confirmed_count++;
  }
  LOG_I("send msg end");

  return RT_EOK;
}

lorawan_mb_msg_t* lorawan_malloc_mb_msg(uint32_t size) {
  lorawan_tx_msg_t* msg = rt_malloc(sizeof(lorawan_tx_msg_t));
  if (msg == RT_NULL) {
    return RT_NULL;
  }

  rt_memset(msg, 0, sizeof(lorawan_tx_msg_t));

  msg->mac_payload.size = size;
  return (lorawan_mb_msg_t*)msg;
}

void lorawan_free_mb_msg(lorawan_mb_msg_t* msg) {
  rt_free(msg);
}

rt_err_t lorawan_send_confirmed(
  lorawan_t*        lorawan,
  uint8_t           fport,
  lorawan_mb_msg_t* mb_msg) {
  if (lorawan->joined == RT_FALSE) {
    return -RT_EBUSY;
  }
  lorawan_tx_msg_t* tx_msg = (lorawan_tx_msg_t*)mb_msg;
  tx_msg->fport            = fport;
  tx_msg->ftype            = LORAWAN_FTYPE_DATA_CONFIRMED_UP;
  return rt_mb_send(lorawan->tx_mb, (rt_uint32_t)tx_msg);
}

rt_err_t lorawan_send_unconfirmed(
  lorawan_t*        lorawan,
  uint8_t           fport,
  lorawan_mb_msg_t* mb_msg) {
  if (lorawan->joined == RT_FALSE) {
    return -RT_EBUSY;
  }
  lorawan_tx_msg_t* tx_msg = (lorawan_tx_msg_t*)mb_msg;
  tx_msg->fport            = fport;
  tx_msg->ftype            = LORAWAN_FTYPE_DATA_UNCONFIRMED_UP;
  return rt_mb_send(lorawan->tx_mb, (rt_uint32_t)tx_msg);
}

static lorawan_t* g_lorawan;

rt_err_t lorawan_run(lorawan_t* lorawan) {
  rt_err_t    err;
  rt_uint32_t lorawan_event;

  err = rt_timer_start(lorawan->app_tx_timer);
  if (err != RT_EOK) {
    LOG_E("app tx start err: %d", err);
    return err;
  }

  g_lorawan = lorawan;

  for (;;) {
    err = rt_event_recv(
      &lorawan->irq_event,
      EV_LORAWAN_RX_WINDOW1
        | EV_LORAWAN_RX_WINDOW2
        | EV_LORAWAN_JOIN
        | EV_LORAWAN_TX_APP,
      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
      RT_WAITING_FOREVER,
      &lorawan_event);
    if (err != RT_EOK) {
      LOG_E("lorawan event recv err: %d", err);
      return err;
    }

    switch (lorawan_event) {
    case EV_LORAWAN_JOIN: {
      err = lorawan_join(lorawan);
      if (err != RT_EOK) {
        LOG_E("thread ev:join err: %d", err);
        return err;
      }
    } break;
    case EV_LORAWAN_RX_WINDOW1: {
      lorawan_region_rx_config_args_t params = {
        .slot       = RX_SLOT_WIN_1,
        .continuous = RT_FALSE,
      };
      LOG_I("EV_LORAWAN_RX_WINDOW1");
      rt_err_t err = lorawan->region->rx_config(lorawan, &params);
      if (err != RT_EOK) {
        LOG_E("rx window1 rx config err(%d)!", err);
        return err;
      }

      err = lora_radio_rx(lorawan->lora_radio_device, lorawan->rx_timeout);
      if (err != RT_EOK) {
        LOG_E("lorawan rx err: %d.", err);
        return err;
      }

      lorawan->rx_slot = RX_SLOT_WIN_1;
    } break;
    case EV_LORAWAN_RX_WINDOW2: {
      if (lorawan->rx_slot == RX_SLOT_WIN_1) {
        break;
      }
      LOG_I("EV_LORAWAN_RX_WINDOW2");
      lorawan_region_rx_config_args_t params = {
        .slot       = RX_SLOT_WIN_2,
        .continuous = RT_FALSE,
      };

      rt_err_t err = lorawan->region->rx_config(lorawan, &params);
      if (err != RT_EOK) {
        LOG_E("rx window2 rx config err(%d)!", err);
        return err;
      }

      err = lora_radio_rx(lorawan->lora_radio_device, lorawan->rx_timeout);
      if (err != RT_EOK) {
        LOG_E("lorawan rx err: %d.", err);
        return err;
      }

      lorawan->rx_slot = RX_SLOT_WIN_2;
    } break;
    case EV_LORAWAN_TX_APP: {
      lorawan_tx_msg_t* tx_msg;
      err = rt_mb_recv(lorawan->tx_mb, (rt_ubase_t*)&tx_msg, RT_WAITING_NO);
      if (err == -RT_ETIMEOUT) {
        break;
      } else if (err != RT_EOK) {
        LOG_E("thread ev:mb recv err: %d", err);
        return err;
      }

      err = lorawan_send_msg(lorawan, tx_msg);
      lorawan_free_mb_msg((lorawan_mb_msg_t*)tx_msg);
      if (err != RT_EOK) {
        LOG_I("thread send msg err %d", err);
        if (err == -RT_EEMPTY) {
          LOG_I("not join");
          break;
        }
        return err;
      }
    } break;
    default:
      LOG_E("thread ev:invalid ev(%d)", lorawan_event);
      return -RT_EINVAL;
    }
  }

  return RT_EOK;
}

static void lorawan_info() {
  if (g_lorawan == RT_NULL) {
    rt_kprintf("lorawan no init\n");
    return;
  }

  rt_kprintf("\033[32m");
  lora_radio_lora_rx_status_t lora_rx_status = lora_radio_get_lora_rx_status(g_lorawan->lora_radio_device);
  rt_kprintf("  snr:        %d\n", lora_rx_status.snr);
  rt_kprintf("  rssi:       %d\n", lora_rx_status.rssi);
  rt_kprintf("  class:      %d\n", g_lorawan->default_class);
  rt_kprintf("  joined:     %d\n", g_lorawan->joined);
  rt_kprintf("  tx time:    %dms\n", g_lorawan->tx_time_on_air);
  rt_kprintf("  fcnt up:    %d\n", g_lorawan->fcnt_list.fcnt_up);
  rt_kprintf("  dev addr:   0x%x\n", g_lorawan->dev_addr);
  rt_kprintf("  fcnt down:  %d\n", g_lorawan->fcnt_list.fcnt_down);
  rt_kprintf("  confirmed:  %d\n", g_lorawan->confirmed_count);
  rt_kprintf("  duty cycle: %dms\n", g_lorawan->tx_duty_cycle);

  rt_kprintf("  join-eui:  ");
  for (uint32_t i = 0; i < 8; i++) {
    rt_kprintf(" 0x%.2x", g_lorawan->secure_element.join_eui[i]);
  }
  rt_kprintf("\n");

  rt_kprintf("  dev-eui:   ");
  for (uint32_t i = 0; i < 8; i++) {
    rt_kprintf(" 0x%.2x", g_lorawan->secure_element.dev_eui[i]);
  }
  rt_kprintf("\n");

  rt_kprintf("  app-key:   ");
  for (uint32_t i = 0; i < 16; i++) {
    rt_kprintf(" 0x%.2x", g_lorawan->secure_element.app_key[i]);
  }
  rt_kprintf("\n");

  rt_kprintf("  chan mask: ");
  for (uint32_t i = 0; i < g_lorawan->region->channels_mask_nb; i++) {
    rt_kprintf(" 0x%.4x", g_lorawan->region->channels_mask[i]);
  }
  rt_kprintf("\n");

  rt_kprintf("  channels:  ");
  for (uint32_t i = 0; i < g_lorawan->region->channels_nb; i++) {
    rt_kprintf(" %d", g_lorawan->region->channels[i].frequency);
    if (!((i + 1) % 8) && (i != (g_lorawan->region->channels_nb - 1))) {
      rt_kprintf("\n             ");
    }
  }
  rt_kprintf("\n\033[0m");
};

MSH_CMD_EXPORT(lorawan_info, show lorawan info);
