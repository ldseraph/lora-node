#include <lorawan.h>
#define DBG_SECTION_NAME "cn470"
#include <ulog.h>

#define CN470_MAX_NB_BANDS 1
#define CN470_MAX_NB_CHANNELS 96
#define CN470_MAX_NB_CHANNELS_MASK 6
#define CN470_MAX_RX_WINDOW 3000

#define CN470_DEFAULT_UPLINK_DWELL_TIME 0
#define CN470_DEFAULT_RECEIVE_DELAY1 1000
#define CN470_DEFAULT_RECEIVE_DELAY2 (CN470_DEFAULT_RECEIVE_DELAY1 + 1000)
#define CN470_DEFAULT_JOIN_ACCEPT_DELAY1 5000
#define CN470_DEFAULT_JOIN_ACCEPT_DELAY2 (CN470_DEFAULT_JOIN_ACCEPT_DELAY1 + 1000)
#define CN470_RX_WINDOW2_FREQ 505300000
#define CN470_RX_WINDOW2_DR DR_0
#define CN470_FIRST_RX1_CHANNEL ((uint32_t)500300000)
#define CN470_STEPWIDTH_RX1_CHANNEL ((uint32_t)200000)

#define CN470_MIN_TX_POWER TX_POWER_7
#define CN470_MAX_TX_POWER TX_POWER_0
#define CN470_DEFAULT_TX_POWER CN470_MAX_TX_POWER

#define CN470_DEFAULT_DATARATE DR_0
#define CN470_RX_MAX_DATARATE DR_5

lorawan_region_band_t region_cn470_bands[CN470_MAX_NB_BANDS] = { {
  .duty_cycle            = 1,
  .tx_max_power          = CN470_MAX_TX_POWER,
  .ready_for_ransmission = RT_TRUE,
} };

lorawan_region_channel_t region_cn470_channels[CN470_MAX_NB_CHANNELS];

static const lora_radio_spreading_factors_t region_cn470_spreading_factors[] = { LORA_SF12, LORA_SF11, LORA_SF10, LORA_SF9, LORA_SF8, LORA_SF7 };
static const lora_radio_lora_bandwidths_t   region_cn470_bandwidths[]        = { LORA_BW_125, LORA_BW_125, LORA_BW_125, LORA_BW_125, LORA_BW_125, LORA_BW_125 };
static const uint8_t                        region_cn470_max_payload[]       = { 51, 51, 51, 115, 242, 242 };

uint16_t region_cn470_channels_mask[CN470_MAX_NB_CHANNELS_MASK];
uint16_t region_cn470_channels_default_mask[CN470_MAX_NB_CHANNELS_MASK];

static void region_cn470_compute_rx_window(lorawan_t* lorawan) {
  uint32_t wakeup_time = lora_radio_wakeup_time(lorawan->lora_radio_device);

  uint32_t symbol_in_us = lorawan_region_compute_symbol_time_lora(
    region_cn470_spreading_factors[lorawan->dr],
    lora_radio_get_bandwidth(region_cn470_bandwidths[lorawan->dr]));

  lorawan->symbol_timeout = LORAWAN_MAX(
    LORAWAN_DIV_CEIL(
      ((2 * lorawan->min_rx_symbols - 8) * symbol_in_us + 2 * (lorawan->system_max_rx_err * 1000)),
      symbol_in_us),
    lorawan->min_rx_symbols);

  lorawan->window_offset = (int32_t)LORAWAN_DIV_CEIL(
    (int32_t)(4 * symbol_in_us) - (int32_t)LORAWAN_DIV_CEIL((lorawan->symbol_timeout * symbol_in_us), 2) - (int32_t)(wakeup_time * 1000),
    1000);

  return;
}

static rt_err_t region_cn470_tx_config(
  lorawan_t*                       lorawan,
  lorawan_region_tx_config_args_t* params) {
  uint32_t frequency = region_cn470_channels[params->channel].frequency;

  rt_err_t err = lora_radio_set_channel(lorawan->lora_radio_device, frequency);
  if (err != RT_EOK) {
    LOG_E("lorawan set channel err: %d.", err);
    return err;
  }

  lora_radio_modems_t            modem             = LORD_RADIO_MODEM_LORA;
  lora_radio_tx_power_t          tx_power_limited  = LORAWAN_MAX(lorawan->tx_power, region_cn470_channels[params->channel].band->tx_max_power);
  lora_radio_spreading_factors_t spreading_factors = region_cn470_spreading_factors[lorawan->dr];
  lora_radio_lora_bandwidths_t   bandwidth         = region_cn470_bandwidths[lorawan->dr];

  lora_radio_tx_config_args_t tx_config_args = {
    .modem              = modem,
    .tx_power           = tx_power_limited,
    .preamble_length    = 8,
    .packet_length_mode = LORA_PACKET_VARIABLE_LENGTH,
    .packet_length      = params->pkt_len,
    .crc_on             = RT_TRUE,
    .modem_args         = {
      .lora = {
        .bandwidth         = bandwidth,
        .spreading_factors = spreading_factors,
        .coding_rate       = LORD_RADIO_CR_4_5,
        .invert_iq         = RT_FALSE,
      },
    },
  };

  err = lora_radio_tx_config(
    lorawan->lora_radio_device,
    &tx_config_args);
  if (err != RT_EOK) {
    LOG_E("lorawan tx config err: %d.", err);
    return err;
  }

  lorawan->tx_power = tx_power_limited;

  return RT_EOK;
}

static lorawan_region_dr_t region_cn470_apply_dr_offset(lorawan_region_dr_t dr, uint32_t drOffset) {
  int8_t datarate = dr - drOffset;

  if (datarate < 0) {
    datarate = DR_0;
  }
  return datarate;
}

static rt_err_t region_cn470_rx_config(
  lorawan_t*                       lorawan,
  lorawan_region_rx_config_args_t* params) {
  uint32_t            frequency = lorawan->rx2_frequency;
  lorawan_region_dr_t dr        = lorawan->rx2_dr;

  if (params->slot == RX_SLOT_WIN_1) {
    frequency = CN470_FIRST_RX1_CHANNEL + (lorawan->channel % 48) * CN470_STEPWIDTH_RX1_CHANNEL;
    dr        = region_cn470_apply_dr_offset(lorawan->dr, lorawan->rx1_dr_offset);
  }

  rt_err_t err = lora_radio_set_channel(lorawan->lora_radio_device, frequency);
  if (err != RT_EOK) {
    LOG_E("lorawan set channel err: %d.", err);
    return err;
  }

  dr = LORAWAN_MIN(dr, CN470_RX_MAX_DATARATE);

  lora_radio_modems_t            modem             = LORD_RADIO_MODEM_LORA;
  lora_radio_spreading_factors_t spreading_factors = region_cn470_spreading_factors[dr];
  lora_radio_lora_bandwidths_t   bandwidth         = region_cn470_bandwidths[dr];

  lora_radio_rx_config_args_t rx_config_args = {
    .modem              = modem,
    .preamble_length    = 8,
    .packet_length_mode = LORA_PACKET_VARIABLE_LENGTH,
    .packet_length      = region_cn470_max_payload[dr] + LORAMAC_FRAME_PAYLOAD_OVERHEAD_SIZE,
    .crc_on             = RT_FALSE,
    .rx_continuous      = params->continuous,
    .symbol_timeout     = lorawan->symbol_timeout,
    .modem_args         = {
      .lora = {
        .bandwidth         = bandwidth,
        .spreading_factors = spreading_factors,
        .coding_rate       = LORD_RADIO_CR_4_5,
        .invert_iq         = RT_TRUE,
      },
    },
  };

  err = lora_radio_rx_config(
    lorawan->lora_radio_device,
    &rx_config_args);
  if (err != RT_EOK) {
    LOG_E("lorawan rx config err: %d.", err);
    return err;
  }

  return RT_EOK;
}

static rt_err_t region_cn470_apply_cf_list(lorawan_t* lorawan, uint8_t* cf_list) {
  if (cf_list[15] != 0x01) {
    return -RT_EINVAL;
  }

  // TODO
  // for (uint8_t chMaskItr = 0, cntPayload = 0; chMaskItr <= 5; chMaskItr++, cntPayload += 2) {
  //   RegionNvmGroup2->ChannelsMask[chMaskItr] = (uint16_t)(0x00FF & applyCFList->Payload[cntPayload]);
  //   RegionNvmGroup2->ChannelsMask[chMaskItr] |= (uint16_t)(applyCFList->Payload[cntPayload + 1] << 8);
  // }

  return RT_EOK;
}

static uint8_t region_cn470_get_max_phy_payload(lorawan_t* lorawan) {
  return region_cn470_max_payload[lorawan->dr];
}

rt_err_t region_cn470_init(lorawan_t* lorawan) {
  for (uint32_t i = 0; i < CN470_MAX_NB_CHANNELS; i++) {
    region_cn470_channels[i].band         = &region_cn470_bands[0];
    region_cn470_channels[i].dr_range.min = DR_5;
    region_cn470_channels[i].dr_range.max = DR_0;
    region_cn470_channels[i].frequency    = 470300000 + i * 200000;
  }

  for (uint32_t i = 0; i < CN470_MAX_NB_CHANNELS_MASK; i++) {
    region_cn470_channels_mask[i]         = 0xFFFF;
    region_cn470_channels_default_mask[i] = 0xFFFF;
  }

  lorawan->tx_power           = CN470_DEFAULT_TX_POWER;
  lorawan->receive_delay1     = CN470_DEFAULT_RECEIVE_DELAY1;
  lorawan->receive_delay2     = CN470_DEFAULT_RECEIVE_DELAY2;
  lorawan->join_accept_delay1 = CN470_DEFAULT_JOIN_ACCEPT_DELAY1;
  lorawan->join_accept_delay2 = CN470_DEFAULT_JOIN_ACCEPT_DELAY2;
  lorawan->rx2_frequency      = CN470_RX_WINDOW2_FREQ;
  lorawan->rx2_dr             = CN470_RX_WINDOW2_DR;
  lorawan->rxc_frequency      = CN470_RX_WINDOW2_FREQ;
  lorawan->rxc_dr             = CN470_RX_WINDOW2_DR;
  lorawan->dr                 = CN470_DEFAULT_DATARATE;
  lorawan->rx1_dr_offset      = LORAWAN_REGION_COMMON_DEFAULT_RX1_DR_OFFSET;
  lorawan->rx_timeout         = CN470_MAX_RX_WINDOW;
  lorawan->uplink_dwell_time  = CN470_DEFAULT_UPLINK_DWELL_TIME;
  lorawan->uplink_dwell_time  = LORAWAN_REGION_COMMON_DEFAULT_DOWNLINK_DWELL_TIME;
  lorawan->fcnt_gap           = LORAWAN_REGION_COMMON_DEFAULT_MAX_FCNT_GAP;
  
  return RT_EOK;
}

lorawan_region_t region_cn470 = {
  .init                  = region_cn470_init,
  .tx_config             = region_cn470_tx_config,
  .rx_config             = region_cn470_rx_config,
  .apply_cf_list         = region_cn470_apply_cf_list,
  .compute_tx_window     = region_cn470_compute_rx_window,
  .get_max_phy_payload   = region_cn470_get_max_phy_payload,
  .bands                 = region_cn470_bands,
  .bands_nb              = CN470_MAX_NB_BANDS,
  .channels              = region_cn470_channels,
  .channels_nb           = CN470_MAX_NB_CHANNELS,
  .channels_mask         = region_cn470_channels_mask,
  .channels_mask_nb      = CN470_MAX_NB_CHANNELS_MASK,
  .channels_default_mask = region_cn470_channels_default_mask,
};
