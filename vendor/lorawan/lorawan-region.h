#ifndef __REGION_H__
#define __REGION_H__
#include <rtthread.h>

#define LORAWAN_REGION_COMMON_DEFAULT_RX1_DR_OFFSET 0
#define LORAWAN_REGION_COMMON_DEFAULT_DOWNLINK_DWELL_TIME 0
#define LORAWAN_REGION_COMMON_DEFAULT_MAX_FCNT_GAP 16384
/*!
 * LoRaMAC region enumeration
 */
typedef enum {
  /*!
     * AS band on 923MHz
     */
  LORAMAC_REGION_AS923,
  /*!
     * Australian band on 915MHz
     */
  LORAMAC_REGION_AU915,
  /*!
     * Chinese band on 470MHz
     */
  LORAMAC_REGION_CN470,
  /*!
     * Chinese band on 779MHz
     */
  LORAMAC_REGION_CN779,
  /*!
     * European band on 433MHz
     */
  LORAMAC_REGION_EU433,
  /*!
     * European band on 868MHz
     */
  LORAMAC_REGION_EU868,
  /*!
     * South korean band on 920MHz
     */
  LORAMAC_REGION_KR920,
  /*!
     * India band on 865MHz
     */
  LORAMAC_REGION_IN865,
  /*!
     * North american band on 915MHz
     */
  LORAMAC_REGION_US915,
  /*!
     * Russia band on 864MHz
     */
  LORAMAC_REGION_RU864,
} lorawan_region_type_t;

typedef enum {
  DR_0 = 0,
  DR_1,
  DR_2,
  DR_3,
  DR_4,
  DR_5,
  DR_6,
  DR_7,
  DR_8,
  DR_9,
  DR_10,
  DR_11,
  DR_12,
  DR_13,
  DR_14,
  DR_15,
} lorawan_region_dr_t;

typedef struct {
  uint16_t              duty_cycle;
  lora_radio_tx_power_t tx_max_power;
  //   TimerTime_t LastBandUpdateTime;

  //   TimerTime_t LastMaxCreditAssignTime;

  //   TimerTime_t TimeCredits;

  //   TimerTime_t MaxTimeCredits;
  rt_bool_t ready_for_ransmission;
} lorawan_region_band_t;

typedef struct {
  lorawan_region_dr_t min;
  lorawan_region_dr_t max;
} lorawan_region_dr_range_t;

typedef struct {
  uint32_t                  frequency;
  uint32_t                  rx1_frequency;
  lorawan_region_dr_range_t dr_range;
  lorawan_region_band_t *   band;
} lorawan_region_channel_t;

typedef struct {
  uint32_t  channel;
  rt_size_t pkt_len;
} lorawan_region_tx_config_args_t;

typedef struct {
  lorawan_rx_slot_t slot;
  rt_bool_t         continuous;
} lorawan_region_rx_config_args_t;

typedef struct {
} lorawan_region_compute_rx_window_args_t;

typedef struct {
  rt_err_t (*init)(lorawan_t *);
  rt_err_t (*tx_config)(lorawan_t *, lorawan_region_tx_config_args_t *);
  rt_err_t (*rx_config)(lorawan_t *, lorawan_region_rx_config_args_t *);
  rt_err_t (*apply_cf_list)(lorawan_t *, uint8_t *);
  uint8_t (*get_max_phy_payload)(lorawan_t *);
  void (*compute_tx_window)(lorawan_t *);

  lorawan_region_band_t *bands;
  rt_size_t              bands_nb;

  lorawan_region_channel_t *channels;
  rt_size_t                 channels_nb;

  uint16_t *channels_mask;
  rt_size_t channels_mask_nb;
  uint16_t *channels_default_mask;
  uint16_t *join_channels_mask;

} lorawan_region_t;

extern lorawan_region_t region_cn470;

static inline uint32_t lorawan_region_compute_symbol_time_lora(
  lora_radio_spreading_factors_t phy_dr,
  uint32_t                       bandwidth_in_hz) {
  return ((uint32_t)(1 << (uint32_t)phy_dr) * 1000000) / bandwidth_in_hz;
}

#endif  // __LORAWAN_H__