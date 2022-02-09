#ifndef __LORA_RADIO_H__
#define __LORA_RADIO_H__
#include <device-tree.h>
#include <lora-radio.h>
#include <rtdevice.h>
#include <stdint.h>

#define LORA_RADIO_PUBLIC_SYNCWORD 0x3444
#define LORD_RADIO_FSK_SYNCWORD \
  { 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 }
#define LORA_RADIO_PRIVATE_SYNCWORD 0x1424
#define LORA_RADIO_DEFAULT_TX_POWER TX_POWER_0
#define LORA_RADIO_DEFAULT_ANTENNA_GAIN 2.15f
#define LORA_RADIO_DEFAULT_MAX_EIRP 19.15f

typedef enum {
  TX_POWER_0 = 0,
  TX_POWER_1,
  TX_POWER_2,
  TX_POWER_3,
  TX_POWER_4,
  TX_POWER_5,
  TX_POWER_6,
  TX_POWER_7,
  TX_POWER_8,
  TX_POWER_9,
  TX_POWER_10,
  TX_POWER_11,
  TX_POWER_12,
  TX_POWER_13,
  TX_POWER_14,
  TX_POWER_15
} lora_radio_tx_power_t;

typedef enum {
  MODE_SLEEP = 0x00,  //! The radio is in sleep mode
  MODE_STDBY,         //! The radio is in standby mode
  MODE_FS,            //! The radio is in frequency synthesis mode
  MODE_TX,            //! The radio is in transmit mode
  MODE_RX,            //! The radio is in receive mode
  MODE_RX_DC,         //! The radio is in receive duty cycle mode
  MODE_CAD            //! The radio is in channel activity detection mode
} lora_radio_operating_modes_t;

typedef enum {
  LORD_RADIO_MODEM_FSK = 0,
  LORD_RADIO_MODEM_LORA,
  LORD_RADIO_MODEM_NONE,
} lora_radio_modems_t;

typedef enum {
  LORD_RADIO_CR_4_5 = 0x01,
  LORD_RADIO_CR_4_6 = 0x02,
  LORD_RADIO_CR_4_7 = 0x03,
  LORD_RADIO_CR_4_8 = 0x04,
} lora_radio_coding_rates_t;

typedef enum {
  LORD_RADIO_PACKET_TYPE_GFSK = 0x00,
  LORD_RADIO_PACKET_TYPE_LORA = 0x01,
  LORD_RADIO_PACKET_TYPE_NONE = 0x0F,
} lora_radio_packet_types_t;

typedef enum {
  LORA_SF5  = 0x05,
  LORA_SF6  = 0x06,
  LORA_SF7  = 0x07,
  LORA_SF8  = 0x08,
  LORA_SF9  = 0x09,
  LORA_SF10 = 0x0A,
  LORA_SF11 = 0x0B,
  LORA_SF12 = 0x0C,
} lora_radio_spreading_factors_t;

typedef enum {
  LORA_BW_500 = 6,
  LORA_BW_250 = 5,
  LORA_BW_125 = 4,
  LORA_BW_062 = 3,
  LORA_BW_041 = 10,
  LORA_BW_031 = 2,
  LORA_BW_020 = 9,
  LORA_BW_015 = 1,
  LORA_BW_010 = 8,
  LORA_BW_007 = 0,
} lora_radio_lora_bandwidths_t;

static inline uint32_t lora_radio_get_bandwidth(lora_radio_lora_bandwidths_t bandwindth) {
  switch (bandwindth) {
  default:
    return 0;
  case LORA_BW_007:
    return 7000;
  case LORA_BW_010:
    return 10000;
  case LORA_BW_015:
    return 15000;
  case LORA_BW_020:
    return 20000;
  case LORA_BW_031:
    return 31000;
  case LORA_BW_041:
    return 41000;
  case LORA_BW_062:
    return 62000;
  case LORA_BW_125:
    return 125000;
  case LORA_BW_250:
    return 250000;
  case LORA_BW_500:
    return 500000;
  }
}

typedef enum {
  IRQ_RADIO_NONE            = 0x0000,
  IRQ_TX_DONE               = 0x0001,
  IRQ_RX_DONE               = 0x0002,
  IRQ_PREAMBLE_DETECTED     = 0x0004,
  IRQ_SYNCWORD_VALID        = 0x0008,
  IRQ_HEADER_VALID          = 0x0010,
  IRQ_HEADER_ERROR          = 0x0020,
  IRQ_CRC_ERROR             = 0x0040,
  IRQ_CAD_DONE              = 0x0080,
  IRQ_CAD_ACTIVITY_DETECTED = 0x0100,
  IRQ_RX_TX_TIMEOUT         = 0x0200,
  IRQ_RADIO_ALL             = 0xFFFF,
} lora_radio_irq_masks_t;

typedef enum {
  LORA_PACKET_VARIABLE_LENGTH = 0x00,
  LORA_PACKET_FIXED_LENGTH    = 0x01,
} lora_radio_packet_length_mode_t;

typedef struct {
  lora_radio_modems_t             modem;
  lora_radio_tx_power_t           tx_power;
  rt_bool_t                       rx_continuous;
  uint32_t                        symbol_timeout;
  uint16_t                        preamble_length;
  uint32_t                        packet_length;
  lora_radio_packet_length_mode_t packet_length_mode;
  rt_bool_t                       crc_on;

  union lora_radio_tx_config_modem_args {
    struct lora_radio_tx_config_modem_fsk_args {
      uint32_t bandwidth;
      uint32_t datarate;
      uint32_t fdev;
      // .PreambleMinD
      // .SyncWordLeng
      // .AddrComp;
      // .PayloadLengt
      // .DcFree;
    } fsk;
    struct lora_radio_tx_config_modem_lora_args {
      lora_radio_lora_bandwidths_t   bandwidth;
      lora_radio_spreading_factors_t spreading_factors;
      rt_bool_t                      invert_iq;
      lora_radio_coding_rates_t      coding_rate;
    } lora;
  } modem_args;
} lora_radio_rx_tx_config_args_t;

typedef lora_radio_rx_tx_config_args_t lora_radio_tx_config_args_t;
typedef lora_radio_rx_tx_config_args_t lora_radio_rx_config_args_t;

typedef struct {
  rt_bool_t wakeup_rtc;
  rt_bool_t reset;
  rt_bool_t warm_start;
} lora_radio_sleep_args_t;

typedef struct lora_radio_s lora_radio_t;

typedef struct {
  rt_err_t (*init)(lora_radio_t *);
  rt_err_t (*reset)(lora_radio_t *);
  rt_err_t (*wakeup)(lora_radio_t *);
  rt_err_t (*standby)(lora_radio_t *);
  rt_err_t (*sleep)(lora_radio_t *, lora_radio_sleep_args_t *);
  rt_err_t (*set_modulation_params)(lora_radio_t *, lora_radio_tx_config_args_t *);
  rt_err_t (*set_packet_params)(lora_radio_t *, lora_radio_tx_config_args_t *);
  rt_err_t (*set_frequency)(lora_radio_t *, uint32_t);
  rt_err_t (*set_packet_type)(lora_radio_t *, lora_radio_packet_types_t);
  rt_err_t (*set_sync_word)(lora_radio_t *, lora_radio_modems_t, uint32_t);
  rt_err_t (*set_tx_power)(lora_radio_t *, int8_t);
  rt_err_t (*after_tx_config)(lora_radio_t *, lora_radio_tx_config_args_t *);
  rt_err_t (*before_rx_config)(lora_radio_t *, lora_radio_rx_config_args_t *);
  rt_err_t (*after_rx_config)(lora_radio_t *, lora_radio_rx_config_args_t *);
  uint32_t (*random)(lora_radio_t *);
  uint32_t (*wakeup_time)(lora_radio_t *);
  rt_err_t (*send)(lora_radio_t *, rt_off_t, const uint8_t *, rt_size_t);
  rt_err_t (*rx)(lora_radio_t *);
  rt_err_t (*rx_done)(lora_radio_t *);
  rt_err_t (*read)(lora_radio_t *, uint8_t *, rt_size_t *, rt_size_t);
  rt_err_t (*get_irq)(lora_radio_t *, uint32_t *);
  rt_err_t (*clean_irq)(lora_radio_t *, uint32_t);
} lora_radio_ops_t;

typedef struct {
  uint8_t rx_status;
  int8_t  rssi_avg;
  int8_t  rssi_sync;
} lora_radio_fsk_rx_status_t;

typedef struct {
  int8_t rssi;
  int8_t snr;
  int8_t signal_rssi;
} lora_radio_lora_rx_status_t;

struct lora_radio_s {
  struct rt_device     parent;
  lora_radio_device_t *device;

  const lora_radio_ops_t *ops;

  struct rt_spi_device *spi_device;
  char *                lora_radio_name;

  void (*dio_irq)(lora_radio_t *);

  struct rt_event irq_event;

  lora_radio_operating_modes_t    mode;
  lora_radio_modems_t             modem;
  uint32_t                        lora_sync_word;
  lora_radio_packet_types_t       packet_type;
  lora_radio_packet_length_mode_t packet_length_mode;
  rt_time_t                       tx_time_on_air;
  rt_timer_t                      timeout_timer;
  rt_bool_t                       rx_continuous;

  lora_radio_fsk_rx_status_t  fsk_rx_status;
  lora_radio_lora_rx_status_t lora_rx_status;

  float antenna_gain;
  float max_eirp;

  void *private;
};

typedef enum {
  LORA_RADIO_EVENT_NONE              = 0,
  LORA_RADIO_EVENT_TX_DONE           = (0x1 << 0),
  LORA_RADIO_EVENT_TX_TIMEOUT        = (0x1 << 1),
  LORA_RADIO_EVENT_RX_DONE           = (0x1 << 2),
  LORA_RADIO_EVENT_RX_TIMEOUT        = (0x1 << 3),
  LORA_RADIO_EVENT_RX_ERROR          = (0x1 << 4),
  LORA_RADIO_EVENT_RX_HEAD_DETECTION = (0x1 << 5),
} lora_radio_event_t;

lora_radio_t *lora_radio_new(lora_radio_device_t *);
rt_err_t      lora_radio_device_register(lora_radio_t *, const char *, void *);
rt_err_t      lora_radio_event_recv(rt_device_t, lora_radio_event_t *);
rt_err_t      lora_radio_set_channel(rt_device_t, uint32_t);
rt_err_t      lora_radio_random(rt_device_t, uint32_t *);
rt_err_t      lora_radio_tx_config(rt_device_t, lora_radio_tx_config_args_t *);
rt_err_t      lora_radio_rx_config(rt_device_t, lora_radio_rx_config_args_t *);
rt_err_t      lora_radio_sleep(rt_device_t);
rt_err_t      lora_radio_rx(rt_device_t, uint32_t);
uint32_t      lora_radio_wakeup_time(rt_device_t);

lora_radio_fsk_rx_status_t  lora_radio_get_fsk_rx_status(rt_device_t);
lora_radio_lora_rx_status_t lora_radio_get_lora_rx_status(rt_device_t);

#endif  // __LORA_RADIO_H__