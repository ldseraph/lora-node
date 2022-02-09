#ifndef __LORAWAN_H__
#define __LORAWAN_H__
#include <lora-radio.h>
#include <lorawan-soft-se.h>
#include <rtthread.h>
#include <stdint.h>

// RT bug
rt_err_t rt_mb_urgent(rt_mailbox_t, rt_ubase_t);

#ifndef LORAMAC_VERSIO
#define LORAMAC_VERSION 0x01000400
#endif

#define LORAWAN_APP_DEFAULT_TX_DUTYCYCLE 36000

#define LORAWAN_RFU 0

#define LORAWAN_JOINEUI_SIZE 8
#define LORAWAN_DEVEUI_SIZE 8
#define LORAWAN_DEVNONCE_SIZE 2
#define LORAWAN_KEY_SIZE 16
#define LORAWAN_MIC_SIZE 4

#define LORAWAN_DEFAULT_JOIN_EUI \
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

#define LORAWAN_DEFAULT_DEVICE_EUI \
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

#define LORAWAN_DEFAULT_APP_KEY \
  { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff }

#define LORAMAC_PHY_MAXPAYLOAD 255
#define LORAMAC_MHDR_FIELD_SIZE 1
#define LORAMAC_DEV_ADDR_FIELD_SIZE 4
#define LORAMAC_FHDR_DEV_ADDR_FIELD_SIZE LORAMAC_DEV_ADDR_FIELD_SIZE
#define LORAMAC_FHDR_F_CTRL_FIELD_SIZE 1
#define LORAMAC_FHDR_F_CNT_FIELD_SIZE 2
#define LORAMAC_F_PORT_FIELD_SIZE 1
#define LORAMAC_MIC_FIELD_SIZE 4
#define LORAMAC_JOIN_NONCE_FIELD_SIZE 3
#define LORAMAC_NET_ID_FIELD_SIZE 3
#define LORAMAC_DL_SETTINGS_FIELD_SIZE 1
#define LORAMAC_RX_DELAY_FIELD_SIZE 1
#define LORAMAC_CF_LIST_FIELD_SIZE 16
#define LORAMAC_FHDR_F_OPTS_MAX_FIELD_SIZE 15

#define LORAWAN_JOIN_REQUEST_SIZE (LORAMAC_MHDR_FIELD_SIZE \
                                   + LORAWAN_MIC_SIZE      \
                                   + LORAWAN_JOINEUI_SIZE  \
                                   + LORAWAN_DEVEUI_SIZE   \
                                   + LORAWAN_DEVNONCE_SIZE)

#define LORAMAC_FRAME_PAYLOAD_OVERHEAD_SIZE (LORAMAC_MHDR_FIELD_SIZE             \
                                             + (LORAMAC_FHDR_DEV_ADDR_FIELD_SIZE \
                                                + LORAMAC_FHDR_F_CTRL_FIELD_SIZE \
                                                + LORAMAC_FHDR_F_CNT_FIELD_SIZE) \
                                             + LORAMAC_F_PORT_FIELD_SIZE         \
                                             + LORAMAC_MIC_FIELD_SIZE)

#define LORAMAC_FRAME_PAYLOAD_MIN_SIZE (LORAMAC_MHDR_FIELD_SIZE             \
                                        + (LORAMAC_FHDR_DEV_ADDR_FIELD_SIZE \
                                           + LORAMAC_FHDR_F_CTRL_FIELD_SIZE \
                                           + LORAMAC_FHDR_F_CNT_FIELD_SIZE) \
                                        + LORAMAC_MIC_FIELD_SIZE)

#define LORAMAC_JOIN_ACCEPT_FRAME_MIN_SIZE (LORAMAC_MHDR_FIELD_SIZE          \
                                            + LORAMAC_JOIN_NONCE_FIELD_SIZE  \
                                            + LORAMAC_NET_ID_FIELD_SIZE      \
                                            + LORAMAC_DEV_ADDR_FIELD_SIZE    \
                                            + LORAMAC_DL_SETTINGS_FIELD_SIZE \
                                            + LORAMAC_RX_DELAY_FIELD_SIZE    \
                                            + LORAMAC_MIC_FIELD_SIZE)

#define LORAMAC_JOIN_ACCEPT_FRAME_MAX_SIZE (LORAMAC_MHDR_FIELD_SIZE          \
                                            + LORAMAC_JOIN_NONCE_FIELD_SIZE  \
                                            + LORAMAC_NET_ID_FIELD_SIZE      \
                                            + LORAMAC_DEV_ADDR_FIELD_SIZE    \
                                            + LORAMAC_DL_SETTINGS_FIELD_SIZE \
                                            + LORAMAC_RX_DELAY_FIELD_SIZE    \
                                            + LORAMAC_CF_LIST_FIELD_SIZE     \
                                            + LORAMAC_MIC_FIELD_SIZE)

#define LORAMAC_MAX_MC_CTX 4
#define LORAMAC_FCNT_DOWN_INITAL_VALUE 0xFFFFFFFF

typedef struct {
  uint8_t join_eui[LORAWAN_JOINEUI_SIZE];
  uint8_t dev_eui[LORAWAN_DEVEUI_SIZE];

  uint8_t app_key[LORAWAN_KEY_SIZE];
  uint8_t mc_root_key[LORAWAN_KEY_SIZE];
  uint8_t mc_ke_key[LORAWAN_KEY_SIZE];

  uint8_t app_s_key[LORAWAN_KEY_SIZE];
  uint8_t nwk_s_enc_key[LORAWAN_KEY_SIZE];
  uint8_t f_nwk_s_int_key[LORAWAN_KEY_SIZE];
  uint8_t s_nwk_s_int_key[LORAWAN_KEY_SIZE];
} lorawan_secure_element_t;

typedef enum {
  /*!
     * LoRaWAN device class A
     *
     * LoRaWAN Specification V1.0.2, chapter 3
     */
  CLASS_A = 0x00,
  /*!
     * LoRaWAN device class B
     *
     * LoRaWAN Specification V1.0.2, chapter 8
     */
  CLASS_B = 0x01,
  /*!
     * LoRaWAN device class C
     *
     * LoRaWAN Specification V1.0.2, chapter 17
     */
  CLASS_C = 0x02,
} lorawan_class_t;

typedef enum {
  LORAWAN_FTYPE_JOIN_REQUEST = 0,
  LORAWAN_FTYPE_JOIN_ACCEPT,
  LORAWAN_FTYPE_DATA_UNCONFIRMED_UP,
  LORAWAN_FTYPE_DATA_UNCONFIRMED_DOWN,
  LORAWAN_FTYPE_DATA_CONFIRMED_UP,
  LORAWAN_FTYPE_DATA_CONFIRMED_DOWN,
  LORAWAN_FTYPE_TYPE_PROPRIETARY = 0x07,
} lorawan_ftype_t;

typedef union {
  struct version_s {
    uint8_t revision;
    uint8_t patch;
    uint8_t minor;
    uint8_t major;
  } fields;
  uint32_t value;
} lorawan_version_t;

typedef enum {
  ACTIVATION_TYPE_NONE = 0,
  ACTIVATION_TYPE_ABP  = 1,
  ACTIVATION_TYPE_OTAA = 2,
} lorawan_activation_type_t;

typedef enum {
  RX_SLOT_WIN_1,
  RX_SLOT_WIN_2,
  RX_SLOT_WIN_CLASS_C,
  RX_SLOT_WIN_CLASS_C_MULTICAST,
  RX_SLOT_WIN_CLASS_B_PING_SLOT,
  RX_SLOT_WIN_CLASS_B_MULTICAST_SLOT,
  RX_SLOT_NONE,
} lorawan_rx_slot_t;

typedef struct {
  uint32_t fcnt_up;
  uint32_t fcnt_down;
  uint32_t n_fcnt_down;
  uint32_t a_fcnt_down;
  uint32_t mc_fcnt_down[LORAMAC_MAX_MC_CTX];
} lorawan_fcnt_list_t;

typedef struct {
  uint32_t size;
  uint8_t  buffer[LORAMAC_PHY_MAXPAYLOAD];
} lorawan_mb_msg_t;

typedef union {
  uint8_t value;

  struct {
    uint8_t fopts_len : 4;
    uint8_t fpending : 1;
    uint8_t ack : 1;
    uint8_t adr_ack : 1;
    uint8_t adr : 1;
  } bits;

} lorawan_frame_ctrl_t;

typedef enum {
  FRAME_TYPE_A,
  FRAME_TYPE_B,
  FRAME_TYPE_C,
  FRAME_TYPE_D,
} lorawan_frame_type_t;

typedef struct {
  lorawan_mb_msg_t     mac_payload;
  lorawan_mb_msg_t     phy_msg;
  uint8_t              fport;
  lorawan_ftype_t      ftype;
  lorawan_frame_ctrl_t fctrl;
  uint32_t             fcnt_up;
  uint8_t              fopts[LORAMAC_FHDR_F_OPTS_MAX_FIELD_SIZE];
  uint8_t*             key;
} lorawan_tx_msg_t;

typedef struct {
  lorawan_mb_msg_t     mac_payload;
  lorawan_mb_msg_t     phy_msg;
  lorawan_ftype_t      ftype;
  uint32_t             dev_addr;
  uint32_t             fcnt_down;
  lorawan_frame_ctrl_t fctrl;
  uint8_t              fport;
  uint8_t              fopts[LORAMAC_FHDR_F_OPTS_MAX_FIELD_SIZE];
  uint8_t*             key;
  uint32_t             mic;
  lorawan_frame_type_t frame_type;
} lorawan_rx_msg_t;

typedef struct lorawan_s lorawan_t;

#include <lorawan-region.h>

struct lorawan_s {
  uint32_t                tx_duty_cycle;
  lorawan_class_t         current_class;
  lorawan_class_t         default_class;
  const lorawan_region_t* region;
  rt_device_t             lora_radio_device;
  rt_thread_t             event_thread;
  struct rt_event         irq_event;
  rt_mailbox_t            tx_mb;

  lorawan_version_t         version;
  lorawan_activation_type_t activation_type;
  lorawan_secure_element_t  secure_element;
  rt_bool_t                 joined;
  rt_bool_t                 adr_on;
  rt_bool_t                 ack_requested;
  lorawan_rx_slot_t         rx_slot;
  uint32_t                  join_nonce;
  uint16_t                  dev_nonce;
  uint32_t                  net_ID;
  uint32_t                  dev_addr;
  uint32_t                  system_max_rx_err;
  uint32_t                  min_rx_symbols;
  uint32_t                  rx_timeout;
  uint32_t                  symbol_timeout;
  int32_t                   window_offset;
  uint32_t                  channel;
  uint32_t                  rx1_dr_offset;
  uint32_t                  rx2_frequency;
  lorawan_region_dr_t       rx2_dr;
  uint32_t                  rxc_frequency;
  lorawan_region_dr_t       rxc_dr;
  lorawan_region_dr_t       dr;
  lora_radio_tx_power_t     tx_power;
  rt_time_t                 tx_time_on_air;
  rt_bool_t                 uplink_dwell_time;
  rt_bool_t                 downlink_dwell_time;
  uint32_t                  fcnt_gap;
  lorawan_fcnt_list_t       fcnt_list;
  uint32_t                  confirmed_count;

  uint32_t receive_delay1;
  uint32_t receive_delay2;
  uint32_t join_accept_delay1;
  uint32_t join_accept_delay2;

  rt_timer_t app_tx_timer;
  rt_timer_t rx_window_timer1;
  rt_timer_t rx_window_timer2;
  rt_timer_t tx_delay_timer;
  rt_timer_t ack_timer;

  rt_tick_t tx_done_tick;
  rt_time_t rx_done_tick;
};

lorawan_t*        lorawan_new(rt_device_t, lorawan_region_type_t);
rt_err_t          lorawan_run(lorawan_t*);
rt_err_t          lorawan_set_channels_mask(lorawan_t*, uint16_t*, rt_size_t);
rt_err_t          lorawan_send_confirmed(lorawan_t*, uint8_t, lorawan_mb_msg_t*);
rt_err_t          lorawan_send_unconfirmed(lorawan_t*, uint8_t, lorawan_mb_msg_t*);
lorawan_mb_msg_t* lorawan_malloc_mb_msg(uint32_t);
void              lorawan_free_mb_msg(lorawan_mb_msg_t*);

#include <lorawan-serialize.h>

void    lorawan_srand(uint32_t);
int32_t lorawan_rand(int32_t, int32_t);

#define LORAWAN_MAX(a, b) \
  (((a) > (b)) ? (a) : (b))

#define LORAWAN_MIN(a, b) \
  (((a) < (b)) ? (a) : (b))

#define LORAWAN_DIV_CEIL(N, D) \
  ((N > 0) ? (((N) + (D)-1) / (D)) : ((N) / (D)))

#endif  // __LORAWAN_H__