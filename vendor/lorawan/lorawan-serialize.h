#ifndef __LORAWAN_SERIALIZE_H__
#define __LORAWAN_SERIALIZE_H__

typedef enum {
  LORAWAN_MAJOR_R1 = 0,
} lorawan_major_t;

typedef union {
  uint8_t value;

  struct {
    lorawan_major_t major : 2;
    uint8_t         rfu : 3;
    lorawan_ftype_t ftype : 3;
  } bits;
} lorawan_mhdr_t;

typedef union {
  uint8_t value;

  struct {
    uint8_t rx2_datarate : 4;
    uint8_t rx1_dr_offset : 3;
    uint8_t rfu : 1;
  } bits;
} lorawan_dl_setting_t;

#define LORAWAN_NWK_S_KEY_COMP_BASE 0x01
#define LORAWAN_APP_S_KEY_COMP_BASE 0x02

lorawan_ftype_t lorawan_serializer_get_ftype(uint8_t);
rt_err_t        lorawan_serializer_join_request(lorawan_t*, uint8_t*);
rt_err_t        lorawan_serializer_join_accept(lorawan_t*, uint8_t*, rt_size_t);
rt_err_t        lorawan_serializer_data(lorawan_t*, lorawan_tx_msg_t*);
rt_err_t        lorawan_deserialize_data(lorawan_t*, lorawan_rx_msg_t*);
rt_err_t        lorawan_serializer_compute_hash(uint8_t*, uint8_t*, uint16_t, uint8_t*, uint8_t*);
#endif  // __LORAWAN_SERIALIZE_H__