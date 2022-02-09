#ifndef __DEVICE_TREE_H__
#define __DEVICE_TREE_H__
#include <stdint.h>

typedef struct {
  const char* adc;
  const char* chg;
  const char* pg;
  uint32_t    channel;
} sensor_bq24040_device_t;

typedef struct {
  const char* pdwn;
  const char* sclk;
  const char* dout;
  const char* avdd;
} sensor_ads1232_device_t;

typedef struct {
  uint32_t spi_base;
} spi_bus_device_t;

typedef struct {
  const char* spi_bus;
  const char* spi_cs;
} lora_radio_device_t;

typedef enum {
  STDBY_RC   = 0x00,
  STDBY_XOSC = 0x01,
} sx126x_standby_modes_t;

typedef enum {
  REGULATOR_LDO  = 0x00,
  REGULATOR_DCDC = 0x01,
} sx126x_regulator_modes_t;

typedef enum {
  SX1261 = 0x00,
  SX1262,
  SX1268
} sx126x_deviceID_t;

typedef enum {
  TCXO_CTRL_1_6V = 0x00,
  TCXO_CTRL_1_7V = 0x01,
  TCXO_CTRL_1_8V = 0x02,
  TCXO_CTRL_2_2V = 0x03,
  TCXO_CTRL_2_4V = 0x04,
  TCXO_CTRL_2_7V = 0x05,
  TCXO_CTRL_3_0V = 0x06,
  TCXO_CTRL_3_3V = 0x07,
} sx126x_tcxo_ctrl_voltage_t;

typedef struct {
  lora_radio_device_t        lora_radio_device;
  const char*                gpio_busy;
  const char*                gpio_dio1;
  const char*                gpio_reset;
  sx126x_standby_modes_t     standby_mode;
  sx126x_regulator_modes_t   regulator_mode;
  sx126x_deviceID_t          device_id;
  sx126x_tcxo_ctrl_voltage_t tcxo_ctrl;
  uint32_t                   tcxo_delay;
  uint8_t                    dio2_as_rf_switch;
  uint32_t                   ramp_times;
  uint32_t                   tcxo_wakeup_time;
  float                      antenna_gain;
  float                      max_eirp;
} sx126x_device_t;

typedef struct {
  const char* name;
  void*       device;
} device_t;

typedef struct {
  device_t* list;
  uint32_t  num;
} devices_t;

#endif