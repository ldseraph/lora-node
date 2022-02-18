#include <board.h>
#include <rtthread.h>
#include <lorawan_app.h>
#define DBG_SECTION_NAME "lorawan.app"
#include <ulog.h>

typedef rt_err_t (*app_init_func_t)(lorawan_t *);


app_init_func_t init_app_list[] = {
#ifdef APP_COMPONENTS_TENSION
  tension_app_init,
#endif
#ifdef APP_COMPONENTS_TILT
  tilt_app_init,
#endif
  RT_NULL
};

typedef struct {
  lorawan_region_type_t region;
  lorawan_class_t class;
  const char *lora_radio_name;
  uint32_t    tx_duty_cycle;
} config_t;

static config_t config = {
  .lora_radio_name = "lora-radio.sx126x0",
  .region          = LORAMAC_REGION_CN470,
  .class           = CLASS_A,
  .tx_duty_cycle   = 10000,
};

rt_err_t lorawan_app(){
  
  const char           *lora_radio_name = config.lora_radio_name;
  uint32_t              tx_duty_cycle   = config.tx_duty_cycle;
  lorawan_region_type_t region          = config.region;
  lorawan_class_t class                 = config.class;

  rt_device_t lora_radio = rt_device_find(lora_radio_name);
  if (lora_radio == RT_NULL) {
    LOG_E("lora_radio: %s find err.", lora_radio_name);
  }

  rt_err_t err = rt_device_open(lora_radio, RT_DEVICE_OFLAG_RDWR);
  if (err != RT_EOK) {
    LOG_E("rt_device: %s open err: %d.", lora_radio, err);
  }

  // lorawan init
  lorawan_t *lorawan = lorawan_new(lora_radio, region);
  if (lorawan == RT_EOK) {
    LOG_E("lorawan new err.");
    return -RT_ENOMEM;
  }

  lorawan->tx_duty_cycle = tx_duty_cycle;
  lorawan->default_class = class;

  uint8_t app_key[] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
  rt_memcpy(lorawan->secure_element.app_key, app_key, sizeof(app_key));

  uint8_t dev_eui[8];
  board_get_unique_ID(dev_eui);

  rt_memcpy(lorawan->secure_element.dev_eui, dev_eui, sizeof(dev_eui));

  uint8_t join_eui[8];
  lorawan_serializer_compute_hash(RT_NULL, dev_eui, sizeof(dev_eui), lorawan->secure_element.app_key, join_eui);

  rt_memcpy(lorawan->secure_element.join_eui, join_eui, sizeof(join_eui));

  // channelsmask init
  uint16_t channels_mask[lorawan->region->channels_mask_nb];
  rt_memset(channels_mask, 0, sizeof(channels_mask));

  channels_mask[0] = 0x00ff;

  err = lorawan_set_channels_mask(lorawan, channels_mask, lorawan->region->channels_mask_nb);
  if (err != RT_EOK) {
    LOG_E("lorawan set channels err(%d).", err);
    return err;
  }

  rt_kprintf("\n\033[32m");
  rt_kprintf("  class:      %d\n", lorawan->default_class);
  rt_kprintf("  duty cycle: %dms\n", lorawan->tx_duty_cycle);

  rt_kprintf("  join-eui:  ");
  for (uint32_t i = 0; i < 8; i++) {
    rt_kprintf(" 0x%.2x", lorawan->secure_element.join_eui[i]);
  }
  rt_kprintf("\n");

  rt_kprintf("  dev-eui:    ");
  for (uint32_t i = 0; i < 8; i++) {
    rt_kprintf("%.2x", lorawan->secure_element.dev_eui[i]);
  }
  rt_kprintf("\n");

  rt_kprintf("  app-key:    ");
  for (uint32_t i = 0; i < 16; i++) {
    rt_kprintf("%.2x", lorawan->secure_element.app_key[i]);
  }
  rt_kprintf("\n");

  rt_kprintf("  chan mask: ");
  for (uint32_t i = 0; i < lorawan->region->channels_mask_nb; i++) {
    rt_kprintf(" 0x%.4x", lorawan->region->channels_mask[i]);
  }
  rt_kprintf("\033[0m\n");

  // flash persistence init
  // activation init
  //  otaa or abp
  // other init
  //  Public Network
  //  class type
  //  adr
  //  dutycycle test
  //  MIB_SYSTEM_MAX_RX_ERROR

  // init app
  for (uint32_t i = 0; i < (sizeof(init_app_list) / sizeof(app_init_func_t)); i++) {
    if (init_app_list[i] != RT_NULL) {
      err = init_app_list[i](lorawan);
      if (err != RT_EOK) {
        LOG_E("app init %d err: %d", i, err);
        return err;
      }
    }
  }

  // route()

  err = lorawan_run(lorawan);
  if (err != RT_EOK) {
    LOG_E("lorawan run err: %d", err);
  }

  return RT_EOK;
}