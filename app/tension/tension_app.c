#include <bq24040.h>
#include <lorawan.h>
#define DBG_SECTION_NAME "tension"
#include <ulog.h>

#define TENSION_APP_THREAD_DELAY_MS 15000
#define TENSION_APP_LED_DELAY_MS 500
#define TENSION_APP_THREAD_STACK_SIZE 2048
#define TENSION_APP_SENSOR_BQ24040_NAME "bq24040:0"
#define TENSION_APP_SENSOR_ADS1232_NAME "ads1232:0"
#define TENSION_APP_GPIO_RED "PB.1"
#define TENSION_APP_GPIO_GREEN "PB.2"

static rt_thread_t tension_app_thread;
static rt_timer_t  tension_app_timer;
static rt_sem_t    tension_app_sem;

static void tension_app_timer_handle(void* param) {
  rt_interrupt_enter();
  rt_sem_release(tension_app_sem);
  rt_interrupt_leave();
}

static void tension_app_thread_handle(void* param) {
  lorawan_t* lorawan = (lorawan_t*)param;
  rt_err_t   err;

  lorawan->exception_handle = board_panic;

  uint32_t  old_battery_value = 100;
  rt_base_t gpio_red          = rt_pin_get(TENSION_APP_GPIO_RED);
  // rt_base_t gpio_green = rt_pin_get(TENSION_APP_GPIO_GREEN);

  rt_pin_mode(gpio_red, GPIO_MODE_OUT_PP);
  // rt_pin_mode(gpio_green, GPIO_MODE_OUT_PP);
  // rt_pin_write(gpio_red, PIN_HIGH);
  rt_pin_write(gpio_red, PIN_LOW);
  rt_thread_mdelay(500);
  rt_pin_mode(gpio_red, GPIO_MODE_IN_FLOATING);

  rt_device_t sensor_bq24040 = rt_device_find(TENSION_APP_SENSOR_BQ24040_NAME);
  if (sensor_bq24040 == RT_NULL) {
    LOG_E("%s: find err.", TENSION_APP_SENSOR_BQ24040_NAME);
    return;
  }

  err = rt_device_open(sensor_bq24040, RT_DEVICE_OFLAG_RDWR);
  if (err != RT_EOK) {
    LOG_E("%s: open err: %d.", RT_DEVICE_OFLAG_RDWR, err);
    return;
  }

  rt_device_t sensor_ads1232 = rt_device_find(TENSION_APP_SENSOR_ADS1232_NAME);
  if (sensor_ads1232 == RT_NULL) {
    LOG_E("%s: find err.", TENSION_APP_SENSOR_ADS1232_NAME);
    return;
  }

  err = rt_device_open(sensor_ads1232, RT_DEVICE_OFLAG_RDWR);
  if (err != RT_EOK) {
    LOG_E("%s: open err: %d.", TENSION_APP_SENSOR_ADS1232_NAME, err);
    return;
  }

  for (;;) {
    rt_sem_take(tension_app_sem, RT_WAITING_FOREVER);

    uint32_t battery_value = 0;
    for (uint8_t i = 0; (old_battery_value > (2 + battery_value)) && i < 5; i++) {
      rt_device_read(sensor_bq24040, 0, &battery_value, sizeof(battery_value));
      if (i) {
        rt_thread_mdelay(100);
      }
    }
    old_battery_value = battery_value;

    LOG_I("%s: battery value: %d", TENSION_APP_SENSOR_BQ24040_NAME, battery_value);

    // if (battery_value < 170) {
    //   battery_value = 0;
    // } else if (battery_value > 200) {
    //   battery_value = 10;
    // } else {
    //   battery_value = (battery_value - 170) / 3;
    // }

    // rt_bool_t charge_ok = sensor_bq24040_charge_ok(sensor_bq24040);

    // if (charge_ok) {
    //   battery_value = 10;
    // }

    int32_t tension_value;
    rt_device_read(sensor_ads1232, 0, &tension_value, sizeof(tension_value));
    LOG_I("%s: tension value: 0x%x", TENSION_APP_SENSOR_ADS1232_NAME, tension_value);

    float ain       = (float)(tension_value * ((0.5 * 3300) / 0x7fffff));
    float slope     = 0.3273;
    float intercept = -0.0032;
    float pull_kg   = (float)((ain - intercept)) / (slope);

    lorawan_mb_msg_t* msg = lorawan_malloc_mb_msg(sizeof(battery_value) + sizeof(pull_kg));

    if (msg == RT_NULL) {
      LOG_E("OOM");
      continue;
    }

    uint32_t* battery = (uint32_t*)msg->buffer;
    float*    pull    = (float*)&msg->buffer[sizeof(battery_value)];

    *battery = battery_value;
    *pull    = pull_kg;

    // LOG_I("battery_value %d", battery_value);
    // rt_bool_t is_charging = sensor_bq24040_is_charging(sensor_bq24040);

    // uint32_t gpio_high;
    // if (battery_value < 4 || (is_charging && !charge_ok)) {
    // rt_pin_write(gpio_red, PIN_HIGH);
    // rt_pin_write(gpio_green, PIN_LOW);
    // gpio_high = gpio_red;
    // } else {
    // rt_pin_write(gpio_green, PIN_HIGH);
    // rt_pin_write(gpio_red, PIN_LOW);
    // gpio_high = gpio_green;
    // }

    err = lorawan_send_confirmed(lorawan, 1, msg);
    if (err != RT_EOK) {
      lorawan_free_mb_msg(msg);
      continue;
    }

    // rt_thread_mdelay(TENSION_APP_LED_DELAY_MS);

    // rt_pin_write(gpio_high, PIN_LOW);
  }

  return;
}

rt_err_t tension_app_init(lorawan_t* lorawan) {
  tension_app_sem = rt_sem_create("app:tension", 1, RT_IPC_FLAG_PRIO);
  if (tension_app_sem == RT_NULL) {
    LOG_E("tension_app_sem err");
    return -RT_ERROR;
  }

  tension_app_thread = rt_thread_create("app:tension",
                                        tension_app_thread_handle,
                                        lorawan,
                                        TENSION_APP_THREAD_STACK_SIZE,
                                        15,
                                        20);
  if (tension_app_thread == RT_NULL) {
    LOG_E("rt_thread_create err");
    return -RT_ERROR;
  }

  rt_err_t err = rt_thread_startup(tension_app_thread);
  if (err != RT_EOK) {
    LOG_E("rt_thread_startup err: %d", err);
    return err;
  }

  tension_app_timer = rt_timer_create("app:tension",
                                      tension_app_timer_handle,
                                      RT_NULL,
                                      rt_tick_from_millisecond(TENSION_APP_THREAD_DELAY_MS),
                                      RT_TIMER_FLAG_PERIODIC);
  if (tension_app_timer == RT_NULL) {
    LOG_E("tension_app_timer err: %d", err);
    return RT_ERROR;
  }

  err = rt_timer_start(tension_app_timer);
  if (err != RT_EOK) {
    LOG_E("tension_app_timer: %d", err);
    return err;
  }

  return RT_EOK;
}
