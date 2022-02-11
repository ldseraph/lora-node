#include <bq24040.h>
#include <lorawan.h>
#define DBG_SECTION_NAME "tilt"
#include <ulog.h>

#define TILT_APP_THREAD_DELAY_MS 10000
#define TILT_APP_LED_DELAY_MS 500
#define TILT_APP_THREAD_STACK_SIZE 2048
#define TILT_APP_SENSOR_BQ24040_NAME "bq24040:0"
#define TILT_APP_GPIO_POWER_UP "PB.1"
#define TILT_APP_GPIO_POWER_DOWN "PB.2"
#define TILT_APP_GPIO_RED "PA.11"

static rt_thread_t tilt_app_thread;
static rt_timer_t  tilt_app_timer;
static rt_sem_t    tilt_app_sem;

static void tilt_app_timer_handle(void* param) {
  rt_interrupt_enter();
  rt_sem_release(tilt_app_sem);
  rt_interrupt_leave();
}

static void tilt_app_power_up() {
  rt_base_t gpio_power = rt_pin_get(TILT_APP_GPIO_POWER_UP);
  rt_pin_mode(gpio_power, GPIO_MODE_OUT_PP);
  rt_pin_write(gpio_power, PIN_HIGH);
}

static void tilt_app_power_down(void* args) {
  rt_base_t gpio_power = rt_pin_get(TILT_APP_GPIO_POWER_UP);
  rt_pin_write(gpio_power, PIN_LOW);
}

static rt_err_t tilt_app_power_down_init() {
  rt_base_t gpio_down = rt_pin_get(TILT_APP_GPIO_POWER_DOWN);
  rt_err_t  err       = rt_pin_attach_irq(
    gpio_down,
    PIN_IRQ_MODE_FALLING,
    tilt_app_power_down,
    RT_NULL);
  if (err != RT_EOK) {
    return err;
  }
  err = rt_pin_irq_enable(gpio_down,PIN_IRQ_ENABLE);
  if (err != RT_EOK){
    return err;
  }
  return err;
}

static void tilt_app_thread_handle(void* param) {
  lorawan_t* lorawan = (lorawan_t*)param;
  rt_err_t   err;

  tilt_app_power_up();
  err = tilt_app_power_down_init();
  if (err != RT_EOK) {
    LOG_E("%s: power down init err.", TILT_APP_SENSOR_BQ24040_NAME);
    return err;
  }

  // rt_base_t gpio_red = rt_pin_get(TILT_APP_GPIO_RED);
  // rt_pin_mode(gpio_red, GPIO_MODE_OUT_PP);
  // rt_pin_write(gpio_red, PIN_LOW);

  rt_device_t sensor_bq24040 = rt_device_find(TILT_APP_SENSOR_BQ24040_NAME);
  if (sensor_bq24040 == RT_NULL) {
    LOG_E("%s: find err.", TILT_APP_SENSOR_BQ24040_NAME);
    return;
  }

  err = rt_device_open(sensor_bq24040, RT_DEVICE_OFLAG_RDWR);
  if (err != RT_EOK) {
    LOG_E("%s: open err: %d.", RT_DEVICE_OFLAG_RDWR, err);
    return;
  }

  for (;;) {
    rt_sem_take(tilt_app_sem, RT_WAITING_FOREVER);
    // rt_pin_write(gpio_red, PIN_LOW);

    uint32_t battery_value;
    rt_device_read(sensor_bq24040, 0, &battery_value, sizeof(battery_value));

    lorawan_mb_msg_t* msg = lorawan_malloc_mb_msg(sizeof(battery_value));

    uint32_t* battery = (uint32_t*)msg->buffer;

    *battery = battery_value;

    // LOG_I("battery_value %d", battery_value);

    err = lorawan_send_confirmed(lorawan, 1, msg);
    if (err != RT_EOK) {
      lorawan_free_mb_msg(msg);
      continue;
    }

    rt_thread_mdelay(TILT_APP_LED_DELAY_MS);

    // rt_pin_write(gpio_red, PIN_HIGH);
  }

  return;
}

rt_err_t tilt_app_init(lorawan_t* lorawan) {
  tilt_app_sem = rt_sem_create("app:tilt", 1, RT_IPC_FLAG_PRIO);
  if (tilt_app_sem == RT_NULL) {
    LOG_E("tilt_app_sem err");
    return -RT_ERROR;
  }

  tilt_app_thread = rt_thread_create("app:tilt",
                                     tilt_app_thread_handle,
                                     lorawan,
                                     TILT_APP_THREAD_STACK_SIZE,
                                     15,
                                     20);
  if (tilt_app_thread == RT_NULL) {
    LOG_E("rt_thread_create err");
    return -RT_ERROR;
  }

  rt_err_t err = rt_thread_startup(tilt_app_thread);
  if (err != RT_EOK) {
    LOG_E("rt_thread_startup err: %d", err);
    return err;
  }

  tilt_app_timer = rt_timer_create("app:tilt",
                                   tilt_app_timer_handle,
                                   RT_NULL,
                                   rt_tick_from_millisecond(TILT_APP_THREAD_DELAY_MS),
                                   RT_TIMER_FLAG_PERIODIC);
  if (tilt_app_timer == RT_NULL) {
    LOG_E("tilt_app_timer err: %d", err);
    return RT_ERROR;
  }

  err = rt_timer_start(tilt_app_timer);
  if (err != RT_EOK) {
    LOG_E("tilt_app_timer: %d", err);
    return err;
  }

  return RT_EOK;
}
