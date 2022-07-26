#include "app.h"
#define DBG_SECTION_NAME "main"
#include <ulog.h>

#define WDT_THREAD_STACK_SIZE 1024
#define WDT_THREAD_DELAY_MS 10000

static rt_thread_t wdt_thread;
static rt_timer_t  wdt_timer;
static rt_sem_t    wdt_sem;

static void tension_app_thread_handle(void* param) {
  for (;;) {
    rt_sem_take(wdt_sem, RT_WAITING_FOREVER);
    LOG_I("board reload");
    board_fwd_reload();
  }
  return;
}

static void wdt_timer_handle(void* param) {
  rt_interrupt_enter();
  rt_sem_release(wdt_sem);
  rt_interrupt_leave();
}

int main() {
  rt_err_t err;

  wdt_sem = rt_sem_create("wdt", 1, RT_IPC_FLAG_PRIO);
  if (wdt_sem == RT_NULL) {
    LOG_E("wdt_sem err");
    return -RT_ERROR;
  }

  wdt_thread = rt_thread_create("wdt",
                                tension_app_thread_handle,
                                RT_NULL,
                                WDT_THREAD_STACK_SIZE,
                                30,
                                20);
  if (wdt_thread == RT_NULL) {
    LOG_E("rt_thread_create err");
    return -RT_ERROR;
  }

  err = rt_thread_startup(wdt_thread);
  if (err != RT_EOK) {
    LOG_E("rt_thread_startup err: %d", err);
    return err;
  }

  wdt_timer = rt_timer_create("wdt",
                              wdt_timer_handle,
                              RT_NULL,
                              rt_tick_from_millisecond(WDT_THREAD_DELAY_MS),
                              RT_TIMER_FLAG_PERIODIC);
  if (wdt_timer == RT_NULL) {
    LOG_E("wdt_timer err: %d", err);
    return RT_ERROR;
  }

  err = rt_timer_start(wdt_timer);
  if (err != RT_EOK) {
    LOG_E("wdt_timer: %d", err);
    return err;
  }

#ifdef RT_USING_LORAWAN
  err = lorawan_app();
  if (err != RT_EOK){
    LOG_E("lorawan app err.");
    return err;
  }
#endif

  return RT_EOK;
}

static void show_id() {
  uint8_t devid[8];
  board_get_unique_ID(devid);
  for (uint32_t i = 0; i < 8; i++) {
    rt_kprintf("0x%.2x ", devid[i]);
  }
  rt_kprintf("\n");
}

static int devid(int argc, char **argv) {
  uint8_t devid[8];
  if (argc == 9) {
    for (uint32_t i = 0; i < 8; i++) {
      int name_len = rt_strlen(argv[i + 1]);
      if (name_len != 4) {
        goto show_id_handle;
      }

      if ((argv[i + 1][0] != '0') || (argv[i + 1][1] != 'x')) {
        goto show_id_handle;
      }

      int       hexadecimal = 0;
      int       finalNumber = 0;
      rt_bool_t flag        = RT_TRUE;
      for (uint32_t m = 0; flag == RT_TRUE && m < 2; m++) {
        if (argv[i + 1][m + 2] >= '0' && argv[i + 1][m + 2] <= '9') {
          hexadecimal = argv[i + 1][m + 2] - '0';
        } else if (argv[i + 1][m + 2] >= 'a' && argv[i + 1][m + 2] <= 'f') {
          hexadecimal = argv[i + 1][m + 2] - 'a' + 10;
        } else if (argv[i + 1][m + 2] >= 'A' && argv[i + 1][m + 2] <= 'F') {
          hexadecimal = argv[i + 1][m + 2] - 'A' + 10;
        } else {
          flag = RT_FALSE;
        }
        if (flag == RT_TRUE) {
          finalNumber = 16 * finalNumber + hexadecimal;
        }
      }
      devid[i] = finalNumber;
    }
    board_write_unique_ID(devid);
    rt_kprintf("board write id\n");
    // return 0;
  }

show_id_handle:
  show_id();

  return 0;
}
MSH_CMD_EXPORT(devid, write dev ID);
