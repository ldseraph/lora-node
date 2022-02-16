#include "app.h"
#define DBG_SECTION_NAME "main"
#include <ulog.h>

int main() {
  rt_err_t err;

#ifdef RT_USING_LORAWAN
  err = lorawan_app();
  if (err != RT_EOK){
    LOG_E("lorawan app err.");
    return err;
  }
#endif

  return RT_EOK;
}
//test
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
