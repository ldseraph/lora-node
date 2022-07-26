/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-01-04     iysheng           first version
 */

#include <board.h>
#include <drv_usart.h>
#include <gd32e10x.h>
#include <rthw.h>
#include <rtthread.h>
#include <stdint.h>
#define DBG_SECTION_NAME "board"
#include <ulog.h>

/*
 * System Clock Configuration
 */
void SystemClock_Config(void) {
  SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);
  NVIC_SetPriority(SysTick_IRQn, 0);
}

/*
 * This is the timer interrupt service routine.
 */
void SysTick_Handler(void) {
  /* enter interrupt */
  rt_interrupt_enter();

  rt_tick_increase();

  /* leave interrupt */
  rt_interrupt_leave();
}

/**
 * This function will initial GD32 board.
 */
void rt_hw_board_init() {
  /* NVIC Configuration */
#define NVIC_VTOR_MASK 0x3FFFFF80
#ifdef VECT_TAB_RAM
  /* Set the Vector Table base location at 0x10000000 */
  SCB->VTOR = (0x10000000 & NVIC_VTOR_MASK);
#else /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */
  SCB->VTOR = (0x08000000 & NVIC_VTOR_MASK);
#endif

  SystemClock_Config();
  rt_system_heap_init((void*)HEAP_BEGIN, (void*)HEAP_END);

#ifdef RT_USING_COMPONENTS_INIT
  rt_components_board_init();
#endif

#ifdef RT_USING_CONSOLE
  rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

  fwdgt_write_enable();
  fwdgt_config(0xFFFF, FWDGT_PSC_DIV256);
  fwdgt_write_disable();
  fwdgt_counter_reload();
  fwdgt_enable();
}

static rt_bool_t panic = RT_FALSE;

void board_fwd_reload() {
  if (!panic) {
    fwdgt_counter_reload();
  }
}

void board_panic() {
  panic = RT_TRUE;
}

#define ID1 (0x1FFFF7E8)
#define ID2 (0x1FFFF7EC)
#define ID3 (0x1FFFF7F0)
#define FLASH_ID (0x0801FC00)
#define FLASH_MAGIC_ID (0x13378868)

void board_write_unique_ID(uint8_t* id) {
  uint32_t* buf = (uint32_t*)id;

  fmc_unlock();
  fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
  fmc_page_erase(FLASH_ID);
  fmc_word_program(FLASH_ID, FLASH_MAGIC_ID);
  fmc_word_program(FLASH_ID + 4, buf[0]);
  fmc_word_program(FLASH_ID + 8, buf[1]);
  fmc_lock();
}

void board_get_unique_ID(uint8_t* id) {
  uint32_t flash_magic_id = *(uint32_t*)FLASH_ID;

  if (flash_magic_id == FLASH_MAGIC_ID) {
    for (uint32_t i = 0; i < 8; i++) {
      id[i] = *(uint8_t*)(FLASH_ID + 4 + i);
    }
    return;
  }

  id[7] = ((*(uint32_t*)ID1) + (*(uint32_t*)ID3)) >> 24;
  id[6] = ((*(uint32_t*)ID1) + (*(uint32_t*)ID3)) >> 16;
  id[5] = ((*(uint32_t*)ID1) + (*(uint32_t*)ID3)) >> 8;
  id[4] = ((*(uint32_t*)ID1) + (*(uint32_t*)ID3));
  id[3] = ((*(uint32_t*)ID2)) >> 24;
  id[2] = ((*(uint32_t*)ID2)) >> 16;
  id[1] = ((*(uint32_t*)ID2)) >> 8;
  id[0] = ((*(uint32_t*)ID2));
}

void board_delay_us(uint32_t us) {
  rt_uint32_t ticks;
  rt_uint32_t told, tnow, tcnt = 0;
  rt_uint32_t reload = SysTick->LOAD;

  ticks = us * (SystemCoreClock / 1000000);
  told  = SysTick->VAL;
  for (;;) {
    tnow = SysTick->VAL;
    if (tnow != told) {
      if (tnow < told) {
        tcnt += told - tnow;
      } else {
        tcnt += reload - tnow + told;
      }
      told = tnow;
      if (tcnt >= ticks) {
        break;
      }
    }
  }
}
