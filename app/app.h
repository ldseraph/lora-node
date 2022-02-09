#ifndef __APP_H__
#define __APP_H__
#include <board.h>
#include <rtthread.h>

#ifdef RT_USING_LORAWAN
rt_err_t lorawan_app();
#endif

#endif