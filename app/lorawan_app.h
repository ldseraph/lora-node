#ifndef __LORAWAN_APP_H__
#define __LORAWAN_APP_H__
#include <board.h>
#include <rtthread.h>

#include <lorawan.h>
rt_err_t tension_app_init(lorawan_t *);
rt_err_t tilt_app_init(lorawan_t*);

#endif