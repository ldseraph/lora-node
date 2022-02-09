/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-09     shelton      first version
 */

#ifndef __DRV_SPI__
#define __DRV_SPI__

#include <device-tree.h>
#include <drivers/spi.h>
#include <drivers/pin.h>
#include <rtthread.h>

#include <gd32e10x.h>

typedef struct {
  struct rt_spi_bus parent;
  spi_bus_device_t* device;
} spi_bus_t;

rt_err_t rt_hw_spi_device_attach(const char *, const char *, const char*);

#endif  // __DRV_SPI__
