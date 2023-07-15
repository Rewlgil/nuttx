/****************************************************************************
 * boards/arm/s32k1xx/s32k146evb/src/s32k1xx_spislavedev.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

#include "s32k1xx_pin.h"
#include "s32k1xx_lpspi.h"
#include "s32k1xx_lpspi_slave.h"

#include <arch/board/board.h>

#include "s32k146evb.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_spislavedev_initialize
 *
 * Description:
 *   Initialize SPI Slave driver and register the /dev/spislv device.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int s32k1xx_spislavedev_initialize(void)
{
  int ret = -ENODEV;

#ifdef CONFIG_S32K1XX_LPSPI0_SLAVE
  struct spi_slave_ctrlr_s *ctrlr0;

  spiinfo("Initializing /dev/spislv%d...\n", 0);

  /* Initialize SPI Slave controller device */

  ctrlr0 = s32k1xx_spi_slave_initialize(0);
  if (ctrlr0 == NULL)
    {
      spierr("Failed to initialize SPI%d as slave.\n", 0);
      return -ENODEV;
    }

  #ifdef CONFIG_SPI_SLAVE_DRIVER

  ret = spi_slave_register(ctrlr0, 0);
  if (ret < 0)
    {
      spierr("Failed to register /dev/spislv%d: %d\n", 0, ret);
    }

  #endif /* CONFIG_SPI_SLAVE_DRIVER */

#endif /* CONFIG_S32K1XX_LPSPI0_SLAVE */

#ifdef CONFIG_S32K1XX_LPSPI1_SLAVE

  struct spi_slave_ctrlr_s *ctrlr1;

  spiinfo("Initializing /dev/spislv%d...\n", 1);

  /* Initialize SPI Slave controller device */

  ctrlr1 = s32k1xx_spi_slave_initialize(1);
  if (ctrlr1 == NULL)
    {
      spierr("Failed to initialize SPI%d as slave.\n", 1);
      return -ENODEV;
    }

  #ifdef CONFIG_SPI_SLAVE_DRIVER

  ret = spi_slave_register(ctrlr1, 1);
  if (ret < 0)
    {
      spierr("Failed to register /dev/spislv%d: %d\n", 1, ret);
    }

  #endif /* CONFIG_SPI_SLAVE_DRIVER */

#endif /* CONFIG_S32K1XX_LPSPI1_SLAVE */

#ifdef CONFIG_S32K1XX_LPSPI2_SLAVE

  struct spi_slave_ctrlr_s *ctrlr2;

  spiinfo("Initializing /dev/spislv%d...\n", 2);

  /* Initialize SPI Slave controller device */

  ctrlr2 = s32k1xx_spi_slave_initialize(2);
  if (ctrlr2 == NULL)
    {
      spierr("Failed to initialize SPI%d as slave.\n", 2);
      return -ENODEV;
    }

  #ifdef CONFIG_SPI_SLAVE_DRIVER

  ret = spi_slave_register(ctrlr2, 2);
  if (ret < 0)
    {
      spierr("Failed to register /dev/spislv%d: %d\n", 2, ret);
    }

  #endif /* CONFIG_SPI_SLAVE_DRIVER */

#endif /* CONFIG_S32K1XX_LPSPI2_SLAVE */

  return ret;
}
