/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_lpspi.h
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
 * The external functions, s32k1xx_lpspiNselect* and s32k1xx_lpspiNstatus
 * must be provided by board-specific logic.  They are implementations of the
 * select and status methods of the SPI interface defined by struct
 * s32k1xx_lpspi_ops_s (see include/nuttx/spi/spi.h).  All other methods
 * (including s32k1xx_lpspibus_initialize()) are provided by common S32K1XX
 * logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in s32k1xx_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide s32k1xx_lpspiNselect() and s32k1xx_lpspiNstatus() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to s32k1xx_lpspibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by s32k1xx_lpspibus_initialize() may then be
 *      used to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_lpspislotinitialize(), for example, will bind the SPI driver
 *      to the SPI MMC/SD driver).
 *
 * NOTE*: If CONFIG_S32K1XX_LPSPI_HWPCS is selected, s32k1xx_lpspiNselect()
 *        does NOT need to provided by board-specific logic.  In this case a
 *        generic implementation is used that switches between native
 *        hardware chip select pins.  It is important that all pins are
 *        configured when the SPI bus is initialized.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_S32K1XX_LPSPI_SLAVE_H
#define __ARCH_ARM_SRC_S32K1XX_S32K1XX_LPSPI_SLAVE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/slave.h>

#include "arm_internal.h"
#include "chip.h"

#include "s32k1xx_pin.h"
#include "hardware/s32k1xx_pinmux.h"
#include "hardware/s32k1xx_lpspi.h"
#include "s32k1xx_periphclocks.h"
#include "s32k1xx_lpspi.h"

#include <arch/board/board.h>

#ifndef CONFIG_SPI_SLAVE_QSIZE
#define CONFIG_SPI_SLAVE_QSIZE 10
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* The overall state of one SPI controller */

struct s32k1xx_lpspidev_s
{
  /* Externally visible part of the SPI slave controller interface */

  struct spi_slave_ctrlr_s ctrlr;

  /* Bound SPI slave device interface */

  struct spi_slave_dev_s *dev;
  xcpt_t handler;              /* SPI interrupt handler */
  uint32_t base;               /* SPI controller register base address */
  mutex_t spilock;             /* Assures mutually exclusive access to SPI */
  uint32_t outval;             /* Default shift-out value */
  uint16_t irq;                /* SPI IRQ number */
  uint8_t mode;                /* Mode 0,1,2,3 */
  uint16_t nbits;              /* Width of word in bits (2 to 4096) */
  uint8_t PCSpin;              /* Peripheral Chip Select Pin 0,1,2,3 */
  bool initialized;            /* True: Controller has been initialized */
  bool nss;                    /* True: Chip selected */

  /* Output queue */

  uint8_t tx_head;                /* Location of next value */
  uint8_t tx_tail;                /* Index of first value */

  uint32_t outq[CONFIG_SPI_SLAVE_QSIZE];

};

/****************************************************************************
 * Public Function Prototype
 ****************************************************************************/

struct spi_slave_ctrlr_s *s32k1xx_spi_slave_initialize(int port);

#endif  /* __ARCH_ARM_SRC_S32K1XX_S32K1XX_LPSPI_SLAVE_H */