/****************************************************************************
 * boards/arm/s32k1xx/s32k146evb/src/s32k146evb.h
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

#ifndef __BOARDS_ARM_S32K1XX_S32K146EVB_SRC_S32K146EVB_H
#define __BOARDS_ARM_S32K1XX_S32K146EVB_SRC_S32K146EVB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include "hardware/s32k1xx_pinmux.h"
#include "s32k1xx_periphclocks.h"
#include "s32k1xx_pin.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* S32K146EVB GPIOs *********************************************************/

/* LEDs.  The S32K146EVB has one RGB LED:
 *
 *   RedLED    PTD15  (FTM0 CH0)
 *   GreenLED  PTD16  (FTM0 CH1)
 *   BlueLED   PTD0   (FTM0 CH2)
 *
 * An output of '1' illuminates the LED.
 */

#define GPIO_LED_R  (PIN_PTD15 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)
#define GPIO_LED_G  (PIN_PTD16 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)
#define GPIO_LED_B  (PIN_PTD0  | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)

/* Buttons.  The S32K146EVB supports two buttons:
 *
 *   SW2  PTC12
 *   SW3  PTC13
 */

#define GPIO_SW2    (PIN_PTC12 | PIN_INT_BOTH)
#define GPIO_SW3    (PIN_PTC13 | PIN_INT_BOTH)

/* VNQ9080AJ High-Side Driver
 *
 * CONTACT1             PTD18   control input pin
 * CONTACT2             PTD19
 * CONTACT3             PTD22
 * CONTACT4             PTD23
 *
 * VNQ_SEN              PTD24   enable CS diagnostic
 * VNQ_FAULT_RST        PTD27   auto restart mode active low
 * VNQ_SEL0             PTD28   CS multiplexer
 * VNQ_CS               PTD29   input pin
 * VNQ_SEL1             PTD30
 */
#define PIN_CONF_CONTACT1        (PIN_PTD18 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO)
#define PIN_CONF_CONTACT2        (PIN_PTD19 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO)
#define PIN_CONF_CONTACT3        (PIN_PTD22 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO)
#define PIN_CONF_CONTACT4        (PIN_PTD23 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO)

#define PIN_CONF_VNQ_SEN         (PIN_PTD24 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO)
#define PIN_CONF_VNQ_FAULT_RST   (PIN_PTD27 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)
#define PIN_CONF_VNQ_SEL0        (PIN_PTD28 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO)
#define PIN_CONF_VNQ_CS          (PIN_PTD29 | GPIO_INPUT)
#define PIN_CONF_VNQ_SEL1        (PIN_PTD30 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO)

/* UCC21750DWR IGBT driver 
 * 
 * UCC_INP      PTE20   Control pin
 * UCC_INN      PTE21   OUTPUT always LOW
 * UCC_RDY      PTE22   INPUT active HIGH
 * UCC_FLT      PTE23   INPUT active low
 * UCC_EN       PTE24   OUTPUT (RST)
 */
#define PIN_CONF_UCC_INP        (PIN_PTE20 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO)
#define PIN_CONF_UCC_INN        (PIN_PTE21 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO)
#define PIN_CONF_UCC_RDY        (PIN_PTE22 | GPIO_INPUT)
#define PIN_CONF_UCC_FLT        (PIN_PTE23 | GPIO_INPUT)
#define PIN_CONF_UCC_EN         (PIN_PTE24 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)

/* MC33664 Isolated network high-speed transceiver for BCC
 * BCC_INT      PTB12   Input interrupt
 * BCC_EN       PTE8    Output chip enable
 */
#define PIN_BCC_EN              (PIN_PTE8  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO)
#define PIN_BCC_INT             (PIN_PTB12 | GPIO_INPUT)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* User peripheral configuration structure 0 */

extern const struct peripheral_clock_config_s g_peripheral_clockconfig0[];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int s32k1xx_bringup(void);

/****************************************************************************
 * Name: s32k1xx_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int s32k1xx_gpio_initialize(void);

/****************************************************************************
 * Name: s32k1xx_i2cdev_initialize
 *
 * Description:
 *   Initialize I2C driver and register /dev/i2cN devices.
 *
 ****************************************************************************/

int s32k1xx_i2cdev_initialize(void);

/****************************************************************************
 * Name: s32k1xx_spidev_initialize
 *
 * Description:
 *   Configure chip select pins, initialize the SPI driver and register
 *   /dev/spiN devices.
 *
 ****************************************************************************/

int s32k1xx_spidev_initialize(void);


int s32k1xx_spislavedev_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_S32K1XX_S32K146EVB_SRC_S32K146EVB_H */
