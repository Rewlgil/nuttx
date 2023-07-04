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
 * IN0          PTB5    control input pin
 * IN1          PTB4
 * IN2          PTB3
 * IN3          PTB2
 * 
 * FaultRST     PTA3    auto restart mode active low
 *  
 * SEn          PTB10   enable CS diagnostic
 * SEL0         PTD13   CS multiplexer
 * SEL1         PTD14
 * CS           PTB0    input pin
 *  
 * VBAT_ADC     PTC0    input pin
 *  
 * In_PullUP    PTB11   output pullup (unused)
 */

#define gpio0       (PIN_PTB5  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO)
#define gpio1       (PIN_PTB4  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO)
#define gpio2       (PIN_PTB3  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO)
#define gpio3       (PIN_PTB2  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO)

#define gpio4       (PIN_PTA3  | GPIO_OUTPUT | GPIO_OUTPUT_ONE)

// #define gpio5       (PIN_PTB10 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO)
// #define gpio6       (PIN_PTD13 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO)
// #define gpio7       (PIN_PTD14 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO)
// #define gpio8       (PIN_PTB0  | GPIO_INPUT)

// #define gpio9       (PIN_PTC0  | PIN_ANALOG)

// #define gpio10      (PIN_PTB11 | GPIO_HIGHDRIVE | GPIO_OUTPUT_ZERO)

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


int s32k1xx_spi_slave_dev_initialize(int bus);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_S32K1XX_S32K146EVB_SRC_S32K146EVB_H */
