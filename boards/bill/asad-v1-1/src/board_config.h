/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * ASAD-v1.1 board internal definitions
 * MCU: STM32H753ZIT6
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* No status LEDs on this board */

/* SPI1 CS - LoRa E22-900MM22S */

#define GPIO_SPI1_CS_LORA       /* PA4 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

/* LoRa control GPIOs */

#define GPIO_LORA_TXEN          /* PD15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)
#define GPIO_LORA_RXEN          /* PD14 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)
#define GPIO_LORA_NRST          /* PD13 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN13)
#define GPIO_LORA_BUSY          /* PC12 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTC|GPIO_PIN12)

/* INA226 alert */

#define GPIO_INA226_ALERT       /* PC9 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN9)

/* Barometer interrupt */

#define GPIO_BARO_DRDY          /* PD10 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTD|GPIO_PIN10)

/* GPS reset via TXS0102 level shifter */

#define GPIO_GPS_RESET          /* PD11 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN11)

/* PWM outputs
 *
 * 6 outputs on PD3-PD8
 * Fixed wing: motor + servos
 */

#define DIRECT_PWM_OUTPUT_CHANNELS   6
#define DIRECT_INPUT_TIMER_CHANNELS  6

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

/* USB OTG FS
 *
 *   PA11  OTG_FS_DM
 *   PA12  OTG_FS_DP
 */

/* RC Serial port - UART8 on PE0/PE1 = ttyS6 */

#define RC_SERIAL_PORT          "/dev/ttyS6"
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

/* High-resolution timer */

#define HRT_TIMER               8  /* use timer 8 for the HRT */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */

/* SD Card */

#define SDIO_SLOTNO             0
#define SDIO_MINOR              0

/* This board provides a DMA pool and APIs */

#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 2

#define PX4_GPIO_INIT_LIST { \
		GPIO_SPI1_CS_LORA, \
		GPIO_LORA_TXEN, \
		GPIO_LORA_RXEN, \
		GPIO_LORA_NRST, \
		GPIO_LORA_BUSY, \
		GPIO_INA226_ALERT, \
		GPIO_BARO_DRDY, \
		GPIO_GPS_RESET, \
	}

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

int stm32_sdio_initialize(void);

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
