/*
 * chip.h
 *
 * Created on: May 24, 2016
 *     Author: Ekawahyu Susilo
 *
 * Copyright (c) 2016, Chongqing Aisenke Electronic Technology Co., Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the copyright holder.
 *
 */

#ifndef INC_CHIP_STM32F072C8_H_
#define INC_CHIP_STM32F072C8_H_

#include <stdbool.h>  /* for wiring constants */

#include "stm32f0xx_hal.h"

#define USB_LP_CAN1_RX0_IRQn USB_IRQn

/* from STM boilerplate:
      Systick timer is used by default as source of time base, but user
            can eventually implement his proper time base source (a general purpose
            timer for example or other time source), keeping in mind that Time base
            duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
            handled in milliseconds basis.
*/
/* returns System clock milliseconds HAL must be active */
#define  millis(a1) HAL_GetTick(a1)

/* from STM boilerplate:
      Care must be taken if HAL_Delay() is called from a
             peripheral ISR process, the Tick interrupt line must have higher priority
            (numerically lower) than the peripheral interrupt. Otherwise the caller
            ISR process will be blocked.
*/
/* delay in micoseconds  Uses HAL system clock */
#define delay(a2) HAL_Delay(a2)

#define CDC_SERIAL_BUFFER_SIZE  128

#define USE_USBSerial

#define USE_USART1
#define USE_USART3
#define USE_USART4

#define USE_TIMER3
#define USE_TIMER14
#define USE_TIMER16
#define USE_TIMER17

#endif /* INC_CHIP_STM32F072C8_H_ */
