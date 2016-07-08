/*
 * variant.h
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

#ifndef INC_VARIANT_H_
#define INC_VARIANT_H_

#include <chip.h>

#ifdef USE_USBSerial
#include "usbd_cdc_if.h"
#endif

#define NO_ADC 0xFFFF

#define NO_PWM 0xFFFF

#include "Arduino.h"
#ifdef __cplusplus
#include "UARTClass.h"

#ifdef USE_USBSerial
#include <USBSerial.h>
#endif

#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef USE_USART1
extern UART_HandleTypeDef huart1;
#endif

#ifdef USE_USART2
extern UART_HandleTypeDef huart2;
#endif

#ifdef USE_USART3
extern UART_HandleTypeDef huart3;
#endif

#ifdef USE_USART4
extern UART_HandleTypeDef huart4;
#endif

#ifdef USE_USBSerial
extern USBD_HandleTypeDef hUsbDeviceFS;
#endif

enum {
  PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
  PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
  PC13, PC14, PC15,
  PF0, PF1
};

/*
  This structure maps the high level settings of the port
  See the HALMX core function variant and wiring_digital
  to see how to use
*/
typedef struct _Pin2PortMapArray
{
  /*
    define the Abstract Hal port reference

    IMPORTANT NOTE: this definition is opaque and
    defined inside the chip family
    do not try and change the include name locally
    unless you know what you are doing

  */
    GPIO_TypeDef *GPIOx_Port;     /* defined in stm32f401xe.h */
    /*
      we only record the pin here as the typedef record
      is normally mutable and only used by the init code
    */
    uint32_t  Pin_abstraction;  /* must match type in GPIO_InitTypeDef struct */

    uint32_t  adc_channel;
    uint32_t  timerNumber;   //Timer1 to Timer4.
    uint32_t  timerChannel;  //Timer channel (1-4).
    uint32_t  altFunction;   //Alternate Function.
} Pin2PortMapArray;

/* Pins table to be instantiated into variant.cpp */
extern const Pin2PortMapArray g_Pin2PortMapArray[];

void Rx1_Handler(void);
void Tx1_Handler(void);
void Rx2_Handler(void);
void Tx2_Handler(void);
void Rx3_Handler(void);
void Tx3_Handler(void);
void Rx4_Handler(void);
void Tx4_Handler(void);
void StartUSBSerial(void);

TIM_HandleTypeDef * variant_get_timer_handle(uint32_t ulPin);
TIM_TypeDef * variant_get_timer_instance(uint32_t ulPin);

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

#ifdef USE_USART1
extern UARTClass Serial1;
#endif

#ifdef USE_USART2
extern UARTClass Serial2;
#endif

#ifdef USE_USART3
extern UARTClass Serial3;
#endif

#ifdef USE_USART4
extern UARTClass Serial4;
#endif

#ifdef USE_USBSerial
extern USBSerial Serial;
#endif

#endif

#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_HARDWARE_OPEN   Serial1
#define SERIAL_PORT_HARDWARE_OPEN1  Serial2
#define SERIAL_PORT_HARDWARE_OPEN2  Serial3
#define SERIAL_PORT_HARDWARE_OPEN3  Serial4
#define SERIAL_PORT_HARDWARE        Serial
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE2       Serial2
#define SERIAL_PORT_HARDWARE3       Serial3

#endif /* INC_VARIANT_H_ */
