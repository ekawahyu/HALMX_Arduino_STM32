/*
 * variant.cpp
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

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const Pin2PortMapArray g_Pin2PortMapArray[]=
{
    {GPIOA, GPIO_PIN_0,  NO_ADC, NO_PWM, NO_PWM, NO_PWM},        /* PA0 */
    {GPIOA, GPIO_PIN_1,  NO_ADC, 2,      TIM_CHANNEL_2, NO_PWM}, /* PA1 */
    {GPIOA, GPIO_PIN_2,  NO_ADC, 2,      TIM_CHANNEL_3, NO_PWM}, /* PA2 */
    {GPIOA, GPIO_PIN_3,  NO_ADC, 2,      TIM_CHANNEL_4, NO_PWM}, /* PA3 */
    {GPIOA, GPIO_PIN_4,  NO_ADC, 14,     TIM_CHANNEL_1, NO_PWM}, /* PA4 */
    {GPIOA, GPIO_PIN_5,  NO_ADC, NO_PWM, NO_PWM,        NO_PWM}, /* PA5 */
    {GPIOA, GPIO_PIN_6,  NO_ADC, 3,      TIM_CHANNEL_1, NO_PWM}, /* PA6 */
    {GPIOA, GPIO_PIN_7,  NO_ADC, 3,      TIM_CHANNEL_2, NO_PWM}, /* PA7 */
    {GPIOA, GPIO_PIN_8,  NO_ADC, 1,      TIM_CHANNEL_1, NO_PWM}, /* PA8 */
    {GPIOA, GPIO_PIN_9,  NO_ADC, 1,      TIM_CHANNEL_2, NO_PWM}, /* PA9 */
    {GPIOA, GPIO_PIN_10, NO_ADC, 1,      TIM_CHANNEL_3, NO_PWM}, /* PA10 */
    {GPIOA, GPIO_PIN_11, NO_ADC, 1,      TIM_CHANNEL_4, NO_PWM}, /* PA11 */
    {GPIOA, GPIO_PIN_12, NO_ADC, NO_PWM, NO_PWM,        NO_PWM}, /* PA12 */
    {GPIOA, GPIO_PIN_13, NO_ADC, NO_PWM, NO_PWM,        NO_PWM}, /* PA13 */
    {GPIOA, GPIO_PIN_14, NO_ADC, NO_PWM, NO_PWM,        NO_PWM}, /* PA14 */
    {GPIOA, GPIO_PIN_15, NO_ADC, NO_PWM, NO_PWM,        NO_PWM}, /* PA15 */

    {GPIOB, GPIO_PIN_0,  NO_ADC, 3,      TIM_CHANNEL_3, GPIO_AF1_TIM3},  /* PB0 */
    {GPIOB, GPIO_PIN_1,  NO_ADC, 3,      TIM_CHANNEL_4, GPIO_AF1_TIM3},  /* PB1 */
    {GPIOB, GPIO_PIN_2,  NO_ADC, NO_PWM, NO_PWM,        NO_PWM},         /* PB2 */
    {GPIOB, GPIO_PIN_3,  NO_ADC, 2,      TIM_CHANNEL_2, NO_PWM},         /* PB3 */
    {GPIOB, GPIO_PIN_4,  NO_ADC, 3,      TIM_CHANNEL_1, NO_PWM},         /* PB4 */
    {GPIOB, GPIO_PIN_5,  NO_ADC, 3,      TIM_CHANNEL_2, NO_PWM},         /* PB5 */
    {GPIOB, GPIO_PIN_6,  NO_ADC, NO_PWM, NO_PWM,        NO_PWM},         /* PB6 */
    {GPIOB, GPIO_PIN_7,  NO_ADC, NO_PWM, NO_PWM,        NO_PWM},         /* PB7 */
    {GPIOB, GPIO_PIN_8,  NO_ADC, 16,     TIM_CHANNEL_1, GPIO_AF2_TIM16}, /* PB8 */
    {GPIOB, GPIO_PIN_9,  NO_ADC, 17,     TIM_CHANNEL_1, GPIO_AF2_TIM17}, /* PB9 */
    {GPIOB, GPIO_PIN_10, NO_ADC, 2,      TIM_CHANNEL_3, NO_PWM},         /* PB10 */
    {GPIOB, GPIO_PIN_11, NO_ADC, 2,      TIM_CHANNEL_4, NO_PWM},         /* PB11 */
    {GPIOB, GPIO_PIN_12, NO_ADC, NO_PWM, NO_PWM,        NO_PWM},         /* PB12 */
    {GPIOB, GPIO_PIN_13, NO_ADC, NO_PWM, NO_PWM,        NO_PWM},         /* PB13 */
    {GPIOB, GPIO_PIN_14, NO_ADC, 15,     TIM_CHANNEL_1, NO_PWM},         /* PB14 */
    {GPIOB, GPIO_PIN_15, NO_ADC, 15,     TIM_CHANNEL_2, NO_PWM},         /* PB15 */

    {GPIOC, GPIO_PIN_13, NO_ADC, NO_PWM, NO_PWM,        NO_PWM},         /* PC13 */
    {GPIOC, GPIO_PIN_14, NO_ADC, NO_PWM, NO_PWM,        NO_PWM},         /* PC14 */
    {GPIOC, GPIO_PIN_15, NO_ADC, NO_PWM, NO_PWM,        NO_PWM},         /* PC15 */

    {GPIOF, GPIO_PIN_0,  NO_ADC, NO_PWM, NO_PWM,        NO_PWM},         /* PF0 */
    {GPIOF, GPIO_PIN_1,  NO_ADC, NO_PWM, NO_PWM,        NO_PWM}          /* PF1 */
};

#ifdef __cplusplus
}
#endif

/* ----------------------------------------------------------------------------
 *     USART objects
 * ----------------------------------------------------------------------------*/

#ifdef USE_USART1
UART_HandleTypeDef huart1;
UARTClass Serial1(&huart1, USART1_IRQn, 0, USART1);
void Tx1_Handler(void){
  Serial1.TxHandler();
}
void Rx1_Handler(void){
  Serial1.RxHandler();
}
#endif

#ifdef USE_USART2
UART_HandleTypeDef huart2;
UARTClass Serial2(&huart2, USART2_IRQn, 1, USART2);
void Tx2_Handler(void){
  Serial2.TxHandler();
}
void Rx2_Handler(void){
  Serial2.RxHandler();
}
#endif

#ifdef USE_USART3
UART_HandleTypeDef huart3;
UARTClass Serial3(&huart3, USART3_4_IRQn, 2, USART3);
void Tx3_Handler(void){
  Serial3.TxHandler();
}
void Rx3_Handler(void){
  Serial3.RxHandler();
}
#endif

#ifdef USE_USART4
UART_HandleTypeDef huart4;
UARTClass Serial4(&huart4, USART3_4_IRQn, 3, USART4);
void Tx4_Handler(void){
  Serial4.TxHandler();
}
void Rx4_Handler(void){
  Serial4.RxHandler();
}
#endif

#ifdef USE_USBSerial
USBSerial Serial;
void StartUSBSerial (void){
  Serial.begin(9600);
}
#endif

#ifdef USE_SPI1
SPI_HandleTypeDef hspi1;
#endif

#ifdef USE_TIMER3
TIM_HandleTypeDef htim3;
#endif

TIM_HandleTypeDef * variant_get_timer_handle(uint32_t ulPin)
{
  TIM_HandleTypeDef * htim;

  switch(g_Pin2PortMapArray[ulPin].timerNumber) {
#ifdef USE_TIMER1
    case 1:
      htim = &htim1;
    break;
#endif
#ifdef USE_TIMER2
    case 2:
      htim = &htim2;
    break;
#endif
#ifdef USE_TIMER3
    case 3:
      htim = &htim3;
    break;
#endif
#ifdef USE_TIMER4
    case 4:
      htim = &htim4;
    break;
#endif
#ifdef USE_TIMER16
    case 16:
      htim = &htim16;
    break;
#endif
#ifdef USE_TIMER17
    case 17:
      htim = &htim17;
    break;
#endif
    default:
      htim = NULL;
    break;
  }

  return htim;
}

TIM_TypeDef * variant_get_timer_instance(uint32_t ulPin)
{
  TIM_TypeDef * instance;

  switch(g_Pin2PortMapArray[ulPin].timerNumber) {
#ifdef USE_TIMER1
    case 1:
      instance = TIM1;
    break;
#endif
#ifdef USE_TIMER2
    case 2:
      instance = TIM2;
    break;
#endif
#ifdef USE_TIMER3
    case 3:
      instance = TIM3;
    break;
#endif
#ifdef USE_TIMER4
    case 4:
      instance = TIM4;
    break;
#endif
#ifdef USE_TIMER16
    case 16:
      instance = TIM16;
    break;
#endif
#ifdef USE_TIMER17
    case 17:
      instance = TIM17;
    break;
#endif
    default:
      instance = NULL;
    break;
  }

  return instance;
}
