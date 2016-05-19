/*
 * variant.cpp
 *
 * Created on: May 17, 2016
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

#include "variant.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const Pin2PortMapArray g_Pin2PortMapArray[]=
{

    {GPIOA, GPIO_PIN_0,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOA, GPIO_PIN_1,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOA, GPIO_PIN_2,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOA, GPIO_PIN_3,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOA, GPIO_PIN_4,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOA, GPIO_PIN_5,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOA, GPIO_PIN_6,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOA, GPIO_PIN_7,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOA, GPIO_PIN_8,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOA, GPIO_PIN_9,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOA, GPIO_PIN_10, NO_ADC, NO_PWM, NO_PWM},
    {GPIOA, GPIO_PIN_11, NO_ADC, NO_PWM, NO_PWM},
    {GPIOA, GPIO_PIN_12, NO_ADC, NO_PWM, NO_PWM},
    {GPIOA, GPIO_PIN_13, NO_ADC, NO_PWM, NO_PWM},
    {GPIOA, GPIO_PIN_14, NO_ADC, NO_PWM, NO_PWM},
    {GPIOA, GPIO_PIN_15, NO_ADC, NO_PWM, NO_PWM},

    {GPIOB, GPIO_PIN_0,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOB, GPIO_PIN_1,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOB, GPIO_PIN_2,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOB, GPIO_PIN_3,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOB, GPIO_PIN_4,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOB, GPIO_PIN_5,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOB, GPIO_PIN_6,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOB, GPIO_PIN_7,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOB, GPIO_PIN_8,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOB, GPIO_PIN_9,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOB, GPIO_PIN_10, NO_ADC, NO_PWM, NO_PWM},
    {GPIOB, GPIO_PIN_11, NO_ADC, NO_PWM, NO_PWM},
    {GPIOB, GPIO_PIN_12, NO_ADC, NO_PWM, NO_PWM},
    {GPIOB, GPIO_PIN_13, NO_ADC, NO_PWM, NO_PWM},
    {GPIOB, GPIO_PIN_14, NO_ADC, NO_PWM, NO_PWM},
    {GPIOB, GPIO_PIN_15, NO_ADC, NO_PWM, NO_PWM},

    {GPIOC, GPIO_PIN_0,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOC, GPIO_PIN_1,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOC, GPIO_PIN_2,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOC, GPIO_PIN_3,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOC, GPIO_PIN_4,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOC, GPIO_PIN_5,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOC, GPIO_PIN_6,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOC, GPIO_PIN_7,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOC, GPIO_PIN_8,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOC, GPIO_PIN_9,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOC, GPIO_PIN_10, NO_ADC, NO_PWM, NO_PWM},
    {GPIOC, GPIO_PIN_11, NO_ADC, NO_PWM, NO_PWM},
    {GPIOC, GPIO_PIN_12, NO_ADC, NO_PWM, NO_PWM},
    {GPIOC, GPIO_PIN_13, NO_ADC, NO_PWM, NO_PWM},
    {GPIOC, GPIO_PIN_14, NO_ADC, NO_PWM, NO_PWM},
    {GPIOC, GPIO_PIN_15, NO_ADC, NO_PWM, NO_PWM},

    {GPIOD, GPIO_PIN_2,  NO_ADC, NO_PWM, NO_PWM},

    {GPIOF, GPIO_PIN_0,  NO_ADC, NO_PWM, NO_PWM},
    {GPIOF, GPIO_PIN_1,  NO_ADC, NO_PWM, NO_PWM}
};

#ifdef __cplusplus
}
#endif
