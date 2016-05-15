/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  - 08 April 2016 Modified by Vassilis Serasidis
    This file is converted for using it with ST HAL + CubeMX + Arduino SAM core files.
*/

#include "arduino.h"
//#include "tim.h"
//#include "variant.h"


#ifdef __cplusplus
extern "C" {
#endif

/* Quick and dirty table based on Blue pill board schematic */

extern const Pin2PortMapArray g_Pin2PortMapArray[]=
{    

    {GPIOA, GPIO_PIN_0,  ADC_CHANNEL_0, hTimer2,  TIM_CHANNEL_1 },  /*-WKUP  */
    {GPIOA, GPIO_PIN_1,  ADC_CHANNEL_1, hTimer2,  TIM_CHANNEL_2 },  /*  */
    {GPIOA, GPIO_PIN_2,  ADC_CHANNEL_2, hTimer2,  TIM_CHANNEL_3 },  /*  */
    {GPIOA, GPIO_PIN_3,  ADC_CHANNEL_3, hTimer2,  TIM_CHANNEL_4 },  /*  */
    {GPIOA, GPIO_PIN_4,  ADC_CHANNEL_4, NO_PWM,   NO_PWM        },  /*SPI1_NSS  SPI1_NSS (opt)*/
    {GPIOA, GPIO_PIN_5,  ADC_CHANNEL_5, NO_PWM,   NO_PWM        },  /*SPI1_SCK  */
    {GPIOA, GPIO_PIN_6,  ADC_CHANNEL_6, NO_PWM,   NO_PWM        },  /*SPI1_MISO  */
    {GPIOA, GPIO_PIN_7,  ADC_CHANNEL_7, NO_PWM,   NO_PWM        },  /*SPI1_MOSI  */
    {GPIOA, GPIO_PIN_8,  NO_ADC       , hTimer1,  TIM_CHANNEL_1 },  /*  */
    {GPIOA, GPIO_PIN_9,  NO_ADC       , hTimer1,  TIM_CHANNEL_2 },  /*USART1_TX  */
    {GPIOA, GPIO_PIN_10, NO_ADC       , hTimer1,  TIM_CHANNEL_3 },  /*USART1_RX  */
    {GPIOA, GPIO_PIN_11, NO_ADC       , hTimer1,  TIM_CHANNEL_4 },  /* USBDM (-)  */
    {GPIOA, GPIO_PIN_12, NO_ADC       , NO_PWM,   NO_PWM        },  /* USBDP (+)  */
    {GPIOA, GPIO_PIN_13, NO_ADC       , NO_PWM,   NO_PWM        },  /*SYS_JTMS-SWDIO  */
    {GPIOA, GPIO_PIN_14, NO_ADC       , NO_PWM,   NO_PWM        },  /*SYS_JTCK-SWCLK  */
    {GPIOA, GPIO_PIN_15, NO_ADC       , NO_PWM,   NO_PWM        },  /*  */
    
    {GPIOB, GPIO_PIN_0,  ADC_CHANNEL_8, hTimer3,  TIM_CHANNEL_3 },  /*  */
    {GPIOB, GPIO_PIN_1,  ADC_CHANNEL_9, hTimer3,  TIM_CHANNEL_4 },  /*Output  GPIO_Output  Blue_LED*/
    {GPIOB, GPIO_PIN_2,  NO_ADC       , NO_PWM,   NO_PWM        },  /* BOOT1 */
    {GPIOB, GPIO_PIN_3,  NO_ADC       , NO_PWM,   NO_PWM        },  /*  */
    {GPIOB, GPIO_PIN_4,  NO_ADC       , hTimer3,  TIM_CHANNEL_1 },  /*  */
    {GPIOB, GPIO_PIN_5,  NO_ADC       , hTimer3,  TIM_CHANNEL_2 },  /*  */
    {GPIOB, GPIO_PIN_6,  NO_ADC       , hTimer4,  TIM_CHANNEL_1 },  /*I2C1_SCL  */
    {GPIOB, GPIO_PIN_7,  NO_ADC       , hTimer4,  TIM_CHANNEL_2 },  /*I2C1_SDA  */
    {GPIOB, GPIO_PIN_8,  NO_ADC       , hTimer4,  TIM_CHANNEL_3 },  /*CAN_RX  */
    {GPIOB, GPIO_PIN_9,  NO_ADC       , hTimer4,  TIM_CHANNEL_4 },  /*CAN_TX  */
    {GPIOB, GPIO_PIN_10, NO_ADC       , NO_PWM,   NO_PWM        },  /* USART3_TX  */
    {GPIOB, GPIO_PIN_11, NO_ADC       , NO_PWM,   NO_PWM        },  /* USART3_RX  */
    {GPIOB, GPIO_PIN_12, NO_ADC       , NO_PWM,   NO_PWM        },  /* SPI2_NSS  */
    {GPIOB, GPIO_PIN_13, NO_ADC       , NO_PWM,   NO_PWM        },  /* SPI2_SCK */
    {GPIOB, GPIO_PIN_14, NO_ADC       , NO_PWM,   NO_PWM        },  /* SPI2_MISO */
    {GPIOB, GPIO_PIN_15, NO_ADC       , NO_PWM,   NO_PWM        },  /* SPI2_MOSI */
    
    {GPIOC, GPIO_PIN_13, NO_ADC       , NO_PWM,   NO_PWM        },  /*-TAMPER-RTC  Output  GPIO_Output  */
    {GPIOC, GPIO_PIN_14, NO_ADC       , NO_PWM,   NO_PWM        },  /*-OSC32_IN    */
    {GPIOC, GPIO_PIN_15, NO_ADC       , NO_PWM,   NO_PWM        }   /*-OSC32_OUT  */
};


#ifdef __cplusplus
}
#endif

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/* ----------------------------------------------------------------------------
 *     USART objects
 * ----------------------------------------------------------------------------*/

#ifdef USE_USART1
UARTClass Serial1(&huart1, USART1_IRQn, 0, USART1);
void Tx1_Handler(void){
  Serial1.TxHandler();
}
void Rx1_Handler(void){
  Serial1.RxHandler();
}
#endif

#ifdef USE_USART2
UARTClass Serial2(&huart2, USART2_IRQn, 1, USART2);
void Tx2_Handler(void){
  Serial2.TxHandler();
}
void Rx2_Handler(void){
  Serial2.RxHandler();
}
#endif

#ifdef USE_USART3
UARTClass Serial3(&huart3, USART3_IRQn, 2, USART3);
void Tx3_Handler(void){
  Serial3.TxHandler();
}
void Rx3_Handler(void){
  Serial3.RxHandler();
}
#endif

#ifdef USE_USBSerial
//uint8_t CDC_RxBuffer[CDC_RX_DATA_SIZE];
//uint8_t CDC_TxBuffer[CDC_TX_DATA_SIZE];

//RingBuffer USB_rx_buffer;
//USBSerial Serial(CDC_RxBuffer, CDC_TxBuffer);
USBSerial Serial;

void USBSerial_Tx_Handler(uint8_t *data, uint16_t len){
  Serial.CDC_TxHandler();
}

void USBSerial_Rx_Handler(uint8_t *data, uint16_t len){
  Serial.CDC_RxHandler(data, len);
}

void StartUSBSerial (void){
  Serial.begin(9600);
}
#endif
