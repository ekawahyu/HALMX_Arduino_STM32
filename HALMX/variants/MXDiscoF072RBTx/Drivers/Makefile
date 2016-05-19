ARM_NONE_EABI_BIN=/Users/nodino/gcc-arm-none-eabi/bin
CC=$(ARM_NONE_EABI_BIN)/arm-none-eabi-gcc
AR=$(ARM_NONE_EABI_BIN)/arm-none-eabi-ar

###########################################

vpath %.c STM32F0xx_HAL_Driver/Src STM32_USB_Device_Library/Core/Src STM32F0xx_HAL_Driver/Src 
vpath %.c ../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src
vpath %.c ../Middlewares/ST/STM32_USB_Device_Library/Core/Src

CFLAGS  = -g -O2 -Wall
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m0 -march=armv6s-m
CFLAGS += -ffreestanding -nostdlib
CFLAGS += -specs=nano.specs
#CFLAGS += -u _printf_float -u _scanf_float
CFLAGS += -include../Inc/stm32f0xx_hal_conf.h -I./ -ICMSIS/Include
CFLAGS += -ICMSIS/Device/ST/STM32F0xx/Include -ISTM32F0xx_HAL_Driver/Inc
CFLAGS += -I../Inc -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc
CFLAGS += -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc

CFLAGS += -DSTM32F072xB ############ need to move/define this somewhere else

SRCS = stm32f0xx_hal_adc_ex.c stm32f0xx_hal_adc.c stm32f0xx_hal_can.c stm32f0xx_hal_cec.c \
	stm32f0xx_hal_comp.c stm32f0xx_hal_cortex.c stm32f0xx_hal_crc_ex.c stm32f0xx_hal_crc.c \
	stm32f0xx_hal_dac_ex.c stm32f0xx_hal_dac.c stm32f0xx_hal_dma.c stm32f0xx_hal_flash_ex.c \
	stm32f0xx_hal_flash.c stm32f0xx_hal_gpio.c stm32f0xx_hal_i2c_ex.c stm32f0xx_hal_i2c.c \
	stm32f0xx_hal_i2s.c stm32f0xx_hal_irda.c stm32f0xx_hal_iwdg.c stm32f0xx_hal_pcd_ex.c \
	stm32f0xx_hal_pcd.c stm32f0xx_hal_pwr_ex.c stm32f0xx_hal_pwr.c stm32f0xx_hal_rcc_ex.c \
	stm32f0xx_hal_rcc.c stm32f0xx_hal_rtc_ex.c stm32f0xx_hal_rtc.c stm32f0xx_hal_smartcard_ex.c \
	stm32f0xx_hal_smartcard.c stm32f0xx_hal_smbus.c stm32f0xx_hal_spi_ex.c stm32f0xx_hal_spi.c \
	stm32f0xx_hal_tim_ex.c stm32f0xx_hal_tim.c stm32f0xx_hal_tsc.c stm32f0xx_hal_uart_ex.c \
	stm32f0xx_hal_uart.c stm32f0xx_hal_usart.c stm32f0xx_hal_wwdg.c stm32f0xx_hal.c

SRCS += usbd_core.c usbd_ctlreq.c usbd_ioreq.c usbd_cdc.c

OBJS = $(SRCS:.c=.o)

.PHONY: libstm32f0.a

all: libstm32f0.a

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $^

libstm32f0.a: $(OBJS)
	$(AR) -r $@ $(OBJS)

clean:
	rm -f $(OBJS) libstm32f0.a
