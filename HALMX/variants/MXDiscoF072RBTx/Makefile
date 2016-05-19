ARM_NONE_EABI_BIN=/Users/nodino/gcc-arm-none-eabi/bin
OPENOCD_HOME=/opt/local
COPY=cp

# put your *.o targets here, make should handle the rest!
SRCS = main.c system_stm32f0xx.c syscalls.c
SRCS += stm32f0xx_hal_msp.c stm32f0xx_it.c
SRCS += usbd_cdc_if.c usbd_conf.c usbd_desc.c usb_device.c
SRCS += wiring.c wiring_digital.c
CXXSRCS = variant.cpp sketch.cpp

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)
PROJ_NAME=main

# Location of the STM32F0xx HAL Drivers
HAL_DRIVER=Drivers

# Location of the STM32F0xx USB Device Library
MIDDLEWARE=Middlewares/ST

# Location of the linker scripts
LDSCRIPT_INC=scripts

# location of OpenOCD Board .cfg files (only used with 'make program')
OPENOCD_BOARD_DIR=$(OPENOCD_HOME)/scripts/board
OPENOCD_INTERFACE_DIR=$(OPENOCD_HOME)/share/openocd/scripts/interface
OPENOCD_TARGET_DIR=$(OPENOCD_HOME)/share/openocd/scripts/target

# Configuration (cfg) file containing programming directives for OpenOCD
OPENOCD_PROC_FILE=scripts/stm32f0-openocd.cfg

# that's it, no need to change anything below this line!

###################################################

CC=$(ARM_NONE_EABI_BIN)/arm-none-eabi-gcc
CXX=$(ARM_NONE_EABI_BIN)/arm-none-eabi-g++
OBJCOPY=$(ARM_NONE_EABI_BIN)/arm-none-eabi-objcopy
OBJDUMP=$(ARM_NONE_EABI_BIN)/arm-none-eabi-objdump
SIZE=$(ARM_NONE_EABI_BIN)/arm-none-eabi-size

CFLAGS  = -Wall -g -O2 -D__STRICT_ANSI__
CFLAGS += -mlittle-endian -mcpu=cortex-m0  -march=armv6-m -mthumb
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wl,--gc-sections -Wl,-Map=$(PROJ_NAME).map
CFLAGS += -specs=nano.specs
#CFLAGS += -u _printf_float -u _scanf_float

CFLAGS += -DSTM32F072xB ############ need to move/define this somewhere else

CXXFLAGS = -fno-rtti -fno-exceptions

###################################################

vpath %.c $(HAL_DRIVER)/CMSIS/Device/ST/STM32F0xx/Source/Templates
vpath %.c Src ../../cores/mapleMX
vpath %.cpp Src ../../cores/mapleMX
vpath %.s Src
vpath %.a $(HAL_DRIVER)

ROOT=$(shell pwd)

CFLAGS += -I. -IInc -I$(HAL_DRIVER) -I$(HAL_DRIVER)/CMSIS/Device/ST/STM32F0xx/Include
CFLAGS += -I$(HAL_DRIVER)/CMSIS/Include -I$(HAL_DRIVER)/STM32F0xx_HAL_Driver/Inc
CFLAGS += -includeInc/stm32f0xx_hal_conf.h
CFLAGS += -I$(MIDDLEWARE)/STM32_USB_Device_Library/Core/Inc
CFLAGS += -I$(MIDDLEWARE)/STM32_USB_Device_Library/Class/CDC/Inc
CFLAGS += -I../../cores/mapleMX

ASMSRCS += startup_stm32f072xb.s ############ need to move/define this somewhere else

OBJS = $(SRCS:.c=.o) $(ASMSRCS:.s=.o) $(CXXSRCS:.cpp=.o)

###################################################

.PHONY: lib proj

all: lib proj

lib:
	$(MAKE) -C $(HAL_DRIVER)

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ -L$(HAL_DRIVER) -lstm32f0 -L$(LDSCRIPT_INC) -TSTM32F072RB_FLASH.ld
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	$(OBJDUMP) -St $(PROJ_NAME).elf >$(PROJ_NAME).lst
	$(SIZE) $(PROJ_NAME).elf
	$(COPY) $(PROJ_NAME).elf ../..
	$(COPY) $(PROJ_NAME).bin ../..

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.s
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.cpp
	$(CXX) $(CFLAGS) $(CXXFLAGS) -c $< -o $@

flash: all $(PROJ_NAME).bin
	$(OPENOCD_HOME)/bin/openocd -f $(OPENOCD_INTERFACE_DIR)/stlink-v2.cfg -f $(OPENOCD_TARGET_DIR)/stm32f0x.cfg -f $(OPENOCD_PROC_FILE) -c "stm_flash `pwd`/$(PROJ_NAME).bin" -c shutdown

clean:
	find ./ -name '*~' | xargs rm -f	
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
	rm -f $(PROJ_NAME).map
	rm -f $(PROJ_NAME).lst
	rm -f ../../$(PROJ_NAME).elf
	rm -f ../../$(PROJ_NAME).bin

distclean: clean
	$(MAKE) -C $(HAL_DRIVER) clean
