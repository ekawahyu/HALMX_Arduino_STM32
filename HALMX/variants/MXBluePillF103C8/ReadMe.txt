

Edit the file 
HALMX_Arduino_STM32\HALMX\variants\MXBluePillF103C8\Drivers\CMSIS\Device\ST\STM32F1xx\Source\Templates\system_stm32f1xx.c
and replace the code:

------------------------------------------------------------------------------------
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x0 /*!< Vector Table base offset field. 
                                  This value must be a multiple of 0x200. */


/**
------------------------------------------------------------------------------------

with

------------------------------------------------------------------------------------
/* #define VECT_TAB_SRAM */
#ifndef VECT_TAB_OFFSET //<------- ADD THIS LINE
#define VECT_TAB_OFFSET  0x0 /*!< Vector Table base offset field. 
                                  This value must be a multiple of 0x200. */
#endif //<------- ADD THIS LINE

/**
------------------------------------------------------------------------------------