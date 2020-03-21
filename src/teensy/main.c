//// Main starting point for LPC176x boards.
////
//// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
////
//// This file may be distributed under the terms of the GNU GPLv3 license.
//
//#include "board/armcm_boot.h" // armcm_main
//#include "internal.h" // enable_pclock
//#include "sched.h" // sched_main
//
//
///****************************************************************
// * watchdog handler
// ****************************************************************/
//
//void
//watchdog_reset(void)
//{
//    LPC_WDT->WDFEED = 0xaa;
//    LPC_WDT->WDFEED = 0x55;
//}
//DECL_TASK(watchdog_reset);
//
//void
//watchdog_init(void)
//{
//    LPC_WDT->WDTC = 4000000 / 2; // 500ms timeout
//    LPC_WDT->WDCLKSEL = 1<<31; // Lock to internal RC
//    LPC_WDT->WDMOD = 0x03; // select reset and enable
//    watchdog_reset();
//}
//DECL_INIT(watchdog_init);
//
//
///****************************************************************
// * misc functions
// ****************************************************************/
//
//// Check if a peripheral clock has been enabled
//int
//is_enabled_pclock(uint32_t pclk)
//{
//    return !!(LPC_SC->PCONP & (1<<pclk));
//}
//
//// Enable a peripheral clock
//void
//enable_pclock(uint32_t pclk)
//{
//    LPC_SC->PCONP |= 1<<pclk;
//    if (pclk < 16) {
//        uint32_t shift = pclk * 2;
//        LPC_SC->PCLKSEL0 = (LPC_SC->PCLKSEL0 & ~(0x3<<shift)) | (0x1<<shift);
//    } else {
//        uint32_t shift = (pclk - 16) * 2;
//        LPC_SC->PCLKSEL1 = (LPC_SC->PCLKSEL1 & ~(0x3<<shift)) | (0x1<<shift);
//    }
//}
//
//// Main entry point - called from armcm_boot.c:ResetHandler()
//void
//armcm_main(void)
//{
//    SystemInit();
//    sched_main();
//}

//int main(){
//    return 0;
//}

#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"

#include "pin_mux.h"
#include "clock_config.h"

#define EXAMPLE_LED_GPIO (GPIO1)
#define EXAMPLE_LED_GPIO_PIN (24)
#define EXAMPLE_DELAY_COUNT 8000000

void delay(void);

/* The PIN status */
volatile bool g_pinSet = false;

void delay(void){
    volatile uint32_t i = 0;
    for (i = 0; i < EXAMPLE_DELAY_COUNT; ++i){
        __asm("NOP"); /* delay */
    }
}

int main(void){

    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    /* Board pin, clock, debug console init */
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /* Print a note to terminal. */
    PRINTF("\r\n GPIO Driver example\r\n");
    PRINTF("\r\n The LED is blinking.\r\n");

    /* Init output LED GPIO. */
    GPIO_PinInit(EXAMPLE_LED_GPIO, EXAMPLE_LED_GPIO_PIN, &led_config);

    while(1){
        SDK_DelayAtLeastUs(100000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
#if (defined(FSL_FEATURE_IGPIO_HAS_DR_TOGGLE) && (FSL_FEATURE_IGPIO_HAS_DR_TOGGLE == 1))
        GPIO_PortToggle(EXAMPLE_LED_GPIO, 1u << EXAMPLE_LED_GPIO_PIN);
#else
        if (g_pinSet){
            GPIO_PinWrite(EXAMPLE_LED_GPIO, EXAMPLE_LED_GPIO_PIN, 0U);
            g_pinSet = false;
        } else {
            GPIO_PinWrite(EXAMPLE_LED_GPIO, EXAMPLE_LED_GPIO_PIN, 1U);
            g_pinSet = true;
        }
#endif /* FSL_FEATURE_IGPIO_HAS_DR_TOGGLE */
    }
}
