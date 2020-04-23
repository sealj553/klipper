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
//#include "fsl_gpio.h"
#include "fsl_lpuart.h"

#undef ARRAY_SIZE

#include "pin_mux.h"
#include "clock_config.h"
#include "sched.h"

//#define EXAMPLE_LED_GPIO (GPIO1)
//#define EXAMPLE_LED_GPIO_PIN (24)

#include "fsl_qtmr.h"


#define QTMR_BASEADDR TMR3
//#define BOARD_FIRST_QTMR_CHANNEL kQTMR_Channel_0
#define QTMR_CHANNEL kQTMR_Channel_0
#define QTMR_ClockCounterOutput kQTMR_ClockCounter0Output

/* Interrupt number and interrupt handler for the QTMR instance used */
#define QTMR_IRQ_ID TMR3_IRQn
#define QTMR_IRQ_HANDLER TMR3_IRQHandler

/* Get source clock for QTMR driver */
#define QTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_IpgClk)

#define QTMR_CLOCK_DIV  (128)
#define QTMR_CLOCK_FREQ (QTMR_SOURCE_CLOCK / QTMR_CLOCK_DIV)

#define TIMER_TOP (65535)

//150MHz / 128 -> 1171875 Hz = 1.1171875 Mhz


volatile bool qtmrIsrFlag = false;

uint32_t overflows = 0;

void QTMR_IRQ_HANDLER(void){
    //time += TIMER_TOP;
    ++overflows;

    //time = QTMR_GetCurrentTimerCount(QTMR_BASEADDR

    /* Clear interrupt flag.*/
    QTMR_ClearStatusFlags(QTMR_BASEADDR, QTMR_CHANNEL, kQTMR_CompareFlag);

    qtmrIsrFlag = true;
}


void main(void){

    //int i = 0;
    //qtmr_config_t qtmrConfig;

    ///* Board pin, clock, debug console init */
    //BOARD_ConfigMPU();
    //BOARD_InitPins();
    //BOARD_BootClockRUN();
    //BOARD_InitDebugConsole();

    //PRINTF("\r\n\r\n*********QUADTIMER EXAMPLE START*********");

    ///*
    // * qtmrConfig.debugMode = kQTMR_RunNormalInDebug;
    // * qtmrConfig.enableExternalForce = false;
    // * qtmrConfig.enableMasterMode = false;
    // * qtmrConfig.faultFilterCount = 0;
    // * qtmrConfig.faultFilterPeriod = 0;
    // * qtmrConfig.primarySource = kQTMR_ClockDivide_2;
    // * qtmrConfig.secondarySource = kQTMR_Counter0InputPin;
    // */
    //QTMR_GetDefaultConfig(&qtmrConfig);
    ///* Use IP bus clock div by 128*/
    //qtmrConfig.primarySource = kQTMR_ClockDivide_128;


    //char buf[32];

    ////uint32_t num = COUNT_TO_USEC(10, QTMR_CLOCK_FREQ);
    ////uint32_t num;
    ////uint32_t num = QTMR_GetCurrentTimerCount(QTMR_BASEADDR, QTMR_CHANNEL);

    //////uint32_t num = QTMR_CLOCK_FREQ;
    //////uint32_t num = (QTMR_SOURCE_CLOCK/64) / 1000;

    ////itoa(num, buf, 10);

    //////QTMR_SetTimerPeriod(QTMR_BASEADDR, QTMR_CHANNEL, MSEC_TO_COUNT(50U, (QTMR_SOURCE_CLOCK / 128)));
    ////PRINTF("\r\n");
    ////PRINTF(buf);

    //QTMR_Init(QTMR_BASEADDR, QTMR_CHANNEL, &qtmrConfig);
    ///* Set timer period */
    //QTMR_SetTimerPeriod(QTMR_BASEADDR, QTMR_CHANNEL, TIMER_TOP);
    ///* Enable at the NVIC */
    //EnableIRQ(QTMR_IRQ_ID);
    ///* Enable timer compare interrupt */
    //QTMR_EnableInterrupts(QTMR_BASEADDR, QTMR_CHANNEL, kQTMR_CompareInterruptEnable);
    ///* Start the second channel to count on rising edge of the primary source clock */
    //QTMR_StartTimer(QTMR_BASEADDR, QTMR_CHANNEL, kQTMR_PriSrcRiseEdge);

    ////SDK_DelayAtLeastUs(100, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);

    //////
    ////uint32_t num = QTMR_GetCurrentTimerCount(QTMR_BASEADDR, QTMR_CHANNEL);
    ////num = COUNT_TO_USEC(num, QTMR_CLOCK_FREQ);
    ////itoa(num, buf, 10);
    ////PRINTF("\r\n");
    ////PRINTF(buf);
    //////

    //PRINTF("\r\n****Timer Test****\n");

    //for (i = 0; i < 10; i++) {
    //    /* Check whether compare interrupt occurs */
    //    while (!(qtmrIsrFlag)) {}
    //    //PRINTF("\r\n interrupt");

    //    itoa(overflows, buf, 10);
    //    PRINTF("\r\n");
    //    PRINTF(buf);

    //    //num = QTMR_GetCurrentTimerCount(QTMR_BASEADDR, QTMR_CHANNEL);
    //    //itoa(num, buf, 10);
    //    //PRINTF("\r\n");
    //    //PRINTF(buf);

    //    //SDK_DelayAtLeastUs(5500, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);

    //    qtmrIsrFlag = false;
    //}
    ////QTMR_Deinit(QTMR_BASEADDR, QTMR_CHANNEL);

    ////PRINTF("\r\n*********QUADTIMER EXAMPLE END.*********");

    //while (1) {}

    /* Board pin, clock, debug console init */
    //SystemInit();
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();

    sched_main();

    /* Define the init structure for the output LED pin*/
    //gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    ///* Print a note to terminal. */
    //PRINTF("\r\n GPIO Driver example\r\n");
    //PRINTF("\r\n The LED is blinking.\r\n");

    //GPIO_PinInit(EXAMPLE_LED_GPIO, EXAMPLE_LED_GPIO_PIN, &led_config);

    //while(1){
    //    SDK_DelayAtLeastUs(400000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    //    GPIO_PortToggle(EXAMPLE_LED_GPIO, 1 << EXAMPLE_LED_GPIO_PIN);
    //}


}
