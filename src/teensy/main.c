#include "board.h"
#include "fsl_debug_console.h"
//#include "fsl_gpio.h"
#include "fsl_lpuart.h"

#undef ARRAY_SIZE

#include "pin_mux.h"
#include "clock_config.h"
#include "sched.h"
#include "command.h" // DECL_CONSTANT
#include "autoconf.h"

///extern void timer_init(void);
///extern uint32_t timer_read_time(void);
///extern uint32_t overflows;
///extern uint32_t timer_from_us(uint32_t us);

//extern void QTMR_IRQ_HANDLER(void);
//extern void TMR3_IRQHandler(void);

//#define EXAMPLE_LED_GPIO (GPIO1)
//#define EXAMPLE_LED_GPIO_PIN (24)

//#include "fsl_qtmr.h"

//#define QTMR_BASEADDR TMR3
////#define BOARD_FIRST_QTMR_CHANNEL kQTMR_Channel_0
//#define QTMR_CHANNEL kQTMR_Channel_0
//#define QTMR_ClockCounterOutput kQTMR_ClockCounter0Output
//
///* Interrupt number and interrupt handler for the QTMR instance used */
//#define QTMR_IRQ_ID TMR3_IRQn
//#define QTMR_IRQ_HANDLER TMR3_IRQHandler
//
///* Get source clock for QTMR driver */
//#define QTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_IpgClk)
//
//#define QTMR_CLOCK_DIV  (128)
//#define QTMR_CLOCK_FREQ (QTMR_SOURCE_CLOCK / QTMR_CLOCK_DIV)

//#define TIMER_TOP (65535)

//150MHz / 128 -> 1171875 Hz = 1.1171875 Mhz


//volatile bool qtmrIsrFlag = false;
//
//uint32_t overflows = 0;
//
//void QTMR_IRQ_HANDLER(void){
//    //time += TIMER_TOP;
//    ++overflows;
//
//    //time = QTMR_GetCurrentTimerCount(QTMR_BASEADDR
//
//    /* Clear interrupt flag.*/
//    QTMR_ClearStatusFlags(QTMR_BASEADDR, QTMR_CHANNEL, kQTMR_CompareFlag);
//
//    qtmrIsrFlag = true;
//}

DECL_CONSTANT_STR("MCU", CONFIG_MCU);

void main(void){

////
////    //PRINTF("\r\n\r\n*********QUADTIMER EXAMPLE START*********");
////
////    ///*
////    // * qtmrConfig.debugMode = kQTMR_RunNormalInDebug;
////    // * qtmrConfig.enableExternalForce = false;
////    // * qtmrConfig.enableMasterMode = false;
////    // * qtmrConfig.faultFilterCount = 0;
////    // * qtmrConfig.faultFilterPeriod = 0;
////    // * qtmrConfig.primarySource = kQTMR_ClockDivide_2;
////    // * qtmrConfig.secondarySource = kQTMR_Counter0InputPin;
////    // */
////    //QTMR_GetDefaultConfig(&qtmrConfig);
////    ///* Use IP bus clock div by 128*/
////    //qtmrConfig.primarySource = kQTMR_ClockDivide_128;
////
////
////    char buf[32];
////
////    ////uint32_t num = COUNT_TO_USEC(10, QTMR_CLOCK_FREQ);
////    ////uint32_t num;
////    ////uint32_t num = QTMR_GetCurrentTimerCount(QTMR_BASEADDR, QTMR_CHANNEL);
////
////    //////uint32_t num = QTMR_CLOCK_FREQ;
////    //////uint32_t num = (QTMR_SOURCE_CLOCK/64) / 1000;
////
////    ////itoa(num, buf, 10);
////
////    //////QTMR_SetTimerPeriod(QTMR_BASEADDR, QTMR_CHANNEL, MSEC_TO_COUNT(50U, (QTMR_SOURCE_CLOCK / 128)));
////    ////PRINTF("\r\n");
////    ////PRINTF(buf);
////
////    //int i = 0;
////    qtmr_config_t qtmrConfig;
////
////    /* Board pin, clock, debug console init */
////    BOARD_ConfigMPU();
////    BOARD_InitPins();
////    BOARD_BootClockRUN();
////    BOARD_InitDebugConsole();
////
////    QTMR_Init(QTMR_BASEADDR, QTMR_CHANNEL, &qtmrConfig);
////    /* Set timer period */
////    QTMR_SetTimerPeriod(QTMR_BASEADDR, QTMR_CHANNEL, TIMER_TOP);
////    /* Enable at the NVIC */
////    EnableIRQ(QTMR_IRQ_ID);
////    /* Enable timer compare interrupt */
////    QTMR_EnableInterrupts(QTMR_BASEADDR, QTMR_CHANNEL, kQTMR_CompareInterruptEnable);
////    /* Start the second channel to count on rising edge of the primary source clock */
////    QTMR_StartTimer(QTMR_BASEADDR, QTMR_CHANNEL, kQTMR_PriSrcRiseEdge);
////
////    //SDK_DelayAtLeastUs(100, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
////
////    ////
////    //uint32_t num = QTMR_GetCurrentTimerCount(QTMR_BASEADDR, QTMR_CHANNEL);
////    //num = COUNT_TO_USEC(num, QTMR_CLOCK_FREQ);
////    //itoa(num, buf, 10);
////    //PRINTF("\r\n");
////    //PRINTF(buf);
////    ////
////
////    PRINTF("\r\n****Timer Test****\n");
////
////    //for (i = 0; i < 10; i++) {
////    while(1){
////        /* Check whether compare interrupt occurs */
////        while (!(qtmrIsrFlag)) {}
////        //PRINTF("\r\n interrupt");
////
////        itoa(overflows, buf, 10);
////        PRINTF("\r\n");
////        PRINTF(buf);
////
////        //num = QTMR_GetCurrentTimerCount(QTMR_BASEADDR, QTMR_CHANNEL);
////        //itoa(num, buf, 10);
////        //PRINTF("\r\n");
////        //PRINTF(buf);
////
////        SDK_DelayAtLeastUs(10000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
////
////        qtmrIsrFlag = false;
////    }
////    QTMR_Deinit(QTMR_BASEADDR, QTMR_CHANNEL);
////
////    /////PRINTF("\r\n*********QUADTIMER EXAMPLE END.*********");
////
////    while (1) {}

    /* Board pin, clock, debug console init */
    //SystemInit();
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();

    //BOARD_InitDebugConsole();

    sched_main();

    /////timer_init();

//////    PRINTF("\r\n\r\ntimer test\r\n");
//////
//////    char buf[64];
//////    //int num = 0;
//////    while(1){
//////        ///if(++num < 20){
//////
//////        uint32_t ticks = timer_read_time();
//////        itoa(ticks, buf, 10);
//////        //itoa(overflows, buf, 10);
//////        PRINTF("\r\n");
//////        PRINTF(buf);
//////
//////
//////        PRINTF("\r\n");
//////        uint32_t time = timer_from_us(COUNT_TO_USEC(ticks, CONFIG_CLOCK_FREQ));
//////        itoa(time, buf, 10);
//////        //itoa(overflows, buf, 10);
//////        PRINTF("\r\n");
//////        PRINTF(buf);
//////
//////        PRINTF("\r\n");
//////        PRINTF("\r\n");
//////
//////        //}
//////
//////        SDK_DelayAtLeastUs(10000000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
//////    }

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

//Here follows the correct setup sequence to get a free-running 32-bit counter using QTIM:
//
//const qtmr_config_t QuadTimer_1_Channel_0_config = {
//.primarySource = kQTMR_ClockCounter1InputPin,
//.secondarySource = kQTMR_Counter0InputPin,
//.enableMasterMode = false,
//.enableExternalForce = false,
//.faultFilterCount = 0,
//.faultFilterPeriod = 0,
//.debugMode = kQTMR_RunNormalInDebug
//};
//const qtmr_config_t QuadTimer_1_Channel_1_config = {
//.primarySource = kQTMR_ClockCounter0Output,
//.secondarySource = kQTMR_Counter0InputPin,
//.enableMasterMode = false,
//.enableExternalForce = false,
//.faultFilterCount = 0,
//.faultFilterPeriod = 0,
//.debugMode = kQTMR_RunNormalInDebug
//};
//
//void QuadTimer_1_init(void) {
///* Quad timer channel Channel_0 peripheral initialization */
//QTMR_Init(QUADTIMER_1_PERIPHERAL, QUADTIMER_1_CHANNEL_0_CHANNEL, &QuadTimer_1_Channel_0_config);
///* Setup the timer period of the channel */
//QTMR_SetTimerPeriod(QUADTIMER_1_PERIPHERAL, QUADTIMER_1_CHANNEL_0_CHANNEL, 0xFFFFU);
///* Quad timer channel Channel_1 peripheral initialization */
//QTMR_Init(QUADTIMER_1_PERIPHERAL, QUADTIMER_1_CHANNEL_1_CHANNEL, &QuadTimer_1_Channel_1_config);
///* Setup the timer period of the channel */
//QTMR_SetTimerPeriod(QUADTIMER_1_PERIPHERAL, QUADTIMER_1_CHANNEL_1_CHANNEL, 0xFFFFU);
//
///* Start the timer - select the timer counting mode */
//QTMR_StartTimer(QUADTIMER_1_PERIPHERAL, QUADTIMER_1_CHANNEL_1_CHANNEL, kQTMR_CascadeCount);
///* Start the timer - select the timer counting mode */
//QTMR_StartTimer(QUADTIMER_1_PERIPHERAL, QUADTIMER_1_CHANNEL_0_CHANNEL, kQTMR_PriSrcRiseEdge);
//}
