#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"

#undef ARRAY_SIZE

#include "pin_mux.h"
#include "clock_config.h"
#include "sched.h"
#include "command.h" // DECL_CONSTANT
#include "autoconf.h"


//#

//#include "fsl_qtmr.h"
///* The QTMR instance/channel used for board */
//#define BOARD_QTMR_BASEADDR TMR3
//#define BOARD_FIRST_QTMR_CHANNEL kQTMR_Channel_0
//#define BOARD_SECOND_QTMR_CHANNEL kQTMR_Channel_1
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
//
//#define TIMER_TOP (65535)

//void QTMR_IRQ_HANDLER(void){
//    /* Clear interrupt flag.*/
//    QTMR_ClearStatusFlags(BOARD_QTMR_BASEADDR, BOARD_SECOND_QTMR_CHANNEL, kQTMR_CompareFlag);
//}

//#

#define EXAMPLE_LED_GPIO (GPIO2)
#define EXAMPLE_LED_GPIO_PIN (3)

DECL_CONSTANT_STR("MCU", CONFIG_MCU);

void main(void){

    //SystemInit();
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    //BOARD_InitDebugConsole();
    sched_main();

    ////////* Define the init structure for the output LED pin*/
    ///////gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    /////////PRINTF("\r\n GPIO Driver example\r\n");

    ///////GPIO_PinInit(EXAMPLE_LED_GPIO, EXAMPLE_LED_GPIO_PIN, &led_config);

    ///////while(1){
    ///////    //SDK_DelayAtLeastUs(400000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    ///////    //GPIO_PortToggle(EXAMPLE_LED_GPIO, 1 << EXAMPLE_LED_GPIO_PIN);
    ///////}

    ///////timer_init();

//
//
//    //uint8_t i = 0;
//    qtmr_config_t qtmrConfig;
//
//    /* Board pin, clock, debug console init */
//    BOARD_ConfigMPU();
//    BOARD_InitPins();
//    BOARD_BootClockRUN();
//    BOARD_InitDebugConsole();
//
//    PRINTF("\r\n*********QUADTIMER EXAMPLE START*********");
//
//    /*
//     * qtmrConfig.debugMode = kQTMR_RunNormalInDebug;
//     * qtmrConfig.enableExternalForce = false;
//     * qtmrConfig.enableMasterMode = false;
//     * qtmrConfig.faultFilterCount = 0;
//     * qtmrConfig.faultFilterPeriod = 0;
//     * qtmrConfig.primarySource = kQTMR_ClockDivide_2;
//     * qtmrConfig.secondarySource = kQTMR_Counter0InputPin;
//     */
//    QTMR_GetDefaultConfig(&qtmrConfig);
//
//    PRINTF("\r\n****Chain Timer use-case, 1 second tick.****\n");
//
//    /* Init the first channel to use the IP Bus clock div by 128 */
//    qtmrConfig.primarySource = kQTMR_ClockDivide_128;
//    QTMR_Init(BOARD_QTMR_BASEADDR, BOARD_FIRST_QTMR_CHANNEL, &qtmrConfig);
//
//    /* Init the second channel to use output of the first channel as we are chaining the first channel and the second channel */
//    qtmrConfig.primarySource = QTMR_ClockCounterOutput;
//    QTMR_Init(BOARD_QTMR_BASEADDR, BOARD_SECOND_QTMR_CHANNEL, &qtmrConfig);
//
//    QTMR_SetTimerPeriod(BOARD_QTMR_BASEADDR, BOARD_FIRST_QTMR_CHANNEL, TIMER_TOP);
//    QTMR_SetTimerPeriod(BOARD_QTMR_BASEADDR, BOARD_SECOND_QTMR_CHANNEL, TIMER_TOP);
//
//    /* Enable the second channel compare interrupt */
//    //QTMR_EnableInterrupts(BOARD_QTMR_BASEADDR, BOARD_SECOND_QTMR_CHANNEL, kQTMR_CompareInterruptEnable);
//
//    /* Start the second channel in cascase mode, chained to the first channel as set earlier via the primary source
//     * selection */
//    QTMR_StartTimer(BOARD_QTMR_BASEADDR, BOARD_SECOND_QTMR_CHANNEL, kQTMR_CascadeCount);
//
//    /* Start the first channel to count on rising edge of the primary source clock */
//    QTMR_StartTimer(BOARD_QTMR_BASEADDR, BOARD_FIRST_QTMR_CHANNEL, kQTMR_PriSrcRiseEdge);
//
//    char buf[32];
//
//    uint32_t time = 0;
//    uint32_t prev_time = 0;
//
//
//    //while(1){
//    for(int i = 0; i < 100; ++i){
//
//        prev_time = time;
//
//        time = QTMR_GetCurrentTimerCount(BOARD_QTMR_BASEADDR, BOARD_SECOND_QTMR_CHANNEL) << 16
//            | QTMR_GetCurrentTimerCount(BOARD_QTMR_BASEADDR, BOARD_FIRST_QTMR_CHANNEL);
//
//        time = COUNT_TO_USEC(time, QTMR_CLOCK_FREQ);
//
//        itoa(time - prev_time, buf, 10);
//        PRINTF("\r\n");
//        PRINTF(buf);
//        //PRINTF("\r\n");
//
//        SDK_DelayAtLeastUs(1000000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
//    }
}


//GPIO mapping
/*
Teensy |  SDK
-------+------
0      |  1.3
1      |  1.2
2      |  4.4
3      |  4.5
4      |  4.6
5      |  4.8
6      |  2.10
7      |  2.17
8      |  2.16
9      |  2.11
10     |  2.0
11     |  2.2
12     |  2.1

13     |  2.3
14     |  1.18 [TX]
15     |  1.19 [RX]
16     |  1.23
17     |  1.22
18     |  1.17
19     |  1.16
20     |  1.26
21     |  1.27
22     |  1.24
23     |  1.25





23     | 1.25








*/
