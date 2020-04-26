#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"
#include "fsl_gpt.h"

#undef ARRAY_SIZE

#include "pin_mux.h"
#include "clock_config.h"
#include "sched.h"
#include "command.h" // DECL_CONSTANT
#include "autoconf.h"


//#

#define GPT_IRQ_ID GPT2_IRQn
#define GPT GPT2
#define GPT_IRQHandler GPT2_IRQHandler

/* Get source clock for GPT driver (GPT prescaler = 0) */
#define GPT_CLK_FREQ CLOCK_GetFreq(kCLOCK_OscClk)

volatile bool gptIsrFlag = false;

void GPT_IRQHandler(void) {
    /* Clear interrupt flag.*/
    GPT_ClearStatusFlags(GPT, kGPT_OutputCompare1Flag);

    GPT_DisableInterrupts(GPT, kGPT_OutputCompare1InterruptEnable);


    //gptIsrFlag = true;
    PRINTF("\r\nbeep");
}


//#

//#define LED_GPIO (GPIO2)
//#define LED_GPIO_PIN (3)

DECL_CONSTANT_STR("MCU", CONFIG_MCU);

void main(void){

    ////SystemInit();
    //BOARD_ConfigMPU();
    //BOARD_InitPins();
    //BOARD_BootClockRUN();
    ////BOARD_InitDebugConsole();
    //sched_main();

    ////////* Define the init structure for the output LED pin*/
    ///////gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    /////////PRINTF("\r\n GPIO Driver example\r\n");

    ///////GPIO_PinInit(LED_GPIO, LED_GPIO_PIN, &led_config);

    ///////while(1){
    ///////    //SDK_DelayAtLeastUs(400000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    ///////    //GPIO_PortToggle(LED_GPIO, 1 << LED_GPIO_PIN);
    ///////}

    ///////timer_init();

    gpt_config_t gptConfig;

    /* Board pin, clock, debug console init */
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();


    char buf[32];

    uint32_t val = GPT_CLK_FREQ;

    itoa(val, buf, 10);
    PRINTF("\r\n");
    PRINTF(buf);

    /*!
     * The default values are:
     *    config->clockSource = kGPT_ClockSource_Periph;
     *    config->divider = 1U;
     *    config->enableRunInStop = true;
     *    config->enableRunInWait = true;
     *    config->enableRunInDoze = false;
     *    config->enableRunInDbg = false;
     *    config->enableFreeRun = true;
     *    config->enableMode  = true;
     */
    GPT_GetDefaultConfig(&gptConfig);
    gptConfig.clockSource = kGPT_ClockSource_Osc;
    gptConfig.enableFreeRun = true;
    /* 1MHz output -> 1 tick per uS */
    gptConfig.divider = GPT_CLK_FREQ/1000000U;

    GPT_Init(GPT, &gptConfig);


    /* Set both GPT modules to 1 second duration */
    GPT_SetOutputCompareValue(GPT, kGPT_OutputCompare_Channel1, 1000000U * 5);

    /* Enable GPT Output Compare1 interrupt */
    GPT_EnableInterrupts(GPT, kGPT_OutputCompare1InterruptEnable);

    /* Enable at the Interrupt */
    EnableIRQ(GPT_IRQ_ID);

    /* Start Timer */
    PRINTF("\r\nStarting GPT timer ...");
    GPT_StartTimer(GPT);

    while (true) {
        ///* Check whether occur interupt and toggle LED */
        //if (true == gptIsrFlag)
        //{
        //    PRINTF("\r\n GPT interrupt is occurred !");
        //    gptIsrFlag = false;

        //    //val = GPT_GetCurrentTimerCount(GPT);

        //    //itoa(val, buf, 10);
        //    //PRINTF("\r\n");
        //    //PRINTF(buf);

        //}
        //else
        //{
        //    __WFI();
        //}

        val = GPT_GetCurrentTimerCount(GPT);

        itoa(val, buf, 10);
        PRINTF("\r\n");
        PRINTF(buf);

        SDK_DelayAtLeastUs(1000000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    }
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
