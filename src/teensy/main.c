#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_wdog.h"

#undef ARRAY_SIZE

#include "pin_mux.h"
#include "clock_config.h"
#include "sched.h"
#include "command.h" // DECL_CONSTANT
#include "autoconf.h"
#include "generic/irq.h"


void
command_reset(uint32_t *args)
{
    irq_disable();

    /* Let the watchdog reset the MCU */
    wdog_config_t config;
    WDOG_GetDefaultConfig(&config);
    config.timeoutValue = 1;
    WDOG_Init(WDOG1, &config);
}
DECL_COMMAND_FLAGS(command_reset, HF_IN_SHUTDOWN, "reset");


//#define LED_GPIO (GPIO2)
//#define LED_GPIO_PIN (3)

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

    ///////GPIO_PinInit(LED_GPIO, LED_GPIO_PIN, &led_config);

    ///////while(1){
    ///////    //SDK_DelayAtLeastUs(400000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    ///////    //GPIO_PortToggle(LED_GPIO, 1 << LED_GPIO_PIN);
    ///////}
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
   */
