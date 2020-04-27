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
