#include "fsl_wdog.h"
#undef ARRAY_SIZE

#include "sched.h"
#include "command.h" // DECL_CONSTANT
#include "autoconf.h"
#include "generic/irq.h"

#include "init.h"


//#
#include "gpio.h"
#include "internal.h"
//#

DECL_CONSTANT_STR("MCU", CONFIG_MCU);

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

#include "board.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"

void
main(void)
{
    SystemInit();
    sched_main();


    //struct gpio_out {
    //    void *regs;
    //    uint32_t bit;
    //};
    //struct gpio_out gpio_out_setup(uint8_t pin, uint8_t val);
    //void gpio_out_reset(struct gpio_out g, uint8_t val);
    //void gpio_out_toggle_noirq(struct gpio_out g);
    //void gpio_out_toggle(struct gpio_out g);
    //void gpio_out_write(struct gpio_out g, uint8_t val);

    //struct gpio_out pin;

    //pin = gpio_out_setup(GPIO(1,3), 1);

    //while(1){
    //    gpio_out_toggle(pin);

    //    PRINTF("\r\nbeep");
    //    PRINTF("\r\n%d",GPIO(2,3));
    //    PRINTF("\r\n%d",GPIO2PORT(GPIO(2,3)));
    //    PRINTF("\r\n%d",GPIO2BIT(GPIO(2,3)));
    //    SDK_DelayAtLeastUs(1000000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    //}
}
