#include "fsl_wdog.h"
#undef ARRAY_SIZE

#include "sched.h"
#include "command.h" // DECL_CONSTANT
#include "autoconf.h"
#include "generic/irq.h"

#include "init.h"

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

void
main(void)
{
    SystemInit();
    sched_main();
}
