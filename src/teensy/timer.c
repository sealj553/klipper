// SAMD21 timer interrupt scheduling
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "board/timer_irq.h" // timer_dispatch_many
#include "internal.h" // enable_pclock
#include "sched.h" // DECL_INIT

//
#include "board.h"

#undef ARRAY_SIZE

#include "pin_mux.h"
#include "clock_config.h"
#include "sched.h"
#include "fsl_gpt.h"
//

#define GPT_IRQ_ID GPT2_IRQn
#define GPT GPT2
#define GPT_IRQHandler GPT2_IRQHandler

/* Get source clock for GPT driver (GPT prescaler = 0) */
#define GPT_CLK_FREQ CLOCK_GetFreq(kCLOCK_OscClk)

// Set the next irq time
    static void
timer_set(uint32_t value)
{
    /* Set both GPT modules to 1 second duration */
    GPT_SetOutputCompareValue(GPT, kGPT_OutputCompare_Channel1, value);
    /* Enable GPT Output Compare1 interrupt */
    GPT_EnableInterrupts(GPT, kGPT_OutputCompare1InterruptEnable);
}

// Return the current time (in absolute clock ticks).
    uint32_t
timer_read_time(void)
{
    return GPT_GetCurrentTimerCount(GPT);
}

// Activate timer dispatch as soon as possible
    void
timer_kick(void)
{
    timer_set(timer_read_time() + 50);
}

void GPT_IRQHandler(void) {
    GPT_ClearStatusFlags(GPT, kGPT_OutputCompare1Flag);

    //GPT_DisableInterrupts(GPT, kGPT_OutputCompare1InterruptEnable);
    irq_disable();
    uint32_t next = timer_dispatch_many();
    timer_set(next);
    irq_enable();
}

    void
timer_init(void)
{
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
    gpt_config_t gptConfig;
    GPT_GetDefaultConfig(&gptConfig);
    /* 24MHz crystal */
    gptConfig.clockSource = kGPT_ClockSource_Osc;
    gptConfig.enableFreeRun = true;
    /* 1MHz output -> 1 tick per uS */
    gptConfig.divider = GPT_CLK_FREQ/1000000U;

    GPT_Init(GPT, &gptConfig);

    EnableIRQ(GPT_IRQ_ID);

    GPT_StartTimer(GPT);

    timer_kick();
}
DECL_INIT(timer_init);
