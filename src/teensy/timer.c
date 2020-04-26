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
//
#include "fsl_qtmr.h"

#define QTMR_BASEADDR TMR3
#define FIRST_QTMR_CHANNEL kQTMR_Channel_0
#define SECOND_QTMR_CHANNEL kQTMR_Channel_1
#define QTMR_ClockCounterOutput kQTMR_ClockCounter0Output

/* Interrupt number and interrupt handler for the QTMR instance used */
#define QTMR_IRQ_ID TMR3_IRQn
#define QTMR_IRQ_HANDLER TMR3_IRQHandler

/* Get source clock for QTMR driver */
#define QTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_IpgClk)

#define QTMR_CLOCK_DIV  (128)
#define QTMR_CLOCK_FREQ (QTMR_SOURCE_CLOCK / QTMR_CLOCK_DIV)

#define TIMER_TOP (65535)

//static void timer_set(uint32_t value);

qtmr_config_t qtmrConfig;

// Set the next irq time
//static void
//timer_set(uint32_t value)
//{
//    //TODO
//    
//
//    //TC4->COUNT32.CC[0].reg = value;
//    //TC4->COUNT32.INTFLAG.reg = TC_INTFLAG_MC0;
//}

// Return the current time (in absolute clock ticks).
uint32_t
timer_read_time(void)
{
    return QTMR_GetCurrentTimerCount(QTMR_BASEADDR, SECOND_QTMR_CHANNEL) << 16
        | QTMR_GetCurrentTimerCount(QTMR_BASEADDR, FIRST_QTMR_CHANNEL);
}

// Activate timer dispatch as soon as possible
void
timer_kick(void)
{
    //timer_set(timer_read_time() + 50);
}

// IRQ handler
//void __aligned(16) // aligning helps stabilize perf benchmarks
//TC4_Handler(void)
//{
//    //irq_disable();
//    //uint32_t next = timer_dispatch_many();
//    //timer_set(next);
//    //irq_enable();
//}

void
timer_init(void)
{
    /*
     * qtmrConfig.debugMode = kQTMR_RunNormalInDebug;
     * qtmrConfig.enableExternalForce = false;
     * qtmrConfig.enableMasterMode = false;
     * qtmrConfig.faultFilterCount = 0;
     * qtmrConfig.faultFilterPeriod = 0;
     * qtmrConfig.primarySource = kQTMR_ClockDivide_2;
     * qtmrConfig.secondarySource = kQTMR_Counter0InputPin;
     */
    QTMR_GetDefaultConfig(&qtmrConfig);

    /* Init the first channel to use the IP Bus clock div by 128 */
    qtmrConfig.primarySource = kQTMR_ClockDivide_128;
    QTMR_Init(QTMR_BASEADDR, FIRST_QTMR_CHANNEL, &qtmrConfig);

    /* Init the second channel to use output of the first channel as we are chaining the first channel and the second channel */
    qtmrConfig.primarySource = QTMR_ClockCounterOutput;
    QTMR_Init(QTMR_BASEADDR, SECOND_QTMR_CHANNEL, &qtmrConfig);

    QTMR_SetTimerPeriod(QTMR_BASEADDR, FIRST_QTMR_CHANNEL, TIMER_TOP);
    QTMR_SetTimerPeriod(QTMR_BASEADDR, SECOND_QTMR_CHANNEL, TIMER_TOP);

    /* Start the second channel in cascase mode, chained to the first channel as set earlier via the primary source
     * selection */
    QTMR_StartTimer(QTMR_BASEADDR, SECOND_QTMR_CHANNEL, kQTMR_CascadeCount);

    /* Start the first channel to count on rising edge of the primary source clock */
    QTMR_StartTimer(QTMR_BASEADDR, FIRST_QTMR_CHANNEL, kQTMR_PriSrcRiseEdge);

    //timer_kick();

}
DECL_INIT(timer_init);
