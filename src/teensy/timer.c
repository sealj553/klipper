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
//150MHz / 128 -> 1171875 Hz = 1.1171875 Mhz

#define TIMER_TOP (65535)

static void timer_set(uint32_t value);


qtmr_config_t qtmrConfig;

uint32_t overflows = 0;

void QTMR_IRQ_HANDLER(void){
    //irq_disable();
    ++overflows;

    /* Clear interrupt flag.*/
    QTMR_ClearStatusFlags(QTMR_BASEADDR, QTMR_CHANNEL, kQTMR_CompareFlag);

    //uint32_t next = timer_dispatch_many();
    //timer_set(next);
    //irq_enable();
}

//TODO: fix this file

// Set the next irq time
static void
timer_set(uint32_t value)
{
    //TODO:
    //
    

    //TC4->COUNT32.CC[0].reg = value;
    //TC4->COUNT32.INTFLAG.reg = TC_INTFLAG_MC0;
}

uint32_t time_offset = 0;

// Return the current time (in absolute clock ticks).
uint32_t
timer_read_time(void)
{
    //return TC4->COUNT32.COUNT.reg;
    uint32_t base = overflows * TIMER_TOP;
    uint32_t count = QTMR_GetCurrentTimerCount(QTMR_BASEADDR, QTMR_CHANNEL);
    return base + count + time_offset;
}

// Activate timer dispatch as soon as possible
void
timer_kick(void)
{
    //timer_set(timer_read_time() + 50);
    time_offset += 50;
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
    /* Use IP bus clock div by 128*/
    qtmrConfig.primarySource = kQTMR_ClockDivide_128;

    QTMR_Init(QTMR_BASEADDR, QTMR_CHANNEL, &qtmrConfig);
    /* Set timer period */
    QTMR_SetTimerPeriod(QTMR_BASEADDR, QTMR_CHANNEL, TIMER_TOP);
    /* Enable at the NVIC */
    EnableIRQ(QTMR_IRQ_ID);
    /* Enable timer compare interrupt */
    QTMR_EnableInterrupts(QTMR_BASEADDR, QTMR_CHANNEL, kQTMR_CompareInterruptEnable);
    /* Start the second channel to count on rising edge of the primary source clock */
    QTMR_StartTimer(QTMR_BASEADDR, QTMR_CHANNEL, kQTMR_PriSrcRiseEdge);

    timer_kick();


    //// Supply power and clock to the timer
    //enable_pclock(TC3_GCLK_ID, ID_TC3);
    //enable_pclock(TC4_GCLK_ID, ID_TC4);

    //// Configure the timer
    //TcCount32 *tc = &TC4->COUNT32;
    //irqstatus_t flag = irq_save();
    //tc->CTRLA.reg = 0;
    //tc->CTRLA.reg = TC_CTRLA_MODE_COUNT32;
    //armcm_enable_irq(TC4_Handler, TC4_IRQn, 2);
    //tc->INTENSET.reg = TC_INTENSET_MC0;
    //tc->COUNT.reg = 0;
    //timer_kick();
    //tc->CTRLA.reg = TC_CTRLA_MODE_COUNT32 | TC_CTRLA_ENABLE;
    //irq_restore(flag);
}
DECL_INIT(timer_init);
