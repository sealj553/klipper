#include "generic/irq.h"

#include "fsl_common.h"

//typedef unsigned long irqstatus_t;

#define QTMR_IRQ_ID TMR3_IRQn
#define LPUART_IRQn LPUART2_IRQn

//TODO:these
void irq_disable(void){
    //DisableIRQ(QTMR_IRQ_ID);
    //DisableIRQ(LPUART_IRQn);
}
void irq_enable(void){
    //EnableIRQ(QTMR_IRQ_ID);
    //EnableIRQ(LPUART_IRQn);
}
irqstatus_t irq_save(void){
    return 0;
}
void irq_restore(irqstatus_t flag){}
void irq_wait(void){}
void irq_poll(void){}
