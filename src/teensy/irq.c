#include "generic/irq.h"

//typedef unsigned long irqstatus_t;

//TODO:these
void irq_disable(void){}
void irq_enable(void){}
irqstatus_t irq_save(void){
    return 0;
}
void irq_restore(irqstatus_t flag){}
void irq_wait(void){}
void irq_poll(void){}
