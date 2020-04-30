#include <string.h> // ffs
#include "board/irq.h" // irq_save
#include "command.h" // shutdown
#include "gpio.h" // gpio_out_setup
#include "internal.h" // gpio_peripheral
#include "sched.h" // sched_shutdown

#include "fsl_gpio.h"

/****************************************************************
 * Pin mappings
 ****************************************************************/

DECL_ENUMERATION_RANGE("pin", "P0.0", GPIO(0, 0), 32);
DECL_ENUMERATION_RANGE("pin", "P1.0", GPIO(1, 0), 32);
DECL_ENUMERATION_RANGE("pin", "P2.0", GPIO(2, 0), 32);
DECL_ENUMERATION_RANGE("pin", "P3.0", GPIO(3, 0), 32);
DECL_ENUMERATION_RANGE("pin", "P4.0", GPIO(4, 0), 32);

static GPIO_Type* const digital_regs[] = {
    0, GPIO1, GPIO2, GPIO3, GPIO4
};

//// Set the mode and extended function of a pin
//void
//gpio_peripheral(uint32_t gpio, int func, int pull_up)
//{
//    uint32_t bank_pos = GPIO2PORT(gpio) * 2, pin_pos = (gpio % 32) * 2;
//    if (pin_pos >= 32) {
//        pin_pos -= 32;
//        bank_pos++;
//    }
//    uint32_t sel_bits = (func & 0x03) << pin_pos, mask = ~(0x03 << pin_pos);
//    uint32_t mode = (pull_up ? (pull_up > 0 ? 0x00 : 0x03) : 0x02) << pin_pos;
//    volatile uint32_t *pinsel = &LPC_PINCON->PINSEL0;
//    volatile uint32_t *pinmode = &LPC_PINCON->PINMODE0;
//    irqstatus_t flag = irq_save();
//    pinsel[bank_pos] = (pinsel[bank_pos] & mask) | sel_bits;
//    pinmode[bank_pos] = (pinmode[bank_pos] & mask) | mode;
//    irq_restore(flag);
//}

// Convert a register and bit location back to an integer pin identifier
//static int
//regs_to_pin(GPIO_Type *regs, uint32_t bit)
//{
//    int i;
//    for (i=0; i<ARRAY_SIZE(digital_regs); i++)
//        if (digital_regs[i] == regs)
//            return GPIO(i, ffs(bit)-1);
//    return 0;
//}


///****************************************************************
// * General Purpose Input Output (GPIO) pins
// ****************************************************************/
//
struct gpio_out
gpio_out_setup(uint8_t pin, uint8_t val)
{
    if (GPIO2PORT(pin) >= ARRAY_SIZE(digital_regs))
        goto fail;
    GPIO_Type *regs = digital_regs[GPIO2PORT(pin)];
    struct gpio_out g = { .regs=regs, .bit=GPIO2BIT(pin) };
    gpio_out_reset(g, val);
    return g;
fail:
    shutdown("Not an output pin");
}

void
gpio_out_reset(struct gpio_out g, uint8_t val)
{
    gpio_pin_config_t pin_config = { kGPIO_DigitalOutput, val, kGPIO_NoIntmode };

    //GPIO_Type *regs = g.regs;
    //int pin = regs_to_pin(regs, g.bit);
    irqstatus_t flag = irq_save();
    //regs->FIOPIN = (regs->FIOSET & ~g.bit) | (val ? g.bit : 0);
    //regs->FIODIR |= g.bit;
    //gpio_peripheral(pin, 0, 0);

    GPIO_PinInit(g.regs, g.bit, &pin_config);
    //GPIO_PinWrite(EXAMPLE_LED_GPIO, EXAMPLE_LED_GPIO_PIN, 0U);

    irq_restore(flag);
}

void
gpio_out_toggle_noirq(struct gpio_out g)
{
//    GPIO_Type *regs = g.regs;
//    regs->FIOPIN = regs->FIOSET ^ g.bit;
    GPIO_PortToggle(g.regs, 1u << g.bit);
}

void
gpio_out_toggle(struct gpio_out g)
{
    irqstatus_t flag = irq_save();
    gpio_out_toggle_noirq(g);
    irq_restore(flag);
}

void
gpio_out_write(struct gpio_out g, uint8_t val)
{
//    GPIO_Type *regs = g.regs;
//    if (val)
//        regs->FIOSET = g.bit;
//    else
//        regs->FIOCLR = g.bit;
    GPIO_PinWrite(g.regs, g.bit, val);
}


struct gpio_in
gpio_in_setup(uint8_t pin, int8_t pull_up)
{
    if (GPIO2PORT(pin) >= ARRAY_SIZE(digital_regs))
        goto fail;
    GPIO_Type *regs = digital_regs[GPIO2PORT(pin)];
    struct gpio_in g = { .regs=regs, .bit=GPIO2BIT(pin) };
    gpio_in_reset(g, pull_up);
    return g;
fail:
    shutdown("Not an input pin");
}

void
gpio_in_reset(struct gpio_in g, int8_t pull_up)
{
//    GPIO_Type *regs = g.regs;
//    int pin = regs_to_pin(regs, g.bit);
//    irqstatus_t flag = irq_save();
//    gpio_peripheral(pin, 0, pull_up);
//    regs->FIODIR &= ~g.bit;
//    irq_restore(flag);

    gpio_pin_config_t pin_config = { kGPIO_DigitalInput, pull_up, kGPIO_NoIntmode };
    irqstatus_t flag = irq_save();
    GPIO_PinInit(g.regs, g.bit, &pin_config);
    //GPIO_PinRead(EXAMPLE_LED_GPIO, EXAMPLE_LED_GPIO_PIN);

    irq_restore(flag);
}

uint8_t
gpio_in_read(struct gpio_in g)
{
//    GPIO_Type *regs = g.regs;
//    return !!(regs->FIOPIN & g.bit);
    return (uint8_t)GPIO_PinRead(g.regs, g.bit);
}
