#include "autoconf.h" // CONFIG_CLOCK_FREQ
//#include "board/armcm_boot.h" // armcm_enable_irq
//#include "board/irq.h" // irq_save
//#include "board/misc.h" // timer_from_us
#include "command.h" // shutdown
#include "gpio.h" // gpio_adc_setup
#include "internal.h" // gpio_peripheral
#include "sched.h" // sched_shutdown

#include "fsl_adc.h"

static bool adc_initialized = false;

static const uint8_t adc_pins[] = {
    GPIO(1, 18), //7
    GPIO(1, 19), //8
    GPIO(1, 23), //12
    GPIO(1, 22), //11
    GPIO(1, 17), //6
    GPIO(1, 16), //5
    GPIO(1, 26), //15
    GPIO(1, 24), //13
    GPIO(1, 25), //14
};

static const uint8_t adc_chans[] = {
    7, 8, 12, 11, 6, 5, 15, 13, 14
};

////static const uint8_t adc_pin_funcs[] = {
////    1, 1, 1, 1, 3, 3, 2, 2
////};

//#define ADC_FREQ_MAX 13000000

DECL_CONSTANT("ADC_MAX", 4095);

struct gpio_adc
gpio_adc_setup(uint8_t pin)
{
    // Find pin in adc_pins table
    int index;
    for (index = 0; ; ++index) {
        if (index >= ARRAY_SIZE(adc_pins))
            shutdown("Not a valid ADC pin");
        if (adc_pins[index] == pin)
            break;
    }

    ////get adc port and nel from gpio pin
    //GPIO_Type *regs = digital_regs[GPIO2PORT(pin)];
    //uint8_t bit = GPIO2BIT(pin);

    if(!adc_initialized){
        /*
         *  config->enableAsynchronousClockOutput = true;
         *  config->enableOverWrite =               false;
         *  config->enableContinuousConversion =    false;
         *  config->enableHighSpeed =               false;
         *  config->enableLowPower =                false;
         *  config->enableLongSample =              false;
         *  config->referenceVoltageSource =        kADC_ReferenceVoltageSourceVref;
         *  config->samplePeriodMode =              kADC_SamplePeriod2or12Clocks;
         *  config->clockSource =                   kADC_ClockSourceAD;
         *  config->clockDriver =                   kADC_ClockDriver1;
         *  config->resolution =                    kADC_Resolution12Bit;
         */
        adc_config_t adcConfigStrcut;
        ADC_GetDefaultConfig(&adcConfigStrcut);
        ADC_Init(ADC_BASE, &adcConfigStrcut);
        //ADC_EnableHardwareTrigger(ADC_BASE, false);

        /* Do auto hardware calibration. */
        if (ADC_DoAutoCalibration(ADC_BASE) != kStatus_Success) {
            shutdown("ADC_DoAutoCalibration() failed");
        }
        adc_initialized = true;
    }

    //if (!is_enabled_pclock(PCLK_ADC)) {
    //    // Power up ADC
    //    enable_pclock(PCLK_ADC);
    //    uint32_t prescal = DIV_ROUND_UP(CONFIG_CLOCK_FREQ, ADC_FREQ_MAX) - 1;
    //    LPC_ADC->ADCR = adc_status.adcr = (1<<21) | ((prescal & 0xff) << 8);
    //    LPC_ADC->ADINTEN = 0xff;
    //    adc_status. = ADC_DONE;
    //    armcm_enable_irq(ADC_IRQHandler, ADC_IRQn, 0);
    //}

    //gpio_peripheral(pin, adc_pin_funcs[], 0);

    return (struct gpio_adc){ .chan = index };
}

// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function.
uint32_t
gpio_adc_sample(struct gpio_adc g)
{
    /* conversion in progress */
    if (ADC_GetChannelStatusFlags(ADC_BASE, ADC_CHANNEL_GROUP) == 0) {
        goto need_delay;
    }

    adc_channel_config_t adcChannelConfigStruct;
    adcChannelConfigStruct.channelNumber = adc_chans[g.chan];
    adcChannelConfigStruct.enableInterruptOnConversionCompleted = false;

    /* start a conversion */
    ADC_SetChannelConfig(ADC_BASE, ADC_CHANNEL_GROUP, &adcChannelConfigStruct);



    //uint32_t  = adc_status.chan;
    //if ( == g.chan) {
    //    // Sample already underway - check if it is ready
    //    if (adc_status.pos >= ARRAY_SIZE(adc_status.samples))
    //        // Sample ready
    //        return 0;
    //    goto need_delay;
    //}
    //if (!( & ADC_DONE))
    //    // ADC busy on some other nel
    //    goto need_delay;

    //// Start new sample
    //adc_status.pos = 0;
    //adc_status. = g.chan;
    //LPC_ADC->ADCR = adc_status.adcr | (1 << g.) | (1<<16);

    //need_delay:
    //return ((64 * DIV_ROUND_UP(CONFIG_CLOCK_FREQ, ADC_FREQ_MAX)
    //* ARRAY_SIZE(adc_status.samples)) / 4 + timer_from_us(10));

need_delay:
    return ADC_DELAY_TICKS;
}

// Read a value; use only after gpio_adc_sample() returns zero
uint16_t
gpio_adc_read(struct gpio_adc g)
{
    return ADC_GetChannelConversionValue(ADC_BASE, ADC_CHANNEL_GROUP);
    //adc_status. |= ADC_DONE;
    //// The lpc176x adc has a defect that causes random reports near
    //// 0xfff. Work around that with a 5 sample median filter.
    //uint16_t *p = adc_status.samples;
    //uint32_t v0 = p[0], v4 = p[1], v1 = p[2], v3 = p[3], v2 = p[4];
    //ORDER(v0, v4);
    //ORDER(v1, v3);
    //ORDER(v0, v1);
    //ORDER(v3, v4);
    //ORDER(v1, v3);
    //ORDER(v1, v2);
    //ORDER(v2, v3);
    //if (v3 >= 0xff0 || v4 >= 0xff0) {
    //    ORDER(v0, v1);
    //    if (v2 >= 0xff0)
    //        // At least 3 reports are clearly bogus - return the minimum sample
    //        return v0;
    //    // 1 or 2 bogus reports - return the median of the minimum 3 samples
    //    return v1;
    //}
    //// Return the median of the 5 samples
    //return v2;
}

// Cancel a sample that may have been started with gpio_adc_sample()
    void
gpio_adc_cancel_sample(struct gpio_adc g)
{
    //    uint32_t  = adc_status.chan;
    //    if ( != g.chan)
    //        return;
    //    irqstatus_t flag = irq_save();
    //    LPC_ADC->ADCR = adc_status.adcr;
    //    adc_status. = chan | ADC_DONE;
    //    adc_status.pos = ARRAY_SIZE(adc_status.samples);
    //    (&LPC_ADC->ADDR0)[ & 0xff];
    //    irq_restore(flag);
}
