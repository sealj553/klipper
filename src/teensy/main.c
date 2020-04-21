//// Main starting point for LPC176x boards.
////
//// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
////
//// This file may be distributed under the terms of the GNU GPLv3 license.
//
//#include "board/armcm_boot.h" // armcm_main
//#include "internal.h" // enable_pclock
//#include "sched.h" // sched_main
//
//
///****************************************************************
// * watchdog handler
// ****************************************************************/
//
//void
//watchdog_reset(void)
//{
//    LPC_WDT->WDFEED = 0xaa;
//    LPC_WDT->WDFEED = 0x55;
//}
//DECL_TASK(watchdog_reset);
//
//void
//watchdog_init(void)
//{
//    LPC_WDT->WDTC = 4000000 / 2; // 500ms timeout
//    LPC_WDT->WDCLKSEL = 1<<31; // Lock to internal RC
//    LPC_WDT->WDMOD = 0x03; // select reset and enable
//    watchdog_reset();
//}
//DECL_INIT(watchdog_init);
//
//
///****************************************************************
// * misc functions
// ****************************************************************/
//
//// Check if a peripheral clock has been enabled
//int
//is_enabled_pclock(uint32_t pclk)
//{
//    return !!(LPC_SC->PCONP & (1<<pclk));
//}
//
//// Enable a peripheral clock
//void
//enable_pclock(uint32_t pclk)
//{
//    LPC_SC->PCONP |= 1<<pclk;
//    if (pclk < 16) {
//        uint32_t shift = pclk * 2;
//        LPC_SC->PCLKSEL0 = (LPC_SC->PCLKSEL0 & ~(0x3<<shift)) | (0x1<<shift);
//    } else {
//        uint32_t shift = (pclk - 16) * 2;
//        LPC_SC->PCLKSEL1 = (LPC_SC->PCLKSEL1 & ~(0x3<<shift)) | (0x1<<shift);
//    }
//}
//
//// Main entry point - called from armcm_boot.c:ResetHandler()
//void
//armcm_main(void)
//{
//    SystemInit();
//    sched_main();
//}

//int main(){
//    return 0;
//}

#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"

#undef ARRAY_SIZE

#include "pin_mux.h"
#include "clock_config.h"
#include "sched.h"

#define EXAMPLE_LED_GPIO (GPIO1)
#define EXAMPLE_LED_GPIO_PIN (24)

#define LPUART_NUM LPUART2
#define LPUART_CLK_FREQ BOARD_DebugConsoleSrcFreq()

#define DEMO_LPUART LPUART2
#define DEMO_LPUART_CLK_FREQ BOARD_DebugConsoleSrcFreq()
#define DEMO_LPUART_IRQn LPUART2_IRQn
#define DEMO_LPUART_IRQHandler LPUART2_IRQHandler

/*! @brief Ring buffer size (Unit: Byte). */
#define DEMO_RING_BUFFER_SIZE 16


uint8_t g_tipString[] =
    "Lpuart functional API interrupt example\r\nBoard receives characters then sends them out\r\nNow please input:\r\n";

/*
  Ring buffer for data input and output, in this example, input data are saved
  to ring buffer in IRQ handler. The main function polls the ring buffer status,
  if there are new data, then send them out.
  Ring buffer full: (((rxIndex + 1) % DEMO_RING_BUFFER_SIZE) == txIndex)
  Ring buffer empty: (rxIndex == txIndex)
*/
uint8_t demoRingBuffer[DEMO_RING_BUFFER_SIZE];
volatile uint16_t txIndex; /* Index of the data to send out. */
volatile uint16_t rxIndex; /* Index of the memory to save new arrived data. */

void DEMO_LPUART_IRQHandler(void)
{
    uint8_t data;
    uint16_t tmprxIndex = rxIndex;
    uint16_t tmptxIndex = txIndex;

    /* If new data arrived. */
    if ((kLPUART_RxDataRegFullFlag)&LPUART_GetStatusFlags(DEMO_LPUART))
    {
        data = LPUART_ReadByte(DEMO_LPUART);

        /* If ring buffer is not full, add data to ring buffer. */
        if (((tmprxIndex + 1) % DEMO_RING_BUFFER_SIZE) != tmptxIndex)
        {
            demoRingBuffer[rxIndex] = data;
            rxIndex++;
            rxIndex %= DEMO_RING_BUFFER_SIZE;
        }
    }
}


void main(void){

    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    /* Board pin, clock, debug console init */
    SystemInit();
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    //BOARD_InitDebugConsole();
    sched_main();

    ///* Print a note to terminal. */
    //PRINTF("\r\n GPIO Driver example\r\n");
    //PRINTF("\r\n The LED is blinking.\r\n");

    //GPIO_PinInit(EXAMPLE_LED_GPIO, EXAMPLE_LED_GPIO_PIN, &led_config);

    //while(1){
    //    SDK_DelayAtLeastUs(400000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    //    GPIO_PortToggle(EXAMPLE_LED_GPIO, 1 << EXAMPLE_LED_GPIO_PIN);
    //}


    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kLPUART_ParityDisabled;
     * config.stopBitCount = kLPUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 0;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    /////lpuart_config_t config;

    /////LPUART_GetDefaultConfig(&config);
    /////config.baudRate_Bps = 115200;
    /////config.enableTx     = true;
    /////config.enableRx     = true;

    /////LPUART_Init(LPUART_NUM, &config, LPUART_CLK_FREQ);

    /////uint8_t txbuff[]   = "\r\nmain\r\n";
    /////LPUART_WriteBlocking(LPUART_NUM, txbuff, sizeof(txbuff) - 1);


    //while(1){
    //    uint8_t ch;
    //    LPUART_ReadBlocking(DEMO_LPUART, &ch, 1);
    //    LPUART_WriteBlocking(DEMO_LPUART, &ch, 1);
    //}

//    lpuart_config_t config;
//    uint16_t tmprxIndex = rxIndex;
//    uint16_t tmptxIndex = txIndex;
//
//    BOARD_ConfigMPU();
//    BOARD_InitPins();
//    BOARD_BootClockRUN();
//
//    /*
//     * config.baudRate_Bps = 115200U;
//     * config.parityMode = kLPUART_ParityDisabled;
//     * config.stopBitCount = kLPUART_OneStopBit;
//     * config.txFifoWatermark = 0;
//     * config.rxFifoWatermark = 0;
//     * config.enableTx = false;
//     * config.enableRx = false;
//     */
//    LPUART_GetDefaultConfig(&config);
//    //config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
//    config.enableTx     = true;
//    config.enableRx     = true;
//
//    LPUART_Init(DEMO_LPUART, &config, DEMO_LPUART_CLK_FREQ);
//
//    /* Send g_tipString out. */
//    LPUART_WriteBlocking(DEMO_LPUART, g_tipString, sizeof(g_tipString) / sizeof(g_tipString[0]));
//
//    /* Enable RX interrupt. */
//    LPUART_EnableInterrupts(DEMO_LPUART, kLPUART_RxDataRegFullInterruptEnable);
//    EnableIRQ(DEMO_LPUART_IRQn);
//
//    while (1) {
//        /* Send data only when LPUART TX register is empty and ring buffer has data to send out. */
//        while (kLPUART_TxDataRegEmptyFlag & LPUART_GetStatusFlags(DEMO_LPUART)) {
//            tmprxIndex = rxIndex;
//            tmptxIndex = txIndex;
//            if (tmprxIndex != tmptxIndex) {
//                LPUART_WriteByte(DEMO_LPUART, demoRingBuffer[txIndex]);
//                txIndex++;
//                txIndex %= DEMO_RING_BUFFER_SIZE;
//            }
//        }
//    }
}
