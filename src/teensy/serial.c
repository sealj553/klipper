// lpc176x serial port
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/armcm_boot.h" // armcm_enable_irq
#include "autoconf.h" // CONFIG_SERIAL_BAUD
#include "board/irq.h" // irq_save
#include "board/serial_irq.h" // serial_rx_data
#include "command.h" // DECL_CONSTANT_STR
#include "internal.h" // gpio_peripheral
#include "sched.h" // DECL_INIT

#include "fsl_iomuxc.h"
#include "fsl_common.h"
#include "fsl_lpuart.h"
#include "fsl_lpuart_edma.h"
#include "fsl_dmamux.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "board.h"

#define EXAMPLE_LED_GPIO (GPIO1)
#define EXAMPLE_LED_GPIO_PIN (24)


//#define LPUART_CLK_FREQ BOARD_DebugConsoleSrcFreq()
//#define LPUART LPUART2
////#define LPUART_IRQn LPUART2_IRQn
////#define LPUART_IRQHandler LPUART2_IRQHandler


#define DEMO_LPUART LPUART2
#define DEMO_LPUART_CLK_FREQ BOARD_DebugConsoleSrcFreq()
#define LPUART_TX_DMA_CHANNEL 0U
#define LPUART_RX_DMA_CHANNEL 1U
#define LPUART_TX_DMA_REQUEST kDmaRequestMuxLPUART2Tx
#define LPUART_RX_DMA_REQUEST kDmaRequestMuxLPUART2Rx
#define EXAMPLE_LPUART_DMAMUX_BASEADDR DMAMUX
#define EXAMPLE_LPUART_DMA_BASEADDR DMA0
#define ECHO_BUFFER_LENGTH 8



lpuart_edma_handle_t g_lpuartEdmaHandle;
edma_handle_t g_lpuartTxEdmaHandle;
edma_handle_t g_lpuartRxEdmaHandle;
AT_NONCACHEABLE_SECTION_INIT(uint8_t g_tipString[]) =
    "LPUART EDMA example\r\nSend back received data\r\nEcho every 8 characters\r\n";
AT_NONCACHEABLE_SECTION_INIT(uint8_t g_txBuffer[ECHO_BUFFER_LENGTH]) = {0};
AT_NONCACHEABLE_SECTION_INIT(uint8_t g_rxBuffer[ECHO_BUFFER_LENGTH]) = {0};
volatile bool rxBufferEmpty                                          = true;
volatile bool txBufferFull                                           = false;
volatile bool txOnGoing                                              = false;
volatile bool rxOnGoing                                              = false;

void LPUART_UserCallback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData) {
    userData = userData;

    if (kStatus_LPUART_TxIdle == status) {
        txBufferFull = false;
        txOnGoing    = false;
    }

    if (kStatus_LPUART_RxIdle == status) {
        rxBufferEmpty = false;
        rxOnGoing     = false;
    }
}

/*
   Ring buffer for data input and output, in this example, input data are saved
   to ring buffer in IRQ handler. The main function polls the ring buffer status,
   if there are new data, then send them out.
   Ring buffer full: (((rxIndex + 1) % _RING_BUFFER_SIZE) == txIndex)
   Ring buffer empty: (rxIndex == txIndex)
   */


// Write tx bytes to the serial port
//    static void
//kick_tx(void)
//{
//    //for (;;) {
//    //    if (!(LPC_UART0->LSR & (1<<5))) {
//    //        // Output fifo full - enable tx irq
//    //        LPC_UART0->IER = 0x03;
//    //        break;
//    //    }
//    //    uint8_t data;
//    //    int ret = serial_get_tx_byte(&data);
//    //    if (ret) {
//    //        // No more data to send - disable tx irq
//    //        LPC_UART0->IER = 0x01;
//    //        break;
//    //    }
//    //    LPC_UART0->THR = data;
//    //}
//}

//    void
//UART0_IRQHandler(void)
//{
//    //uint32_t iir = LPC_UART0->IIR, status = iir & 0x0f;
//    //if (status == 0x04)
//    //    serial_rx_byte(LPC_UART0->RBR);
//    //else if (status == 0x02)
//    //    kick_tx();
//}

    void
serial_enable_tx_irq(void)
{
    //if (LPC_UART0->LSR & (1<<5)) {
    //    irqstatus_t flag = irq_save();
    //    kick_tx();
    //    irq_restore(flag);
    //}
}

DECL_CONSTANT_STR("RESERVE_PINS_serial", "P0.3,P0.2");

    void
serial_init(void)
{




    lpuart_config_t lpuartConfig;
    edma_config_t config;
    lpuart_transfer_t xfer;
    lpuart_transfer_t sendXfer;
    lpuart_transfer_t receiveXfer;

    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();

    /* Initialize the LPUART. */
    /*
     * lpuartConfig.baudRate_Bps = 115200U;
     * lpuartConfig.parityMode = kLPUART_ParityDisabled;
     * lpuartConfig.stopBitCount = kLPUART_OneStopBit;
     * lpuartConfig.txFifoWatermark = 0;
     * lpuartConfig.rxFifoWatermark = 0;
     * lpuartConfig.enableTx = false;
     * lpuartConfig.enableRx = false;
     */
    LPUART_GetDefaultConfig(&lpuartConfig);
    lpuartConfig.baudRate_Bps = 115200;
    lpuartConfig.enableTx     = true;
    lpuartConfig.enableRx     = true;

    LPUART_Init(DEMO_LPUART, &lpuartConfig, DEMO_LPUART_CLK_FREQ);

#if defined(FSL_FEATURE_SOC_DMAMUX_COUNT) && FSL_FEATURE_SOC_DMAMUX_COUNT
    /* Init DMAMUX */
    DMAMUX_Init(EXAMPLE_LPUART_DMAMUX_BASEADDR);
    /* Set channel for LPUART */
    DMAMUX_SetSource(EXAMPLE_LPUART_DMAMUX_BASEADDR, LPUART_TX_DMA_CHANNEL, LPUART_TX_DMA_REQUEST);
    DMAMUX_SetSource(EXAMPLE_LPUART_DMAMUX_BASEADDR, LPUART_RX_DMA_CHANNEL, LPUART_RX_DMA_REQUEST);
    DMAMUX_EnableChannel(EXAMPLE_LPUART_DMAMUX_BASEADDR, LPUART_TX_DMA_CHANNEL);
    DMAMUX_EnableChannel(EXAMPLE_LPUART_DMAMUX_BASEADDR, LPUART_RX_DMA_CHANNEL);
#endif
    /* Init the EDMA module */
    EDMA_GetDefaultConfig(&config);
    EDMA_Init(EXAMPLE_LPUART_DMA_BASEADDR, &config);
    EDMA_CreateHandle(&g_lpuartTxEdmaHandle, EXAMPLE_LPUART_DMA_BASEADDR, LPUART_TX_DMA_CHANNEL);
    EDMA_CreateHandle(&g_lpuartRxEdmaHandle, EXAMPLE_LPUART_DMA_BASEADDR, LPUART_RX_DMA_CHANNEL);

    /* Create LPUART DMA handle. */
    LPUART_TransferCreateHandleEDMA(DEMO_LPUART, &g_lpuartEdmaHandle, LPUART_UserCallback, NULL, &g_lpuartTxEdmaHandle, &g_lpuartRxEdmaHandle);

    /* Send g_tipString out. */
    xfer.data     = g_tipString;
    xfer.dataSize = sizeof(g_tipString) - 1;
    txOnGoing     = true;
    LPUART_SendEDMA(DEMO_LPUART, &g_lpuartEdmaHandle, &xfer);

    /* Wait send finished */
    while (txOnGoing) {}

    /* Start to echo. */
    sendXfer.data        = g_txBuffer;
    sendXfer.dataSize    = ECHO_BUFFER_LENGTH;
    receiveXfer.data     = g_rxBuffer;
    receiveXfer.dataSize = ECHO_BUFFER_LENGTH;

    while (1) {
        /* If RX is idle and g_rxBuffer is empty, start to read data to g_rxBuffer. */
        if ((!rxOnGoing) && rxBufferEmpty) {
            rxOnGoing = true;
            LPUART_ReceiveEDMA(DEMO_LPUART, &g_lpuartEdmaHandle, &receiveXfer);
        }

        /* If TX is idle and g_txBuffer is full, start to send data. */
        if ((!txOnGoing) && txBufferFull) {
            txOnGoing = true;
            LPUART_SendEDMA(DEMO_LPUART, &g_lpuartEdmaHandle, &sendXfer);
        }

        /* If g_txBuffer is empty and g_rxBuffer is full, copy g_rxBuffer to g_txBuffer. */
        if ((!rxBufferEmpty) && (!txBufferFull)) {
            memcpy(g_txBuffer, g_rxBuffer, ECHO_BUFFER_LENGTH);
            rxBufferEmpty = true;
            txBufferFull  = true;
        }
    }










    //IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_02_LPUART2_TX, 0);
    //IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_03_LPUART2_RX, 0);
    //IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_02_LPUART2_TX, 0x10B0);
    //IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_03_LPUART2_RX, 0x10B0);

    //// Setup baud
    //LPC_UART0->LCR = (1<<7); // set DLAB bit
    //enable_pclock(PCLK_UART0);
    //uint32_t pclk = SystemCoreClock;
    //uint32_t div = pclk / (CONFIG_SERIAL_BAUD * 16);
    //LPC_UART0->DLL = div & 0xff;
    //LPC_UART0->DLM = (div >> 8) & 0xff;
    //LPC_UART0->FDR = 0x10;
    //LPC_UART0->LCR = 3; // 8N1 ; clear DLAB bit

    //// Enable fifo
    //LPC_UART0->FCR = 0x01;

    //// Setup pins
    //gpio_peripheral(GPIO(0, 3), 1, 0);
    //gpio_peripheral(GPIO(0, 2), 1, 0);

    //// Enable receive irq
    //armcm_enable_irq(UART0_IRQHandler, UART0_IRQn, 0);
    //LPC_UART0->IER = 0x01;
}
DECL_INIT(serial_init);
