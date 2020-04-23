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

//#include "fsl_iomuxc.h"
//#include "fsl_common.h"
//#include "fsl_lpuart.h"
////#include "fsl_lpuart_edma.h"
////#include "fsl_dmamux.h"
//#include "fsl_gpio.h"
//#include "pin_mux.h"
//#include "board.h"

#include "board.h"
#include "fsl_lpuart.h"

#include "pin_mux.h"
#include "clock_config.h"

#define DEMO_LPUART LPUART2
#define DEMO_LPUART_CLK_FREQ BOARD_DebugConsoleSrcFreq()
#define UART_BUFFER_LENGTH 1

void LPUART_UserCallback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);

//static void kick_tx(void);

lpuart_handle_t g_lpuartHandle;
lpuart_config_t config;
lpuart_transfer_t xfer;
lpuart_transfer_t sendXfer;
lpuart_transfer_t receiveXfer;

uint8_t g_txBuffer[UART_BUFFER_LENGTH] = {0};
uint8_t g_rxBuffer[UART_BUFFER_LENGTH] = {0};
volatile bool rxBufferEmpty            = true;
volatile bool txBufferFull             = false;
volatile bool txOnGoing                = false;
volatile bool rxOnGoing                = false;

void LPUART_UserCallback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData) {
//top:
    switch(status) {
        /* IDLE */
        case kStatus_LPUART_TxIdle:
            {
                uint8_t data;
                int ret = serial_get_tx_byte(&data);
                //if(ret){
                g_txBuffer[0] = data;
                //g_txBuffer[0] = g_rxBuffer[0];
                LPUART_TransferSendNonBlocking(DEMO_LPUART, &g_lpuartHandle, &sendXfer);
                //}
            }
            break;

        case kStatus_LPUART_RxIdle:
            {
                //finished receiving, get another
                serial_rx_byte(g_rxBuffer[0]);
                LPUART_TransferReceiveNonBlocking(DEMO_LPUART, &g_lpuartHandle, &receiveXfer, NULL);
            }
            break;
            //case kStatus_LPUART_IdleLineDetected:
            //    break;

            /* BUSY */
        //case kStatus_LPUART_TxBusy:
        //    {
        //    }
        //    break;
        //case kStatus_LPUART_RxBusy:
        //    {
        //    }
        //    break;

            /* ERROR */
            // case kStatus_LPUART_Error:
            //     break;
            // case kStatus_LPUART_FramingError:
            //     break;
            // case kStatus_LPUART_NoiseError:
            //     break;
            // case kStatus_LPUART_ParityError:
            //     break;
    };



    //if (status == kStatus_LPUART_TxIdle) {
    //    txBufferFull = false;
    //    txOnGoing    = false;

    //    //finished sending a byte - send another
    //    //kick_tx();

    //} else if (status == kStatus_LPUART_RxIdle) {
    //    //rxBufferEmpty = false;
    //    //rxOnGoing     = false;

    //    rxBufferEmpty = true;
    //    //received a byte - get it
    //    serial_rx_byte(g_rxBuffer[0]);

    //    rxOnGoing = true;
    //    LPUART_TransferReceiveNonBlocking(DEMO_LPUART, &g_lpuartHandle, &receiveXfer, NULL);

    //}

    ///* If RX is idle and g_rxBuffer is empty, start to read data to g_rxBuffer. */
    ////if ((!rxOnGoing) && rxBufferEmpty) {
    ////}

    ///* If TX is idle and g_txBuffer is full, start to send data. */
    //if ((!txOnGoing) && txBufferFull) {
    //    txOnGoing = true;
    //    LPUART_TransferSendNonBlocking(DEMO_LPUART, &g_lpuartHandle, &sendXfer);
    //}

    ///* If g_txBuffer is empty and g_rxBuffer is full, copy g_rxBuffer to g_txBuffer. */
    //if ((!rxBufferEmpty) && (!txBufferFull)) {
    //    uint8_t data;
    //    int ret = serial_get_tx_byte(&data);
    //    g_txBuffer[0] = data;
    //    txBufferFull  = true;
    //    //rxBufferEmpty = true;

    //    //if(ret){
    //    //goto top;
    //    //}

    //    //g_txBuffer[0] = g_rxBuffer[0];
    //    //rxBufferEmpty = true;
    //    //txBufferFull  = true;

    //}
}


//Write tx bytes to the serial port
//static void
//kick_tx(void)
//{
//    //while(1){
//        uint8_t data;
//        serial_get_tx_byte(&data);
//
//        g_txBuffer[0] = data;
//        sendXfer.dataSize = UART_BUFFER_LENGTH;
//        LPUART_TransferSendNonBlocking(DEMO_LPUART, &g_lpuartHandle, &xfer);
//    //}
//
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
    //if(!rxOnGoing){
    //    irqstatus_t flag = irq_save();
    //    kick_tx();
    //    irq_restore(flag);
    //}
}

DECL_CONSTANT_STR("RESERVE_PINS_serial", "P0.3,P0.2");

    void
serial_init(void)
{
    /* Default config:
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kLPUART_ParityDisabled;
     * config.stopBitCount = kLPUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 0;
     * config.enableTx = false;
     * config.enableRx = false; */
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = CONFIG_SERIAL_BAUD;
    config.enableTx     = true;
    config.enableRx     = true;

    LPUART_Init(DEMO_LPUART, &config, DEMO_LPUART_CLK_FREQ);
    LPUART_TransferCreateHandle(DEMO_LPUART, &g_lpuartHandle, LPUART_UserCallback, NULL);

    /* Send g_tipString out. */
    //xfer.data     = (uint8_t*)"\r\ninit\r\n";
    //xfer.dataSize = 8;
    //txOnGoing     = true;
    //LPUART_TransferSendNonBlocking(DEMO_LPUART, &g_lpuartHandle, &xfer);

    /* Wait send finished */
    //while (txOnGoing) {}

    /* Start to echo. */
    sendXfer.data        = g_txBuffer;
    sendXfer.dataSize    = UART_BUFFER_LENGTH;
    receiveXfer.data     = g_rxBuffer;
    receiveXfer.dataSize = UART_BUFFER_LENGTH;

    LPUART_UserCallback(DEMO_LPUART, &g_lpuartHandle, kStatus_LPUART_TxIdle, NULL);

    //while (1) {
    //    /* If RX is idle and g_rxBuffer is empty, start to read data to g_rxBuffer. */
    //    if ((!rxOnGoing) && rxBufferEmpty) {
    //        rxOnGoing = true;
    //        LPUART_TransferReceiveNonBlocking(DEMO_LPUART, &g_lpuartHandle, &receiveXfer, NULL);
    //    }

    //    /* If TX is idle and g_txBuffer is full, start to send data. */
    //    if ((!txOnGoing) && txBufferFull) {
    //        txOnGoing = true;
    //        LPUART_TransferSendNonBlocking(DEMO_LPUART, &g_lpuartHandle, &sendXfer);
    //    }

    //    /* If g_txBuffer is empty and g_rxBuffer is full, copy g_rxBuffer to g_txBuffer. */
    //    if ((!rxBufferEmpty) && (!txBufferFull)) {
    //        memcpy(g_txBuffer, g_rxBuffer, UART_BUFFER_LENGTH);
    //        rxBufferEmpty = true;
    //        txBufferFull  = true;
    //    }
    //}


}
DECL_INIT(serial_init);
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
