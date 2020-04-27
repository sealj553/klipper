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

#define LPUART LPUART2
#define LPUART_CLK_FREQ BOARD_DebugConsoleSrcFreq()

#define LPUART_IRQn LPUART2_IRQn
#define LPUART_IRQHandler LPUART2_IRQHandler

#define UART_BUFFER_LENGTH 1

void serial_disable_tx_irq(void);
void serial_enable_tx_irq(void);

lpuart_config_t config;

void LPUART_IRQHandler(void) {
    uint8_t data;
    
    if(LPUART_GetStatusFlags(LPUART) & kLPUART_RxDataRegFullFlag) { /* rx finished */
        serial_rx_byte(LPUART_ReadByte(LPUART));
    } else if(LPUART_GetStatusFlags(LPUART) & kLPUART_TxDataRegEmptyFlag) { /* tx finished */
        int ret = serial_get_tx_byte(&data);
        if(ret){
            serial_disable_tx_irq();
        } else {
            LPUART_WriteByte(LPUART, data);
        }
    }
}

void serial_enable_tx_irq(void) {
    LPUART_EnableInterrupts(LPUART, kLPUART_TxDataRegEmptyInterruptEnable);
}
void serial_disable_tx_irq(void) {
    LPUART_DisableInterrupts(LPUART, kLPUART_TxDataRegEmptyInterruptEnable);
}

//TODO: add proper dynamic serial pin config
/////#ifdef LPUART2
DECL_CONSTANT_STR("RESERVE_PINS_serial", "P1.18,P1.19");

void serial_init(void) {
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

    LPUART_Init(LPUART, &config, LPUART_CLK_FREQ);
    //LPUART_TransferCreateHandle(LPUART, &g_lpuartHandle, LPUART_UserCallback, NULL);

    /* Enable RX interrupt. */
    LPUART_EnableInterrupts(LPUART, kLPUART_RxDataRegFullInterruptEnable);
    EnableIRQ(LPUART_IRQn);

    //LPUART_EnableInterrupts(LPUART, kLPUART_TxDataRegEmptyInterruptEnable);
    //LPUART_DisableInterrupts(LPUART, kLPUART_TxDataRegEmptyInterruptEnable);


    /* Send g_tipString out. */
    //xfer.data     = (uint8_t*)"\r\ninit\r\n";
    //xfer.dataSize = 8;
    //txOnGoing     = true;
    //LPUART_TransferSendNonBlocking(LPUART, &g_lpuartHandle, &xfer);

    /* Wait send finished */
    //while (txOnGoing) {}

    /* Start to echo. */
    //sendXfer.data        = g_txBuffer;
    //sendXfer.dataSize    = UART_BUFFER_LENGTH;
    //receiveXfer.data     = g_rxBuffer;
    //receiveXfer.dataSize = UART_BUFFER_LENGTH;

    //LPUART_UserCallback(LPUART, &g_lpuartHandle, kStatus_LPUART_TxIdle, NULL);

    //while (1) {
    //    /* If RX is idle and g_rxBuffer is empty, start to read data to g_rxBuffer. */
    //    if ((!rxOnGoing) && rxBufferEmpty) {
    //        rxOnGoing = true;
    //        LPUART_TransferReceiveNonBlocking(LPUART, &g_lpuartHandle, &receiveXfer, NULL);
    //    }

    //    /* If TX is idle and g_txBuffer is full, start to send data. */
    //    if ((!txOnGoing) && txBufferFull) {
    //        txOnGoing = true;
    //        LPUART_TransferSendNonBlocking(LPUART, &g_lpuartHandle, &sendXfer);
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
