/*
 * grok_code.c
 *
 *  Created on: 25-Feb-2025
 *      Author: 14169
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"
#include "driverlib/crc.h"
#include "driverlib/udma.h"
#include "inc/hw_uart.h"
#include "inc/hw_types.h"
#include "inc/hw_udma.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"

uint32_t g_ui32SysClock;
#define DATA_ALIGN(var, align)  __attribute__((aligned(align))) var
#pragma DATA_ALIGN(ControlTable, 1024)
uint8_t ControlTable[1024];
uint8_t bufferA[64];
uint8_t bufferB[64];
unsigned int rxbuffA = 0;
unsigned int rxbuffB = 0;

uint32_t file_size = 0;
uint32_t received_size = 0;
volatile bool transfer_complete = false;

void sysClock(void) {
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_320), 20000000);
}

void configureUART0(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC); // 16 MHz
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);
    UARTStdioConfig(0, 115200, 16000000);
}

void configureGPIOINPUT(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIODirModeSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);
}

void CRC_init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CCM0);
    CRCConfigSet(CCM0_BASE, CRC_CFG_INIT_0 | CRC_CFG_SIZE_8BIT | CRC_CFG_TYPE_P1021);
}

void dma_init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    HWREG(UDMA_CFG) = 0x00000001; // Enable DMA controller
    HWREG(UDMA_CTLBASE) = (uint32_t)&ControlTable;
    HWREG(UART0_BASE + UART_O_DMACTL) = 0x00000007; // RX DMA enabled
    HWREG(UDMA_CHMAP1) = 0; // UART0 RX on channel 8
    HWREG(UDMA_PRIOSET) = (1 << 8); // High priority
    HWREG(UDMA_ALTCLR) = (1 << 8); // Use primary control structure
    HWREG(UDMA_USEBURSTCLR) = (1 << 8); // Allow burst/single
    HWREG(UDMA_REQMASKCLR) = (1 << 8); // Enable peripheral request

    tDMAControlTable* ct = (tDMAControlTable*)HWREG(UDMA_CTLBASE);
    ct[8].ui32Control = (3 << 26) | (0 << 24) | (0 << 30) | (0 << 28) | (3 << 14) | (63 << 4) | (3 << 0); // Ping-pong, 64 bytes
    ct[8].pvSrcEndAddr = (void*)(UART0_BASE + UART_O_DR);
    ct[8].pvDstEndAddr = (void*)(bufferA + 63);
    ct[40].ui32Control = (0 << 30) | (0 << 28) | (3 << 26) | (0 << 24) | (3 << 14) | (63 << 4) | (3 << 0);
    ct[40].pvSrcEndAddr = (void*)(UART0_BASE + UART_O_DR);
    ct[40].pvDstEndAddr = (void*)(bufferB + 63);

    HWREG(UDMA_ENASET) = (1 << 8); // Enable channel
    IntMasterEnable();
    UARTIntEnable(UART0_BASE, UART_INT_DMARX);
    IntEnable(INT_UART0);
}

void buffA_process(void) {
    received_size += 64;
    UARTprintf("\nBUFFER A: %s", bufferA);
}

void buffB_process(void) {
    received_size += 64;
    UARTprintf("\nBUFFER B: %s", bufferB);
}

void UART0_IntHandler(void) {
    uint32_t status = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, status);

    tDMAControlTable* ct = (tDMAControlTable*)HWREG(UDMA_CTLBASE);
    uint8_t modeA = ct[8].ui32Control & 0x07;  // Primary (bufferA)
    uint8_t modeB = ct[40].ui32Control & 0x07; // Alternate (bufferB)

    if (modeA == UDMA_MODE_STOP) {
        rxbuffA++;
        buffA_process();
        ct[8].pvSrcEndAddr = (void*)(UART0_BASE + UART_O_DR);
        ct[8].pvDstEndAddr = (void*)(bufferA + 63);
        ct[8].ui32Control = (ct[8].ui32Control & ~0x07) | UDMA_MODE_PINGPONG; // Explicitly set ping-pong mode
    }

    if (modeB == UDMA_MODE_STOP) {
        rxbuffB++;
        buffB_process();
        ct[40].pvSrcEndAddr = (void*)(UART0_BASE + UART_O_DR);
        ct[40].pvDstEndAddr = (void*)(bufferB + 63);
        ct[40].ui32Control = (ct[40].ui32Control & ~0x07) | UDMA_MODE_PINGPONG; // Explicitly set ping-pong mode
    }

    if (received_size >= file_size && file_size > 0) {
        transfer_complete = true;
    }
}

void request_file_info(void) {
    UARTwrite("FIL_SIZE", 9);
    char buff[32];
    UARTReceive(buff, 32); // Expect "size crc" response
    sscanf(buff, "%lu", &file_size); // Simplified: only parse size for now
    UARTprintf("File size: %lu\r\n", file_size);
    UARTwrite("BEGIN", 6);
    received_size = 0;
    transfer_complete = false;
}

int main(void) {
    sysClock();
    configureUART0();
    configureGPIOINPUT();
    CRC_init();
    dma_init();

    UARTprintf("UART transmitting\r\n");

    while (1) {
        uint32_t BUTTON_PRESS_0 = GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0);

        if (!(BUTTON_PRESS_0 & GPIO_PIN_0)) {
            request_file_info();
            SysCtlDelay(g_ui32SysClock / 12); // Debounce
            while (!transfer_complete) {
                // Wait for DMA to complete
            }
            UARTprintf("Transfer complete, received %lu bytes\r\n", received_size);
        }

        SysCtlDelay(g_ui32SysClock / 200);
    }
}


