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
#include "inc/hw_uart.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include <string.h>
#include <math.h>
/**
 * main.c
 */
#define TXSIZE 128
#if TXSIZE > 1024
#undef TXSIZE
#define TXSIZE 1024
#endif

uint32_t g_ui32SysClock;
uint8_t count;
#define DATA_ALIGN(var, align)  __attribute__((aligned(align))) var
#pragma DATA_ALIGN(ControlTable, 1024)
uint8_t ControlTable[1024];
uint8_t bufferA[128] = {0};
uint8_t bufferB[128] = {0};
unsigned int rxbuffB;
unsigned int rxbuffA;
uint32_t received_size = 0;
uint32_t file_size = 0;
char file[1024];

typedef enum uart_states{
    IDLE,
    INIT,
    RECEIVING,
    RECV_COMPLETE,
} state;
state dma_state = IDLE;

void sysClock(void){
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_320), 20000000);
}

void configureUART0(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC); //16 Mhz
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0|GPIO_PIN_1);
    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8); // used by udma for burst request
    UARTStdioConfig(0, 115200, 16000000);
}

// buttons PJ0 and PJ1 is configured
void configureGPIOINPUT(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);//enable port j
    while(! SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));//check clock enable
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);// config port j bit 0 to input
    GPIODirModeSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);
}

void CRC_init(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CCM0);
    CRCConfigSet(CCM0_BASE, CRC_CFG_INIT_0|CRC_CFG_SIZE_8BIT|CRC_CFG_TYPE_P1021);
}


void
UARTReceive(uint8_t *data, uint32_t size)
{
    // loop until RX FIFO is empty
    while (!(HWREG(UART0_BASE + UART_O_FR) & UART_FR_RXFE))
    {
        (void)HWREG(UART0_BASE + UART_O_DR);  // Read and discard
    }
    while(size--)
    {
        while((HWREG(UART0_BASE + UART_O_FR) & UART_FR_RXFE));
        *data++ = HWREG(UART0_BASE + UART_O_DR);
    }
}

void dma_init(){
    //channel 8 for UART0 RX
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    //DMA Init
//    HWREG(SYSCTL_RCGCBASE + 0x0000060C) = 0x00000001; // enable dma module clock - takes 5 clock cyles
    HWREG(UDMA_CFG) = 0x00000001; // enable dma controller, set masteren bit
    HWREG(UDMA_CTLBASE) = (uint32_t)&ControlTable;
    HWREG(UART0_BASE + UART_O_DMACTL) = 0x00000007; // rxFIFO dma enabled, recv req disbale if recv error
    //channel assign
    HWREG(UDMA_CHMAP1) = 0; //uart0 rx channel 8[3:0], encoding 0 to select UART-0
    // configure channel attributes
    //channel prio set - high - 1 channel 8
    HWREG(UDMA_PRIOSET) = (1<<8);
    //use primary control structure , channel 8
    HWREG(UDMA_ALTCLR) = (1<<8);
    // use burst or single transfer
    HWREG(UDMA_USEBURSTCLR) = (1<<8);
    // request dma transfer by peripheral
    HWREG(UDMA_REQMASKCLR) = (1<<8);

    /*
    Control Structure   Offset Formula                  Example for Channel 8
    Primary             Base + (Channel * 16)           Base + (8 * 16) = Base + 0x080
    Alternate           Base + 0x200 + (Channel * 16)   Base + 0x200 + (8 * 16) = Base + 0x280
    */

    tDMAControlTable* ct;
    ct = (tDMAControlTable*)HWREG(UDMA_CTLBASE);

    // configure control structures
    ct[0 + 8].ui32Control =
            (3 << 26) |  // SRCINC: No Increment (UART)
            (0 << 24) |  // SRCSIZE: 8-bit
            (0 << 30) |  // DSTINC: 8-bit Increment
            (0 << 28) |  // DSTSIZE: 8-bit
            (3 << 14) |  // ARBSIZE: 8 Transfers
            (TXSIZE-1 << 4) |  // XFERSIZE: TXSIZE-1 Transfers
            (3 << 0);    // XFERMODE: Ping-Pong;//primary Control mode
    ct[0 + 8].pvSrcEndAddr = (void*)(UART0_BASE + UART_O_DR);//primary source pointer
    ct[0 + 8].pvDstEndAddr = (void*)(bufferA + TXSIZE-1);//primary dest pointer

    ct[32 + 8].ui32Control =
            (0 << 30) |  // destination address increment
            (0 << 28) |  // Destination data size: 8-bit
            (3 << 26) |  // SRCINC: No Increment (UART)
            (0 << 24) |  // SRCSIZE: 8-bit
            (3 << 14) |  // ARBSIZE: 8 Transfers
            (TXSIZE-1 << 4) |  // XFERSIZE: 64 Transfers
            (3 << 0);    // XFERMODE: Ping-Pong;//alt control mode
    ct[32 + 8].pvSrcEndAddr = (void*)(UART0_BASE + UART_O_DR);//alt source pointer
    ct[32 + 8].pvDstEndAddr = (void*)(bufferB + TXSIZE-1);//alt dest pointer

    IntMasterEnable();
    UARTIntEnable(UART0_BASE, UART_INT_DMARX);
    IntEnable(INT_UART0);

    //enable channel, bit[n] - channel n
    HWREG(UDMA_ENASET) = (1<<8);
    while(!(HWREG(UDMA_ENASET) & (1 << 8)));

}


void inline buffA_process(){
    memcpy(file + received_size, bufferA, sizeof(bufferA));
    UARTprintf("\nBUFFER A: %s",bufferA);
}
void inline buffB_process(){
    memcpy(file + received_size, bufferB, sizeof(bufferB));
    UARTprintf("\nBUFFER B: %s",bufferB);
}
void UART0_IntHandler(void){
    uint32_t status = UARTIntStatus(UART0_BASE, true); // get interrupts status
    UARTIntClear(UART0_BASE, status); //  clear those interrupts

    //check the mode of primary
    tDMAControlTable* ct;
    ct = (tDMAControlTable*)HWREG(UDMA_CTLBASE);

    uint8_t mode = ct[8].ui32Control & 0x00000007; //extarct first three bits - mode
    dma_state = RECEIVING;
    if(mode == UDMA_MODE_STOP){
        rxbuffA++;

        buffA_process(); //process data recieved
        received_size += 128;
        //resrt transfer
        ct[0 + 8].pvSrcEndAddr = (void*)(UART0_BASE + UART_O_DR);//primary source pointer
        ct[0 + 8].pvDstEndAddr = (void*)(bufferA + TXSIZE-1);//primary dest pointer
        ct[0 + 8].ui32Control =
                (3 << 26) |  // SRCINC: No Increment (UART)
                (2 << 14) |  // ARBSIZE: 4 Transfers
                (63 << 4) |  // XFERSIZE: 64 Transfers
                (3 << 0);    // XFERMODE: Ping-Pong;//primary Control mode
    }

    mode = ct[32 + 8].ui32Control & 0x00000007; //extarct first three bits - mode

    if(mode == UDMA_MODE_STOP){
        rxbuffB++;

        buffB_process(); //process data recieved
        received_size += 128;
        //resrt transfer
        ct[32 + 8].pvSrcEndAddr = (void*)(UART0_BASE + UART_O_DR);//primary source pointer
        ct[32 + 8].pvDstEndAddr = (void*)(bufferB + TXSIZE-1);//primary dest pointer
        ct[32 + 8].ui32Control =
                (3 << 26) |  // SRCINC: No Increment (UART)
                (2 << 14) |  // ARBSIZE: 4 Transfers
                (63 << 4) |  // XFERSIZE: 64 Transfers
                (3 << 0);    // XFERMODE: Ping-Pong;//primary Control mode
    }

    if(received_size >= file_size){
        dma_state = RECV_COMPLETE;
        HWREG(UDMA_ENACLR) = (1 << 8); // Disable DMA
    }
}

void request_file_info(){
UARTwrite("FIL_SIZE", 9);
}

void buttons(){
    uint32_t BUTTON_PRESS_0 = GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0);
    uint32_t BUTTON_PRESS_1 = GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1);

    if(!(BUTTON_PRESS_0 & GPIO_PIN_0)){
        UARTprintf("count = %d\r\n", count++);
        SysCtlDelay(g_ui32SysClock/12); // 250 ms
    }
    else if(!(BUTTON_PRESS_1 & GPIO_PIN_1)){
        UARTprintf("count = %d\r\n", count--);
        SysCtlDelay(g_ui32SysClock/12);
    }
}
//
uint32_t convert(char* src){
    int i = 0;
    uint32_t dest = 0;
    while(src[i] != 's'){
        (dest) *= 10;
        (dest) += src[i++] - '0';
    }
    return dest;
}

int main(void)
{
    sysClock();
    configureUART0();
    configureGPIOINPUT();
    CRC_init();
    dma_init();
    char command[20];
    unsigned int crc = 0;
    UARTprintf("\nuart start \r\n");

    dma_state = INIT;
    strncpy(command, "FILE_INFO",10);
    UARTwrite(command, sizeof(command));
    while(bufferA[0] == 0);
    file_size = convert(bufferA);
    //RECV the file size and store in file_size variable
    crc = convert(bufferA + (uint32_t)(log10(file_size) + 2)); //log10 + 1 - no of bytes, skip s- plus one
    //receive the crc and store in crc variable
    while(1){

//        //clear dma buffer before recv file
//        memset(bufferA, 0, sizeof(bufferA));
//        memset(bufferB, 0, sizeof(bufferB));
//        dma_state = RECEIVING;

        if(dma_state == RECV_COMPLETE) {
            UARTprintf("\nFile received: %s\n", file);
            crc = CRCDataProcess(CCM0_BASE, file, received_size, false);
            UARTprintf("CRC: %u\n", crc);
            while(1); // Or reset for next file
        }

//        SysCtlDelay(g_ui32SysClock/200);
    }
//  return 0;
}
