#include <lpc214x.h>
#include <stdlib.h>
#include <stdint.h>
#include "sys_time.h"

#include "uart0.h"

typedef struct{
    uint8_t cbuf[UART_CBUFF_SIZE];
    uint8_t tail;
    uint8_t head;
} circular_buffer;

volatile circular_buffer rxbuf = {{0}, 0, 0};

// FIXME make circular_buffer for sends as well (with sends on interruptions)
void UARTWriteChar(uint8_t ch){
    while ((UART0_LSR & 0x20) == 0);
    UART0_THR = ch;
}

// blocking read
uint8_t UARTReadChar(){
    uint8_t c;

    while(rxbuf.tail == rxbuf.head);

    c = rxbuf.cbuf[rxbuf.tail];
    rxbuf.tail = (rxbuf.tail + 1)%UART_CBUFF_SIZE;

    return c;
}

int UARTReadAvailable(){
    return rxbuf.tail != rxbuf.head;
}

void uartISR() __attribute__((interrupt("IRQ")));
void uartISR(){
    // Read IIR to clear interrupt and find out the cause
    unsigned iir = UART0_IIR;
    unsigned char tmp;

    if (!(iir & 0x1)){
        // Handle UART interrupt
        switch ((iir >> 1) & 0x7){
        case 1:
            // THRE interrupt
            break;
        case 2:
            // RDA interrupt
            tmp = (rxbuf.head+1)%UART_CBUFF_SIZE;

            if(tmp != rxbuf.tail){
                rxbuf.cbuf[rxbuf.head] = UART0_RBR;
                rxbuf.head = tmp;
            }
            else{ // byte lost
                // FIXME, build some error stats & tell user
            }
            break;
        case 3:
            // RLS interrupt, error condition detected
            tmp = UART0_LSR; // read LSR to clear interrupt
            // FIXME, build some error stats & tell user
            break;
        case 6:
            // CTI interrupt
            break;
        }
    }

    VIC_VectAddr = (unsigned)0; // updates priority hardware
}

unsigned int uart0_init(unsigned int baud){
    VIC_IntEnClr = VIC_IntEnClr_UART0;

    // Configure UART
    unsigned int divisor = PCLK / (16 * baud);
    UART0_LCR = 0x83; // 8 bit, 1 stop bit, no parity, enable DLAB
    UART0_DLL = divisor & 0xFF;
    UART0_DLM = (divisor >> 8) & 0xFF;
    UART0_LCR &= ~0x80; // Disable DLAB
    PCB_PINSEL0 = (PCB_PINSEL0 & (~0xF)) | 0x5; // setup TXD0 and RXD0 pins
    UART0_FCR = 1; // enable FIFO, interrupt is activated for each received character

    // Setup UART RX interrupt
    VIC_VectCntl12 = VIC_VectCntl_ENABLE | VIC_Channel_UART0;
    VIC_VectAddr12 = (unsigned int)uartISR;
    VIC_IntSelect &= ~VIC_IntSelect_UART0; // IRQ (not FIQ)
    VIC_IntEnable = VIC_IntEnable_UART0;

    UART0_IER = 1;

    return PCLK / (16 * divisor);
}
