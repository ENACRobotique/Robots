/*
 * bufferized_UART.c
 *
 *  Created on: 12 mai 2014
 *      Author: quentin
 */

#include "UART_bufferized.h"
#include <stdint.h>


typedef struct{
    uint8_t cbuf[UART_BUFFERIZED_CBUFF_SIZE];
    uint8_t tail;
    uint8_t head;
} circular_buffer;

volatile circular_buffer rxbuf = {{0}, 0, 0};

void uartb_init(unsigned long speed){
    // Initialize the UART. Set the baud rate, number of data bits, turn off
    // parity, number of stop bits, and stick mode.
    UARTConfigSetExpClk(UART_BUFFERIZED_BASE, SysCtlClockGet(), speed, UART_BUFFERIZED_FLAGS);

    // Enable the UART.
    UARTEnable(UART_BUFFERIZED_BASE);

    // Use hardware FIFO to reduce the amount of interruptions
    UARTFIFOEnable(UART_BUFFERIZED_BASE);

    // Set fifo limit for interruption triggering at 7/8 th of the fifo size
    UARTFIFOLevelSet(UART_BUFFERIZED_BASE,UART_FIFO_TX7_8,UART_FIFO_RX7_8);

    // Flush characters. Spin here until bus is empty
    while(UARTCharGetNonBlocking(UART_BUFFERIZED_BASE)>=0);

    // Register UART interrupt
    UARTIntRegister(UART_BUFFERIZED_BASE,uartb_intHandler);


    // Enable UART interruptions
    UARTIntEnable(UART_BUFFERIZED_BASE,UART_INT_RT|UART_INT_RX);

}

void uartb_deinit(){
    UARTDisable(UART_BUFFERIZED_BASE);
}


void uartb_intHandler(){
    int tmp=0;

    // detect the event that triggered the interrupt
    unsigned long intStatus=UARTIntStatus(UART_BUFFERIZED_BASE,1);
    // Clear the interrupt (done early because rtfm)
    UARTIntClear(UART_BUFFERIZED_BASE,intStatus);

    // if it is on RX fifo limit or RX timeout, put these bits in circular buffer
    if (intStatus==(UART_INT_RT | UART_INT_RX)){
        UARTIntDisable(UART_BUFFERIZED_BASE,(UART_INT_RT | UART_INT_RX));
        while (UARTCharsAvail(UART_BUFFERIZED_BASE)){
            // RDA interrupt
            tmp = (rxbuf.head+1)%UART_BUFFERIZED_CBUFF_SIZE;

            if(tmp != rxbuf.tail){
                rxbuf.cbuf[rxbuf.head] = UARTCharGetNonBlocking(UART_BUFFERIZED_BASE);
                rxbuf.head = tmp;
            }
        }
        UARTIntEnable(UART_BUFFERIZED_BASE,(UART_INT_RT | UART_INT_RX));
    }
    // xxx if it is on TX fifo limit, what should we do ? loop until fifo is free again ? useless if blocking writes are used.

    // otherwise, discard whitout doing anything (done at the beginning of the function, see doc for "why ?".
}


int  uartb_readAvail(){
    return rxbuf.tail != rxbuf.head;
}

// blocking read
char uartb_readChar(){
    uint8_t c;

    while(rxbuf.tail == rxbuf.head); //blocking

    UARTIntDisable(UART_BUFFERIZED_BASE,(UART_INT_RT | UART_INT_RX));

    c = rxbuf.cbuf[rxbuf.tail];
    rxbuf.tail = (rxbuf.tail + 1)%UART_BUFFERIZED_CBUFF_SIZE;

    UARTIntEnable(UART_BUFFERIZED_BASE,(UART_INT_RT | UART_INT_RX));

    return c;
}

// blocking write
void  uartb_writeChar(char c){
    UARTCharPut(UART_BUFFERIZED_BASE,c);
}

// non-blocking write, return 1 if success (char put into TX hardware FIFO), 0 otherwise (TX FIFO full)
int  uartb_writeCharNonBlocking(char c){
    return UARTCharPutNonBlocking(UART_BUFFERIZED_BASE,c);
}
