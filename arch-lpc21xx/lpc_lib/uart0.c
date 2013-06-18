#include <targets/LPC2000.h>
#include <ctl_api.h>

static unsigned char *rxchar=NULL;

void UARTWriteChar(unsigned char ch) {
  while ((U0LSR & 0x20) == 0);
  U0THR = ch;
}

unsigned char UARTReadChar(void) {
  while ((U0LSR & 0x01) == 0);
  return U0RBR;
}

int UARTReadAvailable(void) {
  return U0LSR & 0x01;
}

static void uartISR(void) {
  /* Read IIR to clear interrupt and find out the cause */
  unsigned iir = U0IIR;

  /* Handle UART interrupt */
  switch ((iir >> 1) & 0x7)
    {
      case 1:
        /* THRE interrupt */
        break;
      case 2:
        /* RDA interrupt */
        if(rxchar)
          *rxchar = U0RBR;
        break;
      case 3:
        /* RLS interrupt */
        break;
      case 6:
        /* CTI interrupt */
        break;
   }
}

void uart0_init(unsigned int baud, unsigned char *rxc) {
  /* Configure UART */
  unsigned int divisor = liblpc2000_get_pclk(liblpc2000_get_cclk(OSCILLATOR_CLOCK_FREQUENCY)) / (16 * baud);
  U0LCR = 0x83; /* 8 bit, 1 stop bit, no parity, enable DLAB */
  U0DLL = divisor & 0xFF;
  U0DLM = (divisor >> 8) & 0xFF;
  U0LCR &= ~0x80; /* Disable DLAB */
  PINSEL0 = PINSEL0 & (~0xF)| 0x5;
  U0FCR = 1;

  /* Setup UART RX interrupt */
  ctl_set_isr(6, 0, CTL_ISR_TRIGGER_FIXED, uartISR, 0);
  ctl_unmask_isr(6);
  U0IER = 1;

  rxchar=rxc;
}

void __putchar(int ch) {
  if (ch == '\n')
    UARTWriteChar('\r');
  UARTWriteChar(ch);
}
