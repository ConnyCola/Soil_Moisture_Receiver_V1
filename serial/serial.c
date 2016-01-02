#include <msp430.h>
#include <stdint.h>
#include "stdarg.h"
#include "serial.h"
#include "../typedefs.h"
#include "../defines.h"

int toSer = 1; // 1: print to Serial   0: print to String
char inde = 0;
char *STR;

void initSerial(void) {
/*
	  UART_DIR_PORT |= UART_TX_PIN;					// P1.6 als Ausgang
	  UART_SEL_PORT |= UART_RX_PIN + UART_TX_PIN;	// P1.5 P1.6 = USCI_A0 rXD/tXD

	  UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
	  UCA0CTL1 |= UCSSEL_1;                     // CLK = ACLK

	  UCA0BR0 = 0x03;                           // 32kHz/9600=3.41 (see User's Guide)
	  UCA0BR1 = 0x00;                           //
	  UCA0MCTL = UCBRS_3 + UCBRF_0;               // Modulation UCBRSx=3, UCBRFx=0
	  UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt


	  UCA0CTL1 &= ~UCSWRST;                     	// **Initialize USCI state machine**
	  //IE2 |= UCA0RXIE;                          	// Enable USCI_A0 RX interrupt
	  //IE2 |= UCA0TXIE;                          	// Enable USCI_A0 TX interrupt

*/


	  PMAPPWD = 0x02D52;                        // Get write-access to port mapping regs
	  P1MAP5 = PM_UCA0RXD;                      // Map UCA0RXD output to P1.5
	  P1MAP6 = PM_UCA0TXD;                      // Map UCA0TXD output to P1.6
	  PMAPPWD = 0;                              // Lock port mapping registers

	  P1DIR |= BIT6;                            // Set P1.6 as TX output
	  P1SEL |= BIT5 + BIT6;                     // Select P1.5 & P1.6 to UART function

	  UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
	  UCA0CTL1 |= UCSSEL_1;                     // CLK = ACLK
	  UCA0BR0 = 0x03;                           // 32kHz/9600=3.41 (see User's Guide)
	  UCA0BR1 = 0x00;                           //
	  UCA0MCTL = UCBRS_3+UCBRF_0;               // Modulation UCBRSx=3, UCBRFx=0
	  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	  UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt

	    //while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
	    //UCA0TXBUF = 'A';

	  //printf("\r\n\r\n  ___       __   ________  ________        \r\n |\\  \\     |\\  \\|\\   ____\\|\\   ___  \\     \r\n \\ \\  \\    \\ \\  \\ \\  \\___|\\ \\  \\\\ \\  \\    \r\n  \\ \\  \\  __\\ \\  \\ \\_____  \\ \\  \\\\ \\  \\   \r\n   \\ \\  \\|\\__\\_\\  \\|____|\\  \\ \\  \\\\ \\  \\  \r\n    \\ \\____________\\____\\_\\  \\ \\__\\\\ \\__\\ \r\n     \\|____________|\\_________\\|__| \\|__| \r\n                   \\|_________|           \r\n    ___           __   __  _ __             \r\n   / _ \\_______ _/ /__/ /_(_/ /____ ____ _  \r\n  / ___/ __/ _ `/  '_/ __/ /  '_/ // /  ' \\ \r\n /_/  /_/  \\_,_/_/\\_\\\\__/_/_/\\_\\\\_,_/_/_/_/ \r\n");
}

static const unsigned long dv[] = {
//  4294967296      // 32 bit unsigned max
    1000000000,     // +0
     100000000,     // +1
      10000000,     // +2
       1000000,     // +3
        100000,     // +4
//       65535      // 16 bit unsigned max
         10000,     // +5
          1000,     // +6
           100,     // +7
            10,     // +8
             1,     // +9
};

void xtoa(unsigned long x, const unsigned long *dp)
{
    char c;
    unsigned long d;
    if(x) {
        while(x < *dp) ++dp;
        do {
            d = *dp++;
            c = '0';
            while(x >= d) ++c, x -= d;
            putc(c);
        } while(!(d & 1));
    } else
        putc('0');
}

void puth(unsigned n)
{
    static const char hex[16] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
    putc(hex[n & 15]);
}

void puts(char *s) {
	char c;

	// Loops through each character in string 's'
	while (c = *s++) {
		if(toSer)
			sendByte(c);
		else
			printToString(c);
	}
}
/**
 * puts() is used by printf() to display or send a character. This function
 *     determines where printf prints to. For this case it sends a character
 *     out over UART.
 **/
void putc(unsigned b) {
	if(toSer)
		sendByte(b);
	else
		printToString(b);
}

/**
 * Sends a single byte out through UART
 **/
void sendByte(unsigned char byte )
{
    while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
	UCA0TXBUF = byte;
}

int printf(char *format, ...)
{
    char c;
    int i;
    long n;

    va_list a;
    va_start(a, format);
    while(c = *format++) {
        if(c == '%') {
            switch(c = *format++) {
                case 's':                       // String
                    puts(va_arg(a, char*));
                    break;
                case 'c':                       // Char
                    putc(va_arg(a, char));
                    break;
                case 'i':                       // 16 bit Integer
                case 'u':                       // 16 bit Unsigned
                    i = va_arg(a, int);
                    if(c == 'i' && i < 0) i = -i, putc('-');
                    xtoa((unsigned)i, dv + 5);
                    break;
                case 'l':                       // 32 bit Long
                case 'd':
                case 'n':                       // 32 bit uNsigned loNg
                    n = va_arg(a, long);
                    if(c == 'l' &&  n < 0) n = -n, putc('-');
                    xtoa((unsigned long)n, dv);
                    break;
                case 'X':                       // 16 bit heXadecimal
                    i = va_arg(a, int);
                    puth(i >> 12);
                    puth(i >> 8);
                    puth(i >> 4);
                    puth(i);
                    break;
                case 'x':                       // 8 bit heXadecimal
                    i = va_arg(a, int);
                    puth(i >> 4);
                    puth(i);
                    break;
                case 0: return inde;
                default: goto bad_fmt;
            }
        }
        else
        	bad_fmt:    putc(c);
    }
    va_end(a);
    return inde;
}

void setToSerial(u_char tst)
{
	toSer = tst? 1:0;
}

void printToString(u_char byte)
{
	STR[inde] = byte;
	inde++;
}

void setStringPointer(void* string)
{
	STR = string;
	inde=0;
}

void printfSTR(void* string, char *format, int i){
	setStringPointer(string);
	toSer = 0;
	printf(format, i);
	toSer = 1;

}

void send_CMD(CMD cmd){

	printf("%c",cmd.cmd);

	if(cmd.val1 < 1000)
		printf(" ");
	if(cmd.val1 < 100)
		printf(" ");
	if(cmd.val1 < 10)
		printf(" ");
	printf("%i",cmd.val1);

	if(cmd.val2 < 1000)
		printf(" ");
	if(cmd.val2 < 100)
		printf(" ");
	if(cmd.val2 < 10)
		printf(" ");
	printf("%i",cmd.val2);

	printf("\n");

}
