#include <msp430.h>
#include <stdint.h>
#include "spi.h"
#include "../defines.h"

#ifndef __MSP430_HAS_USCI_A0__
#error "Error! This MCU doesn't have a USCI peripheral"
#endif

#define SPI_MODE_0 UCCKPH | UCMSB| UCMST | UCSYNC /* CPOL=0 CPHA=0 */
#define SPI_MODE_3 UCCKPL | UCMSB| UCMST | UCSYNC /* CPOL=1 CPHA=1 */

/**
 * spi_initialize() - Initialize and enable the SPI module
 */
void initSPI(void)
{

	SPI_SEL_PORT  |= MOSI_PIN | MISO_PIN | SCLK_PIN;	// configure P3.1, P3.2, P3.3

	UCB0CTL1 |= UCSWRST;            // **Put state machine in reset**
	UCB0CTL0 |= SPI_MODE_0;			// SPI MODE 0 - CPOL=0 CPHA=0

	UCB0CTL1 = UCSSEL_2;			// source USCI clock from SMCLK, put USCI in reset mode
									// note: UCCKPH is inverted from CPHA


	UCB0BR0 = LOBYTE(SPI_400kHz);	// set initial speed 16MHz/400000 = 400kHz
	UCB0BR1 = HIBYTE(SPI_400kHz);
	//UCB0MCTL = 0;                   // No modulation


	SPI_OUT_PORT |= CS_PIN;					// CS on P2.0. start out disabled
	SPI_DIR_PORT |= CS_PIN;					// CS configured as output
	SPI_OUT_PORT |= CS_PIN;

	UCB0CTL1 &= ~UCSWRST;			// release for operation
}

/**
 * spi_send() - send a byte and recv response
 */
uint8_t spi_send(const uint8_t c)
{
    while (!(UCB0IFG & UCTXIFG));           // USCI_B0 TX buffer ready?
		; // wait for previous tx to complete

	UCB0TXBUF = c; // setting TXBUF clears the TXIFG flag

    while (!(UCB0IFG & UCRXIFG));           // USCI_B0 RX buffer ready?
		; // wait for an rx character?

	return UCB0RXBUF; // reading clears RXIFG flag
}

/**
 * spi_receive() - send dummy btye then recv response
 */
uint8_t spi_receive(void) {

    while (!(UCB0IFG & UCTXIFG));           // USCI_B0 TX buffer ready?
		; // wait for previous tx to complete

	UCB0TXBUF = 0xFF; // setting TXBUF clears the TXIFG flag

    while (!(UCB0IFG & UCRXIFG));           // USCI_B0 RX buffer ready?
		; // wait for an rx character?

	return UCB0RXBUF; // reading clears RXIFG flag
}

/**
 * spi_setspeed() - set new clock divider for USCI
 *
 * USCI speed is based on the SMCLK divided by BR0 and BR1
 * initially we start slow (400kHz) to conform to SDCard
 * specifications then we speed up once initialized (16Mhz)
 *
 */
void spi_set_divisor(const uint16_t clkdiv)
{
	UCB0CTL1 |= UCSWRST;		// go into reset state
	UCB0BR0 = LOBYTE(clkdiv);
	UCB0BR1 = HIBYTE(clkdiv);

	UCB0CTL1 &= ~UCSWRST;		// release for operation
}

