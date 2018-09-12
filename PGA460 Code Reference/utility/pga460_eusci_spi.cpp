/**
 * File: eusci_spi.c - msp430 USCI SPI implementation
 *
 * EUSCI flavor implementation by Robert Wessels <robertinant@yahoo.com>
 * Copyright (c) 2012 by Rick Kimball <rick@kimballsoftware.com>
 * spi abstraction api for msp430
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 *
 */

#include <msp430.h>
#include <stdint.h>
#include "pga460_spi_430.h"
#include <Energia.h>

// AW: re-allocate SPI port from B0 to B1 pins
static const uint8_t PGA460_SS      = 33;  	/* P4.0 */
static const uint8_t PGA460_SCK     = 34;  	/* P4.3 */
static const uint8_t PGA460_MOSI    = 10;  	/* P4.2 */
static const uint8_t PGA460_MISO    = 9;  	/* P4.1 */

#ifdef __MSP430_HAS_EUSCI_B1__

/**
 * USCI flags for various the SPI MODEs
 *
 * Note: The msp430 UCCKPL tracks the CPOL value. However,
 * the UCCKPH flag is inverted when compared to the CPHA
 * value described in Motorola documentation.
 */

#define SPI_MODE_0 (UCCKPH)		/* CPOL=0 CPHA=0 */
#define SPI_MODE_1 (0)			/* CPOL=0 CPHA=1 */
#define SPI_MODE_2 (UCCKPL | UCCKPH)	/* CPOL=1 CPHA=0 */
#define SPI_MODE_3 (UCCKPL)		/* CPOL=1 CPHA=1 */

#define SPI_MODE_MASK (UCCKPL | UCCKPH)

/**
 * spi_initialize_pga460() - Configure USCI UCB1 for SPI mode
 *
 * P2.0 - CS (active low)
 * P1.5 - SCLK
 * P1.6 - MISO aka SOMI
 * P1.7 - MOSI aka SIMO
 *
 */
void spi_initialize_pga460(void)
{
	/* Put USCI in reset mode, source USCI clock from SMCLK. */
	UCB1CTLW0 = UCSWRST | UCSSEL_2;

	/* SPI in master MODE 0 - CPOL=0 SPHA=0. */
	UCB1CTLW0 |= SPI_MODE_0 | UCMSB | UCSYNC | UCMST;

	/* Set pins to SPI mode. */
	pinMode_int(PGA460_SCK, SPISCK_SET_MODE);
	pinMode_int(PGA460_MOSI, SPIMOSI_SET_MODE);
	pinMode_int(PGA460_MISO, SPIMISO_SET_MODE);


	/* Set initial speed to 4MHz. */
	UCB1BR0 = SPI_CLOCK_DIV4 & 0xFF;
	UCB1BR1 = (SPI_CLOCK_DIV4 >> 8 ) & 0xFF;

	/* Release USCI for operation. */
	UCB1CTLW0 &= ~UCSWRST;
}

/**
 * spi_disable_pga460() - put USCI into reset mode.
 */
void spi_disable_pga460(void)
{
	/* Put USCI in reset mode. */
	UCB1CTLW0 |= UCSWRST;
}

/**
 * spi_send() - send a byte and recv response.
 */
uint8_t spi_send_pga460(const uint8_t _data)
{
	/* Wait for previous tx to complete. */
	while (!(UCB1IFG & UCTXIFG))
		;

	/* Setting TXBUF clears the TXIFG flag. */
	UCB1TXBUF = _data;

	/* Wait for a rx character? */
	while (!(UCB1IFG & UCRXIFG))
		;

	/* Reading clears RXIFG flag. */
	return UCB1RXBUF;
}

/***SPI_MODE_0
 * spi_set_divisor_pga460() - set new clock divider for USCI.
 *
 * USCI speed is based on the SMCLK divided by BR0 and BR1.
 *
 */
void spi_set_divisor_pga460(const uint16_t clkdiv)
{
	/* Hold UCB1 in reset. */
	UCB1CTLW0 |= UCSWRST;

	UCB1BR0 = clkdiv & 0xFF;
	UCB1BR1 = (clkdiv >> 8 ) & 0xFF;

	/* Release for operation. */
	UCB1CTLW0 &= ~UCSWRST;
}

/**
 * spi_set_bitorder_pga460(LSBFIRST=0 | MSBFIRST=1).
 */
void spi_set_bitorder_pga460(const uint8_t order)
{
	/* Hold UCB1 in reset. */
	UCB1CTLW0 |= UCSWRST;

	UCB1CTLW0 = (UCB1CTLW0 & ~UCMSB) | ((order == 1 /*MSBFIRST*/) ? UCMSB : 0); /* MSBFIRST = 1 */

	/* Release for operation. */
	UCB1CTLW0 &= ~UCSWRST;
}

/**
 * spi_set_datamode_pga460() - mode 0 - 3.
 */
void spi_set_datamode_pga460(const uint8_t mode)
{
	/* Hold UCB1 in reset. */
	UCB1CTL1 |= UCSWRST;
	switch(mode) {
	case 0: /* SPI_MODE0 */
		UCB1CTLW0 = (UCB1CTLW0 & ~SPI_MODE_MASK) | SPI_MODE_0;
		break;
	case 1: /* SPI_MODE1 */
		UCB1CTLW0 = (UCB1CTLW0 & ~SPI_MODE_MASK) | SPI_MODE_1;
		break;
	case 2: /* SPI_MODE2 */
		UCB1CTLW0 = (UCB1CTLW0 & ~SPI_MODE_MASK) | SPI_MODE_2;
		break;
	case 4: /* SPI_MODE3 */
		UCB1CTLW0 = (UCB1CTLW0 & ~SPI_MODE_MASK) | SPI_MODE_3;
		break;
	default:
		break;
	}

	/* Release for operation. */
	UCB1CTL1 &= ~UCSWRST;
}
#endif
