#include "lpc246x.h"
#include "init.h"

/** Do not step through this function. Run through the whole function at once when debugging. **/

int SystemInit(void)
{
	unsigned int MValue, NValue;

	MValue = 0;
	NValue = 0;

	PINSEL0 |= 0x00000000;
	PINSEL1 |= 0x00000000;

	MEMMAP = 1;

	PCONP |= 0x80000000;

	if(PLLSTAT & (1<<25))
	{
		PLLCON = 1;
		PLLFEED = 0xAA;
		PLLFEED = 0x55;
	}

	PLLCON = 0;						/* Disable PLL, disconnected */
	PLLFEED = 0xaa;
	PLLFEED = 0x55;

	SCS |= 0x20;					/* enable main oscialltor */
	while(!(SCS & 0x40));			/* wait until ready */

	CLKSRCSEL = 0x1;				/* use main oscillator */

	PLLCFG = 15 | (0 << 16);
	PLLFEED = 0xAA;
	PLLFEED = 0x55;

	PLLCON = 1;
	PLLFEED = 0xAA;
	PLLFEED = 0x55;

	CCLKCFG = 7; /*configure cclk divisor so that the pll output results in 48MHz (former config resulted in 96MHz)*/
	USBCLKCFG = 3;

	while(((PLLSTAT & (1 << 26)) == 0));

	MValue = PLLSTAT & 0x00007FFF;
	NValue = (PLLSTAT & 0x00FF0000) >> 16;

	while((MValue != 15) && (NValue != 0));

	PLLCON = 3;
	PLLFEED = 0xAA;
	PLLFEED = 0x55;

	while(((PLLSTAT & (1 << 25)) == 0));

	PCLKSEL0 = 0xAAAAAAAA;
	PCLKSEL1 = 0xAAAAAAAA;


	MAMCR = 0;

	MAMTIM = 4;					/* wait 4 clock cycles */

	MAMCR = 2;					/* fully enabled */

	return (0);
}
