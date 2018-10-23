#include "lpc246x.h"
#include "lpc2478_lib.h"

void setPortPinDir(char port, char pin)
{
	switch(port){
		case 1: /* configure pin of Port 1 */
				break;
		case 2: FIO2DIR = (1<<pin);
				break;
		case 3: /* configure pin of Port 3 */
				break;
		default:
				break;
	}
}

void clrPortPin(char port, char pin)
{
	switch(port){
		case 1: /* configure pin of Port 1 */
				break;
		case 2: FIO2CLR = (1<<pin);
				break;
		case 3: /* configure pin of Port 3 */
				break;
		default:
				break;
	}
}

void setPortPin(char port, char pin)
{
	switch(port){
		case 1: /* configure pin of Port 1 */
				break;
		case 2: FIO2SET = (1<<pin);
				break;
		case 3: /* configure pin of Port 3 */
				break;
		default:
				break;
	}
}

void delayMs(unsigned short delayInMs)
{
	T1TCR = 0x02;							/* stop and reset timer  */
	T1PR = 0x00;							/* set prescaler to zero */
	T1MR0 = delayInMs * (25000000 / 1000);	/* calculate Timer Match Register Value */
	T1IR = 0xFF;							/* reset all interrupt flags */
	T1MCR = 0x04;							/* stop timer on match */
	T1TCR = 0x01;							/* start timer */

	while(T1TCR & 0x01);					/* wait until delay time has elapsed */
}
