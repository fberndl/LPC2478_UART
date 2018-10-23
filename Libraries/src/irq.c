/*
 * irq.c
 *
 *  Created on: Dec 1, 2010
 *      Author: tw
 */

#include "lpc246x.h"
#include "irq.h"

void ISRDummyHandler(void){

	/*** Do Interrupt handling here ***/

	VICVectAddr = 0;       /* Acknowledge Interrupt */
}


/*** initialize the interrupt controller ***/
void init_VIC(void)
{
    unsigned int i = 0;
    unsigned int *vect_addr, *vect_cntl;

    /* initialize VIC */
    VICIntEnClr = 0xffffffff;
    VICVectAddr = 0;
    VICIntSelect = 0;

    /* set all the vector and vector control register to 0 */
    for ( i = 0; i < VIC_SIZE; i++ )
    {
		vect_addr = (unsigned int *)(VIC_BASE_ADDR + VECT_ADDR_INDEX + i*4);
		vect_cntl = (unsigned int *)(VIC_BASE_ADDR + VECT_CNTL_INDEX + i*4);
		*vect_addr = 0x0;
		*vect_cntl = 0xF;
    }
    return;
}

/*** routine to install the interupt ***/
unsigned int install_irq (unsigned int irq_nr, void (*HandlerAddr)(void), unsigned int priority)
{
		unsigned int *vect_addr;
		unsigned int *vect_cntl;

    VICIntEnClr = 1 << irq_nr;	/* Disable Interrupt */
    if ( irq_nr >= VIC_SIZE )
    {
		return ( FALSE );
    }
    else
    {
		/* find first un-assigned VIC address for the handler */
		vect_addr = (unsigned int *)(VIC_BASE_ADDR + VECT_ADDR_INDEX + irq_nr*4);
		vect_cntl = (unsigned int *)(VIC_BASE_ADDR + VECT_CNTL_INDEX + irq_nr*4);
		*vect_addr = (unsigned int)HandlerAddr;	/* set interrupt vector */
		*vect_cntl = priority;
		VICIntEnable = 1 << irq_nr;	/* Enable Interrupt */
		return( TRUE );
    }
}
