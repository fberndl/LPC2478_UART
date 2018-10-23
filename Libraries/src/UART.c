/*
 * UART.c
 *
 *  Created on: Dec 20, 2013
 *      Author: Floran Berndl; Bernd Glatz;
 */
#include <stdio.h>

#include "UART.h"
#include "cb.h"
#include "irq.h"

/* DEBUG FLAG */
/*#define DEBUG */

/* Fractional UART LUTs */
#define BR_LUT_SIZE 72

/*Tables for looking up fractional baud rate values.*/
const float FRLut[BR_LUT_SIZE] = {
	1.000, 1.067, 1.071, 1.077, 1.083, 1.091, 1.100, 1.111, 1.125, 1.133, 1.143, 1.154, 1.167, 1.182, 1.200, 1.214, 1.222, 1.231,
	1.250, 1.267, 1.273, 1.286, 1.300, 1.308, 1.333, 1.357, 1.364, 1.375, 1.385, 1.400, 1.417, 1.429, 1.444, 1.455, 1.462, 1.467,
	1.500, 1.533, 1.538, 1.545, 1.556, 1.571, 1.583, 1.600, 1.615, 1.625, 1.636, 1.643, 1.667, 1.692, 1.700, 1.714, 1.727, 1.733,
	1.750, 1.769, 1.778, 1.786, 1.800, 1.818, 1.833, 1.846, 1.857, 1.867, 1.875, 1.889, 1.900, 1.909, 1.917, 1.923, 1.929, 1.933 };

const float DIVADDVALLut[BR_LUT_SIZE] = {
	0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 1.0, 2.0, 1.0, 2.0, 1.0, 3.0, 2.0, 3.0,
	1.0, 4.0, 3.0, 2.0, 3.0, 4.0, 1.0, 5.0, 4.0, 3.0, 5.0, 2.0, 5.0, 3.0, 4.0, 5.0, 6.0, 7.0,
	1.0, 8.0, 7.0, 6.0, 5.0, 4.0, 7.0, 3.0, 8.0, 5.0, 7.0, 9.0, 2.0, 9.0, 7.0, 5.0, 8.0, 11.0,
	3.0, 10.0, 7.0, 11.0, 4.0, 9.0, 5.0, 11.0, 6.0, 13.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0 };

const float MULVALLut[BR_LUT_SIZE] = {
	1.0, 15.0, 14.0, 13.0, 12.0, 11.0, 10.0, 9.0, 8.0, 15.0, 7.0, 13.0, 6.0, 11.0, 5.0, 14.0, 9.0, 13.0,
	4.0, 15.0, 11.0, 7.0, 10.0, 13.0, 3.0, 14.0, 11.0, 8.0, 13.0, 5.0, 12.0, 7.0, 9.0, 11.0, 13.0, 15.0,
	2.0, 15.0, 13.0, 11.0, 9.0, 7.0, 12.0, 5.0, 13.0, 8.0, 11.0, 14.0, 3.0, 13.0, 10.0, 7.0, 11.0, 15.0,
	4.0, 13.0, 9.0, 14.0, 5.0, 11.0, 6.0, 13.0, 7.0, 15.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0 };

/**
 * @brief 		Interrupt handler for UART0
 *
 * This function is called if an interrupt for UART0 occurs
 */
void UART0Handler(void)
{
	UART_STAT[0].callback_UART();
	if(LPC_UART0->REG3.IIR == 0x4) /* Receive Data Available */
	{
		if(putRecvCB(LPC_UART0, LPC_UART0->REG1.RBR)!=EOK)
		{
			/* get too much characters -
			 * some characters maybe lost */
		}
	}
	if(LPC_UART0->REG3.IIR == 0x02) /* THRE */
	{
		/* THRE Interrupt */
	}
	VICVectAddr = 0;		/* Acknowledge Interrupt */
}

/**
 * @brief 		Interrupt handler for UART1
 *
 * This function is called if an interrupt for UART1 occurs
 */
void UART1Handler()
{
	UART_STAT[1].callback_UART();
	if(LPC_UART1->REG3.IIR == 0x4) /* Receive Data Available */
	{
		if(putRecvCB(LPC_UART1, LPC_UART1->REG1.RBR)!=EOK)
		{
			/* get too much characters -
			 * some characters maybe lost */
		}
	}
	if(LPC_UART1->REG3.IIR == 0x02) /* THRE */
	{
		/* THRE Interrupt */
	}
	VICVectAddr = 0;		/* Acknowledge Interrupt */
}

/**
 * @brief 		Interrupt handler for UART2
 *
 * This function is called if an interrupt for UART2 occurs
 */
void UART2Handler()
{
	UART_STAT[2].callback_UART();
	if(LPC_UART2->REG3.IIR == 0x4) /* Receive Data Available */
	{
		if(putRecvCB(LPC_UART2, LPC_UART2->REG1.RBR)!=EOK)
		{
			/* get too much characters -
			 * some characters maybe lost */
		}
	}
	if(LPC_UART2->REG3.IIR == 0x02) /* THRE */
	{
		/* THRE Interrupt */
	}
	VICVectAddr = 0;		/* Acknowledge Interrupt */
}

/**
 * @brief		Interrupt handler for UART3
 *
 * This function is called if an interrupt for UART3 occurs
 */
void UART3Handler()
{
	UART_STAT[3].callback_UART();
	if(LPC_UART3->REG3.IIR == 0x4) /* Receive Data Available */
	{
		if(putRecvCB(LPC_UART3, LPC_UART3->REG1.RBR)!=EOK)
		{
			/* get too much characters -
			 * some characters maybe lost */
		}
	}
	if(LPC_UART3->REG3.IIR == 0x02) /* THRE */
	{
		/* THRE Interrupt */
	}
	VICVectAddr = 0;		/* Acknowledge Interrupt */
}

/**
 * @brief		Setup power for specific device
 *
 * This function set/enables the power for the specific
 * UART device
 *
 * @param[in]	dev	UART peripheral selected, should be:
 * 					- LPC_UART0: UART0 peripheral
 * 					- LPC_UART1: UART1 peripheral
 * 					- LPC_UART2: UART2 peripheral
 * 					- LPC_UART3: UART3 peripheral
 * @return		None
 */
static void uart_enable_power(UART_T *dev)
{
	/* Set up power for UART module */
	if(dev == LPC_UART0)
	{
		/* Power */
		PCONP |= PCUART0;
	}
	if(dev == LPC_UART1)
	{
		/* Power */
		PCONP |= PCUART1;
	}
	if(dev == LPC_UART2)
	{
		/* Power */
		PCONP |= PCUART2;
	}
	if(dev == LPC_UART3)
	{
		/* Power */
		PCONP |= PCUART3;
	}
}

/**
 * @brief		Setup clock for specific device
 *
 * This function set/enables the clock for the specific
 * UART device
 *
 * @param[in]	dev	- UART peripheral selected, should be:
 * 					- LPC_UART0: UART0 peripheral
 * 					- LPC_UART1: UART1 peripheral
 * 					- LPC_UART2: UART2 peripheral
 * 					- LPC_UART3: UART3 peripheral
 * @param[in]	divisor - Divisor for the Core Clock
 * @return		None
 */
static void uart_set_clock(UART_T *dev, uint8_t divisor)
{
	uint8_t uClk;

	switch (divisor)
	{
	case 1:
		uClk = 0x01;
		break;
	case 2:
		uClk = 0x02;
		break;
	case 4:
		uClk = 0x00;
		break;
	case 8:
		uClk = 0x03;
		break;
	default:
		uClk = 0x01;
		break;
	}

	/* Set up clock UART module */
	if(dev == LPC_UART0)
	{
		/* Clock */
		PCLKSEL0 &= ~((uint32_t)(3<< PCLK_UART0));
		PCLKSEL0 |= ((uint32_t)(uClk << PCLK_UART0));
	}
	if(dev == LPC_UART1)
	{
		/* Clock */
		PCLKSEL0 &= ~((uint32_t)(3<< PCLK_UART1));
		PCLKSEL0 |= ((uint32_t)(uClk << PCLK_UART1));
	}
	if(dev == LPC_UART2)
	{
		/* Clock */
		PCLKSEL1 &= ~((uint32_t)(3<< PCLK_UART2));
		PCLKSEL1 |= ((uint32_t)(uClk << PCLK_UART2));
	}
	if(dev == LPC_UART3)
	{
		/* Clock */
		PCLKSEL1 &= ~((uint32_t)(3<< PCLK_UART3));
		PCLKSEL1 |= ((uint32_t)(uClk << PCLK_UART3));
	}
}

/*
 * @brief		Calculate divaddval and mulval for fractional divider
 *
 * Get the fraction values (mulVal and divAddVal) for
 * the given FRest value from Look up tables. Look through
 * the FR LUT and find the entry which have the smallest
 * difference between the FRest value and the lookup
 * table value.
 *
 * @param[in]	FRest 		- estimated fractional rest,
 * 							  calculated from DLest
 * @param[in]	divaddval	- found register value in LUT
 * @param[in]	mulval		- found register value in LUT
 *
 * @return		EOK if useful values are found in LUT
 * 				EINTERNAL if no useful value is stored in LUT
 */
static int getFRval(double FRest, float *divaddval, float *mulval) {
	float lastdiff = -1;
	float thisdiff;
	int index;

	for (index = 0; index<BR_LUT_SIZE; index++)
	{
		/*Difference between LUT and estimated value*/
		if (FRest > FRLut[index])
		{
			thisdiff = FRest - FRLut[index];
		}
		else
		{
			thisdiff = FRLut[index] - FRest;
		}

		/*If the difference is bigger than the last time,
		* the last value is the nearest one:
		* e.g.FRest = 1.075
		* index=0 FR=1.000 thisdiff=0.75  lastdiff=-1
		* index=1 FR=1.067 thisdiff=0.008 lastdiff=0.75
		* index=2 FR=1.071 thisdiff=0.004 lastdiff=0.008
		* index=3 FR=1.077 thisdiff=0.002 lastdiff=0.004
		* index=4 FR=1.083 thisdiff=0.008 lastdiff=0.002
		*
		*/
		if (lastdiff != -1 && thisdiff > lastdiff)
		{
			/* return fractional calc values */
			*divaddval = DIVADDVALLut[index - 1];
			*mulval = MULVALLut[index - 1];
			return EOK;
		}
		lastdiff = thisdiff;
	}
	return EINTERNAL;
}

/**
 * @brief		Calculate different values for fractional divider
 * 				and prescaler for a specific baudrate for UART module
 *
 * 	This function calculate with the algorithm from the
 * 	datasheet p.440 the best values for divaddval, mulval,
 * 	dlm and dll. Due to the algorithm, the Baudrate has an max
 * 	accuracy of 1.1%
 *
 * @param[in]	dev	- UART peripheral selected, should be:
 * 					- LPC_UART0: UART0 peripheral
 * 					- LPC_UART1: UART1 peripheral
 * 					- LPC_UART2: UART2 peripheral
 * 					- LPC_UART3: UART3 peripheral
 * @param[in]	baudrate - specific baudrate, given from user
 *
 * @return		EOK if successful
 * 				 EINTERNAL if not successful
 */
static int32_t uart_set_divisors(UART_T *dev, uint32_t baudrate, uint8_t udivisor)
{
	uint32_t PCLK;
	uint16_t DLest;
#ifdef DEBUG
	uint16_t uartBaudRate;
#endif
	float  divaddval, mulval;
	uint8_t dlm=0, dll=0;
	uint8_t	 cnt;

	/* calculate peripheral clock */
	PCLK = CLK_HZ / udivisor;

	/* check if result of DLest calculation
	 *  is an integer, then DIVADDVAL = 0, MULVAL = 1 */
	if( ( PCLK % ( 16 * baudrate ) ) == 0)
	{
		divaddval = 0;
		mulval = 1;
		DLest = (uint32_t)(PCLK / (float)(16 * baudrate));
		dlm = DLest / 256;
		dll = DLest % 256;
	}
	else
	{
		float FRest = 1.5;

		for(cnt = 0; cnt < 9; cnt++)
		{
			/* calculate DLest */
			DLest = (uint32_t)(PCLK / (float)(16 * baudrate * FRest));
			FRest = (float)((float)PCLK / (float)(16 * baudrate * DLest));

			/* if FRest is out of range */
			if( (FRest>1.9) || (FRest<1.1) )
			{
				/* choose another FRest from 1.1 to 1.9 */
				FRest = 1.1 + (cnt / 10);
				/* if FRest above 1.9, then end with an error */
				if(cnt == 9)
				{
					/* cannot find correct values */
					return ECALCBAUD;
				}
			}
			else
			{
				/* value found, end for-loop */
				break;
			}
		}

		/* found FRest ,lookup Table from Datasheet p.440*/
		if(getFRval(FRest, &divaddval, &mulval)!=EOK)
		{
			return EINTERNAL;
		}

#ifdef DEBUG
		/*Calc actual baud rate achieved*/
		uartBaudRate = (uint16_t)(PCLK/ (16 * DLest*(1 + (divaddval / mulval))));
#endif

		/* DLM = DLest[15:8] and DLL = DLest[7:0] */
		dlm = (DLest & 0xFF00) >> 8;
		dll = (DLest & 0x00FF);
	}

	/* Set up fractional UART module */
	dev->LCR = UART_LCR_DLAB_EN; /*Enable access to DLM,DLL and FDR*/
    dev->REG2.DLM = dlm;
	dev->REG1.DLL = dll;
	dev->FDR = (((uint8_t)mulval) << 4) | (uint8_t)divaddval;
	dev->LCR &= ~UART_LCR_DLAB_EN; /*DLAB = 0*/

	return EOK;
}

/**
 * @brief		Routine to install the interrupts for UART 0-3
 *
 *
 * @param[in]	dev	- UART peripheral selected, should be:
 * 					- LPC_UART0: UART0 peripheral
 * 					- LPC_UART1: UART1 peripheral
 * 					- LPC_UART2: UART2 peripheral
 * 					- LPC_UART3: UART3 peripheral
 *@param void Pointer to user interrupt routine callback
 *
 * @return		EOK if successful
 * 				 EINTERNAL if not successful
 */
static int32_t uart_installIrq(UART_T *dev, void(*callback)(void))
{
	if(dev == LPC_UART0)
		{
			if ( install_irq( UART0_INT, UART0Handler, 5  ) == FALSE )
			{
				return EINTERNAL;
			}
			UART_STAT[0].callback_UART = callback;
		}
		if(dev == LPC_UART1)
		{
			if ( install_irq( UART1_INT, UART1Handler, 3  ) == FALSE )
			{
				return EINTERNAL;
			}
			UART_STAT[1].callback_UART = callback;
		}
		if(dev == LPC_UART2)
		{
			if ( install_irq( UART2_INT, UART2Handler, 4  ) == FALSE )
			{
				return EINTERNAL;
			}
			UART_STAT[2].callback_UART = callback;
		}
		if(dev == LPC_UART3)
		{
			if ( install_irq( UART3_INT, UART3Handler, HIGHEST_PRIORITY  ) == FALSE )
			{
				return EINTERNAL;
			}
			UART_STAT[3].callback_UART = callback;
		}

		return EOK;
}

/**
 * @brief		Initialises the dev UART peripheral according to the specified
 *               parameters in the UART_cfg struct.
 *
 * This function initialises a given Uart peripheral
 * according to the specified parameters in the cfg struct
 * and calculate the best possible baudrate.
 *
 * @param[in]	dev	UART peripheral selected, should be:
 *   			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @param[in]	UART_CFG_T Pointer to a UART_CFG_Type structure
 *                    that contains the configuration information for the
 *                    specified UART peripheral.
 *
 *@param void Pointer to user interrupt routine callback
 *
 * @return		EOK if successful
 * 				EINTERNAL if not successful
 */
int32_t uart_init(UART_T *dev, UART_CFG_T *cfg, void(*callback)(void))
{
	uint32_t tmp;
	uint8_t divisor = 1;

	/* Check input parameter */
	if(PARAM_UARTx(dev)==EPARAM)
	{
		return EPARAM;
	}

	/* set up power */
	uart_enable_power(dev);

	/* set up clock */
	uart_set_clock(dev, divisor);

	/* set all unused registers to 0 when the specific
	 * module is used, otherwise -> don't touch
	 */
	if(dev == LPC_UART1)
	{
		dev->MCR = 0;
	}
	if(dev == LPC_UART3)
	{
		dev->ICR = 0;
	}

	/* calculate accurate values for best baudrate */
	if(uart_set_divisors(dev, cfg->Baud_rate, divisor)!=EOK)
	{
		return EINTERNAL;
	}

	tmp = (dev->LCR & (UART_LCR_DLAB_EN | UART_LCR_BREAK_EN)) & UART_LCR_BITMASK;

	/* set Databits */
	switch (cfg->Databits){
	case UART_DATABIT_5:
		tmp |= UART_LCR_WLEN5;
		break;
	case UART_DATABIT_6:
		tmp |= UART_LCR_WLEN6;
		break;
	case UART_DATABIT_7:
		tmp |= UART_LCR_WLEN7;
		break;
	case UART_DATABIT_8:
	default:
		tmp |= UART_LCR_WLEN8;
		break;
	}

	/* set Parity mode */
	if (cfg->Parity == UART_PARITY_NONE)
	{
		/* Do nothing... */
	}
	else
	{
		tmp |= UART_LCR_PARITY_EN;
		switch (cfg->Parity)
		{
		case UART_PARITY_ODD:
			tmp |= UART_LCR_PARITY_ODD;
			break;

		case UART_PARITY_EVEN:
			tmp |= UART_LCR_PARITY_EVEN;
			break;

		case UART_PARITY_SP1:
			tmp |= UART_LCR_PARITY_F_1;
			break;

		case UART_PARITY_SP0:
			tmp |= UART_LCR_PARITY_F_0;
			break;
		default:
			break;
		}
	}

	/* set stop bits */
	switch (cfg->Stopbits){
	case UART_STOPBIT_2:
		tmp |= UART_LCR_STOPBIT_SEL;
		break;
	case UART_STOPBIT_1:
	default:
		/* Do no thing... */
		break;
	}

	/* write back to LCR, configure FIFO and disable Tx */
	dev->LCR = (uint8_t)(tmp & UART_LCR_BITMASK);

	/* check which buffer should be used */
	if(cfg->Buffer==UART_SW_BUF)	/* Software buffer when selected */
	{
		initCB(dev);
		dev->REG3.FCR = 0x00;
	}
	else 	/* otherwise always hardware buffer are used */
	{
		/* Enable and reset TX and RX FIFO. */
		dev->REG3.FCR = 0x07;
	}

	/* install irq routine */
	if(uart_installIrq(dev, callback)!=EOK)
	{
		return EINTERNAL;
	}

	/*  enable IRQs: Receive Data Available and THR Empty Interrupt */
	dev->REG2.IER = (UART_IER_RBRINT_EN|UART_IER_THREINT_EN);

	return EOK;
}

/**
 * @brief		Enable specific UART Pins
 *
 * This function enables specific UART pins with the right function
 *
 * @param[in]	dev	UART peripheral selected, should be:
 * 					- LPC_UART0: UART0 peripheral
 * 					- LPC_UART1: UART1 peripheral
 * 					- LPC_UART2: UART2 peripheral
 * 					- LPC_UART3: UART3 peripheral
 * @param[in]	flag LSB is for TX enable and one left of LSB is for
 * 					RX enable
 *
 * @return		None
 */
void uart_enable(UART_T *dev, uint8_t flag)
{
	if(dev == LPC_UART0)
	{
		if(flag && PIN_RX_ENABLE)
		{
			PINSEL0 &= ~((uint32_t)(PIN_FUNC_MASK<<PIN_RX_UART0));
			PINSEL0 |= ((uint32_t)(PIN_FUNC_SEL_1<<PIN_RX_UART0));
		}
		if(flag && PIN_TX_ENABLE)
		{
			PINSEL0 &= ~((uint32_t)(PIN_FUNC_MASK<<PIN_TX_UART0));
			PINSEL0 |= ((uint32_t)(PIN_FUNC_SEL_1<<PIN_TX_UART0));
		}
	}
	if(dev == LPC_UART1)
	{
		if(flag && PIN_RX_ENABLE)
		{
			PINSEL0 &= ~((uint32_t)(PIN_FUNC_MASK<<PIN_RX_UART1));
			PINSEL0 |= ((uint32_t)(PIN_FUNC_SEL_1<<PIN_RX_UART1));
		}
		if(flag && PIN_TX_ENABLE)
		{
			PINSEL1 &= ~((uint32_t)(PIN_FUNC_MASK<<PIN_TX_UART1));
			PINSEL1 |= ((uint32_t)(PIN_FUNC_SEL_1<<PIN_TX_UART1));
		}
	}
	if(dev == LPC_UART2)
	{
		if(flag && PIN_RX_ENABLE)
		{
			PINSEL0 &= ~((uint32_t)(PIN_FUNC_MASK<<PIN_RX_UART2));
			PINSEL0 |= ((uint32_t)(PIN_FUNC_SEL_1<<PIN_RX_UART2));
		}
		if(flag && PIN_TX_ENABLE)
		{
			PINSEL0 &= ~((uint32_t)(PIN_FUNC_MASK<<PIN_TX_UART2));
			PINSEL0 |= ((uint32_t)(PIN_FUNC_SEL_1<<PIN_TX_UART2));
		}
	}
	if(dev == LPC_UART3)
	{
		if(flag && PIN_RX_ENABLE)
		{
			PINSEL0 &= ~((uint32_t)(PIN_FUNC_MASK<<PIN_RX_UART3));
			PINSEL0 |= ((uint32_t)(PIN_FUNC_SEL_2<<PIN_RX_UART3));
		}
		if(flag && PIN_TX_ENABLE)
		{
			PINSEL0 &= ~((uint32_t)(PIN_FUNC_MASK<<PIN_TX_UART3));
			PINSEL0 |= ((uint32_t)(PIN_FUNC_SEL_2<<PIN_TX_UART3));
		}
	}
}

/**
 * @brief		Disable specific UART Pins
 *
 * Set specific UART RX TX Pin to GPIOs.
 *
 * @param[in]	dev	UART peripheral selected, should be:
 * 					- LPC_UART0: UART0 peripheral
 * 					- LPC_UART1: UART1 peripheral
 * 					- LPC_UART2: UART2 peripheral
 * 					- LPC_UART3: UART3 peripheral
 * @param[in]	flag 0x03 and 0x04 are used to separate disable and
 * 					enable flags.
 *
 * @return		None
 */
void uart_disable(UART_T *dev, uint8_t flag)
{
	if(dev == LPC_UART0)
	{
		if(flag && PIN_RX_DISABLE)
		{
			PINSEL0 &= ~((uint32_t)(PIN_FUNC_MASK<<PIN_RX_UART0));
		}
		if(flag && PIN_TX_DISABLE)
		{
			PINSEL0 &= ~((uint32_t)(PIN_FUNC_MASK<<PIN_TX_UART0));
		}
	}
	if(dev == LPC_UART1)
	{
		if(flag && PIN_RX_DISABLE)
		{
			PINSEL0 &= ~((uint32_t)(PIN_FUNC_MASK<<PIN_RX_UART1));
		}
		if(flag && PIN_TX_DISABLE)
		{
			PINSEL1 &= ~((uint32_t)(PIN_FUNC_MASK<<PIN_TX_UART1));
		}
	}
	if(dev == LPC_UART2)
	{
		if(flag && PIN_RX_DISABLE)
		{
			PINSEL0 &= ~((uint32_t)(PIN_FUNC_MASK<<PIN_RX_UART2));
		}
		if(flag && PIN_TX_DISABLE)
		{
			PINSEL0 &= ~((uint32_t)(PIN_FUNC_MASK<<PIN_TX_UART2));
		}
	}
	if(dev == LPC_UART3)
	{
		if(flag && PIN_RX_DISABLE)
		{
			PINSEL0 &= ~((uint32_t)(PIN_FUNC_MASK<<PIN_RX_UART3));
		}
		if(flag && PIN_TX_DISABLE)
		{
			PINSEL0 &= ~((uint32_t)(PIN_FUNC_MASK<<PIN_TX_UART3));
		}
	}
}

/**
 * @brief		Receive a single data from UART peripheral
 *
 * This function waits until a character arrive in the RecvCB buffer.
 * If a character arrives, this character will returned.
 * This function only works with SW Buffers.
 *
 * @param[in]	dev	UART peripheral selected, should be:
 *  			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @return 		Data received
 */
uint8_t get_char(UART_T *dev)
{
	unsigned char ret;
	uint8_t udev;

	udev = uart_getDevInInt(dev); 	/* get device number */
	if(udev >= 4)					/* error */
	{
		/* ERROR, *dev is wrong */
		return EINTERNAL;
	}

	while (UART_STAT[udev].recv.RecvCB_Empty == TRUE); /* Nothing received so just block */

	if(getRecvCB(dev, &ret)!=EOK)
	{
		/* empty */
		return EINTERNAL;
	}

	return ret;
}

/**
 * @brief		Start sending via UART
 *
 * This function start sending character via UART. Therefore
 * this function gets a character from the SendCB and write them
 * into THR register.
 * This function only works with SW Buffers.
 *
 * @param[in]	dev	UART peripheral selected, should be:
 *  			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @return 		None
 */
static void uart_startTransmit(UART_T *dev)
{
	unsigned char i;

	if(getSendCB(dev, &i)==EOK)
	{
		dev->REG1.THR = i;
	}
}

/**
 * @brief		Transmit a single data through SendCB and furthermore
 * 				to UART peripheral
 *
 * This function put a single byte into the Send Buffer and start
 * transmitting them.
 * This function only works with SW Buffers.
 *
 * @param[in]	dev	UART peripheral selected, should be:
 *   			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @param[in]	c	Data to transmit (must be 8-bit long)
 * @return 		None
 */
void put_char(UART_T *dev, unsigned char c)
{
	if(putSendCB(dev, c)==EOK)
	{
		uart_startTransmit(dev);
	}
}

/**
 * @brief		Transmit a strings through SendCB and furthermore
 * 				to UART peripheral
 *
 * This function sends a string via UART peripheral.
 * Therefore the data is stored in SendCb buffer. If more than
 * CB_SIZE elements should be sent, this function blocks until
 * the first CB_SIZE elements are sent.
 * This function only works with SW Buffers.
 *
 * @param[in]	dev	UART peripheral selected, should be:
 *   			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @param[in]	str		String to transmit
 * @return 		EOK if nothing gone wrong
 */
int32_t send_string(UART_T *dev, char *str)
{
	uint32_t bSent = 0;
	uint8_t udev;

	udev = uart_getDevInInt(dev);	/* get device number */
	if(udev >= 4)					/* error */
	{
		/* ERROR, *dev is wrong */
		return EINTERNAL;
	}

	/* wait for THR empty */
	while(!isready());

	while(*str) {
		put_char(dev, *str);
		str++;
		bSent++;
		/* if buffer is full, wait for emptiing the buffer and
		 * resume writting string into buffer
		 */
		if(((bSent) % CB_SIZE)==0)
		{
			/* wait for THR empty */
			while(!isready());
		}
	}

	/* if string does not end with next line character */
	if(*(--str)!='\n')
	{
		put_char(dev, '\n');
	}

	while(UART_STAT[udev].send.SendCB_Empty==FALSE);

	return EOK;
}

/**
 * @brief		Transmit a buffer with specific length
 * 				through SendCB and furthermore to UART peripheral
 *
 * This function sends len bytes from buf via UART peripheral.
 * Therefore the data is stored in SendCb buffer. If more than
 * CB_SIZE elements should be sent, this function blocks until
 * the first CB_SIZE elements are sent.
 * This function only works with SW Buffers.
 *
 * @param[in]	dev	UART peripheral selected, should be:
 *   			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @param[in]	buf		Pointer to Buffer
 * 				len		Number of Bytes to send
 * @return 		EOK if nothing gone wrong
 */
int32_t send_buf(UART_T *dev, uint8_t *buf, uint32_t len)
{
	uint32_t bToSend, bSent;
	uint8_t udev;

	udev = uart_getDevInInt(dev);	/* get device number */
	if(udev >= 4)					/* error */
	{
		/* ERROR, *dev is wrong */
		return EINTERNAL;
	}


	/* wait for THR empty */
	while(!uart_isready());

	bToSend = len;
	bSent = 0;
	while(bToSend) {
		put_char(dev, (*buf++));
		bSent++;
		bToSend--;

		if(((bSent) % CB_SIZE)==0)
		{
			/* wait for THR empty */
			while(!isready());
		}
	}

	while(UART_STAT[udev].send.SendCB_Empty==FALSE);

	return EOK;
}

/**
 * @brief		Receive String via UART peripheral and
 * 				write them into RecvCB
 *
 *  This function receives a String via UART peripheral
 * 	and store each char in the Recv circular buffer. Furthermore
 * 	this function is blocking -> wait until a len bytes arrives.
 * 	This function automatically place a \n at the last position.
 * 	This function only works with SW Buffers.
 *
 * @param[in]	dev	Selected UART peripheral used to send data,
 * 				should be:
 *   			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @param[out]	str 	Pointer to String
 * @param[in]	len 	Length of String including and \n at the end.
 * @return 		EOK if nothing gone wrong
 */
int32_t recv_string(UART_T *dev, char *str, uint32_t len)
{
	uint32_t bToRecv;

	bToRecv = len;

	while(bToRecv-1)
	{
		(*str++) = (char)get_char(dev);
		bToRecv--;
	}

	*str = '\0';

	return EOK;
}

/**
 * @brief		Receive a block of data via UART peripheral and
 * 				write them into RecvCB
 *
 * 	This function receives a block of data via UART peripheral
 * 	and store each uint8_t in the Recv circular buffer. Furthermore
 * 	this function is blocking -> wait until a len bytes arrives.
 * 	This function only works with SW Buffers.
 *
 * @param[in]	dev	Selected UART peripheral used to send data,
 * 				should be:
 *   			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @param[out]	buf 	Pointer to Received buffer
 * @param[in]	len 	Length of Received buffer
 * @return 		EOK if nothing gone wrong
 */
int32_t recv_buf(UART_T *dev, uint8_t *buf, uint32_t len)
{
	uint32_t bToRecv;

	bToRecv = len;

	while(bToRecv)
	{
		(*buf++) = get_char(dev);
		bToRecv--;
	}

	return EOK;
}

/**
 * @brief		Check if UART is ready
 *
 * This function checks, if a given UART is already
 * sending something.
 *
 * @param[in]	dev	UART peripheral selected, should be:
 * 					- LPC_UART0: UART0 peripheral
 * 					- LPC_UART1: UART1 peripheral
 * 					- LPC_UART2: UART2 peripheral
 * 					- LPC_UART3: UART3 peripheral
 *
 * @return		BUSY if the UART is sendig
 * 				NOTBUSY if the UART is waiting
 */
int32_t uart_isready(UART_T *dev)
{
	if(dev->LSR & UART_LSR_THRE)
	{
		return NOTBUSY;
	}
	else
	{
		return BUSY;
	}
}

/**
 * @brief		Send a character direct via UART
 *
 * This function sends a character without using a
 * circual buffer. This function is blocking.
 * This function only works with HW Buffers.
 *
 * @param[in]	dev	UART peripheral selected, should be:
 * 					- LPC_UART0: UART0 peripheral
 * 					- LPC_UART1: UART1 peripheral
 * 					- LPC_UART2: UART2 peripheral
 * 					- LPC_UART3: UART3 peripheral
 * @param[in]   c Character to send
 *
 * @return		None
 */
void uart_sendchar(UART_T *dev, char c)
{
	while (!isready()); /* Block until tx empty */
	dev->REG1.THR = c;
}

/**
 * @brief		Get a character direct via UART
 *
 * This function receive a character without using a
 * circual buffer. This function is blocking-> wait
 * until a character arrives. This function only
 * works with HW Buffers.
 *
 * @param[in]	dev	UART peripheral selected, should be:
 * 					- LPC_UART0: UART0 peripheral
 * 					- LPC_UART1: UART1 peripheral
 * 					- LPC_UART2: UART2 peripheral
 * 					- LPC_UART3: UART3 peripheral
 *
 * @return		c	return the received character
 */
char uart_getchar(UART_T *dev)
{
	char c;
	while ((dev->LSR & UART_LSR_RDR) == 0); /* Nothing received so just block */
	c = dev->REG1.RBR; /* Read Receiver buffer register */
	return c;
}

/**
 * @brief		Send a String direct via UART
 *
 * This function sends a string without using a
 * circual buffer. This function only works with HW Buffers.
 *
 * @param[in]	dev	UART peripheral selected, should be:
 * 					- LPC_UART0: UART0 peripheral
 * 					- LPC_UART1: UART1 peripheral
 * 					- LPC_UART2: UART2 peripheral
 * 					- LPC_UART3: UART3 peripheral
 * @param[in]   s String to send
 *
 * @return		None
 */
void uart_puts(UART_T *dev, char *s)
{
	while(*s) {
		uart_sendchar(dev, *s);
		s++;
	}

	/* if string does not end with next line character */
	if(*(--s)!='\n')
	{
		uart_sendchar(dev, '\n');
	}
}

/**
 * @brief		Return the device number in integer
 *
 * This function returns the device number in an integer
 * format.
 *
 * @param[in]	dev	UART peripheral selected, should be:
 * 					- LPC_UART0: UART0 peripheral
 * 					- LPC_UART1: UART1 peripheral
 * 					- LPC_UART2: UART2 peripheral
 * 					- LPC_UART3: UART3 peripheral
 * @return		uint8_t 0 if LPC_UART0
 * 				uint8_t 1 if LPC_UART1
 * 				uint8_t 2 if LPC_UART2
 * 				uint8_t 3 if LPC_UART3
 */
uint8_t uart_getDevInInt(UART_T *dev)
{
	uint8_t ret = 255;
	if(dev == LPC_UART0)
	{
		ret = 0;
	} else if(dev == LPC_UART1)
	{
		ret = 1;
	} else if(dev == LPC_UART2)
	{
		ret = 2;
	} else if(dev == LPC_UART3)
	{
		ret = 3;
	}

	return ret;
}
