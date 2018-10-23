/*
 * cb.c
 *
 *  Created on: Jan 2, 2014
 *      Author: Floran Berndl; Bernd Glatz;
 */

#include "cb.h"
#include "UART.h"

/**
 * @brief		Initialize send circular buffer
 *
 * This function initializes the send circular buffer
 *
 * @param[in]	dev	UART peripheral selected, should be:
 * 					- LPC_UART0: UART0 peripheral
 * 					- LPC_UART1: UART1 peripheral
 * 					- LPC_UART2: UART2 peripheral
 * 					- LPC_UART3: UART3 peripheral
 *
 * @return		None
 */
static void initSendCB(uint8_t udev)
{
	UART_STAT[udev].send.SendCB_Full  = FALSE;
	UART_STAT[udev].send.SendCB_Empty = TRUE;
	UART_STAT[udev].send.SendCB_In	 = 0;
	UART_STAT[udev].send.SendCB_Out	 = 0;
}

/**
 * @brief		Initialize receive circular buffer
 *
 * This function initializes the receive circular buffer
 *
 * @param[in]	dev	UART peripheral selected, should be:
 * 					- LPC_UART0: UART0 peripheral
 * 					- LPC_UART1: UART1 peripheral
 * 					- LPC_UART2: UART2 peripheral
 * 					- LPC_UART3: UART3 peripheral
 *
 * @return		None
 */
static void initRecvCB(uint8_t udev)
{
	UART_STAT[udev].recv.RecvCB_Full  = FALSE;
	UART_STAT[udev].recv.RecvCB_Empty = TRUE;
	UART_STAT[udev].recv.RecvCB_In	 = 0;
	UART_STAT[udev].recv.RecvCB_Out	 = 0;
}

/**
 * @brief		Initialize senda and receive circular buffer
 *
 * This function calls the initSendCb and initRecvCb functions
 * to initialze them.
 *
 * @param[in]	dev	UART peripheral selected, should be:
 * 					- LPC_UART0: UART0 peripheral
 * 					- LPC_UART1: UART1 peripheral
 * 					- LPC_UART2: UART2 peripheral
 * 					- LPC_UART3: UART3 peripheral
 *
 * @return		None
 */
void initCB(UART_T *dev)
{
	uint8_t udev;

	udev = uart_getDevInInt(dev);	/* get device number */
	if(udev < 4)					/* between 0 and 3 */
	{
		initSendCB(udev);
		initRecvCB(udev);
	}
}

/**
 * @brief		Send character to outgoing circular buffers
 *
 * This function Put a character in the send circular buffer.
 *
 * @param[in]	dev	UART peripheral selected, should be:
 * 					- LPC_UART0: UART0 peripheral
 * 					- LPC_UART1: UART1 peripheral
 * 					- LPC_UART2: UART2 peripheral
 * 					- LPC_UART3: UART3 peripheral
 *
 * @return		EOK if successful otherwise ECB!
 */
int32_t putSendCB(UART_T *dev, unsigned char c)
{
	int32_t ret = ECB;
	uint8_t udev;

	udev = uart_getDevInInt(dev);	/* get device number */
	if(udev >= 4)					/* error */
	{
		/* ERROR, *dev is wrong */
		return EINTERNAL;
	}

	if(UART_STAT[udev].send.SendCB_Full == FALSE){
		UART_STAT[udev].send.SendCB_Array[UART_STAT[udev].send.SendCB_In] = c;
		UART_STAT[udev].send.SendCB_In = (UART_STAT[udev].send.SendCB_In + 1) % CB_SIZE;
		if(UART_STAT[udev].send.SendCB_In == UART_STAT[udev].send.SendCB_Out){
			UART_STAT[udev].send.SendCB_Full = TRUE;
		}
		ret = EOK;
	}
	UART_STAT[udev].send.SendCB_Empty = FALSE;
	return ret;
}

/**
 * @brief		Get character from outgoing circular buffers
 *
 * This function get a character in the send circular buffer.
 *
 * @param[in]	dev	UART peripheral selected, should be:
 * 					- LPC_UART0: UART0 peripheral
 * 					- LPC_UART1: UART1 peripheral
 * 					- LPC_UART2: UART2 peripheral
 * 					- LPC_UART3: UART3 peripheral
 *
 * @return		EOK if successful otherwise ECB!
 */
int32_t getSendCB(UART_T *dev, unsigned char *i)
{
	int32_t ret = ECB;
	uint8_t udev;

	udev = uart_getDevInInt(dev);	/* get device number */
	if(udev >= 4)					/* error */
	{
		/* ERROR, *dev is wrong */
		return EINTERNAL;
	}

	if(UART_STAT[udev].send.SendCB_Empty == FALSE){
		*i = UART_STAT[udev].send.SendCB_Array[UART_STAT[udev].send.SendCB_Out];
		UART_STAT[udev].send.SendCB_Out = (UART_STAT[udev].send.SendCB_Out + 1) % CB_SIZE;
		if(UART_STAT[udev].send.SendCB_In == UART_STAT[udev].send.SendCB_Out){
			UART_STAT[udev].send.SendCB_Empty = TRUE;
		}
		ret = EOK;
	}
	UART_STAT[udev].send.SendCB_Full = FALSE;
	return ret;
}

/**
 * @brief		Send character to incoming circular buffers
 *
 * This function Put a character in the receive circular buffer.
 *
 * @param[in]	dev	UART peripheral selected, should be:
 * 					- LPC_UART0: UART0 peripheral
 * 					- LPC_UART1: UART1 peripheral
 * 					- LPC_UART2: UART2 peripheral
 * 					- LPC_UART3: UART3 peripheral
 *
 * @return		EOK if successful otherwise ECB!
 */
int32_t putRecvCB(UART_T *dev, unsigned char c)
{
	int32_t ret = ECB;
	uint8_t udev;

	udev = uart_getDevInInt(dev);	/* get device number */
	if(udev >= 4)					/* error */
	{
		/* ERROR, *dev is wrong */
		return EINTERNAL;
	}

	if(UART_STAT[udev].recv.RecvCB_Full == FALSE){
		UART_STAT[udev].recv.RecvCB_Array[UART_STAT[udev].recv.RecvCB_In] = c;
		UART_STAT[udev].recv.RecvCB_In = (UART_STAT[udev].recv.RecvCB_In + 1) % CB_SIZE;
		if(UART_STAT[udev].recv.RecvCB_In == UART_STAT[udev].recv.RecvCB_Out){
			UART_STAT[udev].recv.RecvCB_Full = TRUE;
		}
		ret = EOK;
	}
	UART_STAT[udev].recv.RecvCB_Empty = FALSE;
	return ret;
}

/**
 * @brief		Get character from incoming circular buffers
 *
 * This function get a character in the receive circular buffer.
 *
 * @param[in]	dev	UART peripheral selected, should be:
 * 					- LPC_UART0: UART0 peripheral
 * 					- LPC_UART1: UART1 peripheral
 * 					- LPC_UART2: UART2 peripheral
 * 					- LPC_UART3: UART3 peripheral
 *
 * @return		EOK if successful otherwise ECB!
 */
int32_t getRecvCB(UART_T *dev, unsigned char *i)
{
	int32_t ret = ECB;
	uint8_t udev;

	udev = uart_getDevInInt(dev);	/* get device number */
	if(udev >= 4)					/* error */
	{
		/* ERROR, *dev is wrong */
		return EINTERNAL;
	}

	if(UART_STAT[udev].recv.RecvCB_Empty == FALSE){
		*i = UART_STAT[udev].recv.RecvCB_Array[UART_STAT[udev].recv.RecvCB_Out];
		UART_STAT[udev].recv.RecvCB_Out = (UART_STAT[udev].recv.RecvCB_Out + 1) % CB_SIZE;
		if(UART_STAT[udev].recv.RecvCB_In == UART_STAT[udev].recv.RecvCB_Out){
			UART_STAT[udev].recv.RecvCB_Empty = TRUE;
		}
		ret = EOK;
	}
	UART_STAT[udev].recv.RecvCB_Full = FALSE;
	return ret;
}
