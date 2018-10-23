/*
 * cb.h
 *
 *  Created on: Jan 2, 2014
 *      Author: Floran Berndl; Bernd Glatz;
 */

#ifndef CB_H_
#define CB_H_

#include "UART.h"

#define FALSE 	0
#define TRUE  	1

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
void initCB(UART_T *dev);

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
int32_t putSendCB(UART_T *dev, unsigned char c);

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
int32_t getSendCB(UART_T *dev, unsigned char *i);

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
int32_t putRecvCB(UART_T *dev, unsigned char c);

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
int32_t getRecvCB(UART_T *dev, unsigned char *i);

#endif /* CB_H_ */
