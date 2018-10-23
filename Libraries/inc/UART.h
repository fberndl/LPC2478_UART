/*
 * UART.h
 *
 *  Created on: Dec 20, 2013
 *      Author: Floran Berndl; Bernd Glatz;
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include "lpc246x.h"
#include "init.h"

#define __I volatile const	/* Read only data */
#define __O volatile		/* Write only data */
#define __IO volatile		/* Read write data */

/**************** ERROR CODES **************************/
#define EOK 			0	/* no error */
#define EINTERNAL		-1  /* error */
#define EPARAM			-2	/* wrong parameter */
#define ECALCBAUD		-3  /* error calculate baudrate */
#define ECB				-4  /* error Circular buffer */

/***************** BUSY / NOT BUSY *********************/
#define BUSY			0
#define NOTBUSY			!BUSY

/****************** BUFFER SIZE ************************/
#define CB_SIZE	16

/********* Receiver Buffer Register ********************/
#define UART_RBR_MASKBIT   	((uint8_t)0xFF) 		/* UART Received Buffer mask bit (8 bits) */

/********* Transmit Holding Register *******************/
#define UART_THR_MASKBIT   	((uint8_t)0xFF) 		/* UART Transmit Holding mask bit (8 bits) */

/*********** Divisor Latch LSB register ****************/
#define UART_LOAD_DLL(div)	((div) & 0xFF)			/* Macro for loading least significant halfs of divisors */
#define UART_DLL_MASKBIT	((uint8_t)0xFF)			/* Divisor latch LSB bit mask */

/*********** Divisor Latch MSB register ****************/
#define UART_DLM_MASKBIT	((uint8_t)0xFF)			/* Divisor latch MSB bit mask */
#define UART_LOAD_DLM(div)  (((div) >> 8) & 0xFF)	/* Macro for loading most significant halfs of divisors */

/*********** interrupt enable register *****************/
#define UART_IER_RBRINT_EN		((uint32_t)(1<<0)) 	/* RBR Interrupt enable*/
#define UART_IER_THREINT_EN		((uint32_t)(1<<1)) 	/* THR Interrupt enable*/
#define UART_IER_RLSINT_EN		((uint32_t)(1<<2)) 	/* RX line status interrupt enable*/
#define UART_IER_ABEOINT_EN		((uint32_t)(1<<8)) 	/* Enables the end of auto-baud interrupt */
#define UART_IER_ABTOINT_EN		((uint32_t)(1<<9)) 	/* Enables the auto-baud time-out interrupt */
#define UART_IER_BITMASK		((uint32_t)(0x307)) /* UART interrupt enable register bit mask */

/********** interrupt identification register **********/
#define UART_IIR_INTSTAT_PEND	((uint32_t)(1<<0))	/* Interrupt Status - Active low */
#define UART_IIR_INTID_RLS		((uint32_t)(3<<1)) 	/* Interrupt identification: Receive line status*/
#define UART_IIR_INTID_RDA		((uint32_t)(2<<1)) 	/* Interrupt identification: Receive data available*/
#define UART_IIR_INTID_CTI		((uint32_t)(6<<1)) 	/* Interrupt identification: Character time-out indicator*/
#define UART_IIR_INTID_THRE		((uint32_t)(1<<1)) 	/* Interrupt identification: THRE interrupt*/
#define UART_IIR_INTID_MASK		((uint32_t)(7<<1))	/* Interrupt identification: Interrupt ID mask */
#define UART_IIR_FIFO_EN		((uint32_t)(3<<6)) 	/* These bits are equivalent to UnFCR[0] */
#define UART_IIR_ABEO_INT		((uint32_t)(1<<8)) 	/* End of auto-baud interrupt */
#define UART_IIR_ABTO_INT		((uint32_t)(1<<9)) 	/* Auto-baud time-out interrupt */
#define UART_IIR_BITMASK		((uint32_t)(0x3CF))	/* UART interrupt identification register bit mask */

/*************** FIFO control register *****************/
#define UART_FCR_FIFO_EN		((uint8_t)(1<<0)) 	/* UART FIFO enable */
#define UART_FCR_RX_RS			((uint8_t)(1<<1)) 	/* UART FIFO RX reset */
#define UART_FCR_TX_RS			((uint8_t)(1<<2)) 	/* UART FIFO TX reset */
#define UART_FCR_DMAMODE_SEL 	((uint8_t)(1<<3)) 	/* UART DMA mode selection */
#define UART_FCR_TRG_LEV0		((uint8_t)(0)) 		/* UART FIFO trigger level 0: 1 character */
#define UART_FCR_TRG_LEV1		((uint8_t)(1<<6)) 	/* UART FIFO trigger level 1: 4 character */
#define UART_FCR_TRG_LEV2		((uint8_t)(2<<6)) 	/* UART FIFO trigger level 2: 8 character */
#define UART_FCR_TRG_LEV3		((uint8_t)(3<<6)) 	/* UART FIFO trigger level 3: 14 character */
#define UART_FCR_BITMASK		((uint8_t)(0xCF))	/* UART FIFO control bit mask */
#define UART_TX_FIFO_SIZE		(16)

/*************** line control register *****************/
#define UART_LCR_WLEN5     		((uint8_t)(0))   	/* UART 5 bit data mode */
#define UART_LCR_WLEN6     		((uint8_t)(1<<0))   /* UART 6 bit data mode */
#define UART_LCR_WLEN7     		((uint8_t)(2<<0))   /* UART 7 bit data mode */
#define UART_LCR_WLEN8     		((uint8_t)(3<<0))   /* UART 8 bit data mode */
#define UART_LCR_STOPBIT_SEL	((uint8_t)(1<<2))   /* UART Two Stop Bits Select */
#define UART_LCR_PARITY_EN		((uint8_t)(1<<3))	/* UART Parity Enable */
#define UART_LCR_PARITY_ODD		((uint8_t)(0))      /* UART Odd Parity Select */
#define UART_LCR_PARITY_EVEN	((uint8_t)(1<<4))	/* UART Even Parity Select */
#define UART_LCR_PARITY_F_1		((uint8_t)(2<<4))	/* UART force 1 stick parity */
#define UART_LCR_PARITY_F_0		((uint8_t)(3<<4))	/* UART force 0 stick parity */
#define UART_LCR_BREAK_EN		((uint8_t)(1<<6))	/* UART Transmission Break enable */
#define UART_LCR_DLAB_EN		((uint8_t)(1<<7))   /* UART Divisor Latches Access bit enable */
#define UART_LCR_BITMASK		((uint8_t)(0xFF))	/* UART line control bit mask */

/**************** line status register *****************/
#define UART_LSR_RDR		((uint8_t)(1<<0)) 		/* Line status register: Receive data ready */
#define UART_LSR_OE			((uint8_t)(1<<1)) 		/* Line status register: Overrun error */
#define UART_LSR_PE			((uint8_t)(1<<2)) 		/* Line status register: Parity error */
#define UART_LSR_FE			((uint8_t)(1<<3)) 		/* Line status register: Framing error */
#define UART_LSR_BI			((uint8_t)(1<<4)) 		/* Line status register: Break interrupt */
#define UART_LSR_THRE		((uint8_t)(1<<5)) 		/* Line status register: Transmit holding register empty */
#define UART_LSR_TEMT		((uint8_t)(1<<6)) 		/* Line status register: Transmitter empty */
#define UART_LSR_RXFE		((uint8_t)(1<<7)) 		/* Error in RX FIFO */
#define UART_LSR_BITMASK	((uint8_t)(0xFF)) 		/* UART Line status bit mask */

/***************** Scratch Pad Register ***************/
#define UART_SCR_BIMASK		((uint8_t)(0xFF))		/* UART Scratch Pad bit mask */

/************ Auto baudrate control register **********/
#define UART_ACR_START			((uint32_t)(1<<0))	/* UART Auto-baud start */
#define UART_ACR_MODE			((uint32_t)(1<<1))	/* UART Auto baudrate Mode 1 */
#define UART_ACR_AUTO_RESTART	((uint32_t)(1<<2))	/* UART Auto baudrate restart */
#define UART_ACR_ABEOINT_CLR	((uint32_t)(1<<8))	/* UART End of auto-baud interrupt clear */
#define UART_ACR_ABTOINT_CLR	((uint32_t)(1<<9))	/* UART Auto-baud time-out interrupt clear */
#define UART_ACR_BITMASK		((uint32_t)(0x307))	/* UART Auto Baudrate register bit mask */

/************* Fractional divider register ************/
#define UART_FDR_DIVADDVAL(n)	((uint32_t)(n))				/* Baud-rate generation pre-scaler divisor */
#define UART_FDR_MULVAL(n)		((uint32_t)((n<<4))			/* Baud-rate pre-scaler multiplier value */
#define UART_FDR_BITMASK		((uint32_t)(0xFF))			/* UART Fractional Divider register bit mask */

/***************** Tx Enable register *****************/
#define UART_TER_TXEN			((uint8_t)(1<<7)) 	/* Transmit enable bit */
#define UART_TER_BITMASK		((uint8_t)(0x80))	/* UART Transmit Enable Register bit mask */

/***************** FIFO Level register ****************/
#define UART_FIFOLVL_RXFIFOLVL(n)	((uint32_t)(n&0x0F))		/* Reflects the current level of the UART receiver FIFO */
#define UART_FIFOLVL_TXFIFOLVL(n)	((uint32_t)((n>>8)&0x0F))	/* Reflects the current level of the UART transmitter FIFO */
#define UART_FIFOLVL_BITMASK		((uint32_t)(0x0F0F))		/* UART FIFO Level Register bit mask */

/***************** Power Configuration ****************/
#define PCUART0					((uint32_t)(1<<3)) 	/* UART0 power/clock control bit */
#define PCUART1					((uint32_t)(1<<4)) 	/* UART1 power/clock control bit */
#define PCUART2					((uint32_t)(1<<24)) /* UART2 power/clock control bit */
#define PCUART3					((uint32_t)(1<<25)) /* UART3 power/clock control bit */

/******************* Clock selection ******************/
#define PCLK_UART0				6					/* Start bit of clock selection */
#define PCLK_UART1				8					/* Start bit of clock selection */
#define PCLK_UART2				16					/* Start bit of clock selection */
#define PCLK_UART3				18					/* Start bit of clock selection */
#define PCLK_UART0_CCLK4		((uint32_t)(0))		/* Peripheral clock selection for UART0  (CCLK / 4)*/
#define PCLK_UART0_CCLK			((uint32_t)(1<<6))	/* Peripheral clock selection for UART0  (CCLK)*/
#define PCLK_UART0_CCLK2		((uint32_t)(2<<6))	/* Peripheral clock selection for UART0  (CCLK / 2)*/
#define PCLK_UART0_CCLK8		((uint32_t)(3<<6))	/* Peripheral clock selection for UART0  (CCLK / 8)*/
#define PCLK_UART1_CCLK4		((uint32_t)(0))		/* Peripheral clock selection for UART1  (CCLK / 4)*/
#define PCLK_UART1_CCLK			((uint32_t)(1<<8))	/* Peripheral clock selection for UART1  (CCLK)*/
#define PCLK_UART1_CCLK2		((uint32_t)(2<<8))	/* Peripheral clock selection for UART1  (CCLK / 2)*/
#define PCLK_UART1_CCLK8		((uint32_t)(3<<8))	/* Peripheral clock selection for UART1  (CCLK / 8)*/
#define PCLK_UART2_CCLK4		((uint32_t)(0))		/* Peripheral clock selection for UART2  (CCLK / 4)*/
#define PCLK_UART2_CCLK			((uint32_t)(1<<16))	/* Peripheral clock selection for UART2  (CCLK)*/
#define PCLK_UART2_CCLK2		((uint32_t)(2<<16))	/* Peripheral clock selection for UART2  (CCLK / 2)*/
#define PCLK_UART2_CCLK8		((uint32_t)(3<<16))	/* Peripheral clock selection for UART2  (CCLK / 8)*/
#define PCLK_UART3_CCLK4		((uint32_t)(0))		/* Peripheral clock selection for UART3  (CCLK / 4)*/
#define PCLK_UART3_CCLK			((uint32_t)(1<<18))	/* Peripheral clock selection for UART3  (CCLK)*/
#define PCLK_UART3_CCLK2		((uint32_t)(2<<18))	/* Peripheral clock selection for UART3  (CCLK / 2)*/
#define PCLK_UART3_CCLK8		((uint32_t)(3<<18))	/* Peripheral clock selection for UART3  (CCLK / 8)*/

/********* Transmission reception features *************/
#define PIN_TX_ENABLE			((uint8_t)(1<<0))	/* Enable TX Pin */
#define PIN_RX_ENABLE			((uint8_t)(1<<1))	/* Enable RX Pin */
#define PIN_TX_DISABLE			((uint8_t)(1<<2))	/* DISable TX Pin */
#define PIN_RX_DISABLE			((uint8_t)(1<<3))	/* DISable RX Pin */
#define PIN_TX_UART0			4					/* UART0 TX Pin */
#define PIN_RX_UART0			6  					/* UART0 RX Pin */
#define PIN_TX_UART1			30					/* UART1 TX Pin */
#define PIN_RX_UART1			0  					/* UART1 RX Pin */
#define PIN_TX_UART2			20					/* UART2 TX Pin */
#define PIN_RX_UART2			22  				/* UART2 RX Pin */
#define PIN_TX_UART3			0					/* UART3 TX Pin */
#define PIN_RX_UART3			2  					/* UART3 RX Pin */
#define PIN_FUNC_MASK			3					/* Delete actual values */
#define PIN_FUNC_SEL_1			1					/* Function 01 */
#define PIN_FUNC_SEL_2			2					/* Function 10 */

/****************** Macros *****************************/
/* check macro for UART - for debug purposes*/
#define PARAM_UARTx(x) ((((uint32_t *)x)==((uint32_t *)UART0_BASE_ADDR)) \
		|| (((uint32_t *)x)==((uint32_t *)UART1_BASE_ADDR)) \
		|| (((uint32_t *)x)==((uint32_t *)UART2_BASE_ADDR)) \
		|| (((uint32_t *)x)==((uint32_t *)UART3_BASE_ADDR))) ? EOK : EPARAM

/**
 * @brief UART Databit type definitions
 */
typedef enum UART_DATABIT_e {
	UART_DATABIT_5 = 0,		/* UART 5 bit data mode */
	UART_DATABIT_6,			/* UART 6 bit data mode */
	UART_DATABIT_7,			/* UART 7 bit data mode */
	UART_DATABIT_8			/* UART 8 bit data mode */
} UART_DATABIT_Type;

/**
 * @brief UART Stop bit type definitions
 */
typedef enum UART_STOPBIT_e {
	UART_STOPBIT_1 = 0, 	/* UART 1 stop bit */
	UART_STOPBIT_2 	 		/* UART 2 stop bit */
} UART_STOPBIT_Type;

/**
 * @brief UART Parity type definitions
 */
typedef enum UART_PARITY_e {
	UART_PARITY_NONE = 0,	/* No parity */
	UART_PARITY_ODD,		/* Odd parity */
	UART_PARITY_EVEN,		/* Even parity */
	UART_PARITY_SP1,		/* Forced "1" stick parity */
	UART_PARITY_SP0			/* Forced "0" stick parity */
} UART_PARITY_Type;

/**
 * @brief UART Clock select definitions
 */
typedef enum UART_CLK_e {
	CCLK_div_4 = 0,			/* Core clock divided through 4 */
	CCLK,					/* Core clock */
	CCLK_div_2,				/* Core clock divided through 2 */
	CCLK_div_8				/* Core clock divided through 8 */
} UART_CLK_Select;

/**
 * @brief UART hardware or software buffer
 */
typedef enum UART_BUF_e {
	UART_HW_BUF = 0,		/* Hardware buffer in use */
	UART_SW_BUF				/* Software buffer in use */
} UART_BUF_Type;

/**
 * @brief UART Register Structure with Modem Module and IrDA module
 *
 * This structure reflects the register structure for all UART specific
 * registers. Therefore be carefule when using this structure and
 * check if the specific register are allowed for use, e.g. MODEM structure
 * only for UART1 and IrDA registers only for UART3.
 *
 * In our purpose, only common registers are used, therefore we can use
 * this simple structure. Setting the Modules to 0 in the init function.
 */
typedef struct __attribute__((__packed__)) UART_S
{
	union {
	__I  uint8_t  RBR;		/* Receive Buffer Register */
	__O  uint8_t  THR;		/* Transmit Holding Register */
	__IO uint8_t  DLL;		/* Divisor Latch LSB */
		 uint32_t RESERVED0;
	} REG1;
	union {
	__IO uint8_t  DLM; 		/* Divisor Latch MSB */
	__IO uint16_t IER;		/* Interrupt Enable Register */
		 uint32_t RESERVED1;
	} REG2;
	union {
	__I  uint16_t IIR;		/* Interrupt Identification Register */
	__O	 uint8_t  FCR;		/* FIFO Control Register */
		 uint32_t RESERVED2;
	} REG3;
	__IO uint8_t  LCR;		/* Line Control Register */
		 uint8_t  RESERVED3[3];
	__IO uint8_t  MCR;   	/* Modem Control Register */
		 uint8_t  RESERVED4[3];
	__I	 uint8_t  LSR;		/* Line Status Register */
		 uint8_t  RESERVED5[3];
	__I  uint8_t  MSR;	 	/* Modem Status Register */
		 uint8_t  RESERVED6[3];
	__IO uint8_t  SCR;		/* Scratch Pad Register */
		 uint8_t  RESERVED7[3];
	__IO uint32_t ACR; 		/* Auto-baud Control Register */
	__IO uint8_t  ICR;		/* IrDA Control Register for UART3 only */
		 uint8_t  RESERVED8[3];
	__IO uint8_t  FDR;		/* Fractional Divider Register */
		 uint8_t  RESERVED9[7];
	__IO uint8_t  TER;		/* Transmit Enable Register */
} UART_T;

/**
 * @brief UART12 Register Structure
 *
 * This structure reflects the register structure of the UART module
 */
typedef struct __attribute__((__packed__)) UART02_S
{
	union {
	__I  uint8_t  RBR;		/* Receive Buffer Register */
	__O  uint8_t  THR;		/* Transmit Holding Register */
	__IO uint8_t  DLL;		/* Divisor Latch LSB */
		 uint32_t RESERVED0;
	} REG1;
	union {
	__IO uint8_t  DLM; 		/* Divisor Latch MSB */
	__IO uint16_t IER;		/* Interrupt Enable Register */
		 uint32_t  RESERVED1;
	} REG2;
	union {
	__I  uint16_t IIR;		/* Interrupt Identification Register */
	__O	 uint8_t  FCR;		/* FIFO Control Register */
		 uint32_t  RESERVED2;
	} REG3;
	__IO uint8_t  LCR;		/* Line Control Register */
		 uint8_t  RESERVED3[7];
	__I	 uint8_t  LSR;		/* Line Status Register */
		 uint8_t  RESERVED4[7];
	__IO uint8_t  SCR;		/* Scratch Pad Register */
		 uint8_t  RESERVED5[3];
	__IO uint32_t ACR; 		/* Auto-baud Control Register */
		 uint8_t  RESERVED6[4];
	__IO uint8_t  FDR;		/* Fractional Divider Register */
		 uint8_t  RESERVED7[7];
	__IO uint8_t  TER;		/* Transmit Enable Register */
} UART02_T;

/**
 * @brief UART1 Register Structure with Modem Module
 *
 * This structure reflects the register structure of the UART1 module
 * including the Modul register
 */
typedef struct __attribute__((__packed__)) UART1_S
{
	union {
	__I  uint8_t  RBR;		/* Receive Buffer Register */
	__O  uint8_t  THR;		/* Transmit Holding Register */
	__IO uint8_t  DLL;		/* Divisor Latch LSB */
		 uint32_t RESERVED0;
	} REG1;
	union {
	__IO uint8_t  DLM; 		/* Divisor Latch MSB */
	__IO uint16_t IER;		/* Interrupt Enable Register */
		 uint32_t  RESERVED1;
	} REG2;
	union {
	__I  uint16_t IIR;		/* Interrupt Identification Register */
	__O	 uint8_t  FCR;		/* FIFO Control Register */
		 uint32_t  RESERVED2;
	} REG3;
	__IO uint8_t  LCR;		/* Line Control Register */
		 uint8_t  RESERVED3[3];
	__IO uint8_t  MCR;   	/* Modem Control Register */
		 uint8_t  RESERVED4[3];
	__I	 uint8_t  LSR;		/* Line Status Register */
		 uint8_t  RESERVED5[3];
	__I  uint8_t  MSR;	 	/* Modem Status Register */
		 uint8_t  RESERVED6[3];
	__IO uint8_t  SCR;		/* Scratch Pad Register */
		 uint8_t  RESERVED7[3];
	__IO uint32_t ACR; 		/* Auto-baud Control Register */
		 uint8_t  RESERVED8[4];
	__IO uint8_t  FDR;		/* Fractional Divider Register */
		 uint8_t  RESERVED9[7];
	__IO uint8_t  TER;		/* Transmit Enable Register */
} UART1_T;

/**
 * @brief UART3 Register Structure with IrDA module
 *
 * This structure reflects the register structure of the UART3 module
 * including IrDA register
 */
typedef struct UART3_S
{
	union {
		__I  uint8_t  RBR;		/* Receive Buffer Register */
		__O  uint8_t  THR;		/* Transmit Holding Register */
		__IO uint8_t  DLL;		/* Divisor Latch LSB */
		uint32_t RESERVED0;
	} REG1;
	union {
		__IO uint8_t  DLM; 		/* Divisor Latch MSB */
		__IO uint16_t IER;		/* Interrupt Enable Register */
		uint32_t  RESERVED1;
	} REG2;
	union {
		__I  uint16_t IIR;		/* Interrupt Identification Register */
		__O	 uint8_t  FCR;		/* FIFO Control Register */
		uint32_t  RESERVED2;
	} REG3;
	__IO uint8_t  LCR;		/* Line Control Register */
	uint8_t  RESERVED3[7];
	__I	 uint8_t  LSR;		/* Line Status Register */
	uint8_t  RESERVED4[7];
	__IO uint8_t  SCR;		/* Scratch Pad Register */
	uint8_t  RESERVED5[3];
	__IO uint32_t ACR; 		/* Auto-baud Control Register */
	__IO uint8_t  ICR;		/* IrDA Control Register for UART3 only */
	uint8_t  RESERVED6[3];
	__IO uint8_t  FDR;		/* Fractional Divider Register */
	uint8_t  RESERVED7[7];
	__IO uint8_t  TER;		/* Transmit Enable Register */
} UART3_T;

/**
 * @brief UART Configuration Structure
 *
 * This structure contain the main parameters for the UART
 * configuration. It contains the Baudrate, the parity, the
 * numbers of databits and the numbers of stopbits for the
 * specific UART
 */
typedef struct UART_CFG_S {
	uint32_t Baud_rate;
	UART_PARITY_Type Parity;
	UART_DATABIT_Type Databits;
	UART_STOPBIT_Type Stopbits;
	UART_BUF_Type Buffer;
} UART_CFG_T;

/**
 * @brief UART Buffer send structure
 *
 * This structure contain all necessary information
 * for the send buffer and the buffer itself.
 */
typedef struct UART_BUF_SEND_S {
	unsigned char SendCB_Array[CB_SIZE];
	unsigned char volatile SendCB_In;
	unsigned char volatile SendCB_Out;
	unsigned char volatile SendCB_Full;
	unsigned char volatile SendCB_Empty;
} UART_BUF_SEND_t;

/**
 * @brief UART Buffer receive structure
 *
 * This structure contain all necessary information
 * for the receive buffer and the buffer itself.
 */
typedef struct UART_BUF_RECV_S {
	unsigned char RecvCB_Array[CB_SIZE];
	unsigned char volatile RecvCB_In;
	unsigned char volatile RecvCB_Out;
	unsigned char volatile RecvCB_Full;
	unsigned char volatile RecvCB_Empty;
} UART_BUF_RECV_t;

/**
 * @brief UART Statistics structe
 *
 * This structure contain both buffers as well as the
 * Callback function.
 */
typedef struct UART_STATS_S {
	UART_BUF_SEND_t send;						/* Send buffer */
	UART_BUF_RECV_t recv; 						/* Receive buffer */
	void 			(*callback_UART)(void);	/* Callback function */
} UART_STATS_t;

/* global buffer structure for circular buffer and store
 * callback function for each UART module
 * UART_STAT[0] = UART 0
 * UART_STAT[1] = UART 1
 * UART_STAT[2] = UART 2
 * UART_STAT[3] = UART 3
 *
 * */
UART_STATS_t UART_STAT[4];

/*************** map defines to registers *****************/
/************************ UARTs ***************************/
#define LPC_UART0		((UART_T *) UART0_BASE_ADDR )
#define LPC_UART1		((UART_T *) UART1_BASE_ADDR )
#define LPC_UART2		((UART_T *) UART2_BASE_ADDR )
#define LPC_UART3		((UART_T *) UART3_BASE_ADDR )

/******************* public Functions ********************/
/**
 * @brief UART0 Interrupt Handler
 */
void  UART0Handler(void) __attribute__ ((interrupt("IRQ")));
/**
 * @brief UART1 Interrupt Handler
 */
void  UART1Handler(void) __attribute__ ((interrupt("IRQ")));
/**
 * @brief UART2 Interrupt Handler
 */
void  UART2Handler(void) __attribute__ ((interrupt("IRQ")));
/**
 * @brief UART3 Interrupt Handler
 */
void  UART3Handler(void) __attribute__ ((interrupt("IRQ")));

/**
 * @brief		Initializes the dev UART peripheral according to the specified
 *               parameters in the UART_cfg struct.
 *
 * This function initializes a given Uart peripheral
 * according to the specified parameters in the cfg struct
 * and calculate the best possible baudrate.
 *
 * @param[in]	dev	UART peripheral selected, should be:
 *   			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @param[in]	UART_CFG Pointer to a UART_CFG_Type structure
 *                    that contains the configuration information for the
 *                    specified UART peripheral.
 *
 * @return		EOK if successful
 * 				EINTERNAL if not successful
 */
int32_t uart_init(UART_T *dev, UART_CFG_T *cfg, void(*callback)(void));

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
void uart_enable(UART_T *dev, uint8_t flag);

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
void uart_disable(UART_T *dev, uint8_t flag);

/**
 * @brief		Receive a single data from UART peripheral
 *
 * This function waits until a character arrive in the RecvCB buffer.
 * If a character arrives, this character will returned.
 * This function only works with SW Buffers.
 *
 * @param[in]	UARTx	UART peripheral selected, should be:
 *  			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @return 		Data received
 */
uint8_t get_char(UART_T *dev);

/**
 * @brief		Transmit a single data through SendCB and furthermore
 * 				to UART peripheral
 *
 * This function put a single byte into the Send Buffer and start
 * transmitting them.
 * This function only works with SW Buffers.
 *
 * @param[in]	UARTx	UART peripheral selected, should be:
 *   			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @param[in]	c	Data to transmit (must be 8-bit long)
 * @return 		None
 */
void put_char(UART_T *dev, unsigned char c);

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
int32_t uart_isready(UART_T *dev);

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
int32_t send_string(UART_T *dev, char *str);

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
int32_t recv_string(UART_T *dev, char *str, uint32_t len);

/**
 * @brief		Receive String via UART peripheral and
 * 				write them into RecvCB
 *
 *  This function receives a String via UART peripheral
 * 	and store each char in the Recv circular buffer. Furthermore
 * 	this function is blocking -> wait until a len bytes arrives.
 * 	This function only works with SW Buffers.
 *
 * @param[in]	dev	Selected UART peripheral used to send data,
 * 				should be:
 *   			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @param[out]	str 	Pointer to String
 * @param[in]	len 	Length of String
 * @return 		EOK if nothing gone wrong
 */
int32_t send_buf(UART_T *dev, uint8_t *buf, uint32_t len);

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
int32_t recv_buf(UART_T *dev, uint8_t *buf, uint32_t len);

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
void uart_puts(UART_T *dev, char *s);

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
char uart_getchar(UART_T *dev);

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
void uart_sendchar(UART_T *dev, char c);

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
uint8_t uart_getDevInInt(UART_T *dev);
#endif /* UART_H_ */
