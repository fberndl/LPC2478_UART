#include <stdio.h>

#include "init.h"
#include "lpc2478_lib.h"
#include "UART.h"
#include "irq.h"

/**
 * @brief User Interrupt Routine
 *
 * This function is called when a interrupt occurs
 */
void interrupt_routine()
{
	/* do something here */
	/* user code */
}

/**
 * @brief main function
 */
int main(void)
{
	UART_CFG_T cfg;
	uint8_t recBytes = 0;
	uint8_t text[18] = { 'E','i','n',' ','l','a','n','g','e','r',' ','T','e','x','t', '!', '!','\0' };
	char string[20];

	(void)SystemInit();

	setPortPinDir(2, 10);

	/* init vic */
	init_VIC();

	/* UART-Configuration */
	cfg.Baud_rate = 115200;
	cfg.Databits = UART_DATABIT_8;
	cfg.Parity = UART_PARITY_NONE;
	cfg.Stopbits = UART_STOPBIT_1;
	cfg.Buffer = UART_SW_BUF;
	if(uart_init(LPC_UART0, &cfg, interrupt_routine)!=EOK)
	{
		/* Something went wrong at UART initialization */
		for(;;);
	}
	uart_enable(LPC_UART0, (PIN_TX_ENABLE|PIN_RX_ENABLE));

	/* send some test strings */
	(void)send_string(LPC_UART0, "UART Treiber - BERNDL / GLATZ\n");
	(void)send_string(LPC_UART0, "Funktioniert! Heureka! Das sind 43 Zeichen\n");
	(void)send_buf(LPC_UART0, text, 5);
	put_char(LPC_UART0, '\n');
	(void)send_buf(LPC_UART0, text, 18);
	put_char(LPC_UART0, '\n');

	for(;;)
	{
		/* put_char get_char test */
		(void)send_string(LPC_UART0, "Echo test:\nEnter 10 characters\n");
		while(recBytes<10)
		{
			(void)put_char(LPC_UART0, get_char(LPC_UART0));
			recBytes++;
		}
		(void)send_string(LPC_UART0, " \n");

		/* send receive string tests */
		(void)send_string(LPC_UART0, "Receive - send string test:\nEnter 4 characters\n");
		(void)recv_string(LPC_UART0, string, 5);
		(void)send_string(LPC_UART0, string);

		/* send receive buffer tests */
		(void)send_string(LPC_UART0, "Receive - send buffer test:\nEnter 5 characters\n");
		(void)recv_buf(LPC_UART0, text, 5);
		(void)send_buf(LPC_UART0, text, 4);
		(void)send_string(LPC_UART0, " \n");

		/* send receive string tests */
		(void)send_string(LPC_UART0, "Receive - send long string test:\nEnter 19 characters\n");
		(void)recv_string(LPC_UART0, string, 20);
		(void)send_string(LPC_UART0, string);

		recBytes = 0;
	}

	/* never get here */
	return 0;
}
