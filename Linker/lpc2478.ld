MEMORY 
{
	flash       : ORIGIN = 0x00000000, LENGTH = 504K 
	ram			: ORIGIN = 0x40000000, LENGTH =  64K
	usbram   	: ORIGIN = 0x7FD00000, LENGTH =  16K
	ethram   	: ORIGIN = 0x7FE00000, LENGTH =  16K
}

__stack_end__ = 0x40000000 + 64K - 4;


SECTIONS 
{

	. = 0;
	
	startup : { *(.startup)} >flash

	prog : 
	{
		*(.text)
		*(.rodata)
		*(.rodata*)
		*(.glue_7)
		*(.glue_7t)
		KEEP(*(.eh_frame))		
	} >flash
	
	.ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >flash
	PROVIDE_HIDDEN (__exidx_start = .);
	.ARM.exidx   : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) } >flash
	PROVIDE_HIDDEN (__exidx_end = .);


	__end_of_text__ = .;



	.data : 
	{
		__data_beg__ = .;
		__data_beg_src__ = __end_of_text__;
		*(.data)
		__data_end__ = .;
	} >ram AT>flash

	.bss : 
	{
		__bss_beg__ = .;
		*(.bss)
	} >ram AT>flash		/*************** vorher >ram */

	/* Align here to ensure that the .bss section occupies space up to
	_end.  Align after .bss to ensure correct alignment even if the
	.bss section disappears because there are no input sections.  */
	. = ALIGN(32 / 8);
	_bss_end__ = . ; __bss_end__ = . ; __end__ = . ;

	PROVIDE(__HEAP_START = __end__ );

	
	.usbram (NOLOAD):
	{
	__usbram_beg__ = .;
	*(.dmaram)
		__usbram_end__ = .;
	} >usbram

	.ethram (NOLOAD):
	{
	__ethram_beg__ = .;
	*(.ethram)
		__ethram_end__ = .;
	} >ethram

}
	. = ALIGN(32 / 8);
	_end = .;
	
	PROVIDE (end = .);


