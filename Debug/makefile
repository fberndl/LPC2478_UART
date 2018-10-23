################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Libraries/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
LPC2478_UART.hex \

SECONDARY_LIST += \
LPC2478_UART.lst \

SECONDARY_SIZE += \
LPC2478_UART.siz \


# All Target
all: LPC2478_UART.elf secondary-outputs

# Tool invocations
LPC2478_UART.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Sourcery Linux GCC C Linker'
	arm-none-eabi-gcc -T"/home/es/Desktop/LPC2478_UART/Linker/lpc2478.ld" -nostartfiles -L"/home/es/Desktop/LPC2478_UART/Linker" -Wl,-Map,LPC2478_UART.map -mcpu=arm7tdmi-s -g3 -gdwarf-2 -o "LPC2478_UART.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

LPC2478_UART.hex: LPC2478_UART.elf
	@echo 'Invoking: ARM Sourcery Linux GNU Create Flash Image'
	arm-none-eabi-objcopy -O binary LPC2478_UART.elf  "LPC2478_UART.hex"
	@echo 'Finished building: $@'
	@echo ' '

LPC2478_UART.lst: LPC2478_UART.elf
	@echo 'Invoking: ARM Sourcery Linux GNU Create Listing'
	arm-none-eabi-objdump -h -S LPC2478_UART.elf > "LPC2478_UART.lst"
	@echo 'Finished building: $@'
	@echo ' '

LPC2478_UART.siz: LPC2478_UART.elf
	@echo 'Invoking: ARM Sourcery Linux GNU Print Size'
	arm-none-eabi-size  --format=berkeley -x -t LPC2478_UART.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SECONDARY_SIZE)$(OBJS)$(C_DEPS)$(ASM_DEPS)$(SECONDARY_FLASH)$(EXECUTABLES)$(SECONDARY_LIST)$(S_UPPER_DEPS) LPC2478_UART.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
