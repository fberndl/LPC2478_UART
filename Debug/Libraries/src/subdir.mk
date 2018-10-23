################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/src/UART.c \
../Libraries/src/cb.c \
../Libraries/src/init.c \
../Libraries/src/irq.c \
../Libraries/src/libnosys_gnu.c \
../Libraries/src/lpc2478_lib.c 

ASM_SRCS += \
../Libraries/src/boot.asm 

OBJS += \
./Libraries/src/UART.o \
./Libraries/src/boot.o \
./Libraries/src/cb.o \
./Libraries/src/init.o \
./Libraries/src/irq.o \
./Libraries/src/libnosys_gnu.o \
./Libraries/src/lpc2478_lib.o 

C_DEPS += \
./Libraries/src/UART.d \
./Libraries/src/cb.d \
./Libraries/src/init.d \
./Libraries/src/irq.d \
./Libraries/src/libnosys_gnu.d \
./Libraries/src/lpc2478_lib.d 

ASM_DEPS += \
./Libraries/src/boot.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/src/%.o: ../Libraries/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -I"/home/es/Desktop/LPC2478_UART/Libraries/inc" -O0 -g -pedantic -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -g -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=arm7tdmi-s -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Libraries/src/%.o: ../Libraries/src/%.asm
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC Assembler'
	arm-none-eabi-gcc -x assembler-with-cpp -pedantic -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=arm7tdmi-s -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


