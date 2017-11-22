################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/MTiControl.c \
../Src/PuttyInterface.c \
../Src/commsfpga.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/main.c \
../Src/myNRF24.c \
../Src/speedcalc.c \
../Src/spi.c \
../Src/stm32f3xx_hal_msp.c \
../Src/stm32f3xx_it.c \
../Src/system_stm32f3xx.c \
../Src/tim.c \
../Src/usart.c 

OBJS += \
./Src/MTiControl.o \
./Src/PuttyInterface.o \
./Src/commsfpga.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/main.o \
./Src/myNRF24.o \
./Src/speedcalc.o \
./Src/spi.o \
./Src/stm32f3xx_hal_msp.o \
./Src/stm32f3xx_it.o \
./Src/system_stm32f3xx.o \
./Src/tim.o \
./Src/usart.o 

C_DEPS += \
./Src/MTiControl.d \
./Src/PuttyInterface.d \
./Src/commsfpga.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/main.d \
./Src/myNRF24.d \
./Src/speedcalc.d \
./Src/spi.d \
./Src/stm32f3xx_hal_msp.d \
./Src/stm32f3xx_it.d \
./Src/system_stm32f3xx.d \
./Src/tim.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F301x8 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -I"D:/Documents/STM/microkomop/Inc" -I"D:/Documents/STM/microkomop/Drivers/STM32F3xx_HAL_Driver/Inc" -I"D:/Documents/STM/microkomop/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"D:/Documents/STM/microkomop/Drivers/CMSIS/Include" -I"D:/Documents/STM/microkomop/Drivers/CMSIS/Device/ST/STM32F3xx/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


