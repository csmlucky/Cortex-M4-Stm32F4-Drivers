################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/004spi_tx_testing.c \
../Src/006spi_receive_it.c \
../Src/007i2c_master_tx.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/004spi_tx_testing.o \
./Src/006spi_receive_it.o \
./Src/007i2c_master_tx.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/004spi_tx_testing.d \
./Src/006spi_receive_it.d \
./Src/007i2c_master_tx.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I"D:/STM32CubeIDE/workspace_1.18.1/stm32f4_drivers/drivers/inc" -I../Inc -I"D:/STM32CubeIDE/workspace_1.18.1/stm32f4_drivers/drivers" -I"D:/STM32CubeIDE/workspace_1.18.1/stm32f4_drivers/drivers/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/004spi_tx_testing.cyclo ./Src/004spi_tx_testing.d ./Src/004spi_tx_testing.o ./Src/004spi_tx_testing.su ./Src/006spi_receive_it.cyclo ./Src/006spi_receive_it.d ./Src/006spi_receive_it.o ./Src/006spi_receive_it.su ./Src/007i2c_master_tx.cyclo ./Src/007i2c_master_tx.d ./Src/007i2c_master_tx.o ./Src/007i2c_master_tx.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

