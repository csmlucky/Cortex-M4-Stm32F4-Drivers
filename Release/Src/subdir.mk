################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/015rtc_lcd_test.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/015rtc_lcd_test.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/015rtc_lcd_test.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I"D:/STM32CubeIDE/workspace_1.18.1/stm32f4_drivers/drivers/inc" -I"D:/STM32CubeIDE/workspace_1.18.1/stm32f4_drivers/BSP/inc" -I../Inc -I"D:/STM32CubeIDE/workspace_1.18.1/stm32f4_drivers/drivers" -I"D:/STM32CubeIDE/workspace_1.18.1/stm32f4_drivers/drivers/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/015rtc_lcd_test.cyclo ./Src/015rtc_lcd_test.d ./Src/015rtc_lcd_test.o ./Src/015rtc_lcd_test.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

