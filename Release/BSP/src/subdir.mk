################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/src/lcd.c 

OBJS += \
./BSP/src/lcd.o 

C_DEPS += \
./BSP/src/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/src/%.o BSP/src/%.su BSP/src/%.cyclo: ../BSP/src/%.c BSP/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I"D:/STM32CubeIDE/workspace_1.18.1/stm32f4_drivers/drivers/inc" -I"D:/STM32CubeIDE/workspace_1.18.1/stm32f4_drivers/BSP/inc" -I../Inc -I"D:/STM32CubeIDE/workspace_1.18.1/stm32f4_drivers/drivers" -I"D:/STM32CubeIDE/workspace_1.18.1/stm32f4_drivers/drivers/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-BSP-2f-src

clean-BSP-2f-src:
	-$(RM) ./BSP/src/lcd.cyclo ./BSP/src/lcd.d ./BSP/src/lcd.o ./BSP/src/lcd.su

.PHONY: clean-BSP-2f-src

