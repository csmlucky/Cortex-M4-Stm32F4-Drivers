################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f446xx_gpio_driver.c \
../drivers/src/stm32f446xx_i2c_driver.c \
../drivers/src/stm32f446xx_rcc_driver.c \
../drivers/src/stm32f446xx_rtc_driver.c \
../drivers/src/stm32f446xx_spi_driver.c \
../drivers/src/stm32f446xx_usart_driver.c 

OBJS += \
./drivers/src/stm32f446xx_gpio_driver.o \
./drivers/src/stm32f446xx_i2c_driver.o \
./drivers/src/stm32f446xx_rcc_driver.o \
./drivers/src/stm32f446xx_rtc_driver.o \
./drivers/src/stm32f446xx_spi_driver.o \
./drivers/src/stm32f446xx_usart_driver.o 

C_DEPS += \
./drivers/src/stm32f446xx_gpio_driver.d \
./drivers/src/stm32f446xx_i2c_driver.d \
./drivers/src/stm32f446xx_rcc_driver.d \
./drivers/src/stm32f446xx_rtc_driver.d \
./drivers/src/stm32f446xx_spi_driver.d \
./drivers/src/stm32f446xx_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o drivers/src/%.su drivers/src/%.cyclo: ../drivers/src/%.c drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I"D:/STM32CubeIDE/workspace_1.18.1/stm32f4_drivers/drivers/inc" -I../Inc -I"D:/STM32CubeIDE/workspace_1.18.1/stm32f4_drivers/drivers" -I"D:/STM32CubeIDE/workspace_1.18.1/stm32f4_drivers/drivers/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-src

clean-drivers-2f-src:
	-$(RM) ./drivers/src/stm32f446xx_gpio_driver.cyclo ./drivers/src/stm32f446xx_gpio_driver.d ./drivers/src/stm32f446xx_gpio_driver.o ./drivers/src/stm32f446xx_gpio_driver.su ./drivers/src/stm32f446xx_i2c_driver.cyclo ./drivers/src/stm32f446xx_i2c_driver.d ./drivers/src/stm32f446xx_i2c_driver.o ./drivers/src/stm32f446xx_i2c_driver.su ./drivers/src/stm32f446xx_rcc_driver.cyclo ./drivers/src/stm32f446xx_rcc_driver.d ./drivers/src/stm32f446xx_rcc_driver.o ./drivers/src/stm32f446xx_rcc_driver.su ./drivers/src/stm32f446xx_rtc_driver.cyclo ./drivers/src/stm32f446xx_rtc_driver.d ./drivers/src/stm32f446xx_rtc_driver.o ./drivers/src/stm32f446xx_rtc_driver.su ./drivers/src/stm32f446xx_spi_driver.cyclo ./drivers/src/stm32f446xx_spi_driver.d ./drivers/src/stm32f446xx_spi_driver.o ./drivers/src/stm32f446xx_spi_driver.su ./drivers/src/stm32f446xx_usart_driver.cyclo ./drivers/src/stm32f446xx_usart_driver.d ./drivers/src/stm32f446xx_usart_driver.o ./drivers/src/stm32f446xx_usart_driver.su

.PHONY: clean-drivers-2f-src

