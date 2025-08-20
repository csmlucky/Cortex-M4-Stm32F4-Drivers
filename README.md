# Cortex-M4-Stm32F4-Drivers
# STM32 Peripheral Driver Development (Udemy Exercises)

This repository contains my hands-on implementation of STM32F4xx peripheral drivers based on the **STM32 Embedded Driver Development Udemy course**.  
All drivers are written in **bare-metal C** at the register level (no HAL/LL libraries).

## Features
- GPIO driver (input/output, interrupts)
- UART driver (blocking, and interrupt-driven)
- SPI driver (master/slave, full-duplex communication)
- I2C driver (master transmit/receive, slave transmit/receive ACK/NACK handling)
- RCC (Reset & Clock Control) configuration

## Tools & Hardware
- **Microcontroller**: STM32F446RE Nucleo-64 board
- **Debugger**: ST-LINK/V2 with STM32CubeIDE (SWD)
- **Logic Analyzer**: USB 24 MHz 8-channel (Saleae-compatible)
- **Languages**: C

## Testing & Validation
Each driver was verified by:
- Capturing waveforms with a **logic analyzer**  
- Comparing protocol behavior (UART, SPI, I2C) against STM32 reference manuals  
- Debugging edge cases such as clock stretching in I2C and chip-select handling in SPI  



## Repository Structure
/Drivers
|--/inc
   |--gpio_driver.h
   |--spi_driver.h
   |--i2c_driver.h
   |--usart_driver.h
   |--stm32f446xx.h
|--/src
   |--gpio_driver.c
   |--spi_driver.c
   |--i2c_driver.c
   |--usart_driver.c


