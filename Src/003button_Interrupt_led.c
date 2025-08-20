/*
 * 003button_Interrupt_led.c
 *
 *  Created on: Aug 11, 2025
 *      Author: csmla
 */




#include <string.h>
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

#define LOW 				0
#define BTN_PRESSED 		LOW

void delay (void){
	/* ~200 msec when sys clock is 16 MHZ */
	for(uint32_t i = 0; i < 500000/2; i ++);
}

int main(void){
	GPIO_Handle_t GpioLed = {
			.pGPIOx = GPIOA,
			.GPIO_PinConfig = {
					.GPIO_PinNumber = GPIO_PIN_NO_5,
					.GPIO_PinMode = GPIO_MODE_OUT,
					.GPIO_PinOPType = GPIO_OP_TYPE_PP,
					.GPIO_PinOPSpeed = GPIO_SPEED_FAST ,
					.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD,

			}
	};

	GPIO_Handle_t GpioButton;
	memset(&GpioButton,0,sizeof(GpioButton));

	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PinOPSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;


	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	/* IRQ Configurations */
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PR13);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1){


	}
}

void EXTI15_10_IRQHandler(void){
	delay();  /* 200msec */

	GPIO_IRQHandling(GPIO_PIN_NO_13);

	/* Toggle led */
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
}
