/*
 * 001led_toggle.c
 *
 *  Created on: Aug 9, 2025
 *      Author: csmla
 */


#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

void delay (void){
	for(uint32_t i = 0; i < 500000; i ++);
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


	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);
	GPIO_Init(&GpioLed);
	while(1){

		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();
	}
}
