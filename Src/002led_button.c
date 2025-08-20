/*
 * 002led_button.c
 *
 *  Created on: Aug 9, 2025
 *      Author: csmla
 */


/*
 * 001led_toggle.c
 *
 *  Created on: Aug 9, 2025
 *      Author: csmla
 */


#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

#define LOW 				0
#define BTN_PRESSED 		LOW

void delay (void){
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

	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinOPSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;


	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	while(1){

		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED){
			delay();  		/* button debounce delay */
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		}
	}
}
