/*
 * bsp_lcd_testing.c
 *
 *  Created on: Sep 3, 2025
 *      Author: csmla
 */


#include "stm32f446xx.h"
#include <stdio.h>
#include "lcd.h"
#include <string.h>


#define LOW 					0
#define BTN_PRESSED 			LOW
#define HIGH 					1
#define BTN_NOT_PRESSED 		HIGH


char * disBuf = "Hello LCD";


/*
 * GPIO-Button INit
 */
void GPIO_Button_Inits(void){
	GPIO_Handle_t GpioButton;

	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinOPSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;

	GPIO_Init(&GpioButton);

}

/*
 * delay
 */
void delay (void){
	/* ~200 msec when sys clock is 16 MHZ */
	for(uint32_t i = 0; i < 500000/2; i ++);
}


/*
 * WAIT_ForButtonPress
 * Polling button
 */
void WAIT_ForButtonPress (void){

	while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_NOT_PRESSED);

	/* debounce */
	delay();
}


int main(void){

	/* Initialize the button */
	GPIO_Button_Inits();

	/* LCD init */
	LCD_Init();

	while(1){

		/* wait for button press */
		WAIT_ForButtonPress();

		LCD_DisplayClr();
		LCD_SetCursor(1, 5);
		LCD_PrintString(disBuf);

	}

	return 0;

}


