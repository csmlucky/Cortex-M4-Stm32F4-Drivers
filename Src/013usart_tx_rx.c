/*
 * 012usart_tx.c
 *
 *  Created on: Aug 25, 2025
 *      Author: csmla
 *
 *      Fn: Test usart 2 in tx mode
 *      pins: Tx ->PA2 , Rx ->PA3
 *      baud rate: 115200
 *      frame : start, 8 bit data, 1 stopbit, no parity
 *
 */

#include "stm32f446xx.h"
#include <string.h>


#define LOW 					0
#define BTN_PRESSED 			LOW
#define HIGH 					1
#define BTN_NOT_PRESSED 		HIGH


char* tx_data = "Testing UART1 in tx mode at 115200 baud rate";
uint8_t rx_data[50];
USART_Handle_t USART2Handle;


/*
 * Initialize GPIO pins as Usart 2 pins
 */
void USART2_GPIOInits(){
	GPIO_Handle_t USART2Pin;

	USART2Pin.pGPIOx = GPIOA;
	USART2Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USART2Pin.GPIO_PinConfig.GPIO_AltrFnMode = 7;
	USART2Pin.GPIO_PinConfig.GPIO_PinOPSpeed = GPIO_SPEED_FAST;
	USART2Pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;

	USART2Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&USART2Pin);

	USART2Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&USART2Pin);

}

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

/*
 * Usart 2 init
 */

void USART2_Inits(){



	USART2Handle.pUSARTx = USART2;

	USART2Handle.USART_Config.USART_BaudRate = USART_BAUD_115200;
	USART2Handle.USART_Config.USART_DataSize = USART_DATA_8BITS;
	USART2Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART2Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	USART2Handle.USART_Config.USART_NoOfStopbits = USART_STOPBITS_1;
	USART2Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

	USART_Init(&USART2Handle);


}



int main (void){

	/* Initialize the button */
	GPIO_Button_Inits();

	/* Initialize I2C pins : Configure the GPIO pins to behave as I2C1 */
	USART2_GPIOInits();

	/* Configure I2C peripheral */
	USART2_Inits();


	/* Enable USART2 peripheral control */
	USART_PeripheralControl(USART2, ENABLE);

	/* configure priority for usart2 irq*/
	USART_IRQPriorityConfig(IRQ_NO_USART2, NVIC_IRQ_PR10);

	/*Enable USART2 IRQ */
	USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);

	while(1){

		WAIT_ForButtonPress();


		while (!(USART_SendDataIT(&USART2Handle, (uint8_t *)tx_data, strlen(tx_data)) == USART_STATUS_BUSY_TX)) ;


	}
}



/**************************Interrupt SR*********************************************/

void USART2_IRQHandler(void){

	USART_IRQHandling(&USART2Handle);

}





