/*
 * 012usart_tx.c
 *
 *  Created on: Aug 25, 2025
 *      Author: csmla
 *
 *      Fn: Send 3 messages to PC and receive them back as response
 *      pins: Tx ->PA2 , Rx ->PA3
 *      baud rate: 115200
 *      frame : start, 8 bit data, 1 stopbit, no parity
 *
 */

#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>


#define LOW 					0
#define BTN_PRESSED 			LOW
#define HIGH 					1
#define BTN_NOT_PRESSED 		HIGH


char* tx_msg[3] = { "Testing uart tx-rx mode",
					  "Sending mag to PC",
					  "Hello hyper terminal"
					};

uint8_t rx_data[1024];
USART_Handle_t USART2Handle;
uint8_t msg_cnt;
uint32_t rx_len;

/* app -flags */
uint8_t rx_cmplt;


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

	uint8_t dummy_read;

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

		/* wait until uart tx free before tranmitting msg*/
		while (USART_SendDataIT(&USART2Handle, (uint8_t *)tx_msg[msg_cnt], strlen(tx_msg[msg_cnt])) == USART_STATUS_BUSY_TX) ;

		/* will receive the same msg that is transmitted */
		rx_len = strlen(tx_msg[msg_cnt]);

		/*uart flush */
		dummy_read = USART2Handle.pUSARTx-> SR;
		dummy_read = USART2Handle.pUSARTx ->DR;
		(void)dummy_read;

		/* wait until uart rx free before receiving the message */
		while(USART_ReceiveDataIT(&USART2Handle, rx_data, rx_len) == USART_STATUS_BUSY_RX);

		/* wait until all the bytes are received from PC */
		/* When all the bytes are received rxCmplt will be SET in application callback */
		while(rx_cmplt != SET);
		rx_cmplt = RESET;

		rx_data[rx_len] = '\0';
		printf("received msg:%s\n",rx_data);
		memset(rx_data,0,strlen((char*)rx_data));

		/* msg count increment */
		msg_cnt = (msg_cnt + 1)% 3;

	}
}



/**************************Interrupt SR*********************************************/

void USART2_IRQHandler(void){

	USART_IRQHandling(&USART2Handle);

}


void USART_ApplicationEventCallback (USART_Handle_t *pUSARTHandle, uint8_t AppEv) {

	if( AppEv == USART_EV_RX_CMPLT){
		rx_cmplt = SET;

	}

}


