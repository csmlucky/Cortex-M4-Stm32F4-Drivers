/*
 * 007i2c_master_tx.c
 *
 *  Created on: Aug 19, 2025
 *      Author: csmla
 *      Fn : I2C1_Master Sends commands to slave and receive data from slave using interrupts
 *      Commands: 0x51 -> gets length of data from slave
 *      		  0x52 -> gets data from slave
 *      I2C-1 standard mode 100khz,
 *      SCLK ->100khz,
 *
 *      Pins Used : PB8 --> I2C1_SCL
 *      			PB9 --> I2C1_SDA
 *       Alternate Fn Mode --> 4
 */

#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>



#define LOW 					0
#define BTN_PRESSED 			LOW
#define HIGH 					1
#define BTN_NOT_PRESSED 		HIGH

#define MY_ADDR 				0x40
#define SLAVE_ADDR				0x65

/* commands */
#define CMD_LEN					0x51
#define CMD_DATA				0x52



I2C_Handle_t I2C1Handle;

uint8_t  receive_data[32];
uint8_t command;
uint16_t receive_len;

/* flag variable */
uint8_t rxComplt;



void delay (void){
	/* ~200 msec when sys clock is 16 MHZ */
	for(uint32_t i = 0; i < 500000/2; i ++);
}



 /* This Fn is used to Initialize the GPIO_PIns to behave as SPI2 */

void I2C1_GPIOInits(void){
	GPIO_Handle_t I2C1_Pins;

	I2C1_Pins.pGPIOx = GPIOB;
	I2C1_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2C1_Pins.GPIO_PinConfig.GPIO_AltrFnMode = 4;
	I2C1_Pins.GPIO_PinConfig.GPIO_PinOPSpeed = GPIO_SPEED_HIGH;
	I2C1_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2C1_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


	/* SCL */
	I2C1_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_Init(& I2C1_Pins);

	/* SDA */
	I2C1_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(& I2C1_Pins);



}


/*
 * SPI2 Initialization
 */
void I2C1_Inits(void){

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCLK_SPEED_SM;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;

	I2C_Init(&I2C1Handle);

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
 * WAIT_ForButtonPress
 * Polling button
 */
void WAIT_ForButtonPress (void){

	while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_NOT_PRESSED);

	/* debounce */
	delay();
}


int main (void){

	/* Initialize the button */
	GPIO_Button_Inits();

	/* Initialize I2C pins : Configure the GPIO pins to behave as I2C1 */
	I2C1_GPIOInits();

	/* Configure I2C peripheral */
	I2C1_Inits();

	/* Enable peripheral control */
	I2C_PeripheralControl(I2C1, ENABLE);

	/* configure ack control in CR1 after enabling the peripheral i2c*/
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	/* configure IRQ Priority for I2C1 event and error interrupts*/
	 I2C_IRQPriorityConfig(IRQ_NO_I2C1_EV, NVIC_IRQ_PR10);
	 I2C_IRQPriorityConfig(IRQ_NO_I2C1_ER, NVIC_IRQ_PR9);

	/* Enable I2C1_EV IRQ and I2C_ER IRQ */
	 I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	 I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);


	while(1){

		/* send command after the button is pressed */
		WAIT_ForButtonPress();
		printf("button is pressed\n");

		/* send CMD_LEN */
		command = CMD_LEN;
		while (I2C_MasterSendDataIT(&I2C1Handle, &command, 1, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_STATUS_READY);

		/*receive len of the data from slave */
		while(I2C_MasterReceiveDataIT(&I2C1Handle, (uint8_t *) &receive_len, 1, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_STATUS_READY);
		rxComplt = RESET;

		/* send CMD_LEN */
		command = CMD_DATA;
		while(I2C_MasterSendDataIT(&I2C1Handle, &command, 1, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_STATUS_READY);

		/*receive data from slave */
		while(I2C_MasterReceiveDataIT(&I2C1Handle, receive_data, receive_len , SLAVE_ADDR, I2C_DISABLE_SR) != I2C_STATUS_READY);

		while(rxComplt == RESET);
		receive_data[receive_len] ='\0';
		printf("Received Data : %s\n", receive_data);
	}



	return 0;
}

/* i2c1 event ISR */
void I2C1_EV_IRQHandler (void){

	I2C_EV_IRQHandling(&I2C1Handle);

}


/* i2c1 error ISR */
void I2C1_ER_IRQHandler (void){

	I2C_ER_IRQHandling(&I2C1Handle);

}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
    if(AppEv == I2C_EV_TX_CMPLT)
    {
        printf("Tx is completed\n");
    }
    else if (AppEv == I2C_EV_RX_CMPLT)
    {
        printf("Rx is completed\n");
        rxComplt = SET;

    }
    else if (AppEv == I2C_ERROR_AF)
    {
        printf("Error : Ack failure\n");
        /* Master ACK failure happens when slave fails to send ACK
    	    for the byte sent from the master */
        I2C_CloseSendData(pI2CHandle);

        /* Generating stop condition to release the bus */
        I2C_GenerateStopCondition(I2C1);

        /* Hanging in infinite loop */
        while(1);
    }
}


