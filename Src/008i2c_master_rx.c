/*
 * 007i2c_master_tx.c
 *
 *  Created on: Aug 19, 2025
 *      Author: csmla
 *      Fn : I2C1_Master Sends commands to slave and receive data from slave
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
#define SLAVE_ADDR				0x60

/* commands */
#define CMD_LEN					0x51
#define CMD_DATA				0x52



I2C_Handle_t I2C1Handle;

uint8_t  receive_data[32];
uint8_t command;
uint16_t receive_len;




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

	while(1){

		/* send command after the button is pressed */
		WAIT_ForButtonPress();

		/* send CMD_LEN */
		command = CMD_LEN;
		I2C_MasterSendData(&I2C1Handle, &command, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		/*receive len of the data from slave */
		I2C_MasterReceiveData(&I2C1Handle, (uint8_t *) &receive_len, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		/* send CMD_LEN */
		command = CMD_DATA;
		I2C_MasterSendData(&I2C1Handle, &command, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		/*receive data from slave */
		I2C_MasterReceiveData(&I2C1Handle, receive_data, receive_len , SLAVE_ADDR, I2C_DISABLE_SR);

	}



	return 0;
}
