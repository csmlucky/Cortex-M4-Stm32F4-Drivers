/*
 * 007i2c_master_tx.c
 *
 *  Created on: Aug 19, 2025
 *      Author: csmla
 *      Fn : I2C1_Slave Sends message  to master using interrupts
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

#define MY_ADDR 				0x65
#define SLAVE_ADDR				0x40

/* commands */
#define CMD_LEN					0x51
#define CMD_DATA				0x52



I2C_Handle_t I2C1Handle;

uint8_t tx_buff[50] = "stm32 slave is transmitting.";
uint8_t command_code;


void delay (void){
	/* ~200 msec when sys clock is 16 MHZ */
	for(uint32_t i = 0; i < 500000/2; i ++);
}



 /* This Fn is used to Initialize the GPIO_PIns to behave as I2C */

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
 * I2C Initialization
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

	/* configure IRQ Priority for I2C1 event and error interrupts*/
	 I2C_IRQPriorityConfig(IRQ_NO_I2C1_EV, NVIC_IRQ_PR10);
	 I2C_IRQPriorityConfig(IRQ_NO_I2C1_ER, NVIC_IRQ_PR9);

	/* Enable I2C1_EV IRQ and I2C_ER IRQ */
	 I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	 I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	 /* Enable I2C1 TXE, RXNE, and error interrupts in CR2 */
	 I2C_SlaveEnableDisableInterrupt(I2C1, ENABLE);

	 /* Enable peripheral control */
	 I2C_PeripheralControl(I2C1, ENABLE);

	 /* configure ack control in CR1 after enabling the peripheral i2c*/
	 I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);


	while(1){


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
	static uint8_t len = 0;

    if(AppEv == I2C_EV_DATA_REQ){
    	/* master requested data. slave has to send it */
    	uint8_t send_data;
    	if(command_code == CMD_LEN){
    		send_data = strlen((char *)tx_buff);
    		I2C_SlaveSendData(pI2CHandle ->pI2Cx, send_data);
    	}
    	else if(command_code == CMD_DATA){
    		send_data = tx_buff[len ++];
    		I2C_SlaveSendData(pI2CHandle ->pI2Cx, send_data);
    	}

    }
    else if (AppEv == I2C_EV_DATA_RCV){
	  /* master sent data. Slave has to read it */
    	command_code = I2C_SlaveReceiveData(pI2CHandle ->pI2Cx);

    }
    else if (AppEv == I2C_ERROR_AF)
    {
        /* Slave ACK failure happens when master sends NACK. so slave has to stop txing */
    	/* this happens only in slave txing */
    	len = 0;
    	command_code = 0xff;

    }
    else if (AppEv == I2C_EV_STOP){
    	/* This happens only slave rxing */
    	/* master has stoped the i2c communication with the slave */
    }
}


