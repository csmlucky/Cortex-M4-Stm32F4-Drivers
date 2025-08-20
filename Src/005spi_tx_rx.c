/*
 * 005spi_master_full_duplex.c
 *
 *  Created on: Aug 13, 2025
 *      Author: csmla
 *      Fn : SPI_Master Sends commands to the slave and receive response
 *      SPI-2 Master Mode,
 *      SCLK ->2MHZ,
 *      DFF -> 8 bit testing
 *      Pins Used : PB12 --> SPI2_NSS
 *      			PB13 --> SPI_SCLK
 *      			PB14 --> SPI_MISO
 *      			PB15 --> SPI_MOSI
 *      Alternate Fn Mode --> 5
 */

#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>

/* To enable semi hosting printf */
//extern void initialise_monitor_handles(void);

#define LOW 					0
#define BTN_PRESSED 			LOW
#define HIGH 					1
#define BTN_NOT_PRESSED 		HIGH


/* Command codes */
#define COMMAND_LED_CTRL        0x50
#define COMMAND_SENSOR_READ     0x51
#define COMMAND_LED_READ        0x52
#define COMMAND_PRINT           0x53
#define COMMAND_ID_READ         0x54

#define LED_ON      1
#define LED_OFF     0

/* Analog pins */
#define ANALOG_PIN0     0
#define ANALOG_PIN1     1
#define ANALOG_PIN2     2
#define ANALOG_PIN3     3
#define ANALOG_PIN4     4

/* Arduino LED */
#define LED_PIN         13

/* ACK  */

#define ACK_BYTE 		0xF5

uint8_t dummy_write = 0xFF;
uint8_t dummy_read;


void delay (void){
	/* ~200 msec when sys clock is 16 MHZ */
	for(uint32_t i = 0; i < 500000/2; i ++);
}



 /* This Fn is used to Initialize the GPIO_PIns to behave as SPI2 */

void SPI2_GPIOInits(void){
	GPIO_Handle_t SPI2_Pins;

	SPI2_Pins.pGPIOx = GPIOB;
	SPI2_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI2_Pins.GPIO_PinConfig.GPIO_AltrFnMode = 5;
	SPI2_Pins.GPIO_PinConfig.GPIO_PinOPSpeed = GPIO_SPEED_FAST;
	SPI2_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI2_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;


	/* SCLK */
	SPI2_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(& SPI2_Pins);

	/* MOSI */
	SPI2_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(& SPI2_Pins);

	/* MISO */
	SPI2_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(& SPI2_Pins);

	/* NSS */
	SPI2_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(& SPI2_Pins);

}


/*
 * SPI2 Initialization
 */
void SPI2_Inits(void){
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_IDLE_LOW;
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_DI;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_CLK_SPEED_DIV8;

	// SPI_PeriClockControl(SPI2, ENABLE);

	SPI_Init(&SPI2Handle);

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
 * SPI_VerifyResponse
 */
uint8_t SPI_VerifyResponse(uint8_t ackbyte){
	if (ackbyte == ACK_BYTE) return 1;
	return 0;

}

/*
 * SPI2_SendingCommand
 * Sends command code and returns ack
 */
uint8_t SPI2_SendCommand(uint8_t command, uint8_t len){

	uint8_t ackbyte;

	SPI_SendData(SPI2, &command, 1);

	/* Dummy read to clear off the RXNE */
	SPI_ReceiveData(SPI2, &dummy_read, 1);

	/*send some dummy byte to fetch the response from the slave */
	SPI_SendData(SPI2, &dummy_write, 1);

	/*Read response from slave */
	SPI_ReceiveData(SPI2, &ackbyte, 1);

	return ackbyte;


}


/*
 * SPI_ReceivePrep
 * Flush the RX buffer, wait and initiate the response transfer
 */
void SPI2_ReceivePrep(void){

	/* Dummy read to clear off the RXNE */
	SPI_ReceiveData(SPI2, &dummy_read, 1);

	/* Wait for Slave to be ready with data */
	delay();

	/*send some dummy byte to fetch the response from the slave */
	SPI_SendData(SPI2, &dummy_write, 1);

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


/**************************************************************/

int main(void){

	/* To enable semi hosting printf */
	//initialise_monitor_handles();

	/* Initialize the button */
	GPIO_Button_Inits();

	/* Intialize the GPIO pins as SPI */
	SPI2_GPIOInits();

	/* Initialize the SPI */
	SPI2_Inits();

	/* Enable SSOE bit to Enable NSS automatically */
	SPI_SSOEControl(SPI2, ENABLE);



	while(1){

		/* send command after the button is pressed */
		WAIT_ForButtonPress();

		/* Enable SPI2 peripheral */
		SPI_PeripheralControl(SPI2, ENABLE);
		//printf("SPI communication started!\n");

		/* 1.CMD_LED_CTRL <pin no(1)> <value(1)> */

		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		/* Send Command */
		ackbyte = SPI2_SendCommand(commandcode, 1);

		/* verify ackbyte == ACK_BYTE */
		if(SPI_VerifyResponse(ackbyte)){
			/* send pin number and value args */
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);
			//printf("CMD_LED is executed!\n");
		}

		/* 2.CMD_SENSOR_READ <pin no(1)> > */

		/* send command after the button is pressed */
		WAIT_ForButtonPress();

		commandcode = COMMAND_SENSOR_READ;

		/* Send Command */
		ackbyte = SPI2_SendCommand(commandcode, 1);

		/* verify ackbyte == ACK_BYTE */
		if(SPI_VerifyResponse(ackbyte)){
			/* send pin number  */
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, args, 1);
			//printf("CMD_SENSOR_READ is executed!\n");

			//Rx Flush , wait and initiate the transfer
			SPI2_ReceivePrep();

			/* Read analog value */
			uint8_t analogRead;
			SPI_ReceiveData(SPI2, &analogRead, 1);
		}

		/* 3.CMD_LED_READ <pin no(1)> > */

		/* send command after the button is pressed */
		WAIT_ForButtonPress();

		commandcode = COMMAND_LED_READ;

		/* Send Command */
		ackbyte = SPI2_SendCommand(commandcode, 1);

		/* verify ackbyte == ACK_BYTE */
		if(SPI_VerifyResponse(ackbyte)){
			/* send pin number */
			args[0] = LED_PIN;
			SPI_SendData(SPI2, args, 1);
			//printf("CMD_LED_READ is executed!\n");

			//Rx Flush , wait and initiate the transfer
			SPI2_ReceivePrep();

			/* Read led value */
			uint8_t ledRead;
			SPI_ReceiveData(SPI2, &ledRead, 1);
		}

		/* 4.CMD_ID_READ <no args> */

		/* send command after the button is pressed */
		WAIT_ForButtonPress();

		commandcode = COMMAND_ID_READ;

		/* Send Command */
		ackbyte = SPI2_SendCommand(commandcode, 1);

		/* verify ackbyte == ACK_BYTE */
		if(SPI_VerifyResponse(ackbyte)){
			//printf("CMD_ID_READ is executed!\n");

			//Rx Flush , wait and initiate the transfer
			SPI2_ReceivePrep();

			/* Read led value */
			uint8_t idRead;
			SPI_ReceiveData(SPI2, &idRead, 1);
		}

		/* 5.CMD_PRINT <Len>, <string> */

		/* send command after the button is pressed */
		WAIT_ForButtonPress();

		commandcode = COMMAND_PRINT;

		/* Send Command */
		ackbyte = SPI2_SendCommand(commandcode, 1);

		/* verify ackbyte == ACK_BYTE */
		if(SPI_VerifyResponse(ackbyte)){
			//printf("CMD_PRINT is executed!\n");

			char *msg = "Hello from Master";
			uint8_t len = strlen(msg);

			/* send the length of message to be printed in slave side*/
			SPI_SendData(SPI2, &len, 1);

			/* send message to be printed in slave side*/
			SPI_SendData(SPI2, (uint8_t *) msg, len);

		}


		/* 1.CHK busy bit in SR before close communication */
		while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY));

		/* 2.Disable SPI2 peripheral */
		SPI_PeripheralControl(SPI2, DISABLE);

	}

	return 0;
}

