/*
 * 006spi_interrupts.c
 *
 *  Created on: Aug 15, 2025
 *      Author: csmla
 *      Fn : SPI_Master Receives message from the slave upon gpio Interrupt. spi uses interrupt mode to receive message.
 *      SPI-2 Master Mode,
 *      SCLK ->2MHZ,
 *      DFF -> 8 bit testing
 *      Pins Used : PB12 --> SPI2_NSS
 *      			PB13 --> SPI_SCLK
 *      			PB14 --> SPI_MISO
 *      			PB15 --> SPI_MOSI
 *      Alternate Fn Mode --> 5
 *      PD6 --> slave interrupt pin
 *
 */

#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>


#define RCV_BUFF_MAX	  	100

uint8_t dummy_write = 0xFF;
uint8_t dummy_read;

SPI_Handle_t SPI2Handle;
char rcvBuff[RCV_BUFF_MAX];

/* this flag will be set in the spi slave gpio pin isr */
volatile uint8_t dataAvailable = 0;

volatile uint8_t rcvStop = 0;
uint8_t readByte;


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
	//SPI_Handle_t SPI2Handle;

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
 * Initialize the gpio pin as external interrupt over which spi slave notifies the data
 * availability
 */
void Slave_GPIO_InterruptPin_Inits(void){
	GPIO_Handle_t SpiInt;
	memset(&SpiInt, 0, sizeof(SpiInt));

	SpiInt.pGPIOx = GPIOD;
	SpiInt.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	SpiInt.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	SpiInt.GPIO_PinConfig.GPIO_PinOPSpeed = GPIO_SPEED_FAST;
	SpiInt.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;

	GPIO_Init(&SpiInt);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PR13);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

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





/**************************************************************/

int main(void){


	/* Initialize the slave gpio pin interrupt  */
	Slave_GPIO_InterruptPin_Inits();

	/* Intialize the GPIO pins as SPI */
	SPI2_GPIOInits();

	/* Initialize the SPI */
	SPI2_Inits();

	/* Enable SSOE bit to Enable NSS automatically */
	SPI_SSOEControl(SPI2, ENABLE);

	/* Enable SPI interrupt IRQ */
	SPI_IRQPriorityConfig(IRQ_NO_SPI2, NVIC_IRQ_PR15);
	SPI_IRQInterruptConfig(IRQ_NO_SPI2, ENABLE);



	while(1){

		rcvStop = 0;

		/*wait until slave data ready */
		while(! dataAvailable);

		/* disable slave interrupt until reception of data is complete */
		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, DISABLE);

		/* Eable SPI peripheral by setting SPE bit */
		SPI_PeripheralControl(SPI2, ENABLE);

		while(!rcvStop){

			/* write dummy byte ti initiate the transfer */
			while(SPI_SendDataIT(&SPI2Handle, &dummy_write, 1) == SPI_STATUS_BUSY_TX);
			while(SPI_ReceiveDataIT(&SPI2Handle, &readByte, 1) == SPI_STATUS_BUSY_RX);
		}

		/* Confirm the SPI is not busy before Disable SPI peripheral */
		while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY) == SET);
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("Received Msg: %s\n", rcvBuff);

		/* Enable Slave GPIO pin interrupt to receive next msg */
		dataAvailable = 0;
		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);




	}


	return 0;
}

/********************************************************************************
 * 			ISR
 **************************************************************************************/
/*
 * slave data available interrupt routine
 */

void EXTI9_5_IRQHandler (void){

	GPIO_IRQHandling(GPIO_PIN_NO_6);
	dataAvailable = 1;

}

void SPI2_IRQHandler (void){

	SPI_IRQHandling(&SPI2Handle);
}


/*******************************************************************************
 * 			Call Back
 ***********************************************************************************/
void SPI_ApplicationEventCallback (SPI_Handle_t *pSPIHandle, uint8_t AppEv){

	static uint8_t i = 0;

	if(AppEv == SPI_EVENT_RX_CMPLT){
		rcvBuff[i++] = readByte;
		if((readByte == '\0') || (i == RCV_BUFF_MAX)){
			rcvStop = 1;
			rcvBuff[i] = '\0';
			i = 0;
		}
	}

}
