/*
 * 004spi_tx_testing.c
 *
 *  Created on: Aug 12, 2025
 *      Author: csmla
 *
 *      SPI-2 Master Mode,
 *      SCLK ->8MHZ,
 *      DFF -> Both 8 and 16 bit testing
 *      Pins Used : PB12 --> SPI2_NSS
 *      			PB13 --> SPI_SCLK
 *      			PB14 --> SPI_MISO
 *      			PB15 --> SPI_MOSI
 *      Alternate Fn Mode --> 5
 */


#include "stm32f446xx.h"
#include <string.h>

/*
 * This Fn is used to Initialize the GPIO_PIns to behave as SPI2
 */

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
	//SPI2_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(& SPI2_Pins);

	/* NSS */
	//SPI2_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(& SPI2_Pins);

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
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_EN;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_CLK_SPEED_DIV128;

	//SPI_PeriClockControl(SPI2, ENABLE);

	SPI_Init(&SPI2Handle);

}


int main(void){

	//char user_data[] = "Hello-World";
	uint8_t test_data[] = {0xAA, 0xAA, 0x55, 0x55};

	/* Intialize the GPIO pins as SPI */
	SPI2_GPIOInits();

	/* Initialize the SPI */
	SPI2_Inits();

	/*Disable Internal slave select */
	SPI_SSIControl(SPI2, DISABLE);

	/* Enable SPI2 peripheral */
	SPI_PeripheralControl(SPI2, ENABLE);


	/* Send SPI TXData */
	//SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));
	SPI_SendData(SPI2, test_data, 4);

	/* CHK busy bit in SR to close communication */
	while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY));

	/* Disable SPI2 peripheral */
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}
