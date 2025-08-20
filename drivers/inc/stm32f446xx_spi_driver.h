/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Aug 12, 2025
 *      Author: csmla
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_


#include "stm32f446xx.h"

/*
 * SPI config structure
 */
typedef struct{
 uint8_t SPI_DeviceMode;  			/* SPI mode Possible values @ SPI_MODE */
 uint8_t SPI_BusConfig;  			/* Bus Config Possible values @ SPI_BUSCFG */
 uint8_t SPI_SclkSpeed;  			/* SPI speed Possible values @ SPI_CLK_SPEED */
 uint8_t SPI_DFF;  					/* Data frame Format Possible values @ SPI_DFF */
 uint8_t SPI_CPOL;  				/* Clock Polarity Possible values @ SPI_CPOL */
 uint8_t SPI_CPHA;  				/* Clock Phase Possible values @ SPI_CPHA */
 uint8_t SPI_SSM;  					/* Slave select Management Possible values @ SPI_SSM */


}SPI_Config_t;


/*
 * Handle structure for a SPI
 */
typedef struct{
	SPI_RegDef_t *pSPIx;   				/* <base address of SPI (x -> 1,2,3,4) */
	SPI_Config_t SPI_Config;
	uint8_t *TxBuffer;
	uint8_t *RxBuffer;
	uint8_t RxState;
	uint8_t TxState;
	uint32_t RxLen;
	uint32_t TxLen;


}SPI_Handle_t;



/*
 * @SPI_MODE
 * SPI_DeviceMode macros
 */
#define SPI_MODE_MASTER				1
#define SPI_MODE_SLAVE				0

/*
 * @SPI_BUSCFG
 * SPI_BusConfig macros
 */
#define SPI_BUS_CONFIG_FD							1  //Full Duplex
#define SPI_BUS_CONFIG_HD							2	//Half Duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY				3


/*
 * @SPI_CLK_SPEED
 * SPI_SclkSpeed macros
 */
#define SPI_CLK_SPEED_DIV2			0
#define SPI_CLK_SPEED_DIV4			1
#define SPI_CLK_SPEED_DIV8			2
#define SPI_CLK_SPEED_DIV16			3
#define SPI_CLK_SPEED_DIV32			4
#define SPI_CLK_SPEED_DIV64			5
#define SPI_CLK_SPEED_DIV128		6
#define SPI_CLK_SPEED_DIV256		7

/*
 * @SPI_DFF
 * SPI_DataFormat macros
 */
#define SPI_DFF_8BITS				0
#define SPI_DFF_16BITS				1

/*
 * @SPI_CPHA
 * SPI_Clk phase macros
 */
#define SPI_CPHA_LOW			0
#define SPI_CPHA_HIGH			1


/*
 * @SPI_CPOL
 * SPI_Clk polarity macros
 */
#define SPI_CPOL_IDLE_LOW			0
#define SPI_CPOL_IDLE_HIGH			1


/*
 * @SPI_SSM
 * SPI_SSM macros
 */
#define SPI_SSM_EN				1
#define SPI_SSM_DI				0

/*
 * @SPI_FLAG
 * SPI_FLAG macros to mask the status reg bit position
 */
#define SPI_FLAG_RXNE				(1 << SPI_SR_RXNE)
#define SPI_FLAG_TXE				(1 << SPI_SR_TXE)
#define SPI_FLAG_CHSIDE				(1 << SPI_SR_CHSIDE)
#define SPI_FLAG_UDR				(1 << SPI_SR_UDR)
#define SPI_FLAG_CRCERR				(1 << SPI_SR_CRCERR)
#define SPI_FLAG_MODF				(1 << SPI_SR_MODF)
#define SPI_FLAG_OVR    			(1 << SPI_SR_OVR)
#define SPI_FLAG_BSY    			(1 << SPI_SR_BSY)
#define SPI_FLAG_FRE				(1 << SPI_SR_FRE)

/*
 * @SPI_STATUS
 * SPI_Status macros
 */
#define SPI_STATUS_READY		0
#define SPI_STATUS_BUSY_TX		1
#define SPI_STATUS_BUSY_RX		2

/*
 * possible SPI Apllication Events
 */
#define SPI_EVENT_TX_CMPLT 		1
#define SPI_EVENT_RX_CMPLT 		2
#define SPI_EVENT_OVR_ERR 		3
#define SPI_EVENT_CRC_ERR		4


/*********************************************************************************
 * API supported by this Driver. For more information refer to the Function definitions
 **************************************************************************************/
/*
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Peripheral init/ deint
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Peripheral read/ write
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/* read / write in interrupt mode */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * peripheral Interrupt
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle);


/*
 * Application call backs
 */
void SPI_ApplicationEventCallback (SPI_Handle_t *pSPIHandle, uint8_t AppEv);

/*
 * peripheral Control
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t flagName);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
