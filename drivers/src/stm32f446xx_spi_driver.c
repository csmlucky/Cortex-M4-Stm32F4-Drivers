/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Aug 12, 2025
 *      Author: csmla
 */


#include "stm32f446xx_spi_driver.h"
#include <stdio.h>

static void SPI_TXE_InterruptHandle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_InterruptHandle(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_InterruptHandle(SPI_Handle_t *pSPIHandle);



/******************************************************************************
 * @fn			- SPI_PeriClockControl
 *
 * @brief		- This fn Enables or Disables peripheral clock for the SPI
 *
 * @param[in]	- SPI port base address
 * @param[in]	- ENABLE or DISABLE Macros
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
			if(pSPIx == SPI1) SPI1_PCLK_EN();
			else if (pSPIx == SPI2) SPI2_PCLK_EN();
			else if (pSPIx == SPI3) SPI3_PCLK_EN();
			else if (pSPIx == SPI4) SPI4_PCLK_EN();
	}else{
			if(pSPIx == SPI1) SPI1_PCLK_DI();
			else if (pSPIx == SPI2) SPI2_PCLK_DI();
			else if (pSPIx == SPI3) SPI3_PCLK_DI();
			else if (pSPIx == SPI4) SPI4_PCLK_DI();

		}

}





/******************************************************************************
 * @fn			- SPI_Init
 *
 * @brief		-
 *
 * @param[in]	- SPI Handle base address
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){

	/* Enable SPI peripheral clock */
	SPI_PeriClockControl(pSPIHandle ->pSPIx, ENABLE);

	/* Configure SPI CR1 Reg */
	uint32_t tempreg = 0;
	/* 1.Configure SPI -Mode */
	tempreg = (pSPIHandle -> SPI_Config.SPI_DeviceMode << 2);

	/* 2. Configure SPI- Bus */
	if(pSPIHandle -> SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		/* clear BIDIMODE bit in SPI-CR1 */
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle -> SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		/* set BIDIMODE bit in SPI-CR1 */
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}

	else if(pSPIHandle -> SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		/* BIDIMODE bit should be cleared in SPI-CR1 */
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		/* Set RXONLY bit */
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	/* 3. Configure SPI-Serial clock speed */
	tempreg |= (pSPIHandle -> SPI_Config.SPI_SclkSpeed  << SPI_CR1_BR);

	/* 4.Configure SPI - DFF */
	tempreg |= (pSPIHandle -> SPI_Config.SPI_DFF  << SPI_CR1_DFF);

	/* 5.Configure SPI - CPOL */
	tempreg |= (pSPIHandle -> SPI_Config.SPI_CPOL  << SPI_CR1_CPOL);

	/* 6.Configure SPI - CPHA */
	tempreg |= (pSPIHandle -> SPI_Config.SPI_CPHA  << SPI_CR1_CPHA);

	/* 7..Configure SPI - SSM */
	tempreg |= (pSPIHandle -> SPI_Config.SPI_SSM  << SPI_CR1_SSM);

	pSPIHandle -> pSPIx-> CR1  = tempreg;


}


/******************************************************************************
 * @fn			- SPI_DeInit
 *
 * @brief		-
 *
 * @param[in]	- SPI HAndle  base address
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx){

	if(pSPIx == SPI1) SPI1_REG_RESET();
	else if (pSPIx == SPI2) SPI2_REG_RESET();
	else if (pSPIx == SPI3) SPI3_REG_RESET();
	else if (pSPIx == SPI4) SPI4_REG_RESET();


}

/******************************************************************************
 * @fn			- SPI_GetFlagStatus
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-FLAG_RESET or FLAG_SET macros
 *
 * @note		-
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t flagName){

	if(pSPIx -> SR & flagName)return FLAG_SET;
	return FLAG_RESET;

}


/******************************************************************************
 * @fn			- SPI_PeripheralControl
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		pSPIx -> CR1 |= (1 << SPI_CR1_SPE);
	}
	else{
		pSPIx -> CR1 &= ~(1 << SPI_CR1_SPE);
	}

}


/******************************************************************************
 * @fn			- SPI_SSIControl
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void SPI_SSIControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == DISABLE){
			pSPIx -> CR1 |= (1 << SPI_CR1_SSI);
		}
		else{
			pSPIx -> CR1 &= ~(1 << SPI_CR1_SSI);
		}


}


/******************************************************************************
 * @fn			- SPI_SSOEControl
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void SPI_SSOEControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
			pSPIx -> CR2 |= (1 << SPI_CR2_SSOE);
		}
		else{
			pSPIx -> CR2 &= ~(1 << SPI_CR2_SSOE);
		}


}





/******************************************************************************
 * @fn			- SPI_SendData
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

	while(Len > 0){
		/* wait until the transmitter buffer empty flag in SR reg */
		while((SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE)) == FLAG_RESET);


		if(pSPIx -> CR1 & (1 << SPI_CR1_DFF)){
			/* 16 bit data frame -> write 16 bit data*/

			pSPIx -> DR = *((uint16_t *)pTxBuffer);
			Len --;
			Len--;
			(uint16_t *)pTxBuffer ++;
		}
		else{
			/* 8 bit Data frame -> write 8 bit */

			pSPIx -> DR = *(pTxBuffer);
			Len --;
			pTxBuffer ++;
		}
	}




}


/******************************************************************************
 * @fn			- SPI_ReceiveData
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){
	while(Len > 0){
			/* wait until the receive buffer Not empty flag in SR reg */
			while((SPI_GetFlagStatus(pSPIx, SPI_FLAG_RXNE)) == FLAG_RESET);


			if(pSPIx -> CR1 & (1 << SPI_CR1_DFF)){
				/* 16 bit data frame -> read 16 bit data*/

				*((uint16_t *)pRxBuffer) = pSPIx -> DR;
				Len --;
				Len--;
				(uint16_t *)pRxBuffer ++;
			}
			else{
				/* 8 bit Data frame -> read 8 bit */

				*(pRxBuffer) = pSPIx -> DR;
				Len --;
				pRxBuffer ++;
			}
		}


}


/******************************************************************************
 * @fn			- SPI_SendDataIT
 *
 * @brief		- send data with tx interrupt
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){

	uint8_t state = pSPIHandle -> TxState;

	if(state != SPI_STATUS_BUSY_TX){

	/* save the tx buffer and len info in some global variables */
		pSPIHandle -> TxBuffer = pTxBuffer;
		pSPIHandle -> TxLen = Len;

		/*Mark the SPI state as busy in transmission so that other code code can not take
		 * over same spi peripheral until the transmission is complete
		 */
		pSPIHandle -> TxState = SPI_STATUS_BUSY_TX;

		/* Enable TXEIE bit in CR2 to get interrupt whenever TXE flag is set in SR*/
		pSPIHandle -> pSPIx -> CR2 |= (1 << SPI_CR2_TXEIE);

	}
	return state;

}


/******************************************************************************
 * @fn			- SPI_ReceiveDataIT
 *
 * @brief		- receives data with Rx interrupt
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){

	uint8_t state = pSPIHandle -> RxState;

	if(state != SPI_STATUS_BUSY_RX){

	   /* save the rx buffer and len info in some global variables */
		pSPIHandle -> RxBuffer = pRxBuffer;
		pSPIHandle -> RxLen = Len;

		/* Mark the SPI state as busy in reception so that other code code can not take
		 * over same spi peripheral until the reception is complete
		 */
		pSPIHandle -> RxState = SPI_STATUS_BUSY_RX;

		/* Enable RXNEIE bit in CR2 to get interrupt whenever RXNE flag is set in SR*/
		pSPIHandle -> pSPIx -> CR2 |= (1 << SPI_CR2_RXNEIE);

	}
	return state;



}







/******************************************************************************
 * @fn			- SPI_IRQInterruptConfig
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			/* configure ISER0 */
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		if(IRQNumber >= 32 && IRQNumber < 64){
			/* configure ISER1 */
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		if(IRQNumber >= 64 && IRQNumber < 96){
			/* configure ISER2 */
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}
		if(IRQNumber >= 96 && IRQNumber < 128){
			/* configure ISER3 */
			*NVIC_ISER3 |= (1 << (IRQNumber % 32));
		}
	}
	/* Clear IRQ */
	else{
		if(IRQNumber <= 31){
			/* configure ICER0 */
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		if(IRQNumber >= 32 && IRQNumber < 64){
			/* configure ICER1 */
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		if(IRQNumber >= 64 && IRQNumber < 96){
			/* configure ICER2 */
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}
		if(IRQNumber >= 96 && IRQNumber < 128){
			/* configure ICER3 */
			*NVIC_ICER3 |= (1 << (IRQNumber % 32));
		}

	}


}

/******************************************************************************
 * @fn			- SPI_IRQPriorityConfig
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

	uint8_t iprx = IRQNumber / 4;  			/* ipr reg number offset address */
	uint8_t iprx_section = IRQNumber % 4;    /* position in iprx reg */

	uint8_t shift_position = (8 * iprx_section) + (8 - NO_IPR_BITS_IMPLEMENTED);  //only upper 4 bits are implemented in IPR
	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << shift_position);


}


/******************************************************************************
 * @fn			- SPI_IRQHandling
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){

	uint8_t state_itflag;
	uint8_t state_iten;

	/* chk for TXE */
	state_itflag = pSPIHandle -> pSPIx -> SR & (1 << SPI_SR_TXE);
	state_iten = pSPIHandle -> pSPIx -> CR2 & (1 << SPI_CR2_TXEIE);

	if(state_iten && state_itflag){
		/* Handle TXE interrupt */
		SPI_TXE_InterruptHandle(pSPIHandle);
	}

	/* chk for RXNE */
		state_itflag = pSPIHandle -> pSPIx -> SR & (1 << SPI_SR_RXNE);
		state_iten = pSPIHandle -> pSPIx -> CR2 & (1 << SPI_CR2_RXNEIE);

		if(state_iten && state_itflag){
			/* Handle RXNE interrupt */
			SPI_RXNE_InterruptHandle(pSPIHandle);
	}

	/* chk for OVR flag */
	state_itflag = pSPIHandle -> pSPIx -> SR & (1 << SPI_SR_OVR);
	state_iten = pSPIHandle -> pSPIx -> CR2 & (1 << SPI_CR2_ERRIE);

	if(state_iten && state_itflag){
		/* Handle RXNE interrupt */
		SPI_OVR_InterruptHandle(pSPIHandle);
	}


}


/******************************************************************************
 * @fn			- SPI_CloseTransmission
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	/* clear TXEIE bit in CR2 to prevent interrupt setting up of TXE flag SR*/
	pSPIHandle -> pSPIx -> CR2 &= ~ (1 << SPI_CR2_TXEIE);

	/* Reinitialize Tx buffer & Length */
	pSPIHandle -> TxBuffer = NULL;
	pSPIHandle -> TxLen = 0;

	/* Clear TxState flag to notify the spi is available for next communication */
	pSPIHandle -> TxState = SPI_STATUS_READY;


}

/******************************************************************************
 * @fn			- SPI_CloseReception
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */

void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	/* clear RXNEIE bit in CR2 to prevent interrupt setting up of TXE flag SR*/
	pSPIHandle -> pSPIx -> CR2 &= ~ (1 << SPI_CR2_RXNEIE);

	/* Reinitialize Rx buffer & Length */
	pSPIHandle -> RxBuffer = NULL;
	pSPIHandle -> RxLen = 0;

	/* Clear RxState flag to notify the spi is available for next communication */
	pSPIHandle -> RxState = SPI_STATUS_READY;


}


/******************************************************************************
 * @fn			- SPI_ClearOVRFlag
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle){

	uint16_t temp;
	/* clear OVR flag if it is not in tranmission ->1. read DR followed by SR */
	if(pSPIHandle -> TxState != SPI_STATUS_BUSY_TX){
		temp = pSPIHandle -> pSPIx -> DR;
		temp = pSPIHandle -> pSPIx -> SR;
	}
	(void)temp;

}




/**************************************************************************************
*				Helper Fns
***************************************************************************************/

/******************************************************************************
 * @fn			- SPI_TXE_InterruptHandle
 *
 * @brief		- Helper fn
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void SPI_TXE_InterruptHandle(SPI_Handle_t *pSPIHandle){

	if(pSPIHandle -> TxLen > 0){

		if(pSPIHandle ->pSPIx -> CR1 & (1 << SPI_CR1_DFF)){
			/* 16 bit data frame */

			pSPIHandle ->pSPIx -> DR = *(uint16_t *)(pSPIHandle -> TxBuffer);
			pSPIHandle -> TxLen --;
			pSPIHandle -> TxLen --;
			(uint16_t *)pSPIHandle -> TxBuffer++;
		}
		else{
			/* 8 bit data */
			pSPIHandle ->pSPIx -> DR = *(pSPIHandle -> TxBuffer);
			pSPIHandle -> TxLen --;
			pSPIHandle -> TxBuffer++;
		}

	}
	if(! pSPIHandle -> TxLen){
		/* close transmission and make it available */
		SPI_CloseTransmission(pSPIHandle);

		/* call back */
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);

	}

}





/******************************************************************************
 * @fn			- SPI_RXNE_InterruptHandle
 *
 * @brief		- Helper fn
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void SPI_RXNE_InterruptHandle(SPI_Handle_t *pSPIHandle){

	if(pSPIHandle -> RxLen > 0){

		if(pSPIHandle ->pSPIx -> CR1 & (1 << SPI_CR1_DFF)){
			/* 16 bit data frame */

		    *(uint16_t *)(pSPIHandle -> RxBuffer) = pSPIHandle ->pSPIx -> DR ;
			pSPIHandle -> RxLen --;
			pSPIHandle -> RxLen --;
			(uint16_t *)pSPIHandle -> RxBuffer++;
		}
		else{
			/* 8 bit data */
			*(pSPIHandle -> RxBuffer) = pSPIHandle ->pSPIx -> DR ;
			pSPIHandle -> RxLen --;
			pSPIHandle -> RxBuffer++;
		}

	}
	if(! pSPIHandle -> RxLen){
		/* close transmission and make it available */
		SPI_CloseReception(pSPIHandle);

		/* call back */
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);

	}

}




/******************************************************************************
 * @fn			- SPI_OVR_InterruptHandle
 *
 * @brief		- Helper fn
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void SPI_OVR_InterruptHandle(SPI_Handle_t *pSPIHandle){

	SPI_ClearOVRFlag(pSPIHandle);

	/* Inform the application by call back fn */
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);


}



/******************************************************************************
 * 						Call back Fns
 ******************************************************************************/
__weak void SPI_ApplicationEventCallback (SPI_Handle_t *pSPIHandle, uint8_t AppEv){

	/* This is a weak function. Application may override it */
}
