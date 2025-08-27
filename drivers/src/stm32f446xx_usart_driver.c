/*
 * stm32f446xx_USART_driver.c
 *
 *  Created on: Aug 15, 2025
 *      Author: csmla
 */


#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_rcc_driver.h"
#include <string.h>





static void USART_HandleRXNEInterrupt (USART_Handle_t *pUSARTHandle);
static void USART_HandleTXEInterrupt (USART_Handle_t *pUSARTHandle);








/******************************************************************************
 * @fn			- USART_PeriClockControl
 *
 * @brief		- This fn Enables or Disables peripheral clock for the USART
 *
 * @param[in]	- USART port base address
 * @param[in]	- ENABLE or DISABLE Macros
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){

	if (EnorDi == ENABLE){
		if(pUSARTx == USART1) USART1_PCLK_EN();
		else if (pUSARTx == USART2) USART2_PCLK_EN();
		else if (pUSARTx == USART3) USART3_PCLK_EN();
		else if (pUSARTx == UART4) UART4_PCLK_EN();
		else if (pUSARTx == UART5) UART5_PCLK_EN();
		else if (pUSARTx == USART6) USART6_PCLK_EN();

	}
	else {
		if(pUSARTx == USART1) USART1_PCLK_EN();
		else if (pUSARTx == USART2) USART2_PCLK_DI();
		else if (pUSARTx == USART3) USART3_PCLK_DI();
		else if (pUSARTx == UART4) UART4_PCLK_DI();
		else if (pUSARTx == UART5) UART5_PCLK_DI();
		else if (pUSARTx == USART6) USART6_PCLK_DI();

	}

}





/******************************************************************************
 * @fn			- USART_Init
 *
 * @brief		-
 *
 * @param[in]	- USART HAndle  base address
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void USART_Init(USART_Handle_t *pUSARTHandle){

	uint32_t tempreg = 0;

	/*Enable Peripheral clock */
	USART_PeriClockControl(pUSARTHandle -> pUSARTx, ENABLE);

	/******************************** Configuration of CR1******************************************/

	/* Enable USART Tx and Rx engines according to the USART_Mode configuration item */
	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		/* enable the Receiver bit field*/
		tempreg |= (1 << USART_CR1_RE);

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		/* enable the Transmitter bit field */
		tempreg |= ( 1 << USART_CR1_TE );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		/* enable the both Transmitter and Receiver bit fields */
		tempreg |= ( ( 1 << USART_CR1_TE) | ( 1 << USART_CR1_RE) );
	}

	/* Word length configuration item */
	tempreg |= pUSARTHandle->USART_Config.USART_DataSize << USART_CR1_M ;


	/*Configuration of parity control bit fields */
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		/* enale the parity control */
		tempreg |= ( 1 << USART_CR1_PCE);

		/*enable EVEN parity */
		/* Not required because by default EVEN parity will be selected once you enable the parity control */

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		/* enable the parity control */
		tempreg |= ( 1 << USART_CR1_PCE);

		/* enable ODD parity */
		tempreg |= ( 1 << USART_CR1_PS);

	}

   /*Program the CR1 register */
	pUSARTHandle->pUSARTx->CR1 |= tempreg;


	/******************************** Configuration of CR2******************************************/

	tempreg=0;

	/* configure the number of stop bits inserted during USART frame transmission */
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopbits << USART_CR2_STOP1_0;

	/* Program the CR2 register */
	pUSARTHandle->pUSARTx->CR2 |= tempreg;

	/******************************** Configuration of CR3******************************************/

	tempreg=0;

	/* Configuration of USART hardware flow control */
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		/* enable CTS flow control */
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		/* enable RTS flow control */
		tempreg |= (1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		/* enable both CTS and RTS Flow control */
		tempreg |= ( 1 << USART_CR3_CTSE);
		tempreg |= (1 << USART_CR3_RTSE);

	}

	/* program the CR3 reg */
	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/******************************** Configuration of BRR(Baudrate register)******************************************/

	/*code to configure the baud rate */

	USART_SetBaudRate(pUSARTHandle ->pUSARTx, pUSARTHandle->USART_Config.USART_BaudRate);

}





/******************************************************************************
 * @fn			- USART_DeInit
 *
 * @brief		-
 *
 * @param[in]	- USART HAndle  base address
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void USART_DeInit(USART_RegDef_t *pUSARTx){

	if(pUSARTx == USART1) USART1_REG_RESET();
	else if (pUSARTx == USART2) USART2_REG_RESET();
	else if (pUSARTx == USART3) USART3_REG_RESET();
	else if (pUSARTx == UART4) UART4_REG_RESET();
	else if (pUSARTx == UART5) UART5_REG_RESET();
	else if (pUSARTx == USART6) USART6_REG_RESET();
}

/******************************************************************************
 * @fn			- USART_GetFlagStatus
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
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t flagName){

	if(pUSARTx ->SR & flagName)return FLAG_SET;
	return FLAG_RESET;

}


/******************************************************************************
 * @fn			- USART_ClearFlagStatus
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-FLAG_RESET macros
 *
 * @note		-
 */
void USART_ClearFlagStatus(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName){

	pUSARTx ->SR &= ~StatusFlagName;


}


/******************************************************************************
 * @fn			- USART_PeripheralControl
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
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		pUSARTx -> CR1 |= (1 << USART_CR1_UE);
	}
	else{
		pUSARTx -> CR1 &= ~(1 << USART_CR1_UE);
	}

}





/******************************************************************************
 * @fn			- USART_EnableDisableInterrupt
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
void USART_EnableDisableInterrupt(USART_RegDef_t *pUSARTx, uint8_t EnorDi){



}





/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Resolve all the TODOs

 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	/* to hold peripheral clock */
	uint32_t usart_clkfreq;

	/* variables to hold mantissa and fraction */
	uint32_t  usartdiv, usartdiv_mpart, usartdiv_fpart;

	uint32_t temp = 0;

	if( (pUSARTx == USART1) || (pUSARTx == USART6)){

		usart_clkfreq = RCC_GetPCLK2Value();
	}
	else{

		usart_clkfreq = RCC_GetPCLK1Value();

	}

	if(pUSARTx ->CR1 & (1 << USART_CR1_OVER8)){

		/* over8 = 1 */
		/* usartdiv = (pclk/(8*(2-OVER8)*Baudrate) * 100 = 12.5*pclk/1* Baudrate  => 25 *pclk/ 2* Baudrate */

		usartdiv = 25 * usart_clkfreq /(2 * BaudRate);
		/* calculate mantissa */
		usartdiv_mpart = usartdiv / 100 ;

		/* calculate fraction. add 50 to round it up */
		usartdiv_fpart = ((usartdiv - (usartdiv_mpart * 100)) * 8) + 50;
		usartdiv_fpart /= 100;

		if(usartdiv_fpart > 7){
			usartdiv_mpart += 1;
		}
		usartdiv_fpart &= (uint8_t)(0x07);
	}
	else {

		/* over8 = 0 */
		/* usartdiv = (pclk/(8*(2-OVER8)*Baudrate) * 100 = 12.5*pclk/2* Baudrate  => 25 *pclk/ 4* Baudrate */

		usartdiv = 25 * usart_clkfreq /(4 * BaudRate);

		/* calculate mantissa */
		usartdiv_mpart = usartdiv / 100 ;

		/* calculate fraction. add 50 to round it up */
		usartdiv_fpart = ((usartdiv - (usartdiv_mpart * 100)) * 16) + 50;
		usartdiv_fpart /= 100;

		if(usartdiv_fpart > 15){
			usartdiv_mpart += 1;
		}
		usartdiv_fpart &= (uint8_t)(0x0F);

	}


	/* program the baud rate usartdiv in BRR */
	temp = (usartdiv_mpart << 4);
	temp |= usartdiv_fpart;

	pUSARTx ->BRR |= temp;


}




/*****************************************************************************************
 * 				Send and Receive data
 ***********************************************************************************************/


/******************************************************************************
 * @fn			- USART_CloseSendData
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
void USART_CloseSendData(USART_Handle_t *pUSARTHandle){



}





/******************************************************************************
 * @fn			- USART_CloseReceiveData
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
void USART_CloseReceiveData(USART_Handle_t *pUSARTHandle){


}




/******************************************************************************
 * @fn			- USART_SendData
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
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){
	uint16_t *pdata;

	/*Loop over until "Len" number of bytes are transferred */
	for(uint32_t i = 0 ; i < Len; i++)
	{
		/* wait until TXE flag is set in the SR */
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

		 /*  USART_WordLength item for 9BIT or 8BIT in a frame */
		if(pUSARTHandle->USART_Config.USART_DataSize == USART_DATA_9BITS)
		{
			/* if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits */
			pdata = (uint16_t*) pTxBuffer;
			*pdata = (*pdata & (uint16_t)0x01FF);

			/* check for USART_ParityControl */
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				/* No parity is used in this transfer. so, 9bits of user data will be sent */
				/* increment pTxBuffer twice */
				pUSARTHandle->pUSARTx->DR = *pdata;
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				/* Parity bit is used in this transfer . so , 8bits of user data will be sent */
				/* The 9th bit will be replaced by parity bit by the hardware */
				pUSARTHandle->pUSARTx->DR = *pTxBuffer;
				pTxBuffer++;
			}
		}
		else
		{
			/* This is 8bit data transfer */
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			/* code to increment the buffer address */
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}






/******************************************************************************
 * @fn			- USART_ReceiveData
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
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len ){

	/* Loop over until "Len" number of bytes are transferred */
	for(uint32_t i = 0 ; i < Len; i++){
		/* code to wait until RXNE flag is set in the SR */
		while (!(USART_GetFlagStatus(pUSARTHandle ->pUSARTx, USART_FLAG_RXNE)));

		/* Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit */
		if(pUSARTHandle->USART_Config.USART_DataSize == USART_DATA_9BITS)
		{
			/*We are going to receive 9bit data in a frame*/

			/* check are we using USART_ParityControl control or not*/
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				/* No parity is used. so, all 9bits will be of user data */

				/* read only first 9 bits. so, mask the DR with 0x01FF */
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x1FF);

				/* increment the pRxBuffer */
				pRxBuffer ++;
				pRxBuffer ++;

			}
			else
			{
				/* Parity is used, so, 8bits will be of user data and 1 bit is parity */
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				 /* Increment the pRxBuffer */
				pRxBuffer ++;
			}
		}
		else
		{
			/* We are going to receive 8bit data in a frame */

			/* check are we using USART_ParityControl control or not */
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				/* No parity is used , so all 8bits will be of user data */

				/* read 8 bits from DR */
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}

			else
			{
				/* Parity is used, so , 7 bits will be of user data and 1 bit is parity */

				/* read only 7 bits , hence mask the DR with 0X7F */
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

			}

			/* increment the pRxBuffer */
			pRxBuffer++;
		}

	}



}








/******************************************************************************
 * @fn			- USART_SendDataIT
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- returns the api state : USART_STATUS_BUSY_TX /USART_STATUS_BUSY_RX /USART_STATUS_READY
 *
 * @note		-
 */
uint8_t  USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){

	uint8_t busystate = pUSARTHandle -> TxState;

	if(pUSARTHandle ->TxState != USART_STATUS_BUSY_TX) {

		pUSARTHandle ->TxState = USART_STATUS_BUSY_TX;
		pUSARTHandle ->TxBuffer = pTxBuffer;
		pUSARTHandle ->TxLen = Len;

		/* Enable TX interrupt */
		pUSARTHandle ->pUSARTx ->CR1 |= (1 << USART_CR1_TXEIE);

		/* Enable TCE */
		pUSARTHandle ->pUSARTx ->CR1 |= (1 << USART_CR1_TCIE);


	}

	return busystate;
}




/******************************************************************************
 * @fn			- USART_ReceiveDataIT
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-returns the api state : USART_STATUS_BUSY_TX /USART_STATUS_BUSY_RX /USART_STATUS_READY
 *
 * @note		-
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){

	uint8_t busystate = pUSARTHandle -> RxState;

	if(pUSARTHandle ->RxState != USART_STATUS_BUSY_RX) {

			pUSARTHandle ->RxState = USART_STATUS_BUSY_RX;
			pUSARTHandle ->RxBuffer = pRxBuffer;
			pUSARTHandle ->RxLen = Len;

			/* Enable RX interrupt */
			pUSARTHandle ->pUSARTx ->CR1 |= (1 << USART_CR1_RXNEIE);

		}

	return busystate;
}











/***********************************************************************************************
 * 				Helper Fns
 ****************************************************************************************/



/*****************************************************************
 * @fn          - USART_HandleRXNEInterrupt
 *
 * @brief       -
 *
 * @param[in]   - Base address of the USART Handle
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void USART_HandleRXNEInterrupt (USART_Handle_t *pUSARTHandle){

	if(pUSARTHandle->RxState == USART_STATUS_BUSY_RX)
	{
		/*RX is set so receive data */
		if(pUSARTHandle->RxLen > 0)
		{
			/*Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit */
			if(pUSARTHandle->USART_Config.USART_DataSize == USART_DATA_9BITS)
			{
				/*We are going to receive 9bit data in a frame */

				/*Now, check are we using USART_ParityControl control or not */
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					/*No parity is used. so, all 9bits will be of user data */

					/*read only first 9 bits so mask the DR with 0x01FF */
					*((uint16_t*) pUSARTHandle ->RxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

					/*Now increment the pRxBuffer two times */
					pUSARTHandle ->RxBuffer++;
					pUSARTHandle ->RxBuffer++;

					/* Implement the code to decrement the length */
					pUSARTHandle ->RxLen --;
					pUSARTHandle ->RxLen --;
				}
				else
				{
					/* Parity is used. so, 8bits will be of user data and 1 bit is parity */
					 *pUSARTHandle ->RxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

					 /* Now increment the pRxBuffer */
					 pUSARTHandle->RxBuffer++;

					 /* Implement the code to decrement the length */
					 pUSARTHandle ->RxLen --;
				}
			}
			else
			{
				/*We are going to receive 8bit data in a frame */

				/*Now, check are we using USART_ParityControl control or not */
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					/*No parity is used , so all 8bits will be of user data */

					/* read 8 bits from DR */
					 *pUSARTHandle ->RxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				}

				else
				{
					/* Parity is used, so , 7 bits will be of user data and 1 bit is parity */

					/* read only 7 bits , hence mask the DR with 0X7F */
					 *pUSARTHandle -> RxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

				}

				/* Now , increment the pRxBuffer */
				pUSARTHandle ->RxBuffer++;

				/* decrement the length */
				pUSARTHandle ->RxLen --;
			}


		}//if of >0

		if(! pUSARTHandle->RxLen)
		{
			/* disable the rxne */
			uint8_t dummy;
			pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
			pUSARTHandle->RxState = USART_STATUS_READY;
			pUSARTHandle ->RxBuffer = NULL;
			pUSARTHandle ->RxLen = 0;
			dummy = pUSARTHandle ->pUSARTx ->SR;
			dummy = pUSARTHandle ->pUSARTx ->DR;
			USART_ApplicationEventCallback(pUSARTHandle,USART_EV_RX_CMPLT);
			(void) dummy;
		}
	}



}



/*****************************************************************
 * @fn          - USART_Handle TXE Interrupt
 *
 * @brief       -
 *
 * @param[in]   - Base address of the USART Handle
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void USART_HandleTXEInterrupt (USART_Handle_t *pUSARTHandle){

	uint16_t *pdata;

	if(pUSARTHandle->TxState == USART_STATUS_BUSY_TX)
	{
		/*Keep sending data until Txlen reaches to zero */
		if(pUSARTHandle->TxLen > 0)
		{
			/*Check the USART_WordLength item for 9BIT or 8BIT in a frame */
			if(pUSARTHandle->USART_Config.USART_DataSize == USART_DATA_9BITS)
			{
				/*if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits */
				pdata = (uint16_t*) pUSARTHandle ->TxBuffer;

				/* check for USART_ParityControl */
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					/*loading only first 9 bits , so we have to mask with the value 0x01FF */
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					/* No parity is used in this transfer , so, 9bits of user data will be sent */
					/*Implement the code to increment pTxBuffer twice */
					pUSARTHandle ->TxBuffer++;
					pUSARTHandle ->TxBuffer++;

					/*Implement the code to decrement the length */
					pUSARTHandle ->TxLen --;
					pUSARTHandle ->TxLen --;

				}
				else
				{
					/* Parity bit is used in this transfer . so , 8bits of user data will be sent */
					/* The 9th bit will be replaced by parity bit by the hardware */
					pUSARTHandle ->pUSARTx ->DR = * pUSARTHandle ->TxBuffer;
					pUSARTHandle ->TxBuffer++;

					/* decrement the length */
					pUSARTHandle ->TxLen --;
				}
			}
			else
			{
				/*This is 8bit data transfer */
				pUSARTHandle->pUSARTx->DR = (*pUSARTHandle ->TxBuffer  & (uint8_t)0xFF);

				/* Increment the buffer address */
				pUSARTHandle ->TxBuffer++;

				/* decrement the length */
				pUSARTHandle ->TxLen --;
			}

		}
		if (pUSARTHandle->TxLen == 0 )
		{
			/* TxLen is zero */
			/* clear the TXEIE bit (disable interrupt for TXE flag ) */
			pUSARTHandle ->pUSARTx ->CR1 &= ~(1 << USART_CR1_TXEIE);

		}
	}



}








/***************************************************************************************
 * 					Interrupt Handling
 *******************************************************************************************/

/******************************************************************************
 * @fn			- USART_IRQInterruptConfig
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
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

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
 * @fn			- USART_IRQPriorityConfig
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

	uint8_t iprx = IRQNumber / 4;  			/* ipr reg number offset address */
	uint8_t iprx_section = IRQNumber % 4;    /* position in iprx reg */

	uint8_t shift_position = (8 * iprx_section) + (8 - NO_IPR_BITS_IMPLEMENTED);  //only upper 4 bits are implemented in IPR
	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << shift_position);


}




/******************************************************************************
 * @fn			- USART_IRQHandling
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		- Interrupt handling for different USART events (refer SR1)
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle){

	uint32_t temp1 , temp2, temp3;

/*************************Check for TC flag ********************************************/

	/* check the state of TC bit in the SR */
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);

	 /*check the state of TCEIE bit */
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{
		/* this interrupt is because of TC */

		/* close transmission and call application callback if TxLen is zero */
		if ( pUSARTHandle->TxState == USART_STATUS_BUSY_TX)
		{
			/* Check the TxLen . If it is zero then close the data transmission */
			if(! pUSARTHandle->TxLen )
			{
				/* clear the TC flag */
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);

				/* clear the TCIE control bit */
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TCIE);

				/*Reset the application state */
				pUSARTHandle->TxState = USART_STATUS_READY;

				/*Reset Buffer address to NULL */
				pUSARTHandle ->TxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle ->TxLen = 0;

				//Call the applicaton call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EV_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	/*check the state of TXE bit in the SR */
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);

	/*check the state of TXEIE bit in CR1 */
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);


	if(temp1 && temp2 )
	{
		/* this interrupt is because of TXE */
		USART_HandleTXEInterrupt(pUSARTHandle);


	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if(temp1 && temp2 )
	{
		/*this interrupt is because of rxne */
		USART_HandleRXNEInterrupt(pUSARTHandle);

	}


/*************************Check for CTS flag ********************************************/
/* Note : CTS feature is not applicable for UART4 and UART5 */

	/* check the status of CTS bit in the SR */
	temp1 = pUSARTHandle ->pUSARTx ->SR & (1 << USART_SR_CTS);

	/* check the state of CTSE bit in CR1 */
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	/* check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.) */
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);
	(void) temp3;

	if(temp1  && temp2 )
	{
		/* clear the CTS flag in SR */
		pUSARTHandle ->pUSARTx ->SR &= ~(1 << USART_SR_CTS);

		/* this interrupt is because of cts */
		USART_ApplicationEventCallback(pUSARTHandle,USART_EV_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	/* check the status of IDLE flag bit in the SR */
	temp1 = pUSARTHandle ->pUSARTx ->SR & (1 << USART_SR_IDLE);

	/* check the state of IDLEIE bit in CR1 */
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR1_IDLEIE);


	if(temp1 && temp2)
	{
		/* clear the IDLE flag. read SR followed by read DR */
		/* dummy read */
		temp1 = pUSARTHandle ->pUSARTx ->SR;
		temp1 = pUSARTHandle ->pUSARTx ->DR;

		/* this interrupt is because of idle */
		USART_ApplicationEventCallback(pUSARTHandle,USART_EV_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	/* check the status of ORE flag  in the SR */
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	/* check the status of RXNEIE  bit in the CR1 */
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		/*Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .*/

		/* this interrupt is because of Overrun error */
		USART_ApplicationEventCallback(pUSARTHandle,USART_EV_ERR_ORE);
	}



/*************************Check for Error Flag ********************************************/

/* Noise Flag, Overrun error and Framing Error in multibuffer communication */
//The blow code will get executed in only if multibuffer mode is used. */

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_EV_ERR_FE);
		}

		if(temp1 & ( 1 << USART_SR_NF) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_EV_ERR_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_EV_ERR_ORE);
		}
	}


}






/******************************************************************************
 * 						Call back Fns
 ******************************************************************************/
__weak void USART_ApplicationEventCallback (USART_Handle_t *pUSARTHandle, uint8_t AppEv) {

	/* This is a weak function. Application may override it */
}
