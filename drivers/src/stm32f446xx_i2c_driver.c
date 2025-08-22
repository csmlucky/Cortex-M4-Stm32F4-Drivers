/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Aug 15, 2025
 *      Author: csmla
 */


#include "stm32f446xx_i2c_driver.h"
#include <string.h>



static void I2C_GenerateStartCondition (I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase (I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t mode);
static void I2C_ClearADDRFlag (I2C_Handle_t *pI2CHandle);

static void I2C_MasterHandleRXNEInterrupt (I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt (I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleBTFInterrupt (I2C_Handle_t *pI2CHandle);


uint16_t ahb_PreScaler[10] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t abp1_PreScaler[4] = {2, 4, 6, 8};


/******************************************************************************
 * @fn			- RCC_GetPCLK1Value
 *
 * @brief		- (PCLK1 = system clk freq / AHB pre scaler) /APB1 pre scaler
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-uint32_t
 *
 * @note		-
 */
uint32_t  I2C_GetPCLK1Value (void){


	uint32_t  pclk1, systemClk;
	uint16_t ahbp;
	uint8_t clkSrc, temp, apb1p;

	clkSrc = (RCC -> CFGR >> 2) & 0x3;

	if(clkSrc == 0) systemClk = HSI_CLK_FREQ;
	else if(clkSrc == 1) systemClk = HSE_CLK_FREQ;

	temp = (RCC -> CFGR >> 4) & 0xF;

	if(temp < 8) ahbp = 1;
	else ahbp = ahb_PreScaler[temp - 8];


	temp = (RCC -> CFGR >> 10) & 0x7;

	if(temp < 4) apb1p = 1;
	else apb1p = abp1_PreScaler[temp - 4];

	pclk1 = (systemClk / ahbp /apb1p);

	return pclk1;


}







/******************************************************************************
 * @fn			- I2C_PeriClockControl
 *
 * @brief		- This fn Enables or Disables peripheral clock for the I2C
 *
 * @param[in]	- I2C port base address
 * @param[in]	- ENABLE or DISABLE Macros
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if (EnorDi == ENABLE){
		if(pI2Cx == I2C1) I2C1_PCLK_EN();
		else if(pI2Cx == I2C2) I2C2_PCLK_EN();
		else if (pI2Cx == I2C3) I2C3_PCLK_EN();
	}
	else {
		if(pI2Cx == I2C1) I2C1_PCLK_DI();
		else if(pI2Cx == I2C2) I2C2_PCLK_DI();
		else if (pI2Cx == I2C3) I2C3_PCLK_DI();

	}

}





/******************************************************************************
 * @fn			- I2C_Init
 *
 * @brief		-
 *
 * @param[in]	- I2C HAndle  base address
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void I2C_Init(I2C_Handle_t *pI2CHandle){

	uint32_t tempreg = 0;

	/*Enable Peripheral clock */
	I2C_PeriClockControl(pI2CHandle -> pI2Cx, ENABLE);

	/* configure ack control in CR1*/
	tempreg |= ((pI2CHandle -> I2C_Config.I2C_AckControl) << 10);
	pI2CHandle -> pI2Cx -> CR1 |= (tempreg & 0xFFFF);

	/* configure FREQ field in CR2 */
	tempreg = 0;
	tempreg |= (I2C_GetPCLK1Value() / 1000000U);
	pI2CHandle -> pI2Cx -> CR2 |= (tempreg & 0x3F);

	/* program the device own address */
	tempreg = 0;
	tempreg |= (pI2CHandle -> I2C_Config.I2C_DeviceAddress << 1);
	/* I2C_OAR1 14 bit always should be kept at 1 as per reference manual */
	tempreg |= (1 << 14);
	pI2CHandle -> pI2Cx -> OAR1 |= tempreg;

	/*CCR calculation */
	uint16_t ccrValue = 0;
	tempreg = 0;

	if(pI2CHandle -> I2C_Config.I2C_SCLSpeed <= I2C_SCLK_SPEED_SM){
		/* standard freq mode*/
		ccrValue = (I2C_GetPCLK1Value() / (2 * pI2CHandle -> I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccrValue & 0xFFF);
	}
	else{
		/* Fast freq mode */

		/* set F/S bit in CCR to enable fast mode*/
		tempreg |= (1 << 15);
		/* configure duty cycle */
		tempreg |= (pI2CHandle ->I2C_Config.I2C_FMDutyCycle << 14);

		/* ccr calc */
		if(pI2CHandle ->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccrValue = (I2C_GetPCLK1Value() / (3 * pI2CHandle -> I2C_Config.I2C_SCLSpeed));
		}
		else{
			ccrValue = (I2C_GetPCLK1Value() / (25 * pI2CHandle -> I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccrValue & 0xFFF);
	}

	pI2CHandle -> pI2Cx -> CCR |= tempreg;

	/* configure TRISE reg */
	if(pI2CHandle -> I2C_Config.I2C_SCLSpeed <= I2C_SCLK_SPEED_SM){
		/* standard mode */
		tempreg = (I2C_GetPCLK1Value() / 1000000U) + 1;
	}
	else{
		/* Fast mode */
		tempreg = (I2C_GetPCLK1Value() * 300/ 1000000000U) + 1;
	}
	pI2CHandle -> pI2Cx ->TRISE = (tempreg & 0x3F);
}





/******************************************************************************
 * @fn			- I2C_DeInit
 *
 * @brief		-
 *
 * @param[in]	- I2C HAndle  base address
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx){

	if(pI2Cx == I2C1) I2C1_REG_RESET();
	else if (pI2Cx == I2C2) I2C2_REG_RESET();
	else if (pI2Cx == I2C3) I2C3_REG_RESET();

}

/******************************************************************************
 * @fn			- I2C_GetFlagStatus
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
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t flagName){

	if(pI2Cx ->SR1 & flagName)return FLAG_SET;
	return FLAG_RESET;

}



/******************************************************************************
 * @fn			- I2C_PeripheralControl
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
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		pI2Cx -> CR1 |= (1 << I2C_CR1_PE);
	}
	else{
		pI2Cx -> CR1 &= ~(1 << I2C_CR1_PE);
	}

}


/******************************************************************************
 * @fn			- I2C_ManageAcking
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
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi == I2C_ACK_ENABLE) pI2Cx -> CR1 |= (1 << 10);
	else pI2Cx -> CR1 &= ~(1 << 10);
}



/*****************************************************************
 * @fn          - I2C_GenerateStopCondition
 *
 * @brief       - Generate stop condition for I2C
 *
 * @param[in]   - Base address of the I2C peripheral
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx-> CR1 |= (1 << I2C_CR1_STOP);
}



/******************************************************************************
 * @fn			- I2C_SlaveEnableDisableInterrupt
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
void I2C_SlaveEnableDisableInterrupt(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		/* enable ITBUFEN Control Bit */
		pI2Cx -> CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		/* enable ITEVFEN Control Bit */
		pI2Cx -> CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		/* ITERREN Control Bit */
		pI2Cx -> CR2 |= ( 1 << I2C_CR2_ITERREN);

	}
	else{
		/* disable ITBUFEN Control Bit */
		pI2Cx -> CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

		/* disable ITEVFEN Control Bit */
		pI2Cx -> CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

		/* disable ITERREN Control Bit */
		pI2Cx -> CR2 &= ~( 1 << I2C_CR2_ITERREN);

	}

}






/*****************************************************************************************
 * 				Send and Receive data
 ***********************************************************************************************/


/******************************************************************************
 * @fn			- I2C_CloseSendData
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
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle){

	/* disable ITBUFEN Control Bit */
	pI2CHandle -> pI2Cx -> CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	/* disable ITEVFEN Control Bit */
	pI2CHandle -> pI2Cx -> CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	/*resets all members in handle structure */
	pI2CHandle ->TxLen = 0;
	pI2CHandle ->TxBuffer = NULL;
	pI2CHandle ->TxRxState = I2C_STATUS_READY;

	if(pI2CHandle ->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE){
		I2C_ManageAcking(pI2CHandle ->pI2Cx, ENABLE);
	}

}





/******************************************************************************
 * @fn			- I2C_CloseReceiveData
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
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle){

	/* disable ITBUFEN Control Bit */
	pI2CHandle -> pI2Cx -> CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	/* disable ITEVFEN Control Bit */
	pI2CHandle -> pI2Cx -> CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	/*resets all members in handle structure */
	pI2CHandle ->RxLen = 0;
	pI2CHandle ->RxSize = 0;
	pI2CHandle ->RxBuffer = NULL;
	pI2CHandle ->TxRxState = I2C_STATUS_READY;

	if(pI2CHandle ->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE){
		I2C_ManageAcking(pI2CHandle ->pI2Cx, ENABLE);
	}

}




/******************************************************************************
 * @fn			- I2C_MasterSendData
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
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){

	/* Generate the start condition */
	I2C_GenerateStartCondition(pI2CHandle ->pI2Cx);

	/*Confirm the start generation is completed by using the SB flag in SR1 reg */
	/* Note: until SB flag is cleared SCL will be stretched (pulled low) */
	while(I2C_GetFlagStatus(pI2CHandle ->pI2Cx, I2C_FLAG_SB) == RESET);

	/* send the slave address with r(1) / w(0) bit as lsb */
	I2C_ExecuteAddressPhase(pI2CHandle ->pI2Cx, SlaveAddr, I2C_MASTER_WR);

	/* Confirm sending address is completed by checking the ADDR bit in SR1 reg */
	while(I2C_GetFlagStatus(pI2CHandle ->pI2Cx, I2C_FLAG_ADDR) == RESET);

	/* clear ADDR bit Note: until ADDR flag is cleared SCL will be stretched (pulled low)*/
	I2C_ClearADDRFlag(pI2CHandle);

	/* Send the Data until Len becomes 0 */
	while (Len > 0){

		/*wait until TXE bit is set*/
		while(I2C_GetFlagStatus(pI2CHandle ->pI2Cx, I2C_FLAG_TXE) == RESET);
		pI2CHandle -> pI2Cx -> DR = *pTxBuffer;
		Len --;
		pTxBuffer ++;
	}

	/* Waiting for TXE=1 and BTF=1 before generating STOP condition */
	while(I2C_GetFlagStatus(pI2CHandle ->pI2Cx, I2C_FLAG_TXE) == RESET);
	while(I2C_GetFlagStatus(pI2CHandle ->pI2Cx, I2C_FLAG_BTF) == RESET);

 /* Generate STOP condition and master not need to wait for the completion of stop condition */
	if (Sr == I2C_DISABLE_SR){
		I2C_GenerateStopCondition(pI2CHandle ->pI2Cx);
	}
}



/******************************************************************************
 * @fn			- I2C_MasterReceiveData
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
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr ){

	/* Generate the start condition */
	I2C_GenerateStartCondition(pI2CHandle ->pI2Cx);

	/*Confirm the start generation is completed by using the SB flag in SR1 reg */
	/* Note: until SB flag is cleared SCL will be stretched (pulled low) */
	while(I2C_GetFlagStatus(pI2CHandle ->pI2Cx, I2C_FLAG_SB) == RESET);

	/* send the slave address with r(1) / w(0) bit as lsb */
	I2C_ExecuteAddressPhase(pI2CHandle ->pI2Cx, SlaveAddr, I2C_MASTER_RD);

	/* Confirm sending address is completed by checking the ADDR bit in SR1 reg */
	while(I2C_GetFlagStatus(pI2CHandle ->pI2Cx, I2C_FLAG_ADDR) == RESET);

	if(Len == 1 ){

		/* Disable ACk bit in CR1 before clears ADDR flag */
		I2C_ManageAcking(pI2CHandle ->pI2Cx, I2C_ACK_DISABLE);

		/* clear ADDR bit Note: until ADDR flag is cleared SCL will be stretched (pulled low)*/
		I2C_ClearADDRFlag(pI2CHandle);

		/*wait until RXNE flag is set */
		while(I2C_GetFlagStatus(pI2CHandle ->pI2Cx, I2C_FLAG_RXNE) == RESET);

		/* Generate STOP condition and master not need to wait for the completion of stop condition */
		if (Sr == I2C_DISABLE_SR){
			I2C_GenerateStopCondition(pI2CHandle ->pI2Cx);
		}
		/*  Read data into buffer */
		*pRxBuffer = pI2CHandle -> pI2Cx -> DR;


	}
	else if (Len > 1){

		/* clear ADDR bit Note: until ADDR flag is cleared SCL will be stretched (pulled low)*/
		I2C_ClearADDRFlag(pI2CHandle);

		/* read data until the len becomes 0 */
		for(uint16_t i = Len; i > 0; i--){

			/*wait until RXNE flag is set */
			while(I2C_GetFlagStatus(pI2CHandle ->pI2Cx, I2C_FLAG_RXNE) == RESET);

			if( i == 2){

				/* Disable ACk bit in CR1 */
				I2C_ManageAcking(pI2CHandle ->pI2Cx, I2C_ACK_DISABLE);

			}
			if( i == 1){

				/* Generate STOP condition and master not need to wait for the completion of stop condition */
				if (Sr == I2C_DISABLE_SR){
					I2C_GenerateStopCondition(pI2CHandle ->pI2Cx);
				}

			}

			/*  Read data into buffer */
			*pRxBuffer = pI2CHandle -> pI2Cx -> DR;

			/* Increment the buffer address */
			pRxBuffer ++;

		}
	}

	/* Re enable Acking */
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE){

		I2C_ManageAcking(pI2CHandle ->pI2Cx, I2C_ACK_ENABLE);
	}


}



/******************************************************************************
 * @fn			- I2C_SlaveReceiveData
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
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx){

	return pI2Cx ->DR;


}




/******************************************************************************
 * @fn			- I2C_SlaveSendData
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
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data){

	/* load data into the DR */
	pI2Cx -> DR = data;


}






/******************************************************************************
 * @fn			- I2C_MasterSendDataIT
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- returns the api state : I2C_STATUS_BUSY_TX /I2C_STATUS_BUSY_RX /I2C_STATUS_READY
 *
 * @note		-
 */
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){

	uint8_t busystate = pI2CHandle -> TxRxState;

		if( (busystate != I2C_STATUS_BUSY_TX) && (busystate != I2C_STATUS_BUSY_RX))
		{
			pI2CHandle -> TxBuffer = pTxBuffer;
			pI2CHandle -> TxLen = Len;
			pI2CHandle -> TxRxState = I2C_STATUS_BUSY_TX;
			pI2CHandle -> DevAddr = SlaveAddr;
			pI2CHandle -> Sr = Sr;

			/* Generate START Condition */
			I2C_GenerateStartCondition(pI2CHandle ->pI2Cx);

			/* enable ITBUFEN Control Bit */
			pI2CHandle -> pI2Cx -> CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			/* enable ITEVFEN Control Bit */
			pI2CHandle -> pI2Cx -> CR2 |= ( 1 << I2C_CR2_ITEVTEN);

			/* ITERREN Control Bit */
			pI2CHandle -> pI2Cx -> CR2 |= ( 1 << I2C_CR2_ITERREN);

		}

		return busystate;
}




/******************************************************************************
 * @fn			- I2C_MasterReceiveDataIT
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-returns the api state : I2C_STATUS_BUSY_TX /I2C_STATUS_BUSY_RX /I2C_STATUS_READY
 *
 * @note		-
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr ){

	uint8_t busystate = pI2CHandle -> TxRxState;

		if( (busystate != I2C_STATUS_BUSY_TX) && (busystate != I2C_STATUS_BUSY_RX))
		{
			pI2CHandle->RxBuffer = pRxBuffer;
			pI2CHandle->RxLen = Len;
			pI2CHandle->TxRxState = I2C_STATUS_BUSY_RX;
			pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			/*  Generate START Condition */
			I2C_GenerateStartCondition(pI2CHandle ->pI2Cx);

			/*  enable ITBUFEN Control Bit */
			pI2CHandle -> pI2Cx -> CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			/* enable ITEVFEN Control Bit */
			pI2CHandle -> pI2Cx -> CR2 |= ( 1 << I2C_CR2_ITEVTEN);

			/* enable ITERREN Control Bit */
			pI2CHandle -> pI2Cx -> CR2 |= ( 1 << I2C_CR2_ITERREN);

		}

		return busystate;
}











/***********************************************************************************************
 * 				Helper Fns
 ****************************************************************************************/
/******************************************************************************
* @fn			- I2C_GenerateStartCondition
*
* @brief		-This Helper Fn generates the start condition
*
* @param[in]	- I2C peripheral base address
* @param[in]	-
* @param[in]	-
*
* @return		-
*
* @note		-
*/
static void I2C_GenerateStartCondition (I2C_RegDef_t *pI2Cx){

	/* set START bit in CR1 */
	pI2Cx -> CR1 |= (1 << I2C_CR1_START);

}


/******************************************************************************
* @fn			- I2C_ExecuteAddressPhase
*
* @brief		-This Helper Fn sends the address
*
* @param[in]	- I2C peripheral base address
* @param[in]	- slave address
* @param[in]	-
*
* @return		- none
*
* @note		-
*/
static void I2C_ExecuteAddressPhase (I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t mode){

	SlaveAddr <<= 1;

	if( mode == I2C_MASTER_WR){
		/*  write mode */
		SlaveAddr &= ~( 1 );
	}
	else if ( mode == I2C_MASTER_RD){
		/*  read mode */
		SlaveAddr |= ( 1 );
	}
	pI2Cx -> DR = SlaveAddr;

}

/******************************************************************************
* @fn			- I2C_ClearADDRFlag
*
* @brief		-This Helper Fn clears ADDR bit
*
* @param[in]	- I2C peripheral base address
* @param[in]	-
* @param[in]	-
*
* @return		- none
*
* @note		-
*/
static void I2C_ClearADDRFlag (I2C_Handle_t *pI2CHandle){

	/* check for device mode */
	if(pI2CHandle -> pI2Cx ->SR2 & (1 << I2C_SR2_MSL)){
		/* device in master mode */
		if(pI2CHandle ->TxRxState == I2C_STATUS_BUSY_RX){
			/* receives 1 byte data */
			if(pI2CHandle ->RxSize == 1){
				/* disable acking  before clears ADDR*/
				I2C_ManageAcking(pI2CHandle ->pI2Cx, DISABLE);
			}
		}

	}
	else{
		/* device in slave mode */
		;
	}

	/* ADDR bit is cleared by reading SR1 followed by SR2 */
	uint32_t dummyread;
	dummyread = pI2CHandle -> pI2Cx -> SR1;
	dummyread = pI2CHandle -> pI2Cx -> SR2;
	(void)dummyread;

}




/*****************************************************************
 * @fn          - I2C_MasterHandleRXNEInterrupt
 *
 * @brief       -
 *
 * @param[in]   - Base address of the I2C Handle
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void I2C_MasterHandleRXNEInterrupt (I2C_Handle_t *pI2CHandle){

	if(pI2CHandle -> RxSize == 1){
		*(pI2CHandle ->RxBuffer) = pI2CHandle -> pI2Cx -> DR;
		pI2CHandle ->RxLen --;
	}
	if(pI2CHandle -> RxSize > 1){
		if(pI2CHandle ->RxLen == 2){
			/* disable acking */
			I2C_ManageAcking(pI2CHandle ->pI2Cx, DISABLE);
		}
		*(pI2CHandle ->RxBuffer) = pI2CHandle -> pI2Cx -> DR;
		pI2CHandle ->RxLen --;
		pI2CHandle ->RxBuffer ++;

	}
	if(pI2CHandle -> RxLen == 0){
		/* generate stop condition if sr disabled */
		if (pI2CHandle -> Sr == I2C_DISABLE_SR ){
			I2C_GenerateStartCondition(pI2CHandle ->pI2Cx);
		}
		/*resets all members of handle */
		I2C_CloseReceiveData(pI2CHandle);

		/* notify the application about rx complete */
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);

	}


}



/*****************************************************************
 * @fn          - I2C_MasterHandle TXE Interrupt
 *
 * @brief       -
 *
 * @param[in]   - Base address of the I2C Handle
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void I2C_MasterHandleTXEInterrupt (I2C_Handle_t *pI2CHandle){

	if(pI2CHandle ->TxRxState == I2C_STATUS_BUSY_TX){
		if(pI2CHandle ->TxLen > 0){
			/* load data */
			pI2CHandle -> pI2Cx ->DR = *(pI2CHandle ->TxBuffer);
			pI2CHandle ->TxLen --;
			pI2CHandle ->TxBuffer ++;
		}
	}

}






/*****************************************************************
 * @fn          - I2C_MasterHandle BTF Interrupt
 *
 * @brief       -
 *
 * @param[in]   - Base address of the I2C Handle
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void I2C_MasterHandleBTFInterrupt (I2C_Handle_t *pI2CHandle){

	if(pI2CHandle ->TxRxState == I2C_STATUS_BUSY_TX){
		/* chk for TXE flag */
		if (pI2CHandle -> pI2Cx -> SR1 & (1 << I2C_SR1_TXE)){
			/* both BTF AND TXE are set */
			if (pI2CHandle ->TxLen == 0){
				/* 1. Generate stop condition */
				if(pI2CHandle ->Sr == I2C_DISABLE_SR){
					I2C_GenerateStopCondition(pI2CHandle ->pI2Cx);
				}

				/* 2. Resets all member elements of handle structure */
				I2C_CloseSendData(pI2CHandle);

				/* 3. notify the application about tx complete */
				I2C_ApplicationEventCallback (pI2CHandle, I2C_EV_TX_CMPLT);
			}

		 }
	}
	else if(pI2CHandle ->TxRxState == I2C_STATUS_BUSY_RX){
		;
	}



}





/***************************************************************************************
 * 					Interrupt Handling
 *******************************************************************************************/

/******************************************************************************
 * @fn			- I2C_IRQInterruptConfig
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
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

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
 * @fn			- I2C_IRQPriorityConfig
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

	uint8_t iprx = IRQNumber / 4;  			/* ipr reg number offset address */
	uint8_t iprx_section = IRQNumber % 4;    /* position in iprx reg */

	uint8_t shift_position = (8 * iprx_section) + (8 - NO_IPR_BITS_IMPLEMENTED);  //only upper 4 bits are implemented in IPR
	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << shift_position);


}




/******************************************************************************
 * @fn			- I2C_EV_IRQHandling
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		- Interrupt handling for different I2C events (refer SR1)
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){

	/*Interrupt handling for both master and slave mode of a device */

	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle -> pI2Cx -> CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle -> pI2Cx -> CR2 & (1 << I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle -> pI2Cx -> SR1 & (1 << I2C_SR1_SB);

	/*1. Handle For interrupt generated by SB event */
	/*	Note : SB flag is only applicable in Master mode */
	if(temp1 && temp3){
		/* Start condition is generated succefully. Send address with read or wr */
		uint8_t mode;
		if(pI2CHandle -> TxRxState == I2C_STATUS_BUSY_TX) mode = I2C_MASTER_WR;
		else if (pI2CHandle -> TxRxState == I2C_STATUS_BUSY_RX) mode = I2C_MASTER_RD;
		I2C_ExecuteAddressPhase(pI2CHandle ->pI2Cx, pI2CHandle ->DevAddr, mode);
	}


	/* 2. Handle For interrupt generated by ADDR event */
	/* Note : When master mode : Address is sent */
	/*	When Slave mode   : Address matched with own address */
	temp3 = pI2CHandle -> pI2Cx -> SR1 & (1 << I2C_SR1_ADDR);
	if(temp1 && temp3){
		/* clear ADDR flag  */
		I2C_ClearADDRFlag(pI2CHandle);

	}


	/* 3. Handle For interrupt generated by BTF(Byte Transfer Finished) event */
	temp3 = pI2CHandle -> pI2Cx -> SR1 & (1 << I2C_SR1_BTF);
	if(temp1 && temp3){
		/* BTF flag is set */
		if(pI2CHandle -> pI2Cx ->SR2 & (1 << I2C_SR2_MSL)){
			/* device in master mode data transmission */
			I2C_MasterHandleBTFInterrupt(pI2CHandle);
		}

	}


	/* 4. Handle For interrupt generated by STOPF event */
	/* Note : Stop detection flag is applicable only slave mode . For master this flag will never be set */
	temp3 = pI2CHandle -> pI2Cx -> SR1 & (1 << I2C_SR1_STOPF);
	if(temp1 && temp3){
		/* STOPF flag is set */
		/* Clear STOPF flag  ie read SR1 folloerd by write CR1 regs*/
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		/*Notify the app that STOPF generated by master */
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);

	}


	/* 5. Handle For interrupt generated by TXE event */
	temp3 = pI2CHandle -> pI2Cx -> SR1 & (1 << I2C_SR1_TXE);
	if(temp1 && temp2 && temp3){
		/* TXE flag is set */
		/*chk for device mode */
		if(pI2CHandle -> pI2Cx ->SR2 & (1 << I2C_SR2_MSL)){
			/* device in master mode data transmission */
			I2C_MasterHandleTXEInterrupt(pI2CHandle);
		}
		else{
			/* devive is in slave mode */
			/* chk if the device is in transmit mode */
			if(pI2CHandle -> pI2Cx -> SR2 & (1 << I2C_SR2_TRA)){
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}

	}


	/* 6. Handle For interrupt generated by RXNE event */
	temp3 = pI2CHandle -> pI2Cx -> SR1 & (1 << I2C_SR1_RXNE);
	if(temp1 && temp2 && temp3){
		/* RXNE flag is set */
		/*chk for device mode */
		if(pI2CHandle -> pI2Cx ->SR2 & (1 << I2C_SR2_MSL)){
			/* device in master mode data reception */
			I2C_MasterHandleRXNEInterrupt(pI2CHandle);

		}
		else{
			/* slave mode */
			/* chk if the device is in transmit mode */
			if(!(pI2CHandle -> pI2Cx -> SR2 & (1 << I2C_SR2_TRA))){
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}


	}


}




/******************************************************************************
 * @fn			- I2C_ER_IRQHandling
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
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){

	uint32_t temp1,temp2;

	/*Know the status of  ITERREN control bit in the CR2 */
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


	/*Check for Bus error */
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		/* This is Bus error */
		/* clear the buss error flag */
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

	   /* notify the application about the error */
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

	/* Check for arbitration lost error */
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		/*This is arbitration lost error*/
		/* clear the arbitration lost error flag */
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		/* notify the application about the error */
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

	/* Check for ACK failure  error */

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		/* This is ACK failure error */
		/* clear the ACK failure error flag */
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		/* notify the application about the error */
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

	/* Check for Overrun/underrun error */
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		/* This is Overrun/underrun */
		/* clear the Overrun/underrun error flag */
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		/* notify the application about the error */
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

	/* Check for Time out error */
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		/* This is Time out error */
		/* clear the Time out error flag */
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		/* notify the application about the error */
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}



}


/******************************************************************************
 * 						Call back Fns
 ******************************************************************************/
__weak void I2C_ApplicationEventCallback (I2C_Handle_t *pI2CHandle, uint8_t AppEv){

	/* This is a weak function. Application may override it */
}
