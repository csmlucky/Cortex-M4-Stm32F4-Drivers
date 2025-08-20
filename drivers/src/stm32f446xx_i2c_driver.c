/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Aug 15, 2025
 *      Author: csmla
 */


#include "stm32f446xx_i2c_driver.h"

static void I2C_GenerateStartCondition (I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase (I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag (I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition (I2C_RegDef_t *pI2Cx);


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
	tempreg |= (pI2CHandle -> I2C_Config.I2C_DeviceAddress) << 1;
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


/*****************************************************************************************
 * 				Send and Receive data
 ***********************************************************************************************/
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
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr){

	/* Generate the start condition */
	I2C_GenerateStartCondition(pI2CHandle ->pI2Cx);

	/*Confirm the start generation is completed by using the SB flag in SR1 reg */
	/* Note: until SB flag is cleared SCL will be stretched (pulled low) */
	while(I2C_GetFlagStatus(pI2CHandle ->pI2Cx, I2C_FLAG_SB) == RESET);

	/* send the slave address with r(1) / w(0) bit as lsb */
	I2C_ExecuteAddressPhase(pI2CHandle ->pI2Cx, SlaveAddr);

	/* Confirm sending address is completed by checking the ADDR bit in SR1 reg */
	while(I2C_GetFlagStatus(pI2CHandle ->pI2Cx, I2C_FLAG_ADDR) == RESET);

	/* clear ADDR bit Note: until ADDR flag is cleared SCL will be stretched (pulled low)*/
	I2C_ClearADDRFlag(pI2CHandle ->pI2Cx);

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
	I2C_GenerateStopCondition(pI2CHandle ->pI2Cx);

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
static void I2C_ExecuteAddressPhase (I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){

	SlaveAddr <<= 1;
	/*  write mode */
	SlaveAddr &= ~( 1 );
	pI2Cx -> DR |= SlaveAddr;

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
static void I2C_ClearADDRFlag (I2C_RegDef_t *pI2Cx){
	/* ADDR bit is cleared by reading SR1 followed by SR2 */

	uint32_t dummyread;
	dummyread = pI2Cx -> SR1;
	dummyread = pI2Cx -> SR2;
	(void)dummyread;

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
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx-> CR1 |= (1 << I2C_CR1_STOP);
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

