/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Aug 8, 2025
 *      Author: csmla
 */


#include "stm32f446xx_gpio_driver.h"


/******************************************************************************
 * @fn			- GPIO_PeriClockControl
 *
 * @brief		- This fn Enables or Disables peripheral clock for the given GPIO port
 *
 * @param[in]	- GPIO port base address
 * @param[in]	- ENABLE or DISABLE Macros
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA) GPIOA_PCLK_EN();
		else if (pGPIOx == GPIOB) GPIOB_PCLK_EN();
		else if (pGPIOx == GPIOC) GPIOC_PCLK_EN();
		else if (pGPIOx == GPIOD) GPIOD_PCLK_EN();
		else if (pGPIOx == GPIOE) GPIOE_PCLK_EN();
		else if (pGPIOx == GPIOF) GPIOF_PCLK_EN();
		else if (pGPIOx == GPIOG) GPIOG_PCLK_EN();
		else if (pGPIOx == GPIOH) GPIOH_PCLK_EN();
	}else{
		if(pGPIOx == GPIOA) GPIOA_PCLK_DI();
		else if (pGPIOx == GPIOB) GPIOB_PCLK_DI();
		else if (pGPIOx == GPIOC) GPIOC_PCLK_DI();
		else if (pGPIOx == GPIOD) GPIOD_PCLK_DI();
		else if (pGPIOx == GPIOE) GPIOE_PCLK_DI();
		else if (pGPIOx == GPIOF) GPIOF_PCLK_DI();
		else if (pGPIOx == GPIOG) GPIOG_PCLK_DI();
		else if (pGPIOx == GPIOH) GPIOH_PCLK_DI();
	}
}



/******************************************************************************
 * @fn			- GPIO_Init
 *
 * @brief		- This fn configure the given GPIO port pin
 *
 * @param[in]	- GPIO Handle address
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 ************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	/* Enable GPIO peripheral Clock */
	GPIO_PeriClockControl(pGPIOHandle -> pGPIOx, ENABLE);

	uint32_t temp = 0;
	/* 1. Configure the mode for the given gpio pin */
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		/*Normal GPIO configurations */
		temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle -> pGPIOx -> MODER &= ~(0x3 << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber)); //need to clear bit field before sets
		pGPIOHandle -> pGPIOx -> MODER |= temp;
	}
	else{
		/* Interrupt configuration */

		if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			/* 1. configure falling edge FTSR */
			EXTI -> FTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);

			/* clear same bit in RTSR */
			EXTI -> RTSR &= ~(1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			/* 1. configure raising edge RTSR */
			EXTI -> RTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);

			/* clear same bit in FTSR */
			EXTI -> FTSR &= ~(1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			/* 1. configure both falling and raising edge FTSR, RTSR */
			EXTI -> RTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			EXTI -> FTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}
		/* 2. Configure the GPIO port selection in SYSCFG_EXTICR */
		uint8_t temp1 = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber) / 4; //selects SYSCFG_EXTIR 0 to 3
		uint8_t temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 4; //selects position in SYSCFG_EXTIR 0 to 3
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle -> pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG -> EXTICR[temp1] |= (portcode << (4 *temp2));

		/* 3. Enable EXTI interrupt delivery using IMR */
		EXTI -> IMR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	}

	/* 3. Configure the speed */
	temp = 0;
	temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinOPSpeed << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx -> OSPEEDR &= ~(0x3 << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle -> pGPIOx -> OSPEEDR |= temp;

	/* 4. Configure the Pull-up / Pull-down */
	temp = 0;
	temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx -> PUPDR &= ~(0x3 << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));	//clearing
	pGPIOHandle -> pGPIOx -> PUPDR |= temp;

	/* 5. Configure the output type */
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ALTFN){
		temp = 0;
		temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinOPType << ( pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle -> pGPIOx -> OTYPER &= ~(0x1 << (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));	//clearing
		pGPIOHandle -> pGPIOx -> OTYPER |= temp;
	}

	/* Configure Alternate Function mode */
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		uint8_t temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 8;

		if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber <= 7){
			temp = 0;
			temp = pGPIOHandle -> GPIO_PinConfig.GPIO_AltrFnMode << (4 * temp1);
			pGPIOHandle -> pGPIOx ->AFRL &= ~(0xf << (4 * temp1));
			pGPIOHandle -> pGPIOx ->AFRL |= temp;
		}
		else{
			temp = 0;
			temp = pGPIOHandle -> GPIO_PinConfig.GPIO_AltrFnMode << (4 * temp1);
			pGPIOHandle -> pGPIOx ->AFRH &= ~(0xf << (4 * temp1)); //clearing
			pGPIOHandle -> pGPIOx ->AFRH |= temp;
		}
	}
}

/******************************************************************************
 * @fn			- GPIO_DeInit
 *
 * @brief		- This fn Resets Regs for the given GPIO port
 *
 * @param[in]	- GPIO port base address
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 ************************************************************************************/

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx == GPIOA) GPIOA_REG_RESET();
	else if (pGPIOx == GPIOB) GPIOB_REG_RESET();
	else if (pGPIOx == GPIOC) GPIOC_REG_RESET();
	else if (pGPIOx == GPIOD) GPIOD_REG_RESET();
	else if (pGPIOx == GPIOE) GPIOE_REG_RESET();
	else if (pGPIOx == GPIOF) GPIOF_REG_RESET();
	else if (pGPIOx == GPIOG) GPIOG_REG_RESET();
	else if (pGPIOx == GPIOH) GPIOH_REG_RESET();

}


/******************************************************************************
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		- This fn read data the given GPIO port pin
 *
 * @param[in]	- GPIO port base address
 * @param[in]	- pinNumber macros
 * @param[in]	-
 *
 * @return		- 0 or 1
 *
 * @note		-
 ************************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value = (uint8_t) ((pGPIOx -> IDR >> PinNumber) & 0x00000001);  //read pin value
	return value;
}

/******************************************************************************
 * @fn			- GPIO_ReadFromInputPort
 *
 * @brief		- This fn reads data from the given GPIO
 *
 * @param[in]	- GPIO port base address
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- (uint16_t)
 *
 * @note		-
 ************************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t) (pGPIOx -> IDR);  //read port value
	return value;
}
/******************************************************************************
 * @fn			- GPIO_WriteToOutputpin
 *
 * @brief		- This fn writes to the given GPIO port pin
 *
 * @param[in]	- GPIO port base address
 * @param[in]	- PinNumber macro
 * @param[in]	- SET/ RESET macros
 *
 * @return		-
 *
 * @note		-
 ************************************************************************************/
void GPIO_WriteToOutputpin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value == SET){
		pGPIOx -> ODR |= (1 << PinNumber);
	}
	else{
		pGPIOx -> ODR &= ~(1 << PinNumber);
	}

}
/******************************************************************************
 * @fn			- GPIO_WriteToOutputPort
 *
 * @brief		- This fn writes to the given GPIO port ODR
 *
 * @param[in]	- GPIO port base address
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 ************************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
	pGPIOx -> ODR = value;

}
/******************************************************************************
 * @fn			- GPIO_ToggleOutputPin
 *
 * @brief		- This fn toggles the given GPIO port pin
 *
 * @param[in]	- GPIO port base address
 * @param[in]	- PinNumber macros
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 ************************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx -> ODR ^= (1 << PinNumber);

}


/******************************************************************************
 * @fn			- GPIO_IRQInterruptConfig
 *
 * @brief		- This fn configure the IRQ set or clear reg
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 ************************************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

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
 * @fn			- GPIO_IRQPriorityConfig
 *
 * @brief		- This fn configure the IRQ Priority Reg
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 ************************************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

uint8_t iprx = IRQNumber / 4;  			/* ipr reg number offset address */
uint8_t iprx_section = IRQNumber % 4;    /* position in iprx reg */

uint8_t shift_position = (8 * iprx_section) + (8 - NO_IPR_BITS_IMPLEMENTED);  //only upper 4 bits are implemented in IPR
*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << shift_position);


}



/******************************************************************************
 * @fn			- GPIO_IRQHandling
 *
 * @brief		- Clears the EXT pending ststus for the given GPIO port pin
 *
 * @param[in]	- GPIO PinNumber
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 ************************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber){
	/*clear the EXTI PR (pending reg) value for the given pin */
	if(EXTI -> PR & (1 << PinNumber)){
		EXTI -> PR |= (1 << PinNumber);   /* setting the pin clears the pending status */
	}
}
