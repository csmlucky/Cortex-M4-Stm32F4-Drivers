/*
 * lcd.c
 *
 *  Created on: Sep 1, 2025
 *      Author: csmla
 */

#include "lcd.h"
#include <stdio.h>

static void LCD_I2C_GPIOInits(void);
static void LCD_I2CInits(void);
static void LCD_StartupCmds(void);
static void LCD_CustmConfigCmds(void);

static void LCD_Write (uint8_t val, uint8_t mode);
static void LCD_Write4Bits(uint8_t nible, uint8_t mode);
static void LCD_Enable(uint8_t val);


static void delay_ms(uint16_t value);
static void delay_us(uint8_t value);

static void LCD_AddrTest(void);
static void LCD_SendDummy(void);


I2C_Handle_t I2CHandle;
GPIO_Handle_t GPIO_I2CPins;

/* flag */
volatile uint8_t i2c_tx_done;


/******************************************************************************
 * @fn			- LCD_Init
 *
 * @brief		- lcd i2c config and send commands to lcd for initialization
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void LCD_Init(void){

	LCD_I2C_GPIOInits();

	LCD_I2CInits();

	delay_ms(50);

	//LCD_AddrTest();
	/* for some reason lcd is not acking for the first byte work around*/
	LCD_SendDummy();

	LCD_StartupCmds();

	LCD_CustmConfigCmds();


}



/******************************************************************************
 * @fn			- LCD_PrintString
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
void LCD_PrintString(char *lcdBuf){

	while(*lcdBuf != '\0'){

		LCD_Write((uint8_t)*lcdBuf, LCD_RS_DATA);
		lcdBuf ++;

	}
}


/******************************************************************************
 * @fn			- LCD_PrintChar
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
void LCD_PrintChar(char lcdChar){

		LCD_Write((uint8_t)lcdChar, LCD_RS_DATA);

}

/******************************************************************************
 * @fn			- LCD_SendCmd
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
void LCD_SendCmd(uint8_t lcdCmd){

		LCD_Write(lcdCmd, LCD_RS_CMD);

}


/******************************************************************************
 * @fn			- LCD display clear
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

/********************* **********************************/
void LCD_DisplayClr(void){

	/* Send Display clear */
	LCD_Write((LCD_CNFG_CMD_DIS_CLEAR), LCD_RS_CMD);
	delay_ms(2);

}



/******************************************************************************
 * @fn			- LCD display Return Home
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

/********************* **********************************/
void LCD_Display_ReturnHome(void){

	/* Send Display clear */
	LCD_Write((LCD_CNFG_CMD_DIS_RETURN_HOME), LCD_RS_CMD);
	delay_ms(2);

}



/******************************************************************************
 * @fn			- LCD Set Cursor
 *
 * @brief		-Set Lcd to a specified location given by row and column information
 *   			Row Number (1 to 2)
 *   			Column Number (1 to 16) Assuming a 2 X 16 characters display
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */

/********************* **********************************/
void LCD_SetCursor(uint8_t row, uint8_t column){

	column--;
	  switch (row)
	  {
	    case 1:
	      /* Set cursor to 1st row address and add index*/
	      LCD_Write((column |= 0x80), LCD_RS_CMD);
	      break;
	    case 2:
	      /* Set cursor to 2nd row address and add index*/
	        LCD_Write((column |= 0xC0), LCD_RS_CMD);
	      break;
	    default:
	      break;
	  }

}



/*****************************************************************************************************
 * Helperfns
 *********************************************************************************************************/

/************************ Configure GPIO pin as I2C fn *******************************************/
static void LCD_I2C_GPIOInits(void){

		GPIO_I2CPins.pGPIOx = LCD_I2C_GPIO_PORT;
		GPIO_I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
		GPIO_I2CPins.GPIO_PinConfig.GPIO_AltrFnMode = 4;
		GPIO_I2CPins.GPIO_PinConfig.GPIO_PinOPSpeed = GPIO_SPEED_HIGH;
		GPIO_I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
		GPIO_I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = LCD_I2C_PU;

		/* SDA */
		GPIO_I2CPins.GPIO_PinConfig.GPIO_PinNumber = LCD_I2C_SDA;
		GPIO_Init(&GPIO_I2CPins);

		/* SCL */
		GPIO_I2CPins.GPIO_PinConfig.GPIO_PinNumber = LCD_I2C_SCL;
		GPIO_Init(&GPIO_I2CPins);

}


/*****************************Configure I2C ***********************************************/
static void LCD_I2CInits(void){

		I2CHandle.pI2Cx = LCD_I2C;
		I2CHandle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
		I2CHandle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
		I2CHandle.I2C_Config.I2C_SCLSpeed = I2C_SCLK_SPEED_SM;

		I2C_Init(&I2CHandle);

		/* Enable I2C peripheral  */
		I2C_PeripheralControl(LCD_I2C, ENABLE);

		/* configure ack control in CR1 after enabling the peripheral i2c*/
		I2C_ManageAcking(LCD_I2C, I2C_ACK_ENABLE);

#if 0
		/* configure IRQ Priority for I2C1 event and error interrupts*/
		I2C_IRQPriorityConfig(LCD_I2C_EV_IRQ, LCD_I2C_EV_PR);
		I2C_IRQPriorityConfig(LCD_I2C_ER_IRQ, LCD_I2C_ER_PR);

		/* Enable I2C1_EV IRQ and I2C_ER IRQ */
		I2C_IRQInterruptConfig(LCD_I2C_ER_IRQ, ENABLE);
		I2C_IRQInterruptConfig(LCD_I2C_EV_IRQ, ENABLE);
#endif

}

/********************************LCD initialization commands **********************************/
static void LCD_StartupCmds(void){


	/* Send 0x3 in cmd mode */
	LCD_Write4Bits((LCD_INIT_CMD1 << 4), LCD_RS_CMD);
	delay_ms(5);

	LCD_Write4Bits((LCD_INIT_CMD1 << 4), LCD_RS_CMD);
	delay_us(150);

	LCD_Write4Bits((LCD_INIT_CMD1 << 4), LCD_RS_CMD);
	delay_us(150);

	LCD_Write4Bits((LCD_INIT_CMD2 << 4), LCD_RS_CMD);
	delay_us(150);

}


/********************************LCD custom Configuration commands **********************************/
static void LCD_CustmConfigCmds(void){

	/* Send Function set cmd */
	LCD_Write((LCD_CNFG_CMD_4BIT_2LN_5X8F), LCD_RS_CMD);
	delay_ms(2);

	/* Send Display on cursor on */
	LCD_Write((LCD_CNFG_CMD_DON_CON), LCD_RS_CMD);
	delay_us(100);

	/* Send Display clear */
	LCD_DisplayClr();

	/* Send Entry mode set */
	LCD_Write((LCD_CNFG_CMD_INCADD), LCD_RS_CMD);
	delay_us(100);

	/* Send DDRAM addr */
	LCD_Write((LCD_DDRAM_LN1), LCD_RS_CMD);
	delay_us(100);


}


/***************************LCD write ***************************************/
static void LCD_Write (uint8_t val, uint8_t mode){

	/* first send upper nibble */
	LCD_Write4Bits((val & 0xF0), mode);

	/* lower nibble */
	LCD_Write4Bits((val << 4), mode);


}


/**************************LCD write nibble **************************************/
static void LCD_Write4Bits(uint8_t nible, uint8_t mode){

	uint8_t temp = 0;

	temp = (nible & 0xF0) | (mode ? 1:0);

	// Always write mode
	temp &= ~(1U << LCD_RW_POSI);

	LCD_Enable(temp);

}


/*********************LCD I2C write with enable **********************************/
static void LCD_Enable(uint8_t val){

	uint8_t temp = 0;
#if 0
	temp = val | (1U << LCD_EN_POSI);

	//while(I2C_MasterSendDataIT(&I2CHandle, &temp, 1, LCD_I2C_ADDR, I2C_DISABLE_SR) != I2C_STATUS_READY);
	I2C_MasterSendDataIT(&I2CHandle, &temp, 1, LCD_I2C_ADDR, I2C_DISABLE_SR);
	while (!i2c_tx_done);
	i2c_tx_done = 0;

	delay_us(100);

	temp = val & ~(1U << LCD_EN_POSI);
	//while(I2C_MasterSendDataIT(&I2CHandle, &temp, 1, LCD_I2C_ADDR, I2C_DISABLE_SR) != I2C_STATUS_READY);
	I2C_MasterSendDataIT(&I2CHandle, &temp, 1, LCD_I2C_ADDR, I2C_DISABLE_SR);
	while (!i2c_tx_done);
	i2c_tx_done = 0;

	delay_us(100);

#endif

	temp = val | (1U << LCD_EN_POSI) |(1U << LCD_BL_POSI);
	I2C_MasterSendData(&I2CHandle, &temp, 1, LCD_I2C_ADDR, I2C_DISABLE_SR);

	delay_us(50);

	temp = (val & ~(1U << LCD_EN_POSI)) | (1U << LCD_BL_POSI);
	I2C_MasterSendData(&I2CHandle, &temp, 1, LCD_I2C_ADDR, I2C_DISABLE_SR);

	delay_us(50);

}






/*********************delay msec**********************************/
static void delay_ms(uint16_t value)
{
    for(uint32_t i = 0 ; i < (value * 16000); i++){
    	__NOP();
    }
}


/*********************delay micro sec**********************************/
static void delay_us(uint8_t value)
{
    for(uint32_t i = 0 ; i < (value * 16); i++){
    	__NOP();
    }
}





/**********************************************************************************
 * 					LCD - I2C interrupt
 * *********************************************************************************/
void I2C1_EV_IRQHandler (void){

	I2C_EV_IRQHandling(&I2CHandle);

}

void I2C1_ER_IRQHandler (void){

	I2C_ER_IRQHandling(&I2CHandle);

}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
    if(AppEv == I2C_EV_TX_CMPLT)
    {
        printf("Tx is completed\n");
        i2c_tx_done = 1;
    }
    if(AppEv == I2C_ERROR_AF){
    	//I2C_CloseSendData(pI2CHandle);

    }
}


/*******************************************work around ***************************************/
static void LCD_SendDummy(void){

	uint8_t addr = 0x27;
	uint32_t guard = 1000;

// If BUSY, try to free with STOP
	if (LCD_I2C->SR2 & (1 << I2C_SR2_BUSY)) {
		LCD_I2C->CR1 |= (1 << I2C_CR1_STOP);
		// small guard to allow release
		for (volatile uint32_t d=0; d<1000; ++d) __asm__ volatile("nop");
	}

	// START
	LCD_I2C->CR1 |= (1 << I2C_CR1_START);
	// Wait SB
	while (!(LCD_I2C->SR1 & (1 << I2C_SR1_SB))) {
		if (!--guard) goto nack_stop;
	}

	// Send address (write): (addr7<<1) | 0
	LCD_I2C->DR = (uint32_t)(addr << 1);

	// Wait for ADDR (ACK) or AF (NACK)
	while (1) {
		uint32_t sr1 = LCD_I2C->SR1;
		if (sr1 & (1 << I2C_SR1_ADDR)) {
			(void)LCD_I2C->SR2;              // clear ADDR
			LCD_I2C->CR1 |= (1 <<I2C_CR1_STOP) ;    // STOP
			printf("ACK at %u\n", addr);// ACKed
			goto end;
		}
		if (sr1 & (1 << I2C_SR1_AF)) {
			// NACK: clear AF and STOP
			LCD_I2C->SR1 &= ~(1 << I2C_SR1_AF);
			goto nack_stop;
		}
		if (!--guard) {
			goto nack_stop;
		}
	}

	nack_stop:
	LCD_I2C->CR1 |= (1 << I2C_CR1_STOP);
	// Optional: wait until BUSY clears (short guard)
	printf("NO ACK at %u\n", addr);// ACKed
	//for (volatile uint32_t d=0; d<2000; ++d) __asm__ volatile("nop");
	end:




}

/*****************************************Test***********************************************/
static void LCD_AddrTest(void){

	uint8_t addr = 0x27;
	for(uint8_t i = 0; i <= 4; i++){

		uint32_t guard = 1000;

	// If BUSY, try to free with STOP
		if (LCD_I2C->SR2 & (1 << I2C_SR2_BUSY)) {
			LCD_I2C->CR1 |= (1 << I2C_CR1_STOP);
			// small guard to allow release
			for (volatile uint32_t d=0; d<1000; ++d) __asm__ volatile("nop");
		}

		// START
		LCD_I2C->CR1 |= (1 << I2C_CR1_START);
		// Wait SB
		while (!(LCD_I2C->SR1 & (1 << I2C_SR1_SB))) {
			if (!--guard) goto nack_stop;
		}

		// Send address (write): (addr7<<1) | 0
		LCD_I2C->DR = (uint32_t)(addr << 1);

		// Wait for ADDR (ACK) or AF (NACK)
		while (1) {
			uint32_t sr1 = LCD_I2C->SR1;
			if (sr1 & (1 << I2C_SR1_ADDR)) {
				(void)LCD_I2C->SR2;              // clear ADDR
				LCD_I2C->CR1 |= (1 <<I2C_CR1_STOP) ;    // STOP
				printf("ACK at %u\n", i);// ACKed
				goto end;
			}
			if (sr1 & (1 << I2C_SR1_AF)) {
				// NACK: clear AF and STOP
				LCD_I2C->SR1 &= ~(1 << I2C_SR1_AF);
				goto nack_stop;
			}
			if (!--guard) {
				goto nack_stop;
			}
		}

	nack_stop:
		LCD_I2C->CR1 |= (1 << I2C_CR1_STOP);
		// Optional: wait until BUSY clears (short guard)
		printf("NO ACK at %u\n", i);// ACKed
		//for (volatile uint32_t d=0; d<2000; ++d) __asm__ volatile("nop");
	end:


	}


}

