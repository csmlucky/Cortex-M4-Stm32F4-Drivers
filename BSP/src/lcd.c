/*
 * lcd.c
 *
 *  Created on: Sep 1, 2025
 *      Author: csmla
 */

#include "lcd.h"

static void LCD_I2C_GPIOInits(void);
static void LCD_I2CInits(void);



I2C_Handle_t I2CHandle;
GPIO_Handle_t GPIO_I2CPins;


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





	/********************************LCD initialization commands **********************************/

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

		/* configure IRQ Priority for I2C1 event and error interrupts*/
		I2C_IRQPriorityConfig(LCD_I2C_EV_IRQ, LCD_I2C_EV_PR);
		I2C_IRQPriorityConfig(LCD_I2C_ER_IRQ, LCD_I2C_ER_PR);

		/* Enable I2C1_EV IRQ and I2C_ER IRQ */
		I2C_IRQInterruptConfig(LCD_I2C_ER_IRQ, ENABLE);
		I2C_IRQInterruptConfig(LCD_I2C_EV_IRQ, ENABLE);


}
