/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Aug 8, 2025
 *      Author: csmla
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"



typedef struct{
	uint8_t GPIO_PinNumber;   		/* possible values from GPIO_PIN_NO */
	uint8_t GPIO_PinMode;			/* possible values from GPIO_MODE */
	uint8_t GPIO_PinOPType;			/* <possible values from GPIO_OP_TYPE */
	uint8_t GPIO_PinOPSpeed;		/* <possible values from GPIO_SPEED */
	uint8_t GPIO_PinPuPdControl;	/* <possible values from GPIO_PIN*/
	uint8_t GPIO_AltrFnMode;		/* <possible values from GPIO_ALTFN */

}GPIO_PinConfig_t;

/*
 * Handle structure for a GPIO pin
 */
typedef struct{

	GPIO_RegDef_t *pGPIOx;  // Holds the base address of the GPIO port to ehich the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;



/*
 * @GPIO_PIN_NO
 * GPIO pin number macros
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15



/*
 * @GPIO_MODE
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_RT		4
#define GPIO_MODE_IT_FT		5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_OP_TYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * @GPIO_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PIN
 * GPIO pin Pull-up and Pull-down
 */
#define GPIO_PIN_NOPUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2



/*********************************************************************************
 * API supported by this Driver. For more information refer to the Function definitions
 **************************************************************************************/
/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Peripheral init/ deint
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Peripheral read/ write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputpin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * peripheral Interrupt
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
