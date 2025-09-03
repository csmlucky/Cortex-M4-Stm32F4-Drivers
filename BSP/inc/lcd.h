/*
 * lcd.h
 *
 *  Created on: Sep 1, 2025
 *      Author: csmla
 *
 *      LCD I2C : The PCF8574 chip on the I2C backpack has 8 parallel
 *      			I/O pins (P0–P7) that connect to the LCD's data and control lines.
 *      P7–P4: Connect to the LCD's data lines (D7–D4).
 *		P3: Connects to the LCD's backlight control (if used).
 *		P2: Connects to the LCD's Enable (E) pin.
 *		P1: Connects to the LCD's Read/Write (R/W) pin (typically tied low for write-only).
 *		P0: Connects to the LCD's Register Select (RS) pin.
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "stm32f446xx.h"


/* LCD Interface config */
#define LCD_I2C							I2C1
#define LCD_I2C_GPIO_PORT				GPIOB
#define LCD_I2C_SDA						GPIO_PIN_NO_9					/* PB9 */
#define LCD_I2C_SCL						GPIO_PIN_NO_8					/* PB8 */
#define LCD_I2C_SPEED					I2C_SCLK_SPEED_SM				/* standard mode 100khz */
#define LCD_I2C_PU  					GPIO_PIN_PU						/* Internal pull up */
#define LCD_I2C_ADDR					0x27

#define LCD_I2C_EV_IRQ						IRQ_NO_I2C1_EV
#define LCD_I2C_ER_IRQ						IRQ_NO_I2C1_ER

#define LCD_I2C_EV_PR						NVIC_IRQ_PR10
#define LCD_I2C_ER_PR						NVIC_IRQ_PR9

/* LCD Controls */
#define LCD_RD								1
#define LCD_WR								0
#define LCD_RS_CMD							0
#define LCD_RS_DATA							1
#define LCD_EN_HIGH							1
#define LCD_EN_LOW							0


/* LCD Control bit position */
#define LCD_RS_POSI							0
#define LCD_R_W_POSI						1
#define LCD_EN_POSI							2
#define LCD_BL_POSI							3

/* LCD Commands */


/**********************************API***********************************************************/

void LCD_Init(void);

#endif /* INC_LCD_H_ */
