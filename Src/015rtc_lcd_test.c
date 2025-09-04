/*
 * 004spi_tx_testing.c
 *
 *  Created on: Aug 12, 2025
 *      Author: csmla
 *
 *      RTC Testing
 *      Set and Get TIme, Date
 *      RTC clk -> 1 HZ
 */


#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>
#include "lcd.h"


#define LOW 					0
#define BTN_PRESSED 			LOW
#define HIGH 					1
#define BTN_NOT_PRESSED 		HIGH

#define RTC_SYNCPRESCALER_1HZ   0xFF
#define RTC_ASYNCPRESCALER_1HZ	0x7F

RTC_Time_t readTimeDate;
RTC_Handle_t rtcHandle;

RTC_WUT_Config_t WUTConfig;

const char *weekDayStr[] = {"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
const char *amPmStr[] = {"AM", "PM"};

/* flag */
uint8_t newRTCRead = 0;

/*
 * GPIO-Button INit
 */
void GPIO_Button_Inits(void){
	GPIO_Handle_t GpioButton;

	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinOPSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;

	GPIO_Init(&GpioButton);

}

/*
 * delay
 */
void delay (void){
	/* ~200 msec when sys clock is 16 MHZ */
	for(uint32_t i = 0; i < 500000/2; i ++);
}


/*
 * WAIT_ForButtonPress
 * Polling button
 */
void WAIT_ForButtonPress (void){

	while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_NOT_PRESSED);

	/* debounce */
	delay();
}



int main(void){

	char lcdBuff[100] = {0};

	/* Initialize the button */
	GPIO_Button_Inits();


	/* LCD init */
	LCD_Init();

	/*rtc clk initialization */
	rtcHandle.RTC_Config.Sync_Prescaler = RTC_SYNCPRESCALER_1HZ;
	rtcHandle.RTC_Config.Clock_Source = RTC_CLKSRC_LSE;
	rtcHandle.RTC_Config.Async_Prescaler = RTC_ASYNCPRESCALER_1HZ;
	rtcHandle.RTC_Config.hour_format = RTC_HOUR_FMT_24;

	/* rtc date and time init */
	rtcHandle.RTC_Time.Day = 30;
	rtcHandle.RTC_Time.Month = 8;
	rtcHandle.RTC_Time.Year = 25;

	rtcHandle.RTC_Time.Hours = 11;
	rtcHandle.RTC_Time.Minutes = 45;
	rtcHandle.RTC_Time.Seconds = 0;
	rtcHandle.RTC_Time.Weekday = 6;

	RTC_Init(&rtcHandle);

	/* wait until the button is pressed */
	WAIT_ForButtonPress();

	/*wake up timer for every 60 sec */
	WUTConfig.Auto_Reload = 4; 		/* wut runs wutr-value +1 secs */
	WUTConfig.Clk = RTC_WUT_CLK_SPRE_1HZ;
	WUTConfig.Interrupt_EnorDi = ENABLE;

	RTC_WUTConfig(&WUTConfig);

	while(1){


		if(newRTCRead){

			/* print Date in LCD row 1 */
			sprintf(lcdBuff, "Date: %02u:%02u:%02u", readTimeDate.Day, readTimeDate.Month, readTimeDate.Year);
			LCD_DisplayClr();
			LCD_SetCursor(1, 1);
			LCD_PrintString(lcdBuff);

			/* print Time in LCD row 2 */
			sprintf(lcdBuff, "Time: %02u:%02u:%02u", readTimeDate.Hours, readTimeDate.Minutes, readTimeDate.Seconds);
			LCD_SetCursor(2, 1);
			LCD_PrintString(lcdBuff);

			newRTCRead = 0;
		}

	}


	return 0;
}



/******************************Interrupt **************************************************************/
void RTC_WKUP_IRQHandler (void){

	RTC_WUTIRQHandling();
}



/*
 * Call backs
 */
void RTC_ApplicationEventCallback (uint8_t AppEv){

	if(AppEv == RTC_EV_WUTI){

		/* Read Time and Date */
		RTC_GetTimeDate(&readTimeDate);

		newRTCRead = 1;


	}
}



