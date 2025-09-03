/*
 * stm32f446xx_rtc_driver.h
 *
 *  Created on: Aug 27, 2025
 *      Author: csmla
 */

#ifndef INC_STM32F446XX_RTC_DRIVER_H_
#define INC_STM32F446XX_RTC_DRIVER_H_

#include "stm32f446xx.h"


typedef enum{
	MON = 1,
	TUE,
	WED,
	THU,
	FRI,
	SAT,
	SUN
}rtc_weekday_e;


/* RTC_Config structure */
typedef struct{

	uint8_t Clock_Source;   			/* possiblevalues @RTC_CLKSRC: LSE / LSI / HSE/128 */
	uint32_t Async_Prescaler;       	/* For 1 HZ e.g., 127 for LSE */
	uint32_t Sync_Prescaler;        	/* For 1 HZ e.g., 255 for LSE */
	uint8_t hour_format;         	    /* possible values @RTC_HOUR_FMT  */
	uint8_t Reset_BackupDomain_EnorDi;  /* reset backup registers on init */

}RTC_Config_t;


/* RTC time structure */
typedef struct{

	uint8_t Hours;    /* 0-23 */
	uint8_t Minutes;  /* 0-59 */
	uint8_t Seconds;  /* 0-59 */
	uint8_t AMPM; 	 /*RTC_HOUR_FMT */
	rtc_weekday_e Weekday;  /*  1=Monday ... 7=Sunday (RTC convention uses 1..7) */
	uint8_t Day;      /* 1-31 */
	uint8_t Month;    /* 1-12 */
	uint8_t Year;     /* 0-99 (20yy) */

}RTC_Time_t;


/* RTC Alaram config structure */
typedef struct{

	uint8_t Alarm;               /*possible values @RTC_ALARM */
	RTC_Time_t Time;
	uint8_t Interrupt_EnorDi;
	uint8_t Output_Select;        /* possible values @RTC_OUT */
	uint8_t Output_Polarity;	/* possible values @RTC_OUT_POL */


}RTC_Alarm_Config_t;




/* RTC Wakeup Timer config structure */
typedef struct{

	uint8_t Clk;  				/*possible vaules @ RTC_WUT_CLK */
	uint32_t Auto_Reload;
	uint8_t Interrupt_EnorDi;
	uint8_t Output_Select;        /* possible values @RTC_OUT */
	uint8_t Output_Polarity;	/* possible values @RTC_OUT_POL */



}RTC_WUT_Config_t;



/* RTC timer Handle structure */
typedef struct{

	RTC_Config_t RTC_Config;
	RTC_Time_t RTC_Time;

}RTC_Handle_t;



/*
 * @RTC_WUT_CLK
 */
#define RTC_WUT_CLK_DIV_16				0
#define RTC_WUT_CLK_DIV_8				1
#define RTC_WUT_CLK_DIV_4				2
#define RTC_WUT_CLK_DIV_2				3
#define RTC_WUT_CLK_SPRE_1HZ			4
#define RTC_WUT_CLK_SPRE_1HZ_PLUS		0xC





/*
 * @RTC_CLKSRC
 */
#define RTC_CLKSRC_DISABLE			0
#define RTC_CLKSRC_LSE				1
#define RTC_CLKSRC_LSI				2
#define RTC_CLKSRC_HSE_DIV128		3

/*
 * @RTC_FLAG
 * RTC_FLAG macros to mask the status reg bit position
 */
#define RTC_FLAG_ALRAWF				(1 << ALRAWF)
#define RTC_FLAG_ALRBWF				(1 << ALRAWF)
#define RTC_FLAG_WUTWF				(1 << ALRAWF)
#define RTC_FLAG_SHPF				(1 << ALRAWF)
#define RTC_FLAG_INITS				(1 << ALRAWF)
#define RTC_FLAG_RSF				(1 << ALRAWF)
#define RTC_FLAG_INITF				(1 << ALRAWF)
#define RTC_FLAG_INIT				(1 << ALRAWF)
#define RTC_FLAG_ALRAF				(1 << ALRAWF)
#define RTC_FLAG_ALRBF				(1 << ALRAWF)
#define RTC_FLAG_WUTF				(1 << ALRAWF)
#define RTC_FLAG_TSF				(1 << ALRAWF)
#define RTC_FLAG_TSOVF				(1 << ALRAWF)
#define RTC_FLAG_TAMP1F				(1 << ALRAWF)
#define RTC_FLAG_TAMP2F				(1 << ALRAWF)
#define RTC_FLAG_RECALPF			(1 << ALRAWF)

/*
 * @RTC_OUT
 */
#define RTC_OUT_DISABLE				0
#define RTC_OUT_ALRMA				1
#define RTC_OUT_ALARMB				2
#define RTC_OUT_WUT					3

/*
 * @RTC_OUT_POL
 */
#define RTC_OUT_POL_HIGH			0 		/* The pin is high when ALRAF/ALRBF/WUTF is asserted */
#define RTC_OUT_POL_LOW				1		/* The pin is low when ALRAF/ALRBF/WUTF is asserted */


/*
 * @RTC_ALARM
 */
#define RTC_ALARM_A				0
#define RTC_ALARM_B				1

/*
 * @RTC_HOUR_FMT
 */
#define RTC_HOUR_FMT_AMPM			1
#define RTC_HOUR_FMT_24 			0

#define RTC_AM			0
#define RTC_PM			1


/*
 * RTC events
 */
#define RTC_EV_WUTI					1




/*********************************************************************************
 * API supported by this Driver. For more information refer to the Function definitions
 **************************************************************************************/

/*
 * Core
 */
void RTC_Init(RTC_Handle_t *pRTCHandle);
void RTC_DeInit(void);


/*
 * Set, Get Time/Date
 */
  /* binary → BCD inside */
void RTC_GetTimeDate(RTC_Time_t *pRTCTime);



/*
 * Alarms
  */
void RTC_AlarmConfig(RTC_Alarm_Config_t *pAlarm);


void RTC_AlarmAControl(uint8_t EnorDi);
void RTC_AlarmBControl(uint8_t EnorDi);
void RTC_AlarmAClearFlag(void);
void RTC_AlarmBClearFlag(void);


/*
 * Wakeup timer
  */
void RTC_WUTConfig(RTC_WUT_Config_t *pWUTConfig);
void RTC_WUTControl(uint8_t EnorDi);
void RTC_WakeupClearFlag(void);



uint8_t RTC_GetFlagStatus(uint8_t flagName);



/*
 * peripheral Interrupt
 */
void RTC_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void RTC_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void RTC_WUTIRQHandling(void);



/*
 * Application call backs
 */
void RTC_ApplicationEventCallback (uint8_t AppEv);


#endif /* INC_STM32F446XX_RTC_DRIVER_H_ */
