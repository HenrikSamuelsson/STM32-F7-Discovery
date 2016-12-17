/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
#define STEP_1 //initialize STemwin
#define STEP_2 //create simple dialog
//#define STEP_3 //implement touch support
//#define STEP_4 //add images into project(bmp convertor)
//#define STEP_5 //put images on display
//#define STEP_6 //put images on buttons
//#define STEP_7 //implement button callback action
//#define STEP_8 //add font into project(Font convertor)
//#define STEP_9 //use fonts
//#define STEP_10 //create hours
//#define STEP_11 //double buffer mode

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */
#include "main.h"

#ifdef STEP_1
#include "WM.h"
#include "FRAMEWIN.h"
#include "TEXT.h"
#include "BUTTON.h"
#include "DIALOG.h"
#endif
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Private variables ---------------------------------------------------------*/
/*debug var start*/
uint8_t test_buffer[200];
/*debug var end*/
TIM_HandleTypeDef TimHandle;
#ifdef STEP_1
uint8_t GUI_Initialized = 0; //check if the STemWin is initialized (necessary for touch support)
#endif

#ifdef STEP_4
/*background image*/
extern GUI_CONST_STORAGE GUI_BITMAP bmbackground;
/*board image*/
extern GUI_CONST_STORAGE GUI_BITMAP bmboard;
/*bulf off image*/
extern GUI_CONST_STORAGE GUI_BITMAP bmbulb_off;
/*bulf on image*/
extern GUI_CONST_STORAGE GUI_BITMAP bmbulb_on;
/*en flag image*/
extern GUI_CONST_STORAGE GUI_BITMAP bmUK;
/*fr flag image*/
extern GUI_CONST_STORAGE GUI_BITMAP bmFR;
/*power image*/
extern GUI_CONST_STORAGE GUI_BITMAP bmpower;
/*mail image*/
extern GUI_CONST_STORAGE GUI_BITMAP bmmail;
/*home image*/
extern GUI_CONST_STORAGE GUI_BITMAP bmhome;
/*light image*/
extern GUI_CONST_STORAGE GUI_BITMAP bmlight;
#endif
#ifdef STEP_8
/*font arial 24*/
extern GUI_CONST_STORAGE GUI_FONT GUI_FontArialNarrow24;
/*font arial 30*/
extern GUI_CONST_STORAGE GUI_FONT GUI_FontArialNarrow30;
#endif

/*********************************************************************
 *
 *       Dialog resources
 *
 **********************************************************************
 */

#ifdef STEP_2
/*assign the ID number of each our component in STemWin, number must be unique*/
#define GUI_ID_POWER (GUI_ID_USER+0)
#define GUI_ID_LIGHT (GUI_ID_USER+1)
#define GUI_ID_MAIL (GUI_ID_USER+2)
#define GUI_ID_HOME (GUI_ID_USER+3)
#define GUI_ID_LANGUAGE (GUI_ID_USER+4)
#define GUI_ID_TEXT_LABEL (GUI_ID_USER+5)
#define GUI_ID_TEXT_TIME (GUI_ID_USER+6)
#define GUI_ID_TEXT_MAIL (GUI_ID_USER+7)
#define GUI_ID_TEXT_HOME (GUI_ID_USER+8)
#define GUI_ID_TEXT_LIGHT (GUI_ID_USER+9)
#endif

#ifdef STEP_2
/*Array with dialog box content*/
static const GUI_WIDGET_CREATE_INFO _aFrameWin[] =
{
{ WINDOW_CreateIndirect, "Demo", 0, 0, 0, 480, 272, 0, 0 }, /*dialog box window*/
{ BUTTON_CreateIndirect, "power", GUI_ID_POWER, 207, 95, 64, 64, 0, 0 }, /*the bower button*/
{ BUTTON_CreateIndirect, "light", GUI_ID_LIGHT, 163, 229, 24, 24, 0, 0 }, /*light button*/
{ BUTTON_CreateIndirect, "mail", GUI_ID_MAIL, 291, 229, 24, 24, 0, 0 }, /*mail button*/
{ BUTTON_CreateIndirect, "home", GUI_ID_HOME, 227, 229, 24, 24, 0, 0 }, /*home button*/
{ BUTTON_CreateIndirect, "language", GUI_ID_LANGUAGE, 430, 0, 50, 44, 0, 0 }, /*language button*/
		{ TEXT_CreateIndirect, "Light Control", GUI_ID_TEXT_LABEL, 15, 7, 320,
				40, 0, 0 }, /*light control label*/
		{ TEXT_CreateIndirect, "XX:XX", GUI_ID_TEXT_TIME, 385, 7, 50, 20,
		TEXT_CF_HCENTER, 0 }, /*label for the clocks*/
		{ TEXT_CreateIndirect, "MAIL", GUI_ID_TEXT_MAIL, 271, 256, 64, 12,
		TEXT_CF_HCENTER, 0 }, /*mail label*/
		{ TEXT_CreateIndirect, "HOME", GUI_ID_TEXT_HOME, 207, 256, 64, 12,
		TEXT_CF_HCENTER, 0 }, /*home label*/
		{ TEXT_CreateIndirect, "LIGHT", GUI_ID_TEXT_LIGHT, 143, 256, 64, 12,
		TEXT_CF_HCENTER, 0 } /*light label*/
};
#endif

#ifdef STEP_2
static WM_HWIN _hDialogWindow; /*the dialog box handle*/
#endif

#ifdef STEP_5
static uint8_t onoff = 0; /*contain information if the bulb is on or off*/
static uint8_t enfr = 0; /*contain information if language is en or fr*/
#endif

#ifdef STEP_10
static uint8_t hour = 0, minute = 0; /*variable contai hour and minute time*/
static uint32_t miliceconds = 0; /*milisecond time*/
static uint8_t time_buffer[10]; /*buffer for time string*/
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void
SystemClock_Config(void);
void
Error_Handler(void);
static void
MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#ifdef STEP_3
void
TouchTimer_Init(void); /*Initialize timer for touch sensor*/
void
BSP_Pointer_Update(void); /*function for the pointer update*/
#endif

#ifdef STEP_2
static void _cbFrameWinControl(WM_MESSAGE * pMsg); /*callback function for the dialog box*/
#endif
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();

	/* USER CODE BEGIN 2 */
#ifdef STEP_3
	TouchTimer_Init();/*Initialize the Touch and enable the Timer for touch periodical reading*/
#endif

#ifdef STEP_1
	BSP_SDRAM_Init(); /* Initializes the SDRAM device */
	__HAL_RCC_CRC_CLK_ENABLE()
	; /* Enable the CRC Module */
	GUI_Init();/*Initialize and start STemWin*/
	GUI_Initialized = 1; /*STemWin initialize allow the pointer update from Touch */
	GUI_DispStringAt("Starting...", 0, 0);/*set strig on display*/
	GUI_Exec();/*show last changes on the screen*/
#endif

#ifdef STEP_11
	WM_MULTIBUF_Enable(1); /*enable double buffer for window manager*/
#endif

#ifdef STEP_2
	_hDialogWindow = GUI_CreateDialogBox(_aFrameWin, GUI_COUNTOF(_aFrameWin),
			&_cbFrameWinControl, WM_HBKWIN, 0, 0); //create dialog with all window items
	GUI_ExecCreatedDialog(_hDialogWindow); //execute the dialog window !! blocking function
#endif
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;

}

/* USER CODE BEGIN 4 */
#ifdef STEP_3

/*function initialize timer which will periodically read the touch controller state in interrupt routine*/
void TouchTimer_Init()
{
	uint32_t uwPrescalerValue = 0;
	/* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
	uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2) / 10000) - 1;

	BSP_TS_Init(480, 272);/*Initialise the TS controller on screen resolution*/

	/* Set TIMx instance */
	TimHandle.Instance = TIM3;

	/* Initialise TIM3 peripheral as follows:
	 + Period = 500 - 1
	 + Prescaler = ((SystemCoreClock/2)/10000) - 1
	 + ClockDivision = 0
	 + Counter direction = Up
	 */
	TimHandle.Init.Period = 500 - 1;
	TimHandle.Init.Prescaler = uwPrescalerValue;
	TimHandle.Init.ClockDivision = 0;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
	{
		while (1)
		{
		}
	}

	/*##-2- Start the TIM Base generation in interrupt mode ####################*/
	/* Start Channel1 */
	if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
	{
		while (1)
		{
		}
	}
}

/*the timer interrupt callback*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/*check if the STemWin is initialize before use STemWin API*/
	if (GUI_Initialized == 1)
	{
		BSP_Pointer_Update();/*handle the touch changes*/
	}
}

/**
 * @brief  Provide the GUI with current state of the touch screen
 * @param  None
 * @retval None
 */
#ifdef __ICCARM__
#pragma optimize=none
#endif
void BSP_Pointer_Update(void)
{
	GUI_PID_STATE TS_State;/*structure which reports the touch state to STemWin*/
	static TS_StateTypeDef prev_state;/*previous touch state from the touch sensor used from BSP package*/
	TS_StateTypeDef ts;/*actual touch state from the touch sensor used from BSP package*/
	uint16_t xDiff, yDiff;/*difference in postitions between touch states*/

	BSP_TS_GetState(&ts);/*read the touch state from touch sensor (BSP API)*/

	TS_State.Pressed = ts.touchDetected; /*store pressed state to STemWin structure*/

	xDiff = (prev_state.touchX[0] > ts.touchX[0]) ?
			(prev_state.touchX[0] - ts.touchX[0]) :
			(ts.touchX[0] - prev_state.touchX[0]);/*calculate x difference*/
	yDiff = (prev_state.touchY[0] > ts.touchY[0]) ?
			(prev_state.touchY[0] - ts.touchY[0]) :
			(ts.touchY[0] - prev_state.touchY[0]);/*calculate y difference*/

	/*check if the touch is pressed*/
	if ((prev_state.touchDetected != ts.touchDetected) || (xDiff > 3)
			|| (yDiff > 3))
	{
		prev_state.touchDetected = ts.touchDetected;

		/*check touch movings*/
		if ((ts.touchX[0] != 0) && (ts.touchY[0] != 0))
		{
			prev_state.touchX[0] = ts.touchX[0];
			prev_state.touchY[0] = ts.touchY[0];
		}

		TS_State.Layer = 0;
		TS_State.x = prev_state.touchX[0];
		TS_State.y = prev_state.touchY[0];

		GUI_TOUCH_StoreStateEx(&TS_State);/*send touch state to STemWin*/
	}
}

#endif

#ifdef STEP_6
/*power button callback*/
static void _cbPower(WM_MESSAGE * pMsg)
{
	WM_HWIN hItem;
	/*decode message type*/
	switch (pMsg->MsgId)
	{
	/*component initialization message*/
	case WM_INIT_DIALOG:
		break;
		/*component repaint message*/
	case WM_PAINT:
		GUI_DrawBitmap(&bmpower, 8, 8);/*draw power button*/
		break;
		/*default callback message*/
	default:
		BUTTON_Callback(pMsg);
	}
}
#endif

#ifdef STEP_6
/*light button callback*/
static void _cbLight(WM_MESSAGE * pMsg)
{
	WM_HWIN hItem;
	/*decode message type*/
	switch (pMsg->MsgId)
	{
	/*component initialization message*/
	case WM_INIT_DIALOG:

		break;
		/*component repaint message*/
	case WM_PAINT:
		GUI_DrawBitmap(&bmlight, 0, 0);/*draw light button*/
		break;
		/*default callback message*/
	default:
		BUTTON_Callback(pMsg);
	}
}
#endif

#ifdef STEP_6
/*mail button callback*/
static void _cbMail(WM_MESSAGE * pMsg)
{
	WM_HWIN hItem;

	switch (pMsg->MsgId)
	{
	/*component initialization message*/
	case WM_INIT_DIALOG:
		break;
		/*component repaint message*/
	case WM_PAINT:
		GUI_DrawBitmap(&bmmail, 0, 0);/*draw mail button*/
		break;
		/*default callback message*/
	default:
		BUTTON_Callback(pMsg);
	}
}
#endif

#ifdef STEP_6
/*home button callback*/
static void _cbHome(WM_MESSAGE * pMsg)
{
	WM_HWIN hItem;
	/*decode message type*/
	switch (pMsg->MsgId)
	{
	/*component initialization message*/
	case WM_INIT_DIALOG:
		break;
		/*component repaint message*/
	case WM_PAINT:
		GUI_DrawBitmap(&bmhome, 0, 0);/*draw home button*/
		break;
		/*default callback message*/
	default:
		BUTTON_Callback(pMsg);
	}
}
#endif

#ifdef STEP_6
/*language button callback*/
static void _cbLanguage(WM_MESSAGE * pMsg)
{
	WM_HWIN hItem;
	/*decode message type*/
	switch (pMsg->MsgId)
	{
	/*component initialization message*/
	case WM_INIT_DIALOG:
		break;
		/*component repaint message*/
	case WM_PAINT:
		if (enfr)
		{
			GUI_DrawBitmap(&bmUK, 13, 7);/*draw uk lanbuage button*/
		}
		else
		{
			GUI_DrawBitmap(&bmFR, 13, 7);/*draw fr language button*/
		}
		break;
		/*default callback message*/
	default:
		BUTTON_Callback(pMsg);
	}
}
#endif

#ifdef STEP_2
/*********************************************************************
 *
 *       _cbFrameWinControl
 *       the dialog box basic callback
 */
static void _cbFrameWinControl(WM_MESSAGE * pMsg)
{
	WM_HWIN hItem;
	int xSize;
	int ySize;
	int NCode;
	int Id;

	/*decode callback event source*/
	switch (pMsg->MsgId)
	{
//  /*if the key was pressed on this item*/
//  case WM_KEY:
//    WM_SendMessage(WM_HBKWIN, pMsg);
//    break;

	/*component initialization message*/
	case WM_INIT_DIALOG:
#ifdef STEP_6

		hItem = WM_GetDialogItem(pMsg->hWin, GUI_ID_POWER);/*get a handle of power button*/
		WM_SetCallback(hItem, _cbPower);/*create a callback for power button*/

		hItem = WM_GetDialogItem(pMsg->hWin, GUI_ID_LIGHT);/*get a handle of light button*/
		WM_SetCallback(hItem, _cbLight);/*create a callback for light button*/

		hItem = WM_GetDialogItem(pMsg->hWin, GUI_ID_MAIL);/*get a handle of mail button*/
		WM_SetCallback(hItem, _cbMail);

		hItem = WM_GetDialogItem(pMsg->hWin, GUI_ID_HOME);/*get a handle of home button*/
		WM_SetCallback(hItem, _cbHome);/*create a callback for home button*/

		hItem = WM_GetDialogItem(pMsg->hWin, GUI_ID_LANGUAGE);/*get a handle of language button*/
		WM_SetCallback(hItem, _cbLanguage);/*create a callback for language button*/
#endif
#ifdef STEP_9
		hItem = WM_GetDialogItem(pMsg->hWin, GUI_ID_TEXT_LABEL);/*get a handle of main label*/
		TEXT_SetTextColor(hItem, 0xFFFFFF);/*set colour of main label */
		TEXT_SetFont(hItem, &GUI_FontArialNarrow30);/*set font of main label*/

		hItem = WM_GetDialogItem(pMsg->hWin, GUI_ID_TEXT_MAIL);/*get a handle of mail label*/
		TEXT_SetTextColor(hItem, 0x8B8D8B);/*set colour of mail label*/

		hItem = WM_GetDialogItem(pMsg->hWin, GUI_ID_TEXT_HOME);/*get a handle of home label*/
		TEXT_SetTextColor(hItem, 0x8B8D8B);/*set colour of home label*/

		hItem = WM_GetDialogItem(pMsg->hWin, GUI_ID_TEXT_LIGHT);/*get a handle of light label*/
		TEXT_SetTextColor(hItem, 0x8B8D8B);/*set colour of light label*/

		hItem = WM_GetDialogItem(pMsg->hWin, GUI_ID_TEXT_TIME);/*get a handle of time label*/
		TEXT_SetTextColor(hItem, 0xCDAAAC);/*set colour of time label*/
		TEXT_SetFont(hItem, &GUI_FontArialNarrow24);/*set font of time label*/

#endif
		break;
#ifdef STEP_5
		/*component repaint message*/
	case WM_PAINT:

		GUI_DrawBitmap(&bmbackground, 0, 0);/*draw background*/
		GUI_DrawBitmap(&bmboard, -7, 64);/*draw board*/

		/*check  if the bulb will be drawn as on or off*/
		if (onoff == 1)
		{
			GUI_DrawBitmap(&bmbulb_on, 329, 74);/*draw bulb_on*/
		}
		else
		{
			GUI_DrawBitmap(&bmbulb_off, 329, 74);/*draw bulb_off*/
		}

		GUI_SetAlpha(191);/*set transparency 75% = 255(100%)-64(25%)*/
		GUI_SetColor(GUI_BLACK);/*set pen colour to black*/
		GUI_FillRect(0, 223, 480, 272);/*draw rectangle for the bottom buttons*/
		GUI_FillRect(207, 95, 271, 159);/*draw rectangle for power button*/
		GUI_SetAlpha(0);/*no transparency*/
		break;
#endif
		/*component message from a child(buttons, labels, ...)*/
	case WM_NOTIFY_PARENT:
		Id = WM_GetId(pMsg->hWinSrc);
		NCode = pMsg->Data.v;
		/*check which child send this message*/
		switch (Id)
		{
#ifdef STEP_7
		case GUI_ID_POWER: /* Notifications sent by 'Button'*/
			/*decode the message which the child sent*/
			switch (NCode)
			{
			/*the button was clicked message*/
			case WM_NOTIFICATION_CLICKED:
				/*chack if the bulb is on or off*/
				if (onoff == 0)
				{
					onoff = 1;/*set bulb on*/
				}
				else
				{
					onoff = 0;/*set bulb off*/
				}
				hItem = WM_GetParent(pMsg->hWinSrc);/*get a handle of parent component(background window)*/
				WM_Invalidate(hItem);/*Force the parent window to repaint*/
				break;
				/*case GUI_ID_NEXT:
				 break;*/
			}
			break;
#endif
#ifdef STEP_7
		case GUI_ID_LANGUAGE:/*notification sent by language button*/
			/*decode the message which the child sent*/
			switch (NCode)
			{
			/*the button was clicked message*/
			case WM_NOTIFICATION_CLICKED:
				/*chack if active language is fr or uk*/
				if (enfr == 0)
				{
					enfr = 1;/*set language as uk*/
				}
				else
				{
					enfr = 0;/*set language to fr*/
				}
				hItem = WM_GetParent(pMsg->hWin);/*get a handle of parent component(button itself)*/
				WM_Invalidate(hItem);/*Force the parent window to repaint*/
				break;
			}
			break;
#endif
		}
		break;
		/*default callback message*/
	default:
		WM_DefaultProc(pMsg);/*if the message type was not decoded in callback we will use default one*/
	}
}
#endif

#ifdef STEP_10
/*the systick callback from hal called each 1ms
 * here we can implement the clock increment
 * */
void HAL_SYSTICK_Callback(void)
{
	miliceconds++;/*increment millisecond variable*/
	/*check if we reach maximum value (1s)*/
	if (miliceconds >= (1000))
	{
		miliceconds = 0;/*reset the millisecond counter*/
		minute++;/*increment minute counter*/
		/*check if we reach 60minutes*/
		if (minute >= 60)
		{
			minute = 0;/*reset minute counter*/
			hour++;/*increment hour counter*/
			/*check if we reached 24 hour*/
			if (hour >= 24)
			{
				hour = 0;/*reset hour counter*/
			}
		}
		/*check if the STemWin is initalized*/
		if (GUI_Initialized)
		{
			WM_HWIN hItem;
			hItem = WM_GetDialogItem(_hDialogWindow, GUI_ID_TEXT_TIME);/*get the time label handle*/
			sprintf(time_buffer, "%02d:%02d", hour, minute);/*convert the minute and hour to string buffer*/
			TEXT_SetText(hItem, time_buffer);/*set new text for the time label*/
			WM_InvalidateWindow(hItem);/*force the label to be repainted*/
		}
	}
}

#endif

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1)
	{
	}
	/* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
