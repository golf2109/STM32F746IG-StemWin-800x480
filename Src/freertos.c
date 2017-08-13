/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include <stdio.h>
#include <string.h>
#include "stm32f7xx_hal.h"
#include "GUI.h"
#include "GUIDEMO.h"
#include "WindowDLG.h"
#include "Window_childDLG.h"
//#include "WindowDLG.c"
//#define ID_TEXT_0 (GUI_ID_USER + 0x02)
uint32_t adc[3];
char text0[64];
char text1[64];
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId GUI_ThreadHandle;
osThreadId my_TaskHandle;
osTimerId TS_TimerHandle;

/* USER CODE BEGIN Variables */
extern	WM_HWIN hWin;
extern	WM_HWIN hWin_child;
TEXT_Handle hTxt0,hTxt1;
extern	void picture_test_irq(void);
extern	void k_TouchUpdate(void);
extern	void read_touch_int(void);
extern	void GUI_TOUCH_Exec_my_old(void);
extern	BUTTON_SKINFLEX_PROPS my_BUTTON_SKINFLEX_PROPS;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void GUIThread(void const * argument);
void myTask(void const * argument);
void TimerCallback(void const * argument);

extern void MX_FATFS_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void k_PeriodicProcesses(void)
{
//  char tmp[16];
  WM_HWIN   hItem; 
  static uint32_t InitTick = 0;
  
//  hItem = WM_GetDialogItem(WM_GetDesktopWindowEx(0), ID_FEATURES_CPU);
  hItem = WM_GetDialogItem(WM_GetDesktopWindowEx(0), 800);	
  
  if(hItem)
  {
    if((WM_IsVisible(hItem)) && ((osKernelSysTick() - InitTick ) > 500))
    {
      InitTick = osKernelSysTick();
      
//      sprintf((char *)tmp , "MCU Load : %d%%",  osGetCPUUsage());
//      TEXT_SetText(hItem, tmp);
      
      WM_InvalidateWindow(hItem);
      WM_Update(hItem);
    }
  }
}
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationTickHook(void);

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
__weak void vApplicationTickHook( void )
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of TS_Timer */
  osTimerDef(TS_Timer, TimerCallback);
  TS_TimerHandle = osTimerCreate(osTimer(TS_Timer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
	osTimerStart(TS_TimerHandle, 100);	
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of GUI_Thread */
  osThreadDef(GUI_Thread, GUIThread, osPriorityHigh, 0, 2048);
  GUI_ThreadHandle = osThreadCreate(osThread(GUI_Thread), NULL);

  /* definition and creation of my_Task */
  osThreadDef(my_Task, myTask, osPriorityNormal, 0, 2048);
  my_TaskHandle = osThreadCreate(osThread(my_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* GUIThread function */
void GUIThread(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN GUIThread */
	
WM_SetCreateFlags(WM_CF_MEMDEV);	
//  //hWin = CreateFramewin();	
//  //CreateFramewin();		
//	
  hWin = CreateWindow();		
////	hWin_child = WM_CreateWindowAsChild(0,0,100,100,hWin,WM_CF_SHOW,0,0);
//	hWin_child = CreateWindow_child();
  hTxt0 = WM_GetDialogItem(hWin, 0x803);		
  hTxt1 = WM_GetDialogItem(hWin, 0x804);		
  /* Infinite loop */
  for(;;)
  {
memset(&text0,0,sizeof(text0));		
		sprintf(text0,"%02d", adc[0]);			


//memset(&text1,0,sizeof(text1));
//		sprintf(text1,"%02d"":""%02d"":""%02d",adc[2],adc[1],adc[0]);			

memset(&text1,0,sizeof(text1));
		sprintf(text1,"%02d"":""%02d"":""%02d",adc[2],adc[1],adc[0]);		
		//sprintf(text1,"%02d"":""%02d",adc[2],adc[1]);		
		
////////    osDelay(1000);
////////taskENTER_CRITICAL();		
////////picture_test_irq();		
////////taskENTER_CRITICAL();
//		TEXT_SetText(hTxt,"ASDFG");		

//    GUI_TOUCH_Exec();             /* Execute Touchscreen support */		
		
		TEXT_SetText(hTxt0,text0);
//    osDelay(10);			
//    GUI_Exec1();		
//    osDelay(10);	
		TEXT_SetText(hTxt1,text1);
//    osDelay(10);			
//		GUI_Exec1();	
//    osDelay(10);	
//    GUI_TOUCH_Exec();             /* Execute Touchscreen support */		
    GUI_Exec();                   /* Execute all GUI jobs ... Return 0 if nothing was done. */
//    GUI_X_ExecIdle();             /* Nothing left to do for the moment ... Idle processing */
		
    osDelay(10);		
//		adc[0]++;

//		TEXT_SetText(hTxt,"ASDFG");				
//		HAL_Delay(1000);					
  }
  /* USER CODE END GUIThread */
}

/* myTask function */
void myTask(void const * argument)
{
  /* USER CODE BEGIN myTask */
  /* Infinite loop */
  for(;;)
  {
//    GUI_TOUCH_Exec();             /* Execute Touchscreen support */				
    osDelay(1000);
		adc[0]++;		
		if(adc[0]>=60){
		adc[0]=0;
		adc[1]++;			
		}
		if(adc[1]>=60){
		adc[0]=0;
		adc[1]=0;
		adc[2]++;
		}
		if(adc[2]>=24){
		adc[0]=0;
		adc[1]=0;
		adc[2]=0;
		}		
  }
  /* USER CODE END myTask */
}

/* TimerCallback function */
void TimerCallback(void const * argument)
{
  /* USER CODE BEGIN TimerCallback */
//	read_touch_int();
GUI_TOUCH_Exec_my_old();	
//  k_TouchUpdate();
  /* USER CODE END TimerCallback */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
