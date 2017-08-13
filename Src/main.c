/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "main.h"
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "crc.h"
#include "dma.h"
#include "dma2d.h"
#include "fatfs.h"
#include "i2c.h"
#include "ltdc.h"
#include "rng.h"
#include "sdmmc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* USER CODE BEGIN Includes */
#include "GUI.h"
#include "GUIDEMO.h"
#include "stdint.h"
//#include "ft5336.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32746g_discovery_ts.h"
//#include "stm32746g_sdram.h"

#include "FramewinDLG.h"
#include "WindowDLG.h"
#include "Window_childDLG.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define REFRESH_COUNT                    ((uint32_t)0x0603)   /* SDRAM refresh counter (100Mhz SD clock) */
#define SDRAM_TIMEOUT                    ((uint32_t)0xFFFF)
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000) 
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200) 
 FMC_SDRAM_CommandTypeDef Command;
 
struct __FILE { int handle; /* Add whatever is needed */ };   //for SWO
 
 
FATFS SDFatFs __attribute__((aligned(4))); /* File system object for SD card logical drive */
FIL MyFile __attribute__((aligned(4))); /* File object */
FIL MyFile1 __attribute__((aligned(4))); /* File object */
FIL MyFile2 __attribute__((aligned(4))); /* File object */
extern char SD_Path[4]; /* SD logical drive path */
FRESULT fresult;
uint8_t _acBuffer[4096] __attribute__((aligned(4)));
uint32_t bytesread = 0;
char str1[30];
uint16_t i,j;

uint8_t* bmp1;
uint8_t sect[4096];

TS_StateTypeDef TS_State;
uint16_t x=0, y=0;
uint32_t tscnt[5]={0};
uint16_t xstart[5]={0}, ystart[5]={0};
uint16_t image_number;
char file_name[]="image000.bmp";
char st_1[]="image000.bmp";
char st_2[]="image000";
char st_3[]=".bmp";


//=================
volatile uint8_t touch_enable = 0;
uint8_t MasterTX[0x10] = {0};
uint8_t SlaveRX[5];
uint8_t uart_RX[0x20];
uint8_t uart_TX[0x60];
uint8_t str[200];
uint8_t touch_receive[0xff];
HAL_StatusTypeDef touch_adr;
uint16_t x_pos[5]={0},y_pos[5]={0};
uint16_t event_flag[5], touch_id[5];
uint16_t x_circle_position[5], y_circle_position[5];
uint8_t  t0,t1,t2,t3;
uint8_t device_mode, gesture_id, number_of_tuchpoint ;
uint8_t test_read[0x20];
uint8_t test_read_00[0x20];

//================
GUI_PID_STATE touch_state;
HAL_StatusTypeDef  touch_send;
extern uint8_t SelLayer;
//FRAMEWIN_Handle hWin;
WM_HWIN hWin;
WM_HWIN hWin_child;
BUTTON_SKINFLEX_PROPS my_BUTTON_SKINFLEX_PROPS;

GUI_FONT XBFFont;
GUI_FONT XBFFont42;
GUI_XBF_DATA XBF_Data;
GUI_XBF_DATA XBF_Data42;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int fputc(int ch, FILE *f) {			//for SWO
ITM_SendChar(ch);//send method for SWV
//uint32_t i=0;
//for(i=0;i<0xFFFF;i++);//waiting method, lower value will stop the SWV	
return(ch);
}

void  SDRAM_Setting(void){	
  __IO uint32_t tmpmrd = 0;
  
  /* Step 1: Configure a clock configuration enable command */
  Command.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = 0;
  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

  /* Step 2: Insert 100 us minimum delay */ 
  /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
  HAL_Delay(1);
    
  /* Step 3: Configure a PALL (precharge all) command */ 
  Command.CommandMode            = FMC_SDRAM_CMD_PALL;
//  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = 0;
  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);  
  
  /* Step 4: Configure an Auto Refresh command */ 
  Command.CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
//  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber      = 8;
  Command.ModeRegisterDefinition = 0;
  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);
  
  /* Step 5: Program the external memory mode register */
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |\
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |\
                     SDRAM_MODEREG_CAS_LATENCY_3           |\
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |\
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;
  
  Command.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
//  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = tmpmrd;
  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);
  
  /* Step 6: Set the refresh rate counter */
  /* Set the device refresh rate */
  HAL_SDRAM_ProgramRefreshRate(&hsdram1, REFRESH_COUNT); 
}

int APP_GetData(void * p, const U8 **ppData, unsigned NumBytesReq, U32 Off){
	//You must create an pointer on structure
	FIL *phFile;
	//and then initialize the pointer value is passed to the function APP_GetData
	phFile=(FIL*) p;
	//And then use this pointer to your function
	f_lseek(phFile,Off);
	fresult=f_read(phFile,_acBuffer,NumBytesReq,(void*)&bytesread);
	*ppData = _acBuffer;
	return bytesread;
}


void picture_test(void){
		
	for(image_number = 1; image_number <= 22;image_number++){
		sprintf(st_2,"image%03d",image_number);
		sprintf(file_name, "%s%s",st_2,st_3 );			
		fresult=f_open(&MyFile,file_name, FA_READ);
//		GUI_MULTIBUF_BeginEx(0);	
		GUI_BMP_DrawEx(APP_GetData, &MyFile,0,0); // Draw image
//		GUI_MULTIBUF_EndEx(0);
		f_close(&MyFile);
		HAL_Delay(1000);	
	}	
}


static int _cbGetData300(U32 Off, U16 NumBytes, void * pVoid, void * pBuffer) {
uint32_t  br;
MyFile1 = *(FIL *)pVoid;
fresult = f_lseek(&MyFile1,Off);
if (fresult != FR_OK) {
return 1; /* Error */
}
fresult=f_read(&MyFile1,pBuffer,NumBytes,&br);
if (fresult != FR_OK) {
return 1; /* Error */
}
if (br != NumBytes) {
return 1; /* Error */
}
return 0; /* Ok */
}


void readFontfromXBFSD300(void){
GUI_FONTINFO fontinfo;
GUI_CHARINFO_EXT charinfo;
fresult = f_open(&MyFile1, "R300.xbf", FA_OPEN_EXISTING | FA_READ);
GUI_XBF_CreateFont(&XBFFont, /* Pointer to GUI_FONT structure in RAM */
&XBF_Data, /* Pointer to GUI_XBF_DATA structure in RAM */
GUI_XBF_TYPE_PROP, /* Font type to be created */
_cbGetData300, /* Pointer to callback function */
&MyFile1); /* Pointer to be passed to GetData function */
GUI_XBF__GetFontInfo(&XBFFont,&fontinfo);
GUI_XBF__GetCharInfo(0x4E2D,&charinfo);

//GUI_SetFont(&XBFFont);		
//////GUI_DispStringAt("12",250,150);		
//fresult = f_close(&MyFile1);
}



static int _cbGetData42(U32 Off, U16 NumBytes, void * pVoid, void * pBuffer) {
uint32_t  br;
MyFile2 = *(FIL *)pVoid;
fresult = f_lseek(&MyFile2,Off);
if (fresult != FR_OK) {
return 1; /* Error */
}
fresult=f_read(&MyFile2,pBuffer,NumBytes,&br);
if (fresult != FR_OK) {
return 1; /* Error */
}
if (br != NumBytes) {
return 1; /* Error */
}
return 0; /* Ok */
}


void readFontfromXBFSD42(void){
GUI_FONTINFO fontinfo;
GUI_CHARINFO_EXT charinfo;
fresult = f_open(&MyFile2, "R42.xbf", FA_OPEN_EXISTING | FA_READ);
GUI_XBF_CreateFont(&XBFFont42, /* Pointer to GUI_FONT structure in RAM */
&XBF_Data42, /* Pointer to GUI_XBF_DATA structure in RAM */
GUI_XBF_TYPE_PROP, /* Font type to be created */
_cbGetData42, /* Pointer to callback function */
&MyFile2); /* Pointer to be passed to GetData function */
GUI_XBF__GetFontInfo(&XBFFont42,&fontinfo);
GUI_XBF__GetCharInfo(0x4E2D,&charinfo);

//GUI_SetFont(&XBFFont42);		
//////GUI_DispStringAt("12",250,150);		
//fresult = f_close(&MyFile2);
}

void picture_test_irq(void){
uint32_t prim;	
	
	for(image_number = 1; image_number <= 2;image_number++){
		sprintf(st_2,"image%03d",image_number);
		sprintf(file_name, "%s%s",st_2,st_3 );			
		fresult=f_open(&MyFile,file_name, FA_READ);
//		GUI_MULTIBUF_BeginEx(0);
			prim = __get_PRIMASK();
			__disable_irq();		
		GUI_BMP_DrawEx(APP_GetData, &MyFile,0,0); // Draw image
		if (!prim) {
			__enable_irq();
    }		
//		GUI_MULTIBUF_EndEx(0);
		f_close(&MyFile);
		HAL_Delay(1000);	
	}	
					
	for(image_number = 1; image_number <= 22;image_number++){
		sprintf(st_2,"image%03d",image_number);
		sprintf(file_name, "%s%s",st_2,st_3 );			
		fresult=f_open(&MyFile,file_name, FA_READ);
		GUI_MULTIBUF_BeginEx(0);
			prim = __get_PRIMASK();
			__disable_irq();		
		GUI_BMP_DrawEx(APP_GetData, &MyFile,0,0); // Draw image
		if (!prim) {
			__enable_irq();
    }		
		GUI_MULTIBUF_EndEx(0);
		f_close(&MyFile);
		HAL_Delay(1000);	
	}				
}
uint8_t CheckForUserInput(void){
 if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) != GPIO_PIN_RESET)
 {
   HAL_Delay(10);
   while (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) != GPIO_PIN_RESET);
   return 1 ;
 }
 return 0;
}
void k_TouchUpdate(void){
  static GUI_PID_STATE TS_State = {0, 0, 0, 0};
  __IO TS_StateTypeDef  ts;
  uint16_t xDiff, yDiff;  

  BSP_TS_GetState((TS_StateTypeDef *)&ts);

  if((ts.touchX[0] >= LCD_GetXSize()) ||(ts.touchY[0] >= LCD_GetYSize()) ) 
  {
    ts.touchX[0] = 0;
    ts.touchY[0] = 0;
    ts.touchDetected =0;
  }

  xDiff = (TS_State.x > ts.touchX[0]) ? (TS_State.x - ts.touchX[0]) : (ts.touchX[0] - TS_State.x);
  yDiff = (TS_State.y > ts.touchY[0]) ? (TS_State.y - ts.touchY[0]) : (ts.touchY[0] - TS_State.y);
  
  
  if((TS_State.Pressed != ts.touchDetected ) ||
     (xDiff > 30 )||
      (yDiff > 30))
  {
    TS_State.Pressed = ts.touchDetected;
//    TS_State.Layer = SelLayer;
//    TS_State.Layer = 1;		
    if(ts.touchDetected) 
    {
      TS_State.x = ts.touchX[0];
      TS_State.y = ts.touchY[0];
      GUI_TOUCH_StoreStateEx(&TS_State);
    }
    else
    {
      GUI_TOUCH_StoreStateEx(&TS_State);
      TS_State.x = 0;
      TS_State.y = 0;
    }
  }
}




void touch_test(void){
	while (CheckForUserInput()==0)
	{
		BSP_TS_GetState(&TS_State);
		if(TS_State.touchDetected)
		{
			x = TS_State.touchX[0];
			y = TS_State.touchY[0];
//			TFT_SetFont(&Font20);
//			TFT_SetTextColor(LCD_COLOR_CYAN);
//			TFT_SetBackColor(LCD_COLOR_BLACK);
			GUI_SetFont(&GUI_Font8x16);
			if(tscnt[0]>0)
			{
//				TFT_DrawLine(xstart[0],ystart[0],x,y,LCD_COLOR_RED);
			  GUI_DrawLine(xstart[0],ystart[0],x,y);				
			}
			else
			{
//				TFT_DrawPixel(x,y,LCD_COLOR_RED);
				GUI_DrawPixel(x,y);
			}
			xstart[0]=x; ystart[0]=y;
			tscnt[0]++;
			sprintf(str1,"1: x=%03d; y=%03d",x,y);
//			TFT_DisplayString(14, 50, (uint8_t *)str1, LEFT_MODE);
			GUI_DispStringAt(str1, 14, 50);
			if (TS_State.touchDetected >= 2)
			{
				x = TS_State.touchX[1];
				y = TS_State.touchY[1];
				if(tscnt[1]>0)
				{
//				  TFT_DrawLine(xstart[1],ystart[1],x,y,LCD_COLOR_GREEN);
					GUI_DrawLine(xstart[1],ystart[1],x,y);
				}
				else
				{
//				  TFT_DrawPixel(x,y,LCD_COLOR_GREEN);
				GUI_DrawPixel(x,y);					
				}
				xstart[1]=x; ystart[1]=y;
				tscnt[1]++;
				sprintf(str1, "2: x=%03d; y=%03d", x, y);
//				TFT_DisplayString(14, 80, (uint8_t *)str1, LEFT_MODE);
			GUI_DispStringAt(str1, 14, 80);				
			}
			else
			{
//				TFT_FillRectangle(14,80,250,109,LCD_COLOR_BLACK);
				GUI_DrawRect(14,80,250,109);
			}
			if (TS_State.touchDetected >= 3)
			{
				x = TS_State.touchX[2];
				y = TS_State.touchY[2];
				if(tscnt[2]>0)
				{
//				  TFT_DrawLine(xstart[2],ystart[2],x,y,LCD_COLOR_YELLOW);
					GUI_DrawLine(xstart[2],ystart[2],x,y);					
				}
				else
				{
//				  TFT_DrawPixel(x,y,LCD_COLOR_YELLOW);
				GUI_DrawPixel(x,y);					
				}
				xstart[2]=x; ystart[2]=y;
				tscnt[2]++;
				sprintf(str1, "3: x=%03d; y=%03d", x, y);
//				TFT_DisplayString(14, 110, (uint8_t *)str1, LEFT_MODE);
			GUI_DispStringAt(str1, 14, 110);				
			}
			else
			{
//				TFT_FillRectangle(14,110,250,139,LCD_COLOR_BLACK);
				GUI_DrawRect(14,110,250,139);				
			}
			if (TS_State.touchDetected >= 4)
			{
				x = TS_State.touchX[3];
				y = TS_State.touchY[3];
				if(tscnt[3]>0)
				{
//				  TFT_DrawLine(xstart[3],ystart[3],x,y,LCD_COLOR_ORANGE);
					GUI_DrawLine(xstart[3],ystart[3],x,y);						
				}
				else
				{
//				  TFT_DrawPixel(x,y,LCD_COLOR_ORANGE);
				GUI_DrawPixel(x,y);					
				}
				xstart[3]=x; ystart[3]=y;
				tscnt[3]++;
				sprintf(str1, "4: x=%03d; y=%03d", x, y);
//				TFT_DisplayString(14, 140, (uint8_t *)str1, LEFT_MODE);
			GUI_DispStringAt(str1, 14, 140);				
			}
			else
			{
//				TFT_FillRectangle(14,140,250,169,LCD_COLOR_BLACK);
				GUI_DrawRect(14,140,250,169);				
			}
			if (TS_State.touchDetected >= 5)
			{
				x = TS_State.touchX[4];
				y = TS_State.touchY[4];
				if(tscnt[4]>0)
				{
//				  TFT_DrawLine(xstart[4],ystart[4],x,y,LCD_COLOR_BLUE);
					GUI_DrawLine(xstart[4],ystart[4],x,y);						
				}
				else
				{
//				  TFT_DrawPixel(x,y,LCD_COLOR_BLUE);
				GUI_DrawPixel(x,y);					
				}
				xstart[4]=x; ystart[4]=y;
				tscnt[4]++;
				sprintf(str1, "5: x=%03d; y=%03d", x, y);
//				TFT_DisplayString(14, 170, (uint8_t *)str1, LEFT_MODE);
			}
			else
			{
//				TFT_FillRectangle(14,170,250,199,LCD_COLOR_BLACK);
				GUI_DrawRect(14,170,250,199);				
			}
		}
		else
		{
			tscnt[0]=0;tscnt[1]=0;tscnt[2]=0;tscnt[3]=0;tscnt[4]=0;
		}
		HAL_Delay(10);
	}
}

uint32_t OpenBMP(uint8_t *ptr, const char* fname)
{
	uint32_t ind=0,sz=0,i1=0,ind1=0;
	static uint32_t bmp_addr;
	if(f_open(&MyFile,fname,FA_READ)!=FR_OK)
	{
//		  TFT_FillScreen(0xFF00FF00); //в случае неудачи окрасим экран в красный цвет
	}
	else
	{
		if(f_read(&MyFile,sect,30,(UINT *)&bytesread)!=FR_OK)
		{
			Error_Handler();
		}
		else
		{
			bmp_addr=(uint32_t)sect;
			/*Get bitmap size*/
			sz=*(uint16_t*)(bmp_addr + 2);
			sz|=(*(uint16_t*)(bmp_addr + 4))<<16;
			/*Get bitmap data address offset*/
			ind=*(uint16_t*)(bmp_addr + 10);
			ind|=(*(uint16_t*)(bmp_addr + 12))<<16;
			f_close(&MyFile);
			f_open(&MyFile,fname,FA_READ);
			ind=0;
			do
			{
				if(sz<512)
				{
					i1=sz;
				}
				else
				{
					i1=512;
				}
				sz-=i1;
				f_lseek(&MyFile,ind1);
				f_read(&MyFile,sect,i1,(UINT *)&bytesread);
				memcpy((void*)(bmp1+ind1),(void*)sect,i1);
				ind1+=i1;
			}
			while(sz>0);
			f_close(&MyFile);
		}
		ind1=0;
	}
	return 0;
}

uint8_t hex_char (uint8_t dig){
uint8_t q;
q = 0xf & dig;
if (q <=0x9){q=0x30|q;}
else{
  switch(q)
  {
    case 0x0a:
      q=0x41;
    break;
    case 0x0b:
      q=0x42;
    break;
    case 0x0c:
      q=0x43;
    break;
    case 0x0d:
      q=0x44;
    break;
    case 0x0e:
      q=0x45;
    break;  
    case 0x0f:
      q=0x46;
    break;     
  }
}  
return q;
}

void	my_touch_test(void){
if(touch_enable ==1){
  
device_mode = touch_receive[0x00]; 
gesture_id = touch_receive[0x01];
number_of_tuchpoint = touch_receive[0x02];    
 
y_circle_position[0]=800*(256*(0x0F & (touch_receive[0x03])) + touch_receive[0x04])/1791;
x_circle_position[0]=480*(256*(0x0F & (touch_receive[0x05])) + touch_receive[0x06])/1024;
y_circle_position[1]=800*(256*(0x0F & (touch_receive[0x09])) + touch_receive[0x0a])/1791;
x_circle_position[1]=480*(256*(0x0F & (touch_receive[0x0b])) + touch_receive[0x0c])/1024;  
y_circle_position[2]=800*(256*(0x0F & (touch_receive[0x0f])) + touch_receive[0x10])/1791;
x_circle_position[2]=480*(256*(0x0F & (touch_receive[0x11])) + touch_receive[0x12])/1024;  
y_circle_position[3]=800*(256*(0x0F & (touch_receive[0x15])) + touch_receive[0x16])/1791;
x_circle_position[3]=480*(256*(0x0F & (touch_receive[0x17])) + touch_receive[0x18])/1024;  
y_circle_position[4]=800*(256*(0x0F & (touch_receive[0x1b])) + touch_receive[0x1c])/1791;
x_circle_position[4]=480*(256*(0x0F & (touch_receive[0x1d])) + touch_receive[0x1e])/1024;  

y_pos[0]=256*(0x0F & (touch_receive[0x03])) + touch_receive[0x04];
x_pos[0]=256*(0x0F & (touch_receive[0x05])) + touch_receive[0x06];
y_pos[1]=256*(0x0F & (touch_receive[0x09])) + touch_receive[0x0a];
x_pos[1]=256*(0x0F & (touch_receive[0x0b])) + touch_receive[0x0c];  
y_pos[2]=256*(0x0F & (touch_receive[0x0f])) + touch_receive[0x10];
x_pos[2]=256*(0x0F & (touch_receive[0x11])) + touch_receive[0x12];  
y_pos[3]=256*(0x0F & (touch_receive[0x15])) + touch_receive[0x16];
x_pos[3]=256*(0x0F & (touch_receive[0x17])) + touch_receive[0x18];  
y_pos[4]=256*(0x0F & (touch_receive[0x1b])) + touch_receive[0x1c];
x_pos[4]=256*(0x0F & (touch_receive[0x1d])) + touch_receive[0x1e]; 

//================================================================
t0 =(hex_char(device_mode >> 12));
uart_TX[0] = t0;
t1 =(hex_char(device_mode >>  8));
uart_TX[1] = t1;
t2 =(hex_char(device_mode >>  4));    
uart_TX[2] = t2;
t3 =(hex_char(device_mode >>  0)); 
uart_TX[3] = t3;
uart_TX[4] = 0x2c;;
//TFT_Draw_Char(210,10,WHITE,BLACK,(const uint8_t*) font8x8, t0 ,2);
GUI_DispCharAt(t0,210,10);
//TFT_Draw_Char(230,10,WHITE,BLACK,(const uint8_t*) font8x8, t1 ,2);
GUI_DispCharAt(t1,230,10);
//TFT_Draw_Char(250,10,WHITE,BLACK,(const uint8_t*) font8x8, t2 ,2);    
GUI_DispCharAt(t2,250,10);
//TFT_Draw_Char(270,10,WHITE,BLACK,(const uint8_t*) font8x8, t3 ,2); 
GUI_DispCharAt(t3,270,10);

//================================================================

t0 =(hex_char(gesture_id >> 12));
uart_TX[5] = t0;
t1 =(hex_char(gesture_id >>  8));
uart_TX[6] = t1;
t2 =(hex_char(gesture_id >>  4));    
uart_TX[7] = t2;
t3 =(hex_char(gesture_id >>  0)); 
uart_TX[8] = t3;
uart_TX[9] = 0x2c;
//TFT_Draw_Char(310,10,WHITE,BLACK,(const uint8_t*) font8x8, t0 ,2);
GUI_DispCharAt(t0,310,10);
//TFT_Draw_Char(330,10,WHITE,BLACK,(const uint8_t*) font8x8, t1 ,2);
GUI_DispCharAt(t1,330,10);
//TFT_Draw_Char(350,10,WHITE,BLACK,(const uint8_t*) font8x8, t2 ,2);    
GUI_DispCharAt(t2,350,10);
//TFT_Draw_Char(370,10,WHITE,BLACK,(const uint8_t*) font8x8, t3 ,2);
GUI_DispCharAt(t3,370,10);

//================================================================
t0 =(hex_char(number_of_tuchpoint >> 12));
uart_TX[0x0a] = t0;
t1 =(hex_char(number_of_tuchpoint >>  8));
uart_TX[0x0b] = t1;
t2 =(hex_char(number_of_tuchpoint >>  4));    
uart_TX[0x0c] = t2;
t3 =(hex_char(number_of_tuchpoint >>  0)); 
uart_TX[0x0d] = t3;
uart_TX[0x0e] = 0x2c;
//TFT_Draw_Char(410,10,WHITE,BLACK,(const uint8_t*) font8x8, t0 ,2);
GUI_DispCharAt(t0,410,10);
//TFT_Draw_Char(430,10,WHITE,BLACK,(const uint8_t*) font8x8, t1 ,2);
GUI_DispCharAt(t1,430,10);
//TFT_Draw_Char(450,10,WHITE,BLACK,(const uint8_t*) font8x8, t2 ,2);    
GUI_DispCharAt(t2,450,10);
//TFT_Draw_Char(470,10,WHITE,BLACK,(const uint8_t*) font8x8, t3 ,2); 
GUI_DispCharAt(t3,470,10);

//=================================================================
///////////////////////////////////////////////////////////////////
t0 =(hex_char((y_pos[0]) >> 12));
uart_TX[0x0f] = t0;
t1 =(hex_char((y_pos[0]) >>  8));
uart_TX[0x10] = t1;
t2 =(hex_char((y_pos[0]) >>  4));   
uart_TX[0x11] = t2;
t3 =(hex_char((y_pos[0]) >>  0));    
uart_TX[0x12] = t3;
uart_TX[0x13] = 0x2c;
//TFT_Draw_Char(10,30,WHITE,BLACK,(const uint8_t*) font8x8, t0 ,2);
GUI_DispCharAt(t0,10,30);
//TFT_Draw_Char(30,30,WHITE,BLACK,(const uint8_t*) font8x8, t1 ,2);
GUI_DispCharAt(t1,30,30);
//TFT_Draw_Char(50,30,WHITE,BLACK,(const uint8_t*) font8x8, t2 ,2);    
GUI_DispCharAt(t2,50,30);
//TFT_Draw_Char(70,30,WHITE,BLACK,(const uint8_t*) font8x8, t3 ,2);     
GUI_DispCharAt(t3,70,30);
t0 =(hex_char((x_pos[0]) >> 12));
uart_TX[0x14] = t0;
t1 =(hex_char((x_pos[0]) >>  8));
uart_TX[0x15] = t1;
t2 =(hex_char((x_pos[0]) >>  4));    
uart_TX[0x16] = t2;
t3 =(hex_char((x_pos[0]) >>  0));    
uart_TX[0x17] = t3;
uart_TX[0x18] = 0x2c;
//TFT_Draw_Char(110,30,WHITE,BLACK,(const uint8_t*) font8x8, t0 ,2);
GUI_DispCharAt(t0,110,30);
//TFT_Draw_Char(130,30,WHITE,BLACK,(const uint8_t*) font8x8, t1 ,2);
GUI_DispCharAt(t1,130,30);
//TFT_Draw_Char(150,30,WHITE,BLACK,(const uint8_t*) font8x8, t2 ,2);    
GUI_DispCharAt(t2,150,30);
//TFT_Draw_Char(170,30,WHITE,BLACK,(const uint8_t*) font8x8, t3 ,2);      
GUI_DispCharAt(t3,170,30);
//
event_flag[0] = touch_receive[0x03];
touch_id[0] = touch_receive[0x05];
t0 =(hex_char((event_flag[0]) >> 6));
uart_TX[0x19] = t0;
t1 =(hex_char((touch_id[0]) >>  4));   
uart_TX[0x1a] = t1;
uart_TX[0x1b] = 0x2c;
//TFT_Draw_Char(270,30,WHITE,BLACK,(const uint8_t*) font8x8, t0 ,2);
GUI_DispCharAt(t0,270,30);
//TFT_Draw_Char(370,30,WHITE,BLACK,(const uint8_t*) font8x8, t1 ,2);
GUI_DispCharAt(t1,370,30);
//----------------------------------------------------------------
t0 =(hex_char((y_pos[1]) >> 12));
uart_TX[0x1c] = t0;
t1 =(hex_char((y_pos[1]) >>  8));
uart_TX[0x1d] = t1;
t2 =(hex_char((y_pos[1]) >>  4));    
uart_TX[0x1e] = t2;
t3 =(hex_char((y_pos[1]) >>  0));    
uart_TX[0x1f] = t3;
uart_TX[0x20] = 0x2c;
//TFT_Draw_Char(10,50,WHITE,BLACK,(const uint8_t*) font8x8, t0 ,2);
GUI_DispCharAt(t0,10,50);
//TFT_Draw_Char(30,50,WHITE,BLACK,(const uint8_t*) font8x8, t1 ,2);
GUI_DispCharAt(t1,30,50);
//TFT_Draw_Char(50,50,WHITE,BLACK,(const uint8_t*) font8x8, t2 ,2);    
GUI_DispCharAt(t2,50,50);
//TFT_Draw_Char(70,50,WHITE,BLACK,(const uint8_t*) font8x8, t3 ,2);     
GUI_DispCharAt(t3,70,50);
t0 =(hex_char((x_pos[1]) >> 12));
uart_TX[0x21] = t0;
t1 =(hex_char((x_pos[1]) >>  8));
uart_TX[0x22] = t1;
t2 =(hex_char((x_pos[1]) >>  4));    
uart_TX[0x23] = t2;
t3 =(hex_char((x_pos[1]) >>  0));    
uart_TX[0x24] = t3;
uart_TX[0x25] = 0x2c;
//TFT_Draw_Char(110,50,WHITE,BLACK,(const uint8_t*) font8x8, t0 ,2);
GUI_DispCharAt(t0,110,50);
//TFT_Draw_Char(130,50,WHITE,BLACK,(const uint8_t*) font8x8, t1 ,2);
GUI_DispCharAt(t1,130,50);
//TFT_Draw_Char(150,50,WHITE,BLACK,(const uint8_t*) font8x8, t2 ,2);    
GUI_DispCharAt(t2,150,50);
//TFT_Draw_Char(170,50,WHITE,BLACK,(const uint8_t*) font8x8, t3 ,2);   
GUI_DispCharAt(t3,170,50);
//
event_flag[1] = touch_receive[0x09];
touch_id[1] = touch_receive[0x0b];
t0 =(hex_char((event_flag[1]) >> 6));
uart_TX[0x26] = t0;
t1 =(hex_char((touch_id[1]) >>  4));   
uart_TX[0x27] = t1;
uart_TX[0x28] = 0x2c;
//TFT_Draw_Char(270,50,WHITE,BLACK,(const uint8_t*) font8x8, t0 ,2);
GUI_DispCharAt(t0,270,50);
//TFT_Draw_Char(370,50,WHITE,BLACK,(const uint8_t*) font8x8, t1 ,2);
GUI_DispCharAt(t1,370,50);
//----------------------------------------------------------------
t0 =(hex_char((y_pos[2]) >> 12));
uart_TX[0x29] = t0;
t1 =(hex_char((y_pos[2]) >>  8));
uart_TX[0x2a] = t1;
t2 =(hex_char((y_pos[2]) >>  4));    
uart_TX[0x2b] = t2;
t3 =(hex_char((y_pos[2]) >>  0));    
uart_TX[0x2c] = t3;
uart_TX[0x2d] = 0x2c;
//TFT_Draw_Char(10,70,WHITE,BLACK,(const uint8_t*) font8x8, t0 ,2);
GUI_DispCharAt(t0,10,70);
//TFT_Draw_Char(30,70,WHITE,BLACK,(const uint8_t*) font8x8, t1 ,2);
GUI_DispCharAt(t1,30,70);
//TFT_Draw_Char(50,70,WHITE,BLACK,(const uint8_t*) font8x8, t2 ,2);    
GUI_DispCharAt(t2,50,70);
//TFT_Draw_Char(70,70,WHITE,BLACK,(const uint8_t*) font8x8, t3 ,2);     
GUI_DispCharAt(t3,70,70);
t0 =(hex_char((x_pos[2]) >> 12));
uart_TX[0x2e] = t0;
t1 =(hex_char((x_pos[2]) >>  8));
uart_TX[0x2f] = t1;
t2 =(hex_char((x_pos[2]) >>  4));    
uart_TX[0x30] = t2;
t3 =(hex_char((x_pos[2]) >>  0));    
uart_TX[0x31] = t3;
uart_TX[0x32] = 0x2c;
//TFT_Draw_Char(110,70,WHITE,BLACK,(const uint8_t*) font8x8, t0 ,2);
GUI_DispCharAt(t0,110,70);
//TFT_Draw_Char(130,70,WHITE,BLACK,(const uint8_t*) font8x8, t1 ,2);
GUI_DispCharAt(t1,130,70);
//TFT_Draw_Char(150,70,WHITE,BLACK,(const uint8_t*) font8x8, t2 ,2);    
GUI_DispCharAt(t2,150,70);
//TFT_Draw_Char(170,70,WHITE,BLACK,(const uint8_t*) font8x8, t3 ,2); 
GUI_DispCharAt(t3,170,70);
//
event_flag[2] = touch_receive[0x0f];
touch_id[2] = touch_receive[0x11];
t0 =(hex_char((event_flag[2]) >> 6));
uart_TX[0x33] = t0;
t1 =(hex_char((touch_id[2]) >>  4));   
uart_TX[0x34] = t1;
uart_TX[0x35] = 0x2c;
//TFT_Draw_Char(270,70,WHITE,BLACK,(const uint8_t*) font8x8, t0 ,2);
GUI_DispCharAt(t0,270,70);
//TFT_Draw_Char(370,70,WHITE,BLACK,(const uint8_t*) font8x8, t1 ,2);
GUI_DispCharAt(t1,370,70);
//----------------------------------------------------------------

t0 =(hex_char((y_pos[3]) >> 12));
uart_TX[0x36] = t0;
t1 =(hex_char((y_pos[3]) >>  8));
uart_TX[0x37] = t1;
t2 =(hex_char((y_pos[3]) >>  4));    
uart_TX[0x38] = t2;
t3 =(hex_char((y_pos[3]) >>  0));    
uart_TX[0x39] = t3;
uart_TX[0x3a] = 0x2c;
//TFT_Draw_Char(10,90,WHITE,BLACK,(const uint8_t*) font8x8, t0 ,2);
GUI_DispCharAt(t0,10,90);
//TFT_Draw_Char(30,90,WHITE,BLACK,(const uint8_t*) font8x8, t1 ,2);
GUI_DispCharAt(t1,30,90);
//TFT_Draw_Char(50,90,WHITE,BLACK,(const uint8_t*) font8x8, t2 ,2);    
GUI_DispCharAt(t2,50,90);
//TFT_Draw_Char(70,90,WHITE,BLACK,(const uint8_t*) font8x8, t3 ,2);     
GUI_DispCharAt(t3,70,90);
t0 =(hex_char((x_pos[3]) >> 12));
uart_TX[0x3b] = t0;
t1 =(hex_char((x_pos[3]) >>  8));
uart_TX[0x3c] = t1;
t2 =(hex_char((x_pos[3]) >>  4));    
uart_TX[0x3d] = t2;
t3 =(hex_char((x_pos[3]) >>  0));    
uart_TX[0x3e] = t3;
uart_TX[0x3f] = 0x2c;
//TFT_Draw_Char(110,90,WHITE,BLACK,(const uint8_t*) font8x8, t0 ,2);
GUI_DispCharAt(t0,110,90);
//TFT_Draw_Char(130,90,WHITE,BLACK,(const uint8_t*) font8x8, t1 ,2);
GUI_DispCharAt(t1,130,90);
//TFT_Draw_Char(150,90,WHITE,BLACK,(const uint8_t*) font8x8, t2 ,2);    
GUI_DispCharAt(t2,150,90);
//TFT_Draw_Char(170,90,WHITE,BLACK,(const uint8_t*) font8x8, t3 ,2);
GUI_DispCharAt(t3,170,90);
//
event_flag[3] = touch_receive[0x15];
touch_id[3] = touch_receive[0x17];
t0 =(hex_char((event_flag[3]) >> 6));
uart_TX[0x40] = t0;
t1 =(hex_char((touch_id[3]) >>  4));   
uart_TX[0x41] = t1;
uart_TX[0x42] = 0x2c;
//TFT_Draw_Char(270,90,WHITE,BLACK,(const uint8_t*) font8x8, t0 ,2);
GUI_DispCharAt(t0,270,90);
//TFT_Draw_Char(370,90,WHITE,BLACK,(const uint8_t*) font8x8, t1 ,2);
GUI_DispCharAt(t1,370,90);
//----------------------------------------------------------------

t0 =(hex_char((y_pos[4]) >> 12));
uart_TX[0x43] = t0;
t1 =(hex_char((y_pos[4]) >>  8));
uart_TX[0x44] = t1;
t2 =(hex_char((y_pos[4]) >>  4));    
uart_TX[0x45] = t2;
t3 =(hex_char((y_pos[4]) >>  0));    
uart_TX[0x46] = t3;
uart_TX[0x47] = 0x2c;
//TFT_Draw_Char(10,110,WHITE,BLACK,(const uint8_t*) font8x8, t0 ,2);
GUI_DispCharAt(t0,10,110);
//TFT_Draw_Char(30,110,WHITE,BLACK,(const uint8_t*) font8x8, t1 ,2);
GUI_DispCharAt(t1,30,110);
//TFT_Draw_Char(50,110,WHITE,BLACK,(const uint8_t*) font8x8, t2 ,2);    
GUI_DispCharAt(t2,50,110);
//TFT_Draw_Char(70,110,WHITE,BLACK,(const uint8_t*) font8x8, t3 ,2);     
GUI_DispCharAt(t3,70,110);
t0 =(hex_char((x_pos[4]) >> 12));
uart_TX[0x48] = t0;
t1 =(hex_char((x_pos[4]) >>  8));
uart_TX[0x49] = t1;
t2 =(hex_char((x_pos[4]) >>  4));    
uart_TX[0x4a] = t2;
t3 =(hex_char((x_pos[4]) >>  0));    
uart_TX[0x4b] = t3;
uart_TX[0x4c] = 0x2c;
//TFT_Draw_Char(110,110,WHITE,BLACK,(const uint8_t*) font8x8, t0 ,2);
GUI_DispCharAt(t0,110,110);
//TFT_Draw_Char(130,110,WHITE,BLACK,(const uint8_t*) font8x8, t1 ,2);
GUI_DispCharAt(t1,130,110);
//TFT_Draw_Char(150,110,WHITE,BLACK,(const uint8_t*) font8x8, t2 ,2);    
GUI_DispCharAt(t2,150,110);
//TFT_Draw_Char(170,110,WHITE,BLACK,(const uint8_t*) font8x8, t3 ,2); 
GUI_DispCharAt(t3,170,110);
//
event_flag[4] = touch_receive[0x1b];
touch_id[4] = touch_receive[0x1d];
t0 =(hex_char((event_flag[4]) >> 6));
uart_TX[0x4d] = t0;
t1 =(hex_char((touch_id[4]) >>  4));   
uart_TX[0x4e] = t1;
uart_TX[0x4f] = 0x2c;
//TFT_Draw_Char(270,110,WHITE,BLACK,(const uint8_t*) font8x8, t0 ,2);
GUI_DispCharAt(t0,270,110);
//TFT_Draw_Char(370,110,WHITE,BLACK,(const uint8_t*) font8x8, t1 ,2);
GUI_DispCharAt(t1,370,110);
//----------------------------------------------------------------
//GUI_GotoXY(100,100);    
//GUI_DispChar(t0); 
//GUI_GotoXY(120,100);     
//GUI_DispChar(t1);     
//GUI_GotoXY(140,100);    
//GUI_DispChar(t2); 
//GUI_GotoXY(160,100);     
//GUI_DispChar(t3);        
uart_TX[0x50] = 0x0d;
uart_TX[0x51] = 0x0a;
   if((x_circle_position[0] < 10 ) && (y_circle_position[0] < 10)){
//  Lcd_ClearScreen(BLACK);      
	GUI_Clear();		 
  }
   else{
//TFT_Draw_Circle(y_circle_position[0], x_circle_position[0], 5);     
GUI_DrawCircle(y_circle_position[0], x_circle_position[0], 5);    		 

///////**
//////  * @brief  Sends an amount of data in non blocking mode. 
//////  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
//////  *                the configuration information for the specified UART module.
//////  * @param  pData: Pointer to data buffer
//////  * @param  Size: Amount of data to be sent
//////  * @retval HAL status
//////  */
//////HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)     
//HAL_UART_Transmit_DMA(&huart2, touch_receive, 0x20);
     

/////////**
////////* @brief  Starts the DMA Transfer.
////////* @param  hdma      : pointer to a DMA_HandleTypeDef structure that contains
////////*                     the configuration information for the specified DMA Stream.
////////* @param  SrcAddress: The source memory Buffer address
////////* @param  DstAddress: The destination memory Buffer address
////////* @param  DataLength: The length of data to be transferred from source to destination
////////* @retval HAL status
////////*/
////////HAL_StatusTypeDef HAL_DMA_Start(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)     
////HAL_DMA_Start(&hdma_usart2_tx, touch_receive, huart2, 0x20)    
   }
////////MasterTX[0]=0x00;
////////MasterTX[1]=0x01;
////////MasterTX[2]=0x02;
////////MasterTX[3]=0x03;    
//////////touch_adr = HAL_I2C_Master_Receive(&hi2c1, 0x6e, touch_receive, 0x03, 1);   
//////////HAL_Delay(1);    
////////touch_adr = HAL_I2C_Master_Transmit(&hi2c1, 0x70, MasterTX, 0x01, 1);   
//////////HAL_Delay(1);   
//////////for(uint8_t ttt=0; ttt<0x1F;ttt++){
////////touch_adr = HAL_I2C_Master_Receive(&hi2c1, 0x71, touch_receive, 0x30, 1);   
////////HAL_Delay(10);      
//HAL_UART_Transmit_DMA(&huart2, uart_TX, 0x52);   
touch_enable = 0;   
 }
}


int GUI_TOUCH_GetState_my(GUI_PID_STATE * pState){
	
if(touch_enable ==1){
  
//////////device_mode = touch_receive[0x00]; 
//////////gesture_id = touch_receive[0x01];
//////////number_of_tuchpoint = touch_receive[0x02];    
 
pState->x = 800*(256*(0x0F & (touch_receive[0x03])) + touch_receive[0x04])/1791;
pState->y = 480*(256*(0x0F & (touch_receive[0x05])) + touch_receive[0x06])/1024;	
//pState->Pressed = 1;
//touch_enable = 0; 	
return 1;

}	
pState->Pressed = 0;
return 0;
}

void GUI_TOUCH_Exec_my_old(void) {
  //TOUCH_STATE touch_state;
//	GUI_PID_STATE touch_state;
  static U8   PressedOld = 0;
  static int  xOld = 0;
  static int  yOld = 0;
         int  x, y;
         int  xDiff, yDiff;
//////////         int  xSize, ySize;
  GUI_TOUCH_GetState_my(&touch_state);
//  if (touch_state.Pressed) {
  if (touch_enable == 1) {	
    // Touch screen is pressed
    y=480*(256*(0x0F & (touch_receive[0x05])) + touch_receive[0x06])/1024;	
		x=800*(256*(0x0F & (touch_receive[0x03])) + touch_receive[0x04])/1791;
		
    if (PressedOld == 1) {
      // Touch screen has already been pressed
      // Calculate difference between new and old position
      xDiff = (x > xOld) ? (x - xOld) : (xOld - x);
      yDiff = (y > yOld) ? (y - yOld) : (yOld - y);
      // Store state if new position differs significantly
      if (xDiff + yDiff > 2) {
				touch_state.x = x;
				touch_state.y = y;				
				touch_state.Layer = 0;	
				touch_state.Pressed = 1;				
        GUI_TOUCH_StoreStateEx(&touch_state);
        xOld = x;
        yOld = y;
      }
    }
    else {
      // Touch screen was previously not pressed
      // Store state regardless of position
				touch_state.x = x;
				touch_state.y = y;				
				touch_state.Layer = 0;				
				touch_state.Pressed = 1;			
      GUI_TOUCH_StoreStateEx(&touch_state);
      xOld = x;
      yOld = y;
      PressedOld = 1;
    }
	touch_enable = 0; 			
  }
  else {
    // Touch screen is not pressed
    // Store state if it was released recently
    if (PressedOld == 1) {
      PressedOld = 0;
				touch_state.x = -1;
				touch_state.y = -1;				
				touch_state.Layer = 0;	
				touch_state.Pressed = 0;
      GUI_TOUCH_StoreStateEx(&touch_state);
    }
  }
}

void read_touch_int(void){
	if (touch_enable == 1){	
	MasterTX[0]=0x00;
	touch_adr = HAL_I2C_Master_Transmit(&hi2c1, 0x70, MasterTX, 0x01, 1);   	
//	touch_adr = HAL_I2C_Master_Receive_DMA(&hi2c1, 0x70, touch_receive, 0x20);
	touch_enable = 0;  
	}
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration----------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache-------------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache-------------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FMC_Init();
  MX_DMA2D_Init();
  MX_CRC_Init();
  MX_LTDC_Init();
  MX_SDMMC1_SD_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_RNG_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	
//===================================================================================
HAL_SDRAM_MspInit(&hsdram1);
	
  /* SDRAM initialization sequence */
//  __HAL_RCC_DMA2D_CLK_ENABLE();   	
	
	SDRAM_Setting();

//=====================================================================

  HAL_LTDC_ProgramLineEvent(&hltdc, 0);
    /* Enable dithering */
  HAL_LTDC_EnableDither(&hltdc);


  BSP_TS_Init(800, 480);

//=====================================================================

//while (1)
//{
//printf("test text\n");
//HAL_Delay(1000);
//}

	/* Init the STemWin GUI Library */
	GUI_Init();

	/* Multi buffering enable */
  WM_MULTIBUF_Enable(1);
  GUI_SetLayerVisEx (1, 0);
  GUI_SelectLayer(0);
	/* Activate the use of memory device feature */
  WM_SetCreateFlags(WM_CF_MEMDEV);
	

	GUI_SetBkColor(GUI_DARKBLUE);
	GUI_Clear();
f_mount(&SDFatFs, (TCHAR const*)SD_Path, 0);

GUI_SetFont(&GUI_Font24_1);		
GUI_DispStringAt("TOUCHSCREEN TEST... TOUCH TO SCREEN",250,450);	
//while(1){	
//GUI_SetFont(&GUI_Font8x18);	
//my_touch_test();	
//}	
//	picture_test();	
	
//while(1){picture_test();}	
//while(1){touch_test();}//narod stream
	
////		for(j=1;j<=22;j++)
////		{	
////			bmp1 = (uint8_t *) 0xD0200000;	
////			sprintf(str1,"image%03d.bmp",j);
////			OpenBMP((uint8_t *)bmp1,str1);	
////			HAL_Delay(1000);		
////					GUI_MULTIBUF_BeginEx(0);	
////			GUI_BMP_Draw(bmp1,0,0); // Draw image	
////					GUI_MULTIBUF_EndEx(0);	
////			HAL_Delay(1000);
////		}
//while(1){}	
	
////picture_test();
//picture_test_irq();

	/* Start Demo */
//	GUIDEMO_Main();

BUTTON_GetSkinFlexProps(&my_BUTTON_SKINFLEX_PROPS, 1);
my_BUTTON_SKINFLEX_PROPS.Radius = 10;

my_BUTTON_SKINFLEX_PROPS.aColorLower[0] = 0x361400;
my_BUTTON_SKINFLEX_PROPS.aColorLower[1] = 0x361400;
my_BUTTON_SKINFLEX_PROPS.aColorUpper[0] = 0x361400;
my_BUTTON_SKINFLEX_PROPS.aColorUpper[1] = 0x361400;

//my_BUTTON_SKINFLEX_PROPS.aColorFrame[0] = 0x361400;
//my_BUTTON_SKINFLEX_PROPS.aColorFrame[1] = 0x361400;
my_BUTTON_SKINFLEX_PROPS.aColorFrame[2] = 0x361400;

//////my_BUTTON_SKINFLEX_PROPS.aColorLower[0] = GUI_TRANSPARENT;
//////my_BUTTON_SKINFLEX_PROPS.aColorLower[1] = GUI_TRANSPARENT;
//////my_BUTTON_SKINFLEX_PROPS.aColorUpper[0] = GUI_TRANSPARENT;
//////my_BUTTON_SKINFLEX_PROPS.aColorUpper[1] = GUI_TRANSPARENT;

BUTTON_SetSkinFlexProps(&my_BUTTON_SKINFLEX_PROPS, 0);
BUTTON_SetSkinFlexProps(&my_BUTTON_SKINFLEX_PROPS, 1);
BUTTON_SetSkinFlexProps(&my_BUTTON_SKINFLEX_PROPS, 2);
BUTTON_SetSkinFlexProps(&my_BUTTON_SKINFLEX_PROPS, 3);

BUTTON_SetDefaultFont(&GUI_Font24_1);
BUTTON_SetDefaultTextAlign(GUI_TA_LEFT | GUI_TA_VERTICAL);//3
TEXT_SetDefaultFont(&GUI_Font24_1);
BUTTON_SetDefaultTextColor(0xFFFFFF,0);
BUTTON_SetDefaultTextColor(0xFFFFFF,1);
BUTTON_SetDefaultTextColor(0xFFFFFF,2);

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
WM_SetCreateFlags(WM_CF_MEMDEV);	
  //hWin = CreateFramewin();	
  //CreateFramewin();		
	
  hWin = CreateWindow();		
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//    GUI_TOUCH_Exec();             /* Execute Touchscreen support */		
    GUI_Exec();                   /* Execute all GUI jobs ... Return 0 if nothing was done. */
    GUI_X_ExecIdle();             /* Nothing left to do for the moment ... Idle processing */		

		
		HAL_Delay(50);				
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_SDMMC1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disables the MPU */
  HAL_MPU_Disable();
    /**Initializes and configures the Region and the memory to be protected 
    */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0xD0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_8MB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
