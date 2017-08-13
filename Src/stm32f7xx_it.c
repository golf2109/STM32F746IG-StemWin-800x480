/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
#include "stm32f7xx_it.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */
#include "GUI.h"
//extern volatile GUI_TIMER_TIME OS_TimeMS;
uint8_t read_value = 0;
uint16_t read_value_xy[2];
extern uint8_t MasterTX[0x10];
extern	HAL_StatusTypeDef touch_adr;
extern uint8_t touch_receive[0xff];
extern volatile uint8_t touch_enable;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA2D_HandleTypeDef hdma2d;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern I2C_HandleTypeDef hi2c1;
extern LTDC_HandleTypeDef hltdc;
extern TIM_HandleTypeDef htim6;

extern TIM_HandleTypeDef htim14;

/******************************************************************************/
/*            Cortex-M7 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */
//	OS_TimeMS++;
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
    HAL_GPIO_EXTI_Callback(GPIO_PIN_4);
  }
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

//////  * @brief  Read an amount of data in blocking mode from a specific memory address
//////  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
//////  *                the configuration information for the specified I2C.
//////  * @param  DevAddress Target device address: The device 7 bits address value
//////  *         in datasheet must be shift at right before call interface
//////  * @param  MemAddress Internal memory address
//////  * @param  MemAddSize Size of internal memory address
//////  * @param  pData Pointer to data buffer
//////  * @param  Size Amount of data to be sent
//////  * @param  Timeout Timeout duration
//////  * @retval HAL status
//////  
//////HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, 
//////																				 uint16_t DevAddress, 
//////																				 uint16_t MemAddress, 
//////	                                       uint16_t MemAddSize, 
//////	                                       uint8_t *pData, 
//////																				 uint16_t Size, 
//////																				 uint32_t Timeout)



///////**
//////  * @brief  Reads multiple data.
//////  * @param  i2c_handler : I2C handler
//////  * @param  Addr: I2C address
//////  * @param  Reg: Reg address 
//////  * @param  MemAddress: Memory address 
//////  * @param  Buffer: Pointer to data buffer
//////  * @param  Length: Length of the data
//////  * @retval Number of read data
//////  */
//////static HAL_StatusTypeDef I2Cx_ReadMultiple(I2C_HandleTypeDef *i2c_handler,
//////                                           uint8_t Addr,
//////                                           uint16_t Reg,
//////                                           uint16_t MemAddress,
//////                                           uint8_t *Buffer,
//////                                           uint16_t Length)
	
	
MasterTX[0]=0x00;
touch_adr = HAL_I2C_Master_Transmit(&hi2c1, 0x70, MasterTX, 0x01, 1);   
//touch_adr = HAL_I2C_Master_Receive(&hi2c1, 0x70, touch_receive, 0x20, 1);   
touch_adr = HAL_I2C_Master_Receive_DMA(&hi2c1, 0x70, touch_receive, 0x20);
//touch_adr = HAL_I2C_Mem_Read(&hi2c1, 0x70,0,3, touch_receive, 0x20, 100);	


/////**
////  * @brief  Reads a single data.
////  * @param  Addr: I2C address
////  * @param  Reg: Reg address
////  * @retval Data to be read
////  */
////uint8_t TS_IO_Read(uint8_t Addr, uint8_t Reg)
////{
////  uint8_t read_value = 0;

//////  I2Cx_ReadMultiple(&hI2cAudioHandler, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&read_value, 1);
////  I2Cx_ReadMultiple(&hi2c1, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&read_value, 1);	

////  return read_value;
////}

//read_value =  TS_IO_Read(0x70,0x02);


/////**
////  * @brief  Get the touch screen X and Y positions values
////  *         Manage multi touch thanks to touch Index global
////  *         variable 'ft5336_handle.currActiveTouchIdx'.
////  * @param  DeviceAddr: Device address on communication Bus.
////  * @param  X: Pointer to X position value
////  * @param  Y: Pointer to Y position value
////  * @retval None.
////  */
////void ft5336_TS_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y)
////ft5336_TS_GetXY(0x70, read_value_xy, read_value_xy+1);

touch_enable = 1;  
  /* USER CODE END EXTI4_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream0 global interrupt.
*/
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
* @brief This function handles I2C1 event interrupt.
*/
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
* @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
*/
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
* @brief This function handles LTDC global interrupt.
*/
void LTDC_IRQHandler(void)
{
  /* USER CODE BEGIN LTDC_IRQn 0 */

  /* USER CODE END LTDC_IRQn 0 */
  HAL_LTDC_IRQHandler(&hltdc);
  /* USER CODE BEGIN LTDC_IRQn 1 */

  /* USER CODE END LTDC_IRQn 1 */
}

/**
* @brief This function handles DMA2D global interrupt.
*/
void DMA2D_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2D_IRQn 0 */

  /* USER CODE END DMA2D_IRQn 0 */
  HAL_DMA2D_IRQHandler(&hdma2d);
  /* USER CODE BEGIN DMA2D_IRQn 1 */

  /* USER CODE END DMA2D_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
