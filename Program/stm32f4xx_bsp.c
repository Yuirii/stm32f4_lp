/**
  *      __            ____
  *     / /__ _  __   / __/                      __  
  *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
  *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
  *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
  *                    /_/   github.com/KitSprout    
  * 
  * @file    stm32f4xx_bsp.c
  * @author  KitSprout
  * @date    16-Nov-2016
  * @brief   
  * 
  */

/* Includes --------------------------------------------------------------------------------*/
#include "drivers\stm32f4_system.h"
#include "drivers\stm32f4_exti.h"
#include "modules\serial.h"
#include "modules\dw1000.h"
#include "stm32f4xx_bsp.h"

/** @addtogroup STM32_Program
  * @{
  */

/* Private typedef -------------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------------*/
/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
/* Private function prototypes -------------------------------------------------------------*/
/* Private functions -----------------------------------------------------------------------*/

void BSP_GPIO_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable all GPIO Clk *******************************************************/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* GPIO all analog input *****************************************************/
  GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin   = GPIO_PIN_All & (~(GPIO_PIN_13 | GPIO_PIN_14));
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* GPIO Pin ******************************************************************/
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

  GPIO_InitStruct.Pin   = LED_R_PIN;
  HAL_GPIO_Init(LED_R_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = LED_G_PIN;
  HAL_GPIO_Init(LED_G_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = LED_VCC_PIN;
  HAL_GPIO_Init(LED_VCC_GPIO_PORT, &GPIO_InitStruct);
	
	/*20190311添加蜂鸣器的端口*/
	GPIO_InitStruct.Pin   = BEEP_PIN;
  HAL_GPIO_Init(BEEP_GPIO_PORT, &GPIO_InitStruct);
	

  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStruct.Pin   = KEY_PIN;
  HAL_GPIO_Init(KEY_GPIO_PORT, &GPIO_InitStruct);

  LED_R_Set();
  LED_G_Set();
  LED_VCC_Set();
	
	/*蜂鸣器拉低20190311*/
	BEEP_Reset();
}

//void BSP_GPIO_DisConfig(void){
//  GPIO_InitTypeDef GPIO_InitStruct;

//  /* GPIO all analog input *****************************************************/
//  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
////	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
//	GPIO_InitStruct.Pin   = GPIO_PIN_All &(~(GPIO_PIN_15|GPIO_PIN_13|GPIO_PIN_14));
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//  GPIO_InitStruct.Pin   = GPIO_PIN_All &(~(GPIO_PIN_3 |GPIO_PIN_4 |GPIO_PIN_5 ));
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//  /* GPIO Pin ******************************************************************/
//  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull  = GPIO_PULLUP;
//  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

//  GPIO_InitStruct.Pin   = LED_R_PIN;
//	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
//  HAL_GPIO_Init(LED_R_GPIO_PORT, &GPIO_InitStruct);

//  GPIO_InitStruct.Pin   = LED_G_PIN;
//  HAL_GPIO_Init(LED_G_GPIO_PORT, &GPIO_InitStruct);

//  GPIO_InitStruct.Pin   = LED_VCC_PIN;
//	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
//  HAL_GPIO_Init(LED_VCC_GPIO_PORT, &GPIO_InitStruct);
//	
//	//beep
//	GPIO_InitStruct.Pin   = BEEP_PIN;
//	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
//  HAL_GPIO_Init(BEEP_GPIO_PORT, &GPIO_InitStruct);
//	
//	//KEYS
//  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pin   = KEY_PIN;
//	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;//按键拉低，省电
//  HAL_GPIO_Init(KEY_GPIO_PORT, &GPIO_InitStruct);
//}

void BSP_GPIO_DisConfig(void){
  GPIO_InitTypeDef GPIO_InitStruct;

//  /* GPIO all analog input *****************************************************/
//  GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull  = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//  GPIO_InitStruct.Pin   = GPIO_PIN_All & (~(GPIO_PIN_13 | GPIO_PIN_14));
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
//	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pin   = GPIO_PIN_All &(~(GPIO_PIN_15|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_0));
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin   = GPIO_PIN_All &(~(GPIO_PIN_3 |GPIO_PIN_4 |GPIO_PIN_5));//DWT_SPI
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* GPIO Pin ******************************************************************/
  GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;

  GPIO_InitStruct.Pin   = LED_R_PIN;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  HAL_GPIO_Init(LED_R_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = LED_G_PIN;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
  HAL_GPIO_Init(LED_G_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = LED_VCC_PIN;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  HAL_GPIO_Init(LED_VCC_GPIO_PORT, &GPIO_InitStruct);
	
	//beep
	GPIO_InitStruct.Pin   = BEEP_PIN;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  HAL_GPIO_Init(BEEP_GPIO_PORT, &GPIO_InitStruct);
	
	//KEYS
  GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pin   = KEY_PIN;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;//按键拉低，省电
  HAL_GPIO_Init(KEY_GPIO_PORT, &GPIO_InitStruct);
	
}

void BSP_UART_Config( pFunc txEven, pFunc rxEven )
{
  Serial_SetTxCallbackFunc(txEven);
  Serial_SetRxCallbackFunc(rxEven);

  Serial_Config();
  printf("\r\nHello World!\r\n\r\n");
}

void BSP_UWB_Config( void )
{
  DWT_Config();
  delay_us(10);
}

void BSP_EXTIx_Config( pFunc extix )
{
  hExtix.EvenCallback = extix;
  EXTIx_Config();
}
/*************************************** END OF FILE ****************************************/
