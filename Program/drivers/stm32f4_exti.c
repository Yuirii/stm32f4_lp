/**
  *      __            ____
  *     / /__ _  __   / __/                      __  
  *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
  *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
  *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
  *                    /_/   github.com/KitSprout    
  * 
  * @file    stm32f4_exti.c
  * @author  KitSprout
  * @date    25-Nov-2016
  * @brief   
  * 
  */

/* Includes --------------------------------------------------------------------------------*/
#include "stm32f4_system.h"
#include "stm32f4_exti.h"

/** @addtogroup STM32_Driver
  * @{
  */

/* Private typedef -------------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------------*/
/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
pFunc DW1000_ExtiCallback = NULL;
//GPIO_PIN_0 ÖÐ¶Ï¿Ú
ExtiHandle_st hExtix = {
  .pin          = GPIO_PIN_0,
  .EvenCallback = NULL,
};
/* Private function prototypes -------------------------------------------------------------*/
/* Private functions -----------------------------------------------------------------------*/

/**
  * @brief  EXTI line detection callbacks.
  */
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
  if (GPIO_Pin == DW1000_IRQ_PIN) {
    DW1000_ExtiCallback();
  }
}

/**
  * @brief  EXTIx_Config
  * @param  None
  * @retval None
  */
void EXTIx_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_PWR_CLK_ENABLE();//421 add_test
	//PA0ÖÐ¶ÏIO
  /* GPIO Pin ******************************************************************/
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Pin   = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  //must enable APB1ENR?
////  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
//	RCC->APB1ENR|=1<<28;
  /* EXTI IT *******************************************************************/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0x0f, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);//EXTI2_IRQn>>EXTI0_IRQn
}
/*************************************** END OF FILE ****************************************/
