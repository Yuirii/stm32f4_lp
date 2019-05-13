/**
  *      __            ____
  *     / /__ _  __   / __/                      __  
  *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
  *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
  *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
  *                    /_/   github.com/KitSprout    
  * 
  * @file    stm32f4xx_bsp.h
  * @author  KitSprout
  * @date    19-Nov-2016
  * @brief   
  * 
  */

/* Define to prevent recursive inclusion ---------------------------------------------------*/
#ifndef __STM32F4XX_BSP_H
#define __STM32F4XX_BSP_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes --------------------------------------------------------------------------------*/
#include "drivers\stm32f4_system.h"

/* Exported types --------------------------------------------------------------------------*/
/* Exported constants ----------------------------------------------------------------------*/
/* Exported functions ----------------------------------------------------------------------*/
void BSP_GPIO_Config( void );
void BSP_GPIO_DisConfig(void);
void BSP_UART_Config( pFunc txEven, pFunc rxEven );
void BSP_UWB_Config( void );
void BSP_EXTIx_Config( pFunc extix );

#ifdef __cplusplus
}
#endif

#endif

/*************************************** END OF FILE ****************************************/
