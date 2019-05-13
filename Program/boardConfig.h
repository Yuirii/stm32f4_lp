/**
  *      __            ____
  *     / /__ _  __   / __/                      __  
  *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
  *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
  *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
  *                    /_/   github.com/KitSprout    
  * 
  * @file    boardConfig.h
  * @author  KitSprout
  * @date    28-Nov-2016
  * @brief   
  * 
  */

/* Define to prevent recursive inclusion ---------------------------------------------------*/
#ifndef __BOARDCONFIG_H
#define __BOARDCONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes --------------------------------------------------------------------------------*/
/* Exported types --------------------------------------------------------------------------*/
/* Exported constants ----------------------------------------------------------------------*/

#define KS_HW_BOARD_NAME              "UWBAdapter"
#define KS_HW_MCU_NAME                "STM32F411xE"

#define KS_HW_CLOCK_SOUCE_HSE
//#define KS_HW_USE_CLOCK_SOUCE_HSI

//#define KS_HW_UART_HAL_LIBRARY
#define KS_HW_SPI_HAL_LIBRARY
//#define KS_HW_I2C_HAL_LIBRARY

/* -------- LED and KEY */
#define LED_R_PIN                     GPIO_PIN_12
#define LED_R_GPIO_PORT               GPIOA
#define LED_R_Set()                   __GPIO_SET(LED_R_GPIO_PORT, LED_R_PIN)
#define LED_R_Reset()                 __GPIO_RST(LED_R_GPIO_PORT, LED_R_PIN)
#define LED_R_Toggle()                __GPIO_TOG(LED_R_GPIO_PORT, LED_R_PIN)

#define LED_G_PIN                     GPIO_PIN_11
#define LED_G_GPIO_PORT               GPIOA
#define LED_G_Set()                   __GPIO_SET(LED_G_GPIO_PORT, LED_G_PIN)
#define LED_G_Reset()                 __GPIO_RST(LED_G_GPIO_PORT, LED_G_PIN)
#define LED_G_Toggle()                __GPIO_TOG(LED_G_GPIO_PORT, LED_G_PIN)

#define LED_VCC_PIN                   GPIO_PIN_12
#define LED_VCC_GPIO_PORT             GPIOB
#define LED_VCC_Set()                 __GPIO_SET(LED_VCC_GPIO_PORT, LED_VCC_PIN)
#define LED_VCC_Reset()               __GPIO_RST(LED_VCC_GPIO_PORT, LED_VCC_PIN)
#define LED_VCC_Toggle()              __GPIO_TOG(LED_VCC_GPIO_PORT, LED_VCC_PIN)

#define BEEP_PIN                     GPIO_PIN_7                              //��ӷ�����IO��
#define BEEP_GPIO_PORT               GPIOA
#define BEEP_Set()                   __GPIO_SET(BEEP_GPIO_PORT, BEEP_PIN)
#define BEEP_Reset()                 __GPIO_RST(BEEP_GPIO_PORT, BEEP_PIN)
#define BEEP_Toggle()                __GPIO_TOG(BEEP_GPIO_PORT, BEEP_PIN)

#define KEY_PIN                       GPIO_PIN_2
#define KEY_GPIO_PORT                 GPIOB
#define KEY_Set()                   __GPIO_SET(KEY_GPIO_PORT, KEY_PIN)
#define KEY_Reset()                 __GPIO_RST(KEY_GPIO_PORT, KEY_PIN)
#define KEY_Read()                    (__GPIO_READ(KEY_GPIO_PORT, KEY_PIN) == KEY_PIN)

/* -------- UART Serial */
#define SERIAL_MAX_TXBUF              16
#define SERIAL_MAX_RXBUF              16

#define SERIAL_UARTx                  USART1
#define SERIAL_UARTx_CLK_ENABLE()     __HAL_RCC_USART1_CLK_ENABLE()
#define SERIAL_UARTx_IRQn             USART1_IRQn
#define SERIAL_UARTx_IRQn_PREEMPT     0x0F
#define SERIAL_UARTx_IRQn_SUB         1

#define SERIAL_UARTx_FORCE_RESET()    __HAL_RCC_USART1_FORCE_RESET()
#define SERIAL_UARTx_RELEASE_RESET()  __HAL_RCC_USART1_RELEASE_RESET()

#define SERIAL_TX_PIN                 GPIO_PIN_6
#define SERIAL_TX_GPIO_PORT           GPIOB
#define SERIAL_TX_AF                  GPIO_AF7_USART1

#define SERIAL_RX_PIN                 GPIO_PIN_7
#define SERIAL_RX_GPIO_PORT           GPIOB
#define SERIAL_RX_AF                  GPIO_AF7_USART1

#define SERIAL_BAUDRATE               115200
#define SERIAL_BYTESIZE               UART_WORDLENGTH_8B
#define SERIAL_STOPBITS               UART_STOPBITS_1
#define SERIAL_PARITY                 UART_PARITY_NONE
#define SERIAL_HARDWARECTRL           UART_HWCONTROL_NONE
#define SERIAL_MODE                   UART_MODE_TX_RX
#define SERIAL_OVERSAMPLE             UART_OVERSAMPLING_16

/* -------- Ultra-Wideband */
#define DW1000_MAX_TXBUF              16
#define DW1000_MAX_RXBUF              16

#define DW1000_SPIx                   SPI1
#define DW1000_SPIx_CLK_ENABLE()      __HAL_RCC_SPI1_CLK_ENABLE()
#define DW1000_SPIx_IRQn              SPI1_IRQn
#define DW1000_SPIx_IRQn_PREEMPT      0x0F
#define DW1000_SPIx_IRQn_SUB          1
#define DW1000_SPIx_FORCE_RESET()     __HAL_RCC_SPI1_FORCE_RESET()
#define DW1000_SPIx_RELEASE_RESET()   __HAL_RCC_SPI1_RELEASE_RESET()

#define DW1000_SPIx_SPEED_HIGH        SPI_BAUDRATEPRESCALER_4
#define DW1000_SPIx_SPEED_LOW         SPI_BAUDRATEPRESCALER_32

#define DW1000_SCK_PIN                GPIO_PIN_3
#define DW1000_SCK_GPIO_PORT          GPIOB
#define DW1000_SCK_AF                 GPIO_AF5_SPI1

#define DW1000_SDO_PIN                GPIO_PIN_4
#define DW1000_SDO_GPIO_PORT          GPIOB
#define DW1000_SDO_AF                 GPIO_AF5_SPI1

#define DW1000_SDI_PIN                GPIO_PIN_5
#define DW1000_SDI_GPIO_PORT          GPIOB
#define DW1000_SDI_AF                 GPIO_AF5_SPI1

#define DW1000_CSD_PIN                GPIO_PIN_15
#define DW1000_CSD_GPIO_PORT          GPIOA
#define DW1000_CSD_H()                __GPIO_SET(DW1000_CSD_GPIO_PORT, DW1000_CSD_PIN)
#define DW1000_CSD_L()                __GPIO_RST(DW1000_CSD_GPIO_PORT, DW1000_CSD_PIN)

#define DW1000_IRQ_PIN                GPIO_PIN_8
#define DW1000_IRQ_GPIO_PORT          GPIOB
#define DW1000_IRQn                   EXTI9_5_IRQn

#define DW1000_RST_PIN                GPIO_PIN_9
#define DW1000_RST_GPIO_PORT          GPIOB
#define DW1000_RST_H()                __GPIO_SET(DW1000_RST_GPIO_PORT, DW1000_RST_PIN)
#define DW1000_RST_L()                __GPIO_RST(DW1000_RST_GPIO_PORT, DW1000_RST_PIN)

#define DW1000_EXT_PIN                GPIO_PIN_13
#define DW1000_EXT_GPIO_PORT          GPIOC

#define DW1000_WAK_PIN                GPIO_PIN_14
#define DW1000_WAK_GPIO_PORT          GPIOC

/* Exported functions ----------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif

/*************************************** END OF FILE ****************************************/
