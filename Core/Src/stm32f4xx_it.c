/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */
  process_irq_spi1();
  /* USER CODE END SPI1_IRQn 0 */
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void EXTI1_IRQHandler(void)
{
    // Kiểm tra nếu c�? ngắt EXTI1 đã được set
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_1))
    {
    	if(LL_GPIO_IsInputPinSet(CS1_GPIO_Port, CS1_Pin))
    		SPI1->CR1 &= ~SPI_CR1_SSI;
    	else
    	{
    		SPI1->CR1 |= SPI_CR1_SSI;
//    		slave_choice(1);
    	}
    		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    }
}

void EXTI2_IRQHandler(void)
{
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_2))
    {
    	if(LL_GPIO_IsInputPinSet(CS2_GPIO_Port, CS2_Pin))
    		SPI1->CR1 &= ~SPI_CR1_SSI;
    	else
    	{
    		SPI1->CR1 |= SPI_CR1_SSI;
//    		slave_choice(2);
    	}


        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
    }
}

void EXTI3_IRQHandler(void)
{
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_3))
    {
//    	if(LL_GPIO_IsInputPinSet(CS3_GPIO_Port, CS3_Pin))
//    		SPI1->CR1 |= SPI_CR1_SSI;
//    	else
//    	{
//    		SPI1->CR1 &= ~SPI_CR1_SSI;
//
////    		slave_choice(3);
//    	}


        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);
    }
}

void EXTI4_IRQHandler(void)
{
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4))
    {
    	if(LL_GPIO_IsInputPinSet(CS4_GPIO_Port, CS4_Pin))
    		SPI1->CR1 &= ~SPI_CR1_SSI;
    	else
    	{
    		SPI1->CR1 |= SPI_CR1_SSI;
//    		slave_choice(4);
    	}
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
    }
}



//void handle_exti_line(uint32_t exti_line, GPIO_TypeDef *CS_GPIO_Port, uint16_t CS_Pin, char slave_id)
//{
//  if (LL_EXTI_IsActiveFlag_0_31(exti_line))
//  {
//    if (LL_GPIO_IsInputPinSet(CS_GPIO_Port, CS_Pin))
//    {
////      uart_print("ext3 up\r\n");
//      SPI1->CR1 &= ~SPI_CR1_SSI;
//    }
//    else
//    {
//      SPI1->CR1 |= SPI_CR1_SSI;
////      uart_print("ext3 down\r\n");
//      slave_choice(slave_id);
//    }
//    LL_EXTI_ClearFlag_0_31(exti_line);
//  }
//}
//
//void EXTI1_IRQHandler(void)
//{
//  handle_exti_line(LL_EXTI_LINE_1, CS1_GPIO_Port, CS1_Pin, 1);
//}
//
//void EXTI2_IRQHandler(void)
//{
//  handle_exti_line(LL_EXTI_LINE_2, CS2_GPIO_Port, CS2_Pin, 2);
//}
//
//void EXTI3_IRQHandler(void)
//{
//  handle_exti_line(LL_EXTI_LINE_3, CS3_GPIO_Port, CS3_Pin, 3);
//}
//
//void EXTI4_IRQHandler(void)
//{
//  handle_exti_line(LL_EXTI_LINE_4, CS4_GPIO_Port, CS4_Pin, 4);
//}

/* USER CODE END 1 */
