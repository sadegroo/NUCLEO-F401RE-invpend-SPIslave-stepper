/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_spi3_rx;

extern DMA_HandleTypeDef hdma_spi3_tx;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */
extern void BSP_MotorControl_StepClockHandler(uint8_t deviceId);
extern void BSP_MotorControl_FlagInterruptHandler(void);
extern void StepClockHandler_L6472_Acceleration_Control(void); // for acceleration control of device 0
/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PA4     ------> SPI3_NSS
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* SPI3 DMA Init */
    /* SPI3_RX Init */
    hdma_spi3_rx.Instance = DMA1_Stream0;
    hdma_spi3_rx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_spi3_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_spi3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi3_rx) != HAL_OK)
    {
      Error_Handler(40);
    }

    __HAL_LINKDMA(hspi,hdmarx,hdma_spi3_rx);

    /* SPI3_TX Init */
    hdma_spi3_tx.Instance = DMA1_Stream5;
    hdma_spi3_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_tx.Init.Mode = DMA_CIRCULAR;
    hdma_spi3_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_spi3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK)
    {
      Error_Handler(41);
    }

    __HAL_LINKDMA(hspi,hdmatx,hdma_spi3_tx);

  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */

  }
  if(hspi->Instance == SPIx)
    {
      /*##-1- Enable peripherals and GPIO Clocks #################################*/
      /* Enable GPIO TX/RX clock */
      SPIx_SCK_GPIO_CLK_ENABLE();
      SPIx_MISO_GPIO_CLK_ENABLE();
      SPIx_MOSI_GPIO_CLK_ENABLE();
      /* Enable SPI clock */
      SPIx_CLK_ENABLE();

      /*##-2- Configure peripheral GPIO ##########################################*/
      /* SPI SCK GPIO pin configuration  */
      GPIO_InitStruct.Pin       = SPIx_SCK_PIN;
      GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull      = GPIO_NOPULL;
      GPIO_InitStruct.Speed     = GPIO_SPEED_MEDIUM;
      GPIO_InitStruct.Alternate = SPIx_SCK_AF;

      HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

      /* SPI MISO GPIO pin configuration  */
      GPIO_InitStruct.Pin = SPIx_MISO_PIN;
      GPIO_InitStruct.Alternate = SPIx_MISO_AF;

      HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);

      /* SPI MOSI GPIO pin configuration  */
      GPIO_InitStruct.Pin = SPIx_MOSI_PIN;
      GPIO_InitStruct.Alternate = SPIx_MOSI_AF;

      HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);
    }

}

/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PA4     ------> SPI3_NSS
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);

    /* SPI3 DMA DeInit */
    HAL_DMA_DeInit(hspi->hdmarx);
    HAL_DMA_DeInit(hspi->hdmatx);
  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
  }
  if(hspi->Instance == SPIx)
   {
     /*##-1- Reset peripherals ##################################################*/
     SPIx_FORCE_RESET();
     SPIx_RELEASE_RESET();

     /*##-2- Disable peripherals and GPIO Clocks ################################*/
     /* Configure SPI SCK as alternate function  */
     HAL_GPIO_DeInit(SPIx_SCK_GPIO_PORT, SPIx_SCK_PIN);
     /* Configure SPI MISO as alternate function  */
     HAL_GPIO_DeInit(SPIx_MISO_GPIO_PORT, SPIx_MISO_PIN);
     /* Configure SPI MOSI as alternate function  */
     HAL_GPIO_DeInit(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_PIN);
   }

}

/**
* @brief TIM_Encoder MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_encoder: TIM_Encoder handle pointer
* @retval None
*/
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_encoder->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA1     ------> TIM2_CH2
    PA15     ------> TIM2_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */

  }

}

/**
* @brief TIM_Encoder MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_encoder: TIM_Encoder handle pointer
* @retval None
*/
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* htim_encoder)
{
  if(htim_encoder->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PA1     ------> TIM2_CH2
    PA15     ------> TIM2_CH1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_15);

  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }

}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */

  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, USART_TX_Pin|USART_RX_Pin);

    /* USART2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */
/**
  * @brief PWM MSP Initialization
  * @param[in] htim_pwm PWM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1)
  {
    /* Peripheral clock enable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1_CLCK_ENABLE();

    /* GPIO configuration */
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PWM_1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM1;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PWM_1_PORT, &GPIO_InitStruct);

    /* Set Interrupt Group Priority of Timer Interrupt*/
    HAL_NVIC_SetPriority(BSP_MOTOR_CONTROL_BOARD_PWM1_IRQn, 4, 0);

    /* Enable the timer global Interrupt */
    HAL_NVIC_EnableIRQ(BSP_MOTOR_CONTROL_BOARD_PWM1_IRQn);
  }
  else if(htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2)
  {
    /* Peripheral clock enable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2_CLCK_ENABLE();

    /* GPIO configuration */
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PWM_2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM2;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PWM_2_PORT, &GPIO_InitStruct);

    /* Set Interrupt Group Priority of Timer Interrupt*/
    HAL_NVIC_SetPriority(BSP_MOTOR_CONTROL_BOARD_PWM2_IRQn, 4, 0);

    /* Enable the timer2 global Interrupt */
    HAL_NVIC_EnableIRQ(BSP_MOTOR_CONTROL_BOARD_PWM2_IRQn);

  }
  else if(htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM3)
  {
    /* Peripheral clock enable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM3_CLCK_ENABLE();

    /* GPIO configuration */
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PWM_3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PWM_3_PORT, &GPIO_InitStruct);

    /* Set Interrupt Group Priority of Timer Interrupt*/
    HAL_NVIC_SetPriority(BSP_MOTOR_CONTROL_BOARD_PWM3_IRQn, 3, 0);

    /* Enable the timer global Interrupt */
    HAL_NVIC_EnableIRQ(BSP_MOTOR_CONTROL_BOARD_PWM3_IRQn);
  }
}

/**
  * @brief PWM MSP De-Initialization
  * @param[in] htim_pwm PWM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
  if(htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1)
  {
    /* Peripheral clock disable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1_CLCK_DISABLE();

    /* GPIO Deconfiguration */
    HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_PWM_1_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_1_PIN);

  }
  else if(htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2)
  {
    /* Peripheral clock disable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2_CLCK_DISABLE();

    /* GPIO Deconfiguration */
    HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_PWM_2_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_2_PIN);

  }
  else if(htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM3)
  {
    /* Peripheral clock disable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM3_CLCK_DISABLE();

    /* GPIO Deconfiguration */
    HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_PWM_3_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_3_PIN);
  }
}

/**
  * @brief PWM Callback
  * @param[in] htim PWM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if ((htim->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1)&& (htim->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM1))
  {
    if (BSP_MotorControl_GetDeviceState(0) != INACTIVE)
    {
#ifndef ACCELERATION_CONTROL
      BSP_MotorControl_StepClockHandler(0);
#endif
#ifdef ACCELERATION_CONTROL
      StepClockHandler_L6472_Acceleration_Control(); // picks standard or alt stepclockhandler
#endif
    }
  }
  if ((htim->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2)&& (htim->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM2))
  {
    if (BSP_MotorControl_GetDeviceState(1) != INACTIVE)
    {
      BSP_MotorControl_StepClockHandler(1);
    }
  }
  if ((htim->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM3)&& (htim->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM3))
  {
    HAL_GPIO_TogglePin(BSP_MOTOR_CONTROL_BOARD_PWM_3_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_3_PIN);
    if ((BSP_MotorControl_GetDeviceState(2) != INACTIVE)&&
        (HAL_GPIO_ReadPin(BSP_MOTOR_CONTROL_BOARD_PWM_3_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_3_PIN) == GPIO_PIN_SET))
    {
      BSP_MotorControl_StepClockHandler(2);
    }
  }
}

/**
  * @brief External Line Callback
  * @param[in] GPIO_Pin pin number
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BSP_MOTOR_CONTROL_BOARD_FLAG_PIN)
  {
    BSP_MotorControl_FlagInterruptHandler();
  }
 }
/* USER CODE END 1 */
