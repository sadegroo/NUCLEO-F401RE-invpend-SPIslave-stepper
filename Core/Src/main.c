/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

char uart_buf[UART_BUFFER_SIZE];
int  uart_buf_len;
// Global flags
volatile uint8_t uart_xmit_flag = 0;
volatile uint8_t uart_err_flag = 0;
volatile uint8_t spi_txrx_flag = 0;
volatile uint8_t spi_err_flag = 0;

static uint8_t spi_rx_buf[SPI_BUFFER_SIZE];
static uint8_t spi_tx_buf[SPI_BUFFER_SIZE];
static int16_t recv_number1=0;   // stepper motor acceleration command in steps/s^2
static int16_t recv_number2=0;   // no use for this yet
static int16_t recv_number3=0;   // no use for this yet
static int16_t send_number1=0;	   // encoder position in increments
static int16_t send_number2=0;       // stepper motor angle in microsteps
static int16_t send_number3=0;	   // stepper motor angle in microsteps per second

static int16_t accel_cmd = 0;
static int16_t current_cmd = 0;
static int16_t prev_current_cmd = 0;
static int32_t encoder_pos=0;
static int32_t rotor_pos=0;
static int16_t rotor_vel=0;

static uint16_t rotor_pos_cnt=0;

static volatile uint16_t gLastError;

L6474_Init_t gL6474InitParams =
{
	MAX_ACCEL,           	/// Acceleration rate in step/s2. Range: (0..+inf).
	MAX_DECEL,           	/// Deceleration rate in step/s2. Range: (0..+inf).
	MAX_SPEED,              /// Maximum speed in step/s. Range: (30..10000].
	MIN_SPEED,              /// Minimum speed in step/s. Range: [30..10000).
	MAX_TORQUE_CONFIG, 		/// Torque regulation current in mA. (TVAL register) Range: 31.25mA to 4000mA.
	OVERCURRENT_THRESHOLD, 	/// Overcurrent threshold (OCD_TH register). Range: 375mA to 6000mA.
   L6474_CONFIG_OC_SD_ENABLE,         ///Overcurrent shutwdown (OC_SD field of CONFIG register).
   L6474_CONFIG_EN_TQREG_TVAL_USED,   /// Torque regulation method (EN_TQREG field of CONFIG register).
   L6474_STEP_SEL_1_16,               /// Step selection (STEP_SEL field of STEP_MODE register).
   L6474_SYNC_SEL_1_2,                /// Sync selection (SYNC_SEL field of STEP_MODE register).
   L6474_FAST_STEP_12us,              /// Fall time value (T_FAST field of T_FAST register). Range: 2us to 32us.
   L6474_TOFF_FAST_8us,               /// Maximum fast decay time (T_OFF field of T_FAST register). Range: 2us to 32us.
   3,                                 /// Minimum ON time in us (TON_MIN register). Range: 0.5us to 64us.
   21,                                /// Minimum OFF time in us (TOFF_MIN register). Range: 0.5us to 64us.
   L6474_CONFIG_TOFF_044us,           /// Target Swicthing Period (field TOFF of CONFIG register).
   L6474_CONFIG_SR_320V_us,           /// Slew rate (POW_SR field of CONFIG register).
   L6474_CONFIG_INT_16MHZ,            /// Clock setting (OSC_CLK_SEL field of CONFIG register).
   (L6474_ALARM_EN_OVERCURRENT      |
    L6474_ALARM_EN_THERMAL_SHUTDOWN |
    L6474_ALARM_EN_THERMAL_WARNING  |
    L6474_ALARM_EN_UNDERVOLTAGE     |
    L6474_ALARM_EN_SW_TURN_ON       |
    L6474_ALARM_EN_WRONG_NPERF_CMD)    /// Alarm (ALARM_EN register).
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint8_t encoder_position_read(Quadrature_Encoder_TypeDef *encoder, TIM_HandleTypeDef *htim3);
static void MyFlagInterruptHandler(void);
int32_t swap_Endians_32(int32_t value);
int16_t swap_Endians_16(int16_t value);
void Debug_TimerStatus(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t encoder_range_error;

  uint8_t state_main = START_STATE_MAIN;
  //uint32_t state_main_counter = 0; // counts the amount of times the same state was entered
  uint32_t state_cycle_cnt = 0; // counts the amount of times the circular part of the main state machine is looped
  static Chrono_TypeDef main_cycletimer= {0,0,0.0}; //Chronometer for circular part of the main state machine
 // uint8_t state_uart = 0;
  uint32_t err_cnt = 0;
  uint32_t overtime_cnt = 0;

  for(int i=0;i<SPI_BUFFER_SIZE;i++)
  {
	  spi_rx_buf[i]=0;
	  spi_tx_buf[i]=0;
  }

  //Declare and initialize encoder
  Quadrature_Encoder_TypeDef encoder_inst = {0,0,0,0,0,0,0,0,0,0,0,0,0,FALSE,FALSE,FALSE,COUNTS_PER_TURN};

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  MX_GPIO_Init();
  //----- Init of the Motor control library
  /* Set the L6474 library to use 1 device */
	BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_L6474, 1);
	/* When BSP_MotorControl_Init is called with NULL pointer,                  */
	/* the L6474 registers and parameters are set with the predefined values from file   */
	/* l6474_target_config.h, otherwise the registers are set using the   */
	/* L6474_Init_t pointer structure                */
	/* The first call to BSP_MotorControl_Init initializes the first device     */
	/* whose Id is 0.                                                           */
	/* The nth call to BSP_MotorControl_Init initializes the nth device         */
	/* whose Id is n-1.                                                         */
	/* Uncomment the call to BSP_MotorControl_Init below to initialize the      */
	/* device with the structure gL6474InitParams declared in the the main.c file */
	/* and comment the subsequent call having the NULL pointer                   */
	BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, &gL6474InitParams);
	//BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, NULL);

  /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
  BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

  /* Attach the function Error_Handler (defined below) to the error Handler*/

  // wake-up move
  /*
  BSP_MotorControl_AttachErrorHandler(Error_Handler);
  BSP_MotorControl_SetHome(0, 0);
  BSP_MotorControl_GoTo(0, 100);
  BSP_MotorControl_WaitWhileActive(0);
  BSP_MotorControl_GoTo(0, 0);
  BSP_MotorControl_WaitWhileActive(0);
  HAL_Delay(2000); // wait for pendulum to settle
  */

  //initialize acceleration control
  Init_L6472_Acceleration_Control(&gL6474InitParams, T_SAMPLE);

  //init chronometer
  Chrono_Init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  //MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Say something
  uart_buf_len = sprintf(uart_buf, "Hello world!\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

  // Initialize encoder
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  // Start RX DMA in Circular Mode
  if (HAL_SPI_TransmitReceive_DMA(&hspi3, spi_tx_buf, spi_rx_buf, SPI_BUFFER_SIZE) != HAL_OK)
  {
      Error_Handler(51); // Handle initialization error
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // UART state machine
	  /*
		switch (state_uart)
		{
		  case 0:
			  // Idle
			  if ((suc_cnt % UART_DECIMATION) == 0 && suc_cnt != 0 )
			  {
				  state_uart = 1;
			  }
			  break;

		  case 1:

			// Print newline
				switch(HAL_UART_Getstate(&huart2))
				{
				case HAL_UART_state_READY:
					uart_buf_len = sprintf(uart_buf, "success: %d, errors: %d, motcmd = %.3f, encoder = %.3f, motfb = %.3f \r\n", (unsigned int)suc_cnt, (unsigned int)err_cnt, recv_number1, send_number1, send_number2);
					HAL_UART_Transmit_IT(&huart2, (uint8_t *)uart_buf, uart_buf_len);

				default:
				  break;
				}

			state_uart = 0;
			break;

		  case 2:
			  // wait for transmit flag
			  if (uart_xmit_flag)
			  {
				  uart_xmit_flag =0;
				  state_uart = 0;
				  break;
			  }
			  if (uart_err_flag && HAL_UART_Getstate(&huart2) == HAL_UART_state_READY)
			  {
			  // Clear flag and try again
				uart_err_flag = 0;
				state_uart = 1;
				break;
			  }
		  default:
			break;

		} //close switch
		*/
		//-----------------------------------------------
		switch(state_main)
		{
		// sequential states
		//-------------------
		  case 0:
			  // START state: wait for DMA SPI to have valid date
			  if (spi_txrx_flag == 1){
				  state_main=1;
				  Chrono_Mark(&main_cycletimer);
				  uart_buf_len = sprintf(uart_buf, "SPI connection established. Starting system... \r\n");
				  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
			  }
			  break;
		// looping states
		//-----------------
		  case 1:
			// Read encoder
			encoder_range_error = encoder_position_read(&encoder_inst, &htim2);
			encoder_pos = encoder_inst.position_steps;

			// read rotor position
			if (rotor_pos_cnt % STEPPER_POSITION_DECIMATION == 0) {
				rotor_pos_cnt = 0;
				rotor_pos = GetPosition_L6472_Acceleration_Control();
			}
			rotor_pos_cnt++;

			// read rotor velocity
			rotor_vel = GetVelocity_L6472_Acceleration_Control();

			__disable_irq();
			send_number1 = swap_Endians_16((int16_t)encoder_pos);
			send_number2 = swap_Endians_16((int16_t)rotor_pos);
			send_number3 = swap_Endians_16(rotor_vel);
			__enable_irq();

			state_main++;
			break;
		  case 2:
			if (spi_txrx_flag == 1){
				spi_txrx_flag = 0;
				state_main=4;
				__disable_irq();
				accel_cmd = swap_Endians_16(recv_number1);
				__enable_irq();
			}
			// keep track of waiting time in this state
			if (Chrono_GetDiffNoMark(&main_cycletimer) > 1.5 * T_SAMPLE) {
				 overtime_cnt++;
				 state_main=3;
				 uart_buf_len = sprintf(uart_buf, "overtime: %" PRIu32 "\r\n", overtime_cnt);
				 //uart_buf_len = sprintf(uart_buf, "overtime: %u\r\n", overtime_cnt);
				 HAL_UART_Transmit_IT(&huart2, (uint8_t *)uart_buf, uart_buf_len);
			 }
			break;

		  case 3:
			  // keep periodically reading rotor to check for overrange
			 if (Chrono_GetDiffNoMark(&main_cycletimer) > 0.05) {
				 Chrono_Mark(&main_cycletimer);
				 rotor_pos = GetPosition_L6472_Acceleration_Control();
			 }
			if (spi_txrx_flag == 1){
				spi_txrx_flag = 0;
				Chrono_Mark(&main_cycletimer);
				state_main=4;
				__disable_irq();
				accel_cmd = swap_Endians_16(recv_number1);
				current_cmd = swap_Endians_16(recv_number2);
				__enable_irq();
			}
			break;
		  case 4:
			  // control actions

#ifdef POSITION_CONTROL
				//run control if motor inactive or standby
				if(BSP_MotorControl_GetDevicestate(0) >= 8) {
					BSP_MotorControl_GoTo(0, (int32_t) (accel_cmd));
				}
#endif
#ifdef ACCELERATION_CONTROL

				Run_L6472_Acceleration_Control(accel_cmd);
				//Debug_TimerStatus();
#endif
				state_main = 1;
				state_cycle_cnt++;
				Chrono_Mark(&main_cycletimer);
			  break;
		  case 50:
			  // Rotor out of safe range
			  BSP_MotorControl_HardStop(0);
			  uart_buf_len = sprintf(uart_buf, "Error 50: Rotor out of safe range\r\n");
			  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
			  state_main=99;
		  case 99:
			  ;// do nothting wait state
		  default:
			  break;
		}

		// update ARR of PWM timer when count is close to 0
		PostProcess_StepClockHandler_L6472_Acceleration_Control();

		// Error checks
		if (state_main!=99 && (float)labs(rotor_pos) > MAX_DEFLECTION_REV * STEPS_PER_TURN) {
			// go to error state 50
			state_main = 50;
			}
	  // count any communication errors
	  if (spi_err_flag)
	  {
		err_cnt++;
		spi_err_flag = 0; // Clear flag

	   }

  } // close while(1)
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler(0);
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler(1);
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler(10);
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler(21);
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler(22);
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler(30);
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
 * Encoder position read (returns signed integer)
 *
 * Includes capability for tracking pendulum excursion for Swing Up control.
 * This is developed and provided by Markus Dauberschmidt.  Please see
 * https://github.com/OevreFlataeker/steval_edukit_swingup
 *
 */


uint8_t encoder_position_read(Quadrature_Encoder_TypeDef *encoder, TIM_HandleTypeDef *htim) {

	int range_error = 0;

	 // Read current counter value
	    encoder->cnt3 = __HAL_TIM_GET_COUNTER(htim);

	    // Calculate delta explicitly with rollover handling
	    int32_t delta = (int32_t)(encoder->cnt3 - encoder->previous_cnt3);

	    encoder->position_steps += delta;

	    // Update previous counter
	    encoder->previous_cnt3 = encoder->cnt3;

	    // Handle range limits
	    if (encoder->position_steps < INT32_MIN) {
	        range_error = -1;
	        encoder->position_steps = INT32_MIN;
	    } else if (encoder->position_steps > INT32_MAX) {
	        range_error = 1;
	        encoder->position_steps = INT32_MAX;
	    }

	    // Subtract zeroing offset
	    encoder->position_steps -= encoder->position_init;

	    // Calculate position in revolutions
	    encoder->position = (float)encoder->position_steps / (float)encoder->counts_per_turn;

	/*
	 *  Detect if we passed the bottom, then re-arm peak flag
	 *  oppositeSigns returns true when we pass the bottom position
	 */


	if (__HAS_OPPOSITE_SIGNS(encoder->position_steps, encoder->previous_position))
	{
		encoder->peaked = FALSE;
		encoder->zero_crossed = TRUE;
	}

	if (!encoder->peaked) // We don't need to evaluate anymore if we hit a maximum when we're still in downward motion and didn't cross the minimum
	{
		// Add global maximum
		if (labs(encoder->position_steps) >= labs(encoder->global_max_position))
		{
			encoder->global_max_position = encoder->position_steps;
		}
		// Check if new maximum
		if (labs(encoder->position_steps) >= labs(encoder->max_position))
		{
			encoder->max_position = encoder->position_steps;
		}
		else
		{
			// We are at the peak and disable further checks until we traversed the minimum position again
			encoder->peaked = TRUE;
			encoder->handled_peak = FALSE;
		}
	}

	encoder->previous_position = encoder->position_steps;


	return range_error;
}

__STATIC_INLINE void DWT_Delay_us(volatile uint32_t microseconds)
{
	uint32_t clk_cycle_start = DWT->CYCCNT;

	/* Go to number of cycles for system */
	microseconds *= (RCC_HCLK_FREQ / 1000000);

	/* Delay till end */
	while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

__STATIC_INLINE void DWT_Delay_until_cycle(volatile uint32_t cycle)
{
	while (DWT->CYCCNT < cycle);
}

int32_t swap_Endians_32(int32_t value)
{

	int32_t leftmost_byte;
	int32_t left_middle_byle;
	int32_t right_middle_byte;
	int32_t rightmost_byte;

	int32_t result;

    leftmost_byte = (value & 0x000000FF) >> 0;
    left_middle_byle = (value & 0x0000FF00) >> 8;
    right_middle_byte = (value & 0x00FF0000) >> 16;
    rightmost_byte = (value & 0xFF000000) >> 24;
    leftmost_byte <<= 24;
    left_middle_byle <<= 16;
    right_middle_byte <<= 8;
    rightmost_byte <<= 0;

    result = (leftmost_byte | left_middle_byle
              | right_middle_byte | rightmost_byte);

    return result;
}

int16_t swap_Endians_16(int16_t value)
{

	int16_t leftmost_byte;
	int16_t rightmost_byte;

	int16_t result;

    leftmost_byte = (value & 0x00FF) >> 0;
    rightmost_byte = (value & 0xFF00) >> 8;
    leftmost_byte <<= 8;
    rightmost_byte <<= 0;

    result = (leftmost_byte | rightmost_byte);

    return result;
}

void Debug_TimerStatus(void) {
    static uint32_t lastDebugTime = 0; // Timestamp of last debug print
    uint32_t currentTime = HAL_GetTick(); // Current system time in milliseconds
    TIM_HandleTypeDef hTimPwm1 = L6474_Board_Pwm1GetHandle();

   // Print debug information at most every 500 ms
   if ((currentTime - lastDebugTime) >= 500 && uart_xmit_flag == 1) {
	   uint32_t CNT = hTimPwm1.Instance->CNT;
	   uint32_t ARR = hTimPwm1.Instance->ARR;
	   uint32_t PSC = hTimPwm1.Instance->PSC;
	   uint32_t SR = hTimPwm1.Instance->SR;
	   uint32_t DIER = hTimPwm1.Instance->DIER;

	   // Combine all debug information into a single string
	   int uart_buf_len = sprintf(
		   uart_buf,
		   "==== Timer Debug Info ====\r\n"
		   "CNT: %lu, ARR: %lu, PSC: %lu, SR: 0x%lX, DIER: 0x%lX\r\n"
		   "==========================\r\n",
		   CNT, ARR, PSC, SR, DIER
	   );

	   // Initiate non-blocking UART transmission
	   if (HAL_UART_Transmit_IT(&huart2, (uint8_t *)uart_buf, uart_buf_len) == HAL_OK) {
		   uart_xmit_flag = 0; // Mark UART as busy
	   }

	   lastDebugTime = currentTime; // Update the timestamp
   }
}


//------------------------------------------------------
// Callbacks

void HAL_SPI_TxRxCpltCallback (SPI_HandleTypeDef * hspi)
{
	spi_txrx_flag = 1;
	// copy send_numberx to tx buffer
	memcpy(&spi_tx_buf[0], &send_number1 , 2);
	memcpy(&spi_tx_buf[2], &send_number2 , 2);
	memcpy(&spi_tx_buf[4], &send_number3 , 2);
	// copy rx buffer to recv_numberx
	memcpy(&recv_number1, &spi_rx_buf[0], 2);
	memcpy(&recv_number2, &spi_rx_buf[2], 2);
	memcpy(&recv_number3, &spi_rx_buf[4], 2);

}

void HAL_SPI_ErrorCallback (SPI_HandleTypeDef * hspi)
{
  // raise flag
  spi_err_flag = 1;
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef * huart)
{
  // raise flag
  uart_xmit_flag = 1;
}

void HAL_UART_ErrorCallback (UART_HandleTypeDef * huart)
{
  // Set CS pin to high and raise flag
	uart_err_flag = 1;
}

void MyFlagInterruptHandler(void)
{
  /* Get the value of the status register via the L6474 command GET_STATUS */
  uint16_t statusRegister = BSP_MotorControl_CmdGetStatus(0);

  /* Check HIZ flag: if set, power brigdes are disabled */
  if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ)
  {
    // HIZ state
    // Action to be customized
  }

  /* Check direction bit */
  if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR)
  {
    // Forward direction is set
    // Action to be customized
  }
  else
  {
    // Backward direction is set
    // Action to be customized
  }

  /* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
  /* This often occures when a command is sent to the L6474 */
  /* while it is in HIZ state */
  if ((statusRegister & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD)
  {
      // Command received by SPI can't be performed
     // Action to be customized
  }

  /* Check WRONG_CMD flag: if set, the command does not exist */
  if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD)
  {
     //command received by SPI does not exist
     // Action to be customized
  }

  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
  if ((statusRegister & L6474_STATUS_UVLO) == 0)
  {
     //undervoltage lock-out
     // Action to be customized
  }

  /* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
  if ((statusRegister & L6474_STATUS_TH_WRN) == 0)
  {
    //thermal warning threshold is reached
    // Action to be customized
  }

  /* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
  if ((statusRegister & L6474_STATUS_TH_SD) == 0)
  {
    //thermal shut down threshold is reached
    // Action to be customized
  }

  /* Check OCD  flag: if not set, there is an overcurrent detection */
  if ((statusRegister & L6474_STATUS_OCD) == 0)
  {
    //overcurrent detection
    // Action to be customized
  }

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  error number of the error
  * @retval None
  */
void Error_Handler(uint16_t error)
{
  /* Backup error number */
  gLastError = error;

  /* Infinite loop */
  while(1)
  {
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
