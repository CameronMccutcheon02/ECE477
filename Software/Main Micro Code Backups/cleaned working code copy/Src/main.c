/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main code for Autonomous Air Hockey Robot
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;			//SPI for 7-segment display
TIM_HandleTypeDef htim1;			//tim1 - MOTORS
TIM_HandleTypeDef htim2;			//tim2 - DECELERATION
TIM_HandleTypeDef htim3;			//tim3 - ACCELERATION
TIM_HandleTypeDef htim15;			//tim15 - MULTIPLEXING for 7Segment
UART_HandleTypeDef huart2;			//UART2 for communications
DMA_HandleTypeDef hdma_usart2_rx;	//DMA RX variable for receiving data over UART

/* Private function prototypes -----------------------------------------------*/
//General Initializations
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//Timers
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
//Communications
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);

/* Variable Declaration */
uint8_t rxdata[4];					//buffer for data sent from PC/camera
int target_ccr_acc = 100; 			//target CCR value for TIM1 when accelerating (top speed)
int target_ccr_dec = 600;			//target CCR value for TIM1 when decelerating (lowest speed before stopping)
int starting_ccr = 250; 			//initial CCR value for TIM1 (starting speed)
int down_flag=0;
int up_flag=0;
int left_flag=0;
int righ_flag=0;
int dl_flag=0;
int ur_flag=0;
int dr_flag=0;
int ul_flag=0;

//SPI PINS and variables
const int latchPin = ((uint16_t)0x0010U);		//variable for pulling latchPin high and low
#define HOLDING_CYCLES 4
uint8_t segvalue[8] = {0, 0, 0, 0, 0, 0, 0, 0};	//array storing the numbers shown on 7-segment display
uint8_t display_select = 0;						//initial display selection (rightmost middle display)
uint8_t hold = 0;								//used for multiplexing larger displays

//Main code
int main(void)
{
	//Initializations
	HAL_Init();					//initialize all peripherals, flash interface, and Systick
	SystemClock_Config();		//configure system clock (8 MHz)

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();

  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();

  /* Start acceleration timer and multiplexing timer */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim15);

  /* Initialize rxdata to receive data over UART  */
  HAL_UART_Receive_DMA(&huart2, rxdata, sizeof(rxdata));

  /* Initialize PWM signals */
  HAL_Delay(3000);				//delay 3s to allow for tolerance w motor drivers
  TIM1->CCR1 = 175;
  TIM1->CCR3 = 175;
  TIM1->ARR = 350;
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //scan for each direction flag and enter the given functions accordingly
	  if(down_flag){down();}
	  else if(up_flag){up();}
	  else if(left_flag){left();}
	  else if(righ_flag){righ();}
	  else if(dl_flag){dl();}
	  else if(ur_flag){ur();}
	  else if(dr_flag){dr();}
	  else if(ul_flag){ul();}
	  }
 }


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function (MOTORS)
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function (DECELERATION)
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 40000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function (ACCELERATION)
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 40000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM15 Initialization Function (MULTIPLEXING)
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 7;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 200;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Ch1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Test_Pin1_Pin|Test_Pin2_Pin|DIR_A_Pin|DIR_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Test_Pin3_Pin|Test_Pin4_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Test_Pin5_Pin|Test_Pin6_Pin|Test_Pin7_Pin|Test_Pin8_Pin
                          |Test_Pin9_Pin|SPI_Latch_Pin|DIR_IC_Pin|EN_A_Pin
                          |EN_B_Pin|OE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Test_Pin1_Pin Test_Pin2_Pin */
  GPIO_InitStruct.Pin = Test_Pin1_Pin|Test_Pin2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Test_Pin3_Pin Test_Pin4_Pin */
  GPIO_InitStruct.Pin = Test_Pin3_Pin|Test_Pin4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Test_Pin5_Pin Test_Pin6_Pin Test_Pin7_Pin Test_Pin8_Pin
                           Test_Pin9_Pin DIR_IC_Pin OE_Pin */
  GPIO_InitStruct.Pin = Test_Pin5_Pin|Test_Pin6_Pin|Test_Pin7_Pin|Test_Pin8_Pin
                          |Test_Pin9_Pin|DIR_IC_Pin|OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_Sensor_IO1_Pin */
  GPIO_InitStruct.Pin = IR_Sensor_IO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IR_Sensor_IO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Limit_SW1_Pin Limit_SW2_Pin Limit_SW3_Pin Limit_SW4_Pin */
  GPIO_InitStruct.Pin = Limit_SW1_Pin|Limit_SW2_Pin|Limit_SW3_Pin|Limit_SW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_A_Pin DIR_B_Pin */
  GPIO_InitStruct.Pin = DIR_A_Pin|DIR_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_Latch_Pin */
  GPIO_InitStruct.Pin = SPI_Latch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_Latch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_A_Pin EN_B_Pin */
  GPIO_InitStruct.Pin = EN_A_Pin|EN_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* Used for displaying each player's score on the 7-segment display */
	UNUSED(GPIO_Pin);
	if(GPIO_Pin == GPIO_PIN_6)	//IR sensor 1
	{
		if(segvalue[6] >= 9){
			segvalue[7] = segvalue[7] + 1;
			segvalue[6] = 0;
		}
		else
		{
		segvalue[6] = segvalue[6] + 1;
		}
	}
	else if(GPIO_Pin == GPIO_PIN_7)	//IR sensor 2
	{
		if(segvalue[4] >= 9){
			segvalue[5] = segvalue[5] + 1;
			segvalue[4] = 0;
		}
		else
		{
		segvalue[4] = segvalue[4] + 1;
		}
	}

}

void down()
{
	/* Called to move mallet down */
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, (GPIO_PIN_9), SET);
	HAL_GPIO_WritePin(GPIOA, (GPIO_PIN_11), SET);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	down_flag=0;
}

void up()
{
	/* Called to move mallet up */
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, (GPIO_PIN_9), RESET);
	HAL_GPIO_WritePin(GPIOA, (GPIO_PIN_11), RESET);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	up_flag=0;
}

void left()
{
	/* Called to move mallet left */
	HAL_GPIO_WritePin(GPIOA, (GPIO_PIN_9), SET);
	HAL_GPIO_WritePin(GPIOA, (GPIO_PIN_11), RESET);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	left_flag=0;
}

void righ()
{
	/* Called to move mallet right */
	HAL_GPIO_WritePin(GPIOA, (GPIO_PIN_9), RESET);
	HAL_GPIO_WritePin(GPIOA, (GPIO_PIN_11), SET);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	righ_flag=0;
}

void dl()
{
	/* Called to move mallet down and left */
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, (GPIO_PIN_9), SET);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	dl_flag=0;
}

void ur()
{
	/* Called to move mallet up and right */
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOA, (GPIO_PIN_9), RESET);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	ur_flag=0;
}

void dr()
{
	/* Called to move mallet down and right */
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, (GPIO_PIN_11), SET);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	dr_flag=0;
}

void ul()
{
	/* Called to move mallet up and left */
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOA, (GPIO_PIN_11), RESET);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	ul_flag=0;
}

void reset_and_accelerate()
{
	/* Called to reset motors to initial speed whenever data is received over UART */
	starting_ccr = 350;
	TIM1->CCR1 = (0.5) * starting_ccr;
	TIM1->CCR3 = (0.5) * starting_ccr;
	TIM1->ARR = starting_ccr;
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)	//rx transfer completed caldrack
{
	/* Callback function triggered when data is received over UART */

	UNUSED(huart);
	HAL_UART_Transmit(&huart2, rxdata, sizeof(rxdata), 100);
	reset_and_accelerate();
	if(strstr(rxdata, "down")){
		down_flag=1;
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Start_IT(&htim3);
	}
	else if(strstr(rxdata, "up00")){
		up_flag=1;
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Start_IT(&htim3);
	}
	else if(strstr(rxdata, "left")){
		left_flag=1;
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Start_IT(&htim3);
	}
	else if(strstr(rxdata, "righ")){
		righ_flag=1;
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Start_IT(&htim3);
	}
	else if(strstr(rxdata, "dl00")){
		dl_flag=1;
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Start_IT(&htim3);
	}
	else if(strstr(rxdata, "ul00")){
		ul_flag=1;
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Start_IT(&htim3);
	}
	else if(strstr(rxdata, "dr00")){
		dr_flag=1;
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Start_IT(&htim3);
	}
	else if(strstr(rxdata, "ur00")){
		ur_flag=1;
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Start_IT(&htim3);
	}
	else{
		HAL_TIM_Base_Start_IT(&htim2);
		HAL_TIM_Base_Stop_IT(&htim3);
	}
}

uint8_t TranslateDigit(int digit) {
	/* Used in translating decimal digits to bytes to be send over SPI */

    uint8_t segments[10] = {0xfc, 0x60, 0xda, 0xf2, 0x66, 0xb6, 0xbe, 0xe0, 0xfe, 0xf6};
    if (digit >= 0 && digit <= 9) {
        return segments[digit];
    } else {
        // Return 0xFF for an invalid digit (all segments off)
        return 0xff;
    }
}
// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* Callback function for all timer interrupts */
	if (htim == &htim2)
	{
		/* Start Deceleration */
	  if (starting_ccr < target_ccr_dec)
	  {
		  starting_ccr = starting_ccr * 1.1;
		  TIM1->CCR1 = (0.5) * starting_ccr;
		  TIM1->CCR3 = (0.5) * starting_ccr;
		  TIM1->ARR = starting_ccr;
	  }
	  if (starting_ccr >= target_ccr_dec)
	  {
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	  }
	}
	if (htim == &htim3)
	{
		/* Start Acceleration  */

		if(starting_ccr > target_ccr_acc)
		{
			starting_ccr = starting_ccr * .9;
			TIM1->CCR1 = (0.5) * starting_ccr;
			TIM1->CCR3 = (0.5) * starting_ccr;
			TIM1->ARR = starting_ccr;
		}
	}

	if(htim == &htim15){
		/* Start Multiplexing  */

		HAL_GPIO_WritePin(GPIOB, latchPin, GPIO_PIN_SET);	// pull latch pin HIGH
		uint8_t segment;
		segment = TranslateDigit(segvalue[display_select]);
		segment = ~segment;

		uint8_t tx_data[2];

		tx_data[0] = segment;
		tx_data[1] = (0x01 << display_select);

		HAL_SPI_Transmit(&hspi1, tx_data, 2, 100);
		HAL_GPIO_WritePin(GPIOB, latchPin, GPIO_PIN_RESET);	// pull latch pin HIGH

		if ((display_select > 3) && (hold < HOLDING_CYCLES)) {
			hold++;
		}
		else {
			display_select++;
			if (display_select == 8) {
			  display_select = 0;
			}
			hold = 0;
		}
	}

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

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
