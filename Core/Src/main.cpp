/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include "Moteus.h"

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

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static const uint32_t LOOP_INTERVAL_MS = 1;

char pbuf[256];
size_t offset = 0;

Moteus moteus1(hfdcan1, huart2, htim2, []() {
	  Moteus::Options options;
	  options.id = 1;
	  options.disable_brs = false;
	  return options;
}());

Moteus moteus2(hfdcan1, huart2, htim2, []() {
	  Moteus::Options options;
	  options.id = 2;
	  options.disable_brs = false;
	  return options;
}());

uint32_t last_send = 0;
uint32_t loop_count = 0;

static const uint8_t AS5600_ADDR = 0x36 << 1;
static const uint8_t AS5600_REG_ANGLE_H = 0x0E;
static const uint8_t AS5600_REG_ANGLE_L = 0x0F;
static const double AS5600_PPR = 4096.0;

uint8_t i2c_buf[16];

double pan_target = 0;
double pan_last = 0;
int32_t pan_revs = 0;

double tilt_target = 0;
double tilt_last = 0;
int32_t tilt_revs = 0;

int pan_speed = 0;
int tilt_speed = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void home() {
	pan_target = 0;
	pan_last = 0;
	pan_revs = 0;

	tilt_target = 0;
	tilt_last = 0;
	tilt_revs = 0;

	sprintf(pbuf, "homed\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) pbuf, strlen(pbuf), 10);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_FDCAN1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_FDCAN_Start(&hfdcan1);

  sprintf(pbuf, "helo\n");
  HAL_UART_Transmit(&huart2, (uint8_t*) pbuf, strlen(pbuf), 10);

  moteus1.SetStop();
  moteus2.SetStop();

  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	const auto now = HAL_GetTick();
	if (now - last_send >= LOOP_INTERVAL_MS) {
//		sprintf(pbuf, "last loop time: %lu\n", now-last_send);
//		HAL_UART_Transmit(&huart2, (uint8_t*) pbuf, strlen(pbuf), 10);

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);



		// handle control panel inputs

		// home button
		if (HAL_GPIO_ReadPin(BTN_HOME_GPIO_Port, BTN_HOME_Pin) == GPIO_PIN_SET) {
			HAL_GPIO_WritePin(BTN_HOME_LED_GPIO_Port, BTN_HOME_LED_Pin, GPIO_PIN_SET);
			home();
		} else {
			HAL_GPIO_WritePin(BTN_HOME_LED_GPIO_Port, BTN_HOME_LED_Pin, GPIO_PIN_RESET);
		}

		// pan speed
		if (HAL_GPIO_ReadPin(PAN_SPEED_1_GPIO_Port, PAN_SPEED_1_Pin)) {
			pan_speed = 1;
		}
		if (HAL_GPIO_ReadPin(PAN_SPEED_2_GPIO_Port, PAN_SPEED_2_Pin)) {
			pan_speed = 2;
		}
		if (HAL_GPIO_ReadPin(PAN_SPEED_3_GPIO_Port, PAN_SPEED_3_Pin)) {
			pan_speed = 3;
		}

		// tilt speed
		if (HAL_GPIO_ReadPin(TILT_SPEED_1_GPIO_Port, TILT_SPEED_1_Pin)) {
			tilt_speed = 1;
		}
		if (HAL_GPIO_ReadPin(TILT_SPEED_2_GPIO_Port, TILT_SPEED_2_Pin)) {
			tilt_speed = 2;
		}
		if (HAL_GPIO_ReadPin(TILT_SPEED_3_GPIO_Port, TILT_SPEED_3_Pin)) {
			tilt_speed = 3;
		}

		if (loop_count % 100 == 0) {
			offset = 0;
			offset += sprintf(pbuf+offset, "mtr pwr: %d\n", HAL_GPIO_ReadPin(SW_MOTOR_POWER_GPIO_Port, SW_MOTOR_POWER_Pin));
			offset += sprintf(pbuf+offset, "pan_speed: %d\n", pan_speed);
			offset += sprintf(pbuf+offset, "tilt_speed: %d\n\n", tilt_speed);
			HAL_UART_Transmit(&huart2, (uint8_t*) pbuf, strlen(pbuf), 10);
			offset = 0;
		}



		//debug
		loop_count += 1;
		last_send = now;
		continue;

		// get sensor data
		// PAN
		i2c_buf[0] = AS5600_REG_ANGLE_H;
		HAL_I2C_Master_Transmit(&hi2c3, AS5600_ADDR, i2c_buf, 1, 1);
		HAL_I2C_Master_Receive(&hi2c3, AS5600_ADDR, i2c_buf, 2, 1);
		uint16_t pan_raw = (i2c_buf[0] << 8) | (i2c_buf[1]);
		double pan_float = pan_raw / AS5600_PPR;
		if (pan_float > 0.75 && pan_last < 0.25) { pan_revs--; }
		if (pan_float < 0.25 && pan_last > 0.75) { pan_revs++; }
		pan_last = pan_float;
//		pan_target = -(pan_revs+pan_float)/19.0;
		pan_target = -(pan_revs+pan_float)/35.5;
//		sprintf(pbuf, "pan_raw: %d\n", pan_raw);
		sprintf(pbuf, "pan_target: %f\n", pan_target);
		HAL_UART_Transmit(&huart2, (uint8_t*) pbuf, strlen(pbuf), 1);

		// TILT
		i2c_buf[0] = AS5600_REG_ANGLE_H;
		HAL_I2C_Master_Transmit(&hi2c1, AS5600_ADDR, i2c_buf, 1, 1);
		HAL_I2C_Master_Receive(&hi2c1, AS5600_ADDR, i2c_buf, 2, 1);
		uint16_t tilt_raw = (i2c_buf[0] << 8) | (i2c_buf[1]);
		double tilt_float = tilt_raw / AS5600_PPR;
		if (tilt_float > 0.75 && tilt_last < 0.25) { tilt_revs--; }
		if (tilt_float < 0.25 && tilt_last > 0.75) { tilt_revs++; }
		tilt_last = tilt_float;
		//		tilt_target = -(tilt_revs+tilt_float)/(4.25*6);
		tilt_target = -(tilt_revs+tilt_float)/(9.25*6);
//		sprintf(pbuf, "tilt_raw: %d\n\n", tilt_raw);
//		sprintf(pbuf, "tilt_target: %f\n", tilt_target);
//		HAL_UART_Transmit(&huart2, (uint8_t*) pbuf, strlen(pbuf), 10);


		// send motor commands

//		if (moteus1.last_result().values.mode != mjbots::moteus::Mode::kPosition) {
//			  moteus1.SetStop();
//		}
//
//		if (moteus2.last_result().values.mode != mjbots::moteus::Mode::kPosition) {
//			  moteus2.SetStop();
//		}

		Moteus::PositionMode::Command cmd;
		cmd.position = pan_target;
		cmd.velocity = 0.0f;
//		cmd.velocity_limit = 0.1f;
		cmd.accel_limit = 1.0f;
		moteus1.SetPosition(cmd);
		cmd.position = tilt_target;
		moteus2.SetPosition(cmd);


		FDCAN_ProtocolStatusTypeDef protocol_status;
		HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status);
	    // If we have gotten into a BusOff state, recover.
	    if (protocol_status.BusOff) {
	      hfdcan1.Instance->CCCR &= ~FDCAN_CCCR_INIT;
	    }


		////////////////////// DEBUG //////////////////////////
		if (loop_count % 50 == 0) {
			  auto print_moteus = [](const Moteus::Query::Result& query) {
			    offset += sprintf(pbuf+offset, "\n\nmode: %d\n", static_cast<int>(query.mode));
//			    offset += sprintf(pbuf+offset, "position: %f\n", query.position);
//			    offset += sprintf(pbuf+offset, "velocity: %f\n", query.velocity);
				HAL_UART_Transmit(&huart2, (uint8_t*) pbuf, strlen(pbuf), 1);
			  };


			  print_moteus(moteus1.last_result().values);
	//		  offset += sprintf(pbuf, " / ");
	//		  print_moteus(moteus2.last_result().values);
			  sprintf(pbuf, "requested position: %f\nrequested velocity: %f\nrequested accel limit: %f\n", cmd.position, cmd.velocity, cmd.accel_limit);
//				HAL_UART_Transmit(&huart2, (uint8_t*) pbuf, strlen(pbuf), 10);




			if (protocol_status.LastErrorCode != FDCAN_PROTOCOL_ERROR_NO_CHANGE || protocol_status.BusOff) {
				sprintf(pbuf,
						"\nlast error code: %lu\ndata last error code: %lu\nactivity: %lu\nbus off: %lu\ndelay comp: %lu\n\n",
						protocol_status.LastErrorCode,
						protocol_status.DataLastErrorCode,
						protocol_status.Activity,
						protocol_status.BusOff,
						protocol_status.TDCvalue);
				HAL_UART_Transmit(&huart2, (uint8_t*) pbuf, strlen(pbuf), 1);
			}

//			auto fdcan_state = HAL_FDCAN_GetState(&hfdcan1);
//			sprintf(pbuf, "fdcan_state: %d\n", fdcan_state);
//			HAL_UART_Transmit(&huart2, (uint8_t*) pbuf, strlen(pbuf), 10);
		}

		loop_count += 1;
		last_send = now;
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 16;
  hfdcan1.Init.NominalTimeSeg1 = 56;
  hfdcan1.Init.NominalTimeSeg2 = 28;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 5;
  hfdcan1.Init.DataTimeSeg1 = 11;
  hfdcan1.Init.DataTimeSeg2 = 5;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  if (HAL_FDCAN_ConfigGlobalFilter(
          &hfdcan1,
          FDCAN_ACCEPT_IN_RX_FIFO0,
          FDCAN_ACCEPT_IN_RX_FIFO0,
          FDCAN_FILTER_REMOTE,
          FDCAN_FILTER_REMOTE) != HAL_OK) {
    Error_Handler();
  }

  HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);
  HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 10, 0);

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00C20F27;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /** I2C Fast mode Plus enable
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00C20F27;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /** I2C Fast mode Plus enable
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C3);
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 170-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(BTN_HOME_LED_GPIO_Port, BTN_HOME_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PAN_SPEED_2_Pin PAN_SPEED_1_Pin */
  GPIO_InitStruct.Pin = PAN_SPEED_2_Pin|PAN_SPEED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TILT_SPEED_3_Pin TILT_SPEED_2_Pin TILT_SPEED_1_Pin BTN_HOME_Pin */
  GPIO_InitStruct.Pin = TILT_SPEED_3_Pin|TILT_SPEED_2_Pin|TILT_SPEED_1_Pin|BTN_HOME_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_HOME_LED_Pin */
  GPIO_InitStruct.Pin = BTN_HOME_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BTN_HOME_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_MOTOR_POWER_Pin PAN_SPEED_3_Pin */
  GPIO_InitStruct.Pin = SW_MOTOR_POWER_Pin|PAN_SPEED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
