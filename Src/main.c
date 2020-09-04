/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "u8g2.h"
#include "screens.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WELDING_TIMEOUT_MAX 2000
#define WELDING_TIMEOUT_MIN 100
#define ENCODER_DEBUONCE_TIMEOUT 100

#define BUTTON_DEBOUNCE_TIMEOUT_PRESS 30
#define BUTTON_DEBOUNCE_TIMEOUT_RELEASE 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint8_t  encoderDebounceCounter = 0;
volatile uint8_t  timer_updated_flag = 0;
volatile uint16_t timer_value = 0;
volatile uint16_t timer_value_shadow = 0;
volatile uint16_t welding_timeout = 500;
volatile uint16_t welding_timeout_counter = 0;


volatile uint16_t welding_timeout_countdown_counter = 0;

volatile uint16_t button_debounce_timeout_counter = 500;
volatile uint8_t button_debouncing_flag = 0;
volatile uint8_t button_pressed_flag = 0;


static u8g2_t u8g2;
struct screen_data_holder test_struct_holder;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */


int8_t test_screen_function(uint16_t reg_addr);
int8_t render_time_value(uint16_t reg_addr);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int8_t test_screen_function(uint16_t reg_addr) {
//	  char string[] = "test";
//		HAL_UART_Transmit(&huart1, string, 4, 50);
//		HAL_UART_Transmit(&huart1, &reg_addr, 1, 50);
//	  	HAL_UART_Transmit(&huart1, " ", 1, 50);

		u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
		u8g2_DrawStr(&u8g2, 1, 20, "Time ");
		u8g2_DrawStr(&u8g2, 120, 20, "s");

	  	return 0;
};

int8_t render_time_value(uint16_t value) {
		uint16_t thouthands = value/1000;
		uint16_t leftover = value%1000;


		char str[10];
		sprintf (str, "%d.%03d", thouthands, leftover);
		u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
		u8g2_DrawStr(&u8g2, 55, 20, str);

	  	return 0;
};

uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
    U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
    U8X8_UNUSED void *arg_ptr)
{
  switch (msg)
  {
  case U8X8_MSG_GPIO_AND_DELAY_INIT:
    HAL_Delay(1);
    break;
  case U8X8_MSG_DELAY_MILLI:
    HAL_Delay(arg_int);
    break;
  case U8X8_MSG_GPIO_DC:
//    HAL_GPIO_WritePin(DISPLAY_DC_GPIO_Port, DISPLAY_DC_Pin, arg_int);
    break;
  case U8X8_MSG_GPIO_RESET:
//    HAL_GPIO_WritePin(DISPLAY_RESET_GPIO_Port, DISPLAY_RESET_Pin, arg_int);
    break;
  }
  return 1;
}

uint8_t u8x8_byte_sw_i2c_my(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  static uint8_t buffer[32];		/* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
  static uint8_t buf_idx;
  uint8_t *data;

  switch(msg)
  {
    case U8X8_MSG_BYTE_SEND:
      data = (uint8_t *)arg_ptr;
      while( arg_int > 0 )
      {
	buffer[buf_idx++] = *data;
	data++;
	arg_int--;
      }
      break;
    case U8X8_MSG_BYTE_INIT:
      /* add your custom code to init i2c subsystem */
      break;
    case U8X8_MSG_BYTE_SET_DC:
      /* ignored for i2c */
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
      buf_idx = 0;
      break;
    case U8X8_MSG_BYTE_END_TRANSFER:
    	HAL_I2C_Master_Transmit(&hi2c1,
    		/* uint16_t DevAddress, */
    		/* 0x3c<<1, correct address */
    			u8x8_GetI2CAddress(u8x8),
			/* uint8_t *pData, */
				buffer,
			/* uint16_t Size, */
				buf_idx,
			/* uint32_t Timeout); */
				50);
      break;
    default:
      return 0;
  }
  return 1;
}


void display_update(void) {
	u8g2_SetFont(&u8g2, u8g2_font_logisoso18_tr );

	char tmp_string[100];
	u8g2_FirstPage(&u8g2);
	do
	 {
		sprintf(tmp_string, "B xxx");
		u8g2_DrawStr(&u8g2, 0, 10, tmp_string);

	 } while (u8g2_NextPage(&u8g2));
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
//  test_struct_holder.screen_index = 1;
  test_struct_holder.screen_renderer[0] = render_time_value;
  test_struct_holder.screen_renderer[1] = test_screen_function;


  //  test_struct_holder.screen_renderer = test_screen_function;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

//  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_1);

  HAL_TIM_Base_Start_IT(&htim1);


  u8g2_Setup_ssd1306_i2c_128x32_univision_1(&u8g2,
		  U8G2_R0,
		  u8x8_byte_sw_i2c_my,
		  u8x8_stm32_gpio_and_delay); //[page buffer, size = 128 bytes]

  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);

  u8g2_FirstPage(&u8g2);

  	do
	{
//		u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
//		u8g2_DrawStr(&u8g2, 5, 20, "Hello world ");

		test_struct_holder.screen_renderer[0](welding_timeout);
  		test_struct_holder.screen_renderer[1](0x00);
	} while (u8g2_NextPage(&u8g2));




//	HAL_StatusTypeDef result;
// 	uint8_t i;
// 	for (i=1; i<128; i++)
// 	{
// 	  /*
// 	   * the HAL wants a left aligned i2c address
// 	   * &hi2c1 is the handle
// 	   * (uint16_t)(i<<1) is the i2c address left aligned
// 	   * retries 2
// 	   * timeout 2
// 	   */
// 	  result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 2);
// 	  if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
// 	  {
//// 		  printf("."); // No ACK received at that address
// 		  uint8_t test = 0x00;
// 		  HAL_UART_Transmit(&huart1, &test, 1, 50);
//
// 	  }
// 	  if (result == HAL_OK)
// 	  {
//// 		  printf("0x%X", i); // Received an ACK at that address
// 		HAL_UART_Transmit(&huart1, &i, 1, 50);
// 	  }
// 	}


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  button_pressed_flag
//	  button_debounce_timeout_counter
//	  BUTTON_DEBOUNCE_TIMEOUT

	if (button_pressed_flag == 0) {
		if (button_debouncing_flag == 0) {
			/* if pressed */
			if ( ! HAL_GPIO_ReadPin(GPIOA, BUTTON_Pin)) {
				button_debouncing_flag = 1;
				button_debounce_timeout_counter = BUTTON_DEBOUNCE_TIMEOUT_PRESS;
			}
		} else {
			if (! HAL_GPIO_ReadPin(GPIOA, BUTTON_Pin)) {
				if (button_debounce_timeout_counter == 0) {
					button_debouncing_flag = 0;
					button_pressed_flag = 1;
					/* pressed action here */


					HAL_GPIO_WritePin(GPIOA, TEST_LED_Pin | TRIAC_CONTROL_Pin, GPIO_PIN_SET);
					HAL_Delay(75);
					HAL_GPIO_WritePin(GPIOA, TEST_LED_Pin | TRIAC_CONTROL_Pin, GPIO_PIN_RESET);
					HAL_Delay(50);

					welding_timeout_counter = welding_timeout;
					HAL_GPIO_WritePin(GPIOA, TEST_LED_Pin | TRIAC_CONTROL_Pin, GPIO_PIN_SET);

//					char str1[10];
//					sprintf (str1, "press \r\n");
//					HAL_UART_Transmit(&huart1, (uint8_t*)str1, 10, 50);
				}

			/* if not pressed but debounce time still running*/
			} else {
				button_debouncing_flag = 0;
				button_pressed_flag = 0;
			}
		}
	} else {
		if (button_debouncing_flag == 0) {
			/* if not pressed */
			if (HAL_GPIO_ReadPin(GPIOA, BUTTON_Pin)) { /* change */
				button_debouncing_flag = 1;
				button_debounce_timeout_counter = BUTTON_DEBOUNCE_TIMEOUT_RELEASE;
			}
		} else {
			if (HAL_GPIO_ReadPin(GPIOA, BUTTON_Pin)) { /* change */
				if (button_debounce_timeout_counter == 0) {
					button_debouncing_flag = 0;
					button_pressed_flag = 0; /* change */
					/* released action here */
//					char str1[10];
//					sprintf (str1, "release \r\n");
//					HAL_UART_Transmit(&huart1, (uint8_t*)str1, 10, 50);
				}
			} else {
				button_debouncing_flag = 0;
				button_pressed_flag = 1; /* change */
			}
		}
	}





	  if (timer_updated_flag == 1) {
		  timer_updated_flag = 0;

		  if (encoderDebounceCounter == 0) {
			  encoderDebounceCounter = ENCODER_DEBUONCE_TIMEOUT;

			  /* timer incrementing value >> */
			  if (timer_value_shadow < timer_value) {
				  welding_timeout += 50;
				  if (welding_timeout > WELDING_TIMEOUT_MAX) {
					  welding_timeout = WELDING_TIMEOUT_MAX;
				  }
			  } else {
				  welding_timeout -= 50;
				  if (welding_timeout < WELDING_TIMEOUT_MIN) {
					  welding_timeout = WELDING_TIMEOUT_MIN;
				  }
			  }
			  timer_value_shadow = timer_value;
			  /* timer incrementing value << */
		  }



//			uint8_t test_data[2] = { (uint8_t)(timer_value >> 8), (uint8_t)(timer_value & 0x00ff) };
			u8g2_FirstPage(&u8g2);
			do
			{
				test_struct_holder.screen_renderer[0](welding_timeout);
				test_struct_holder.screen_renderer[1](0);
			} while (u8g2_NextPage(&u8g2));

			char str[10];
			sprintf (str, "%d \r\n", timer_value);
			HAL_UART_Transmit(&huart1, (uint8_t*)str, 10, 50);
	  }

//	HAL_GPIO_WritePin(GPIOA, TEST_LED_Pin, GPIO_PIN_SET);
//	HAL_Delay(1500);
//	HAL_GPIO_WritePin(GPIOA, TEST_LED_Pin, GPIO_PIN_RESET);
//	HAL_Delay(1500);
//	display_update();
//	uint8_t test_data[2] = { 0x05, 0x02 };
//	HAL_UART_Transmit(&huart1, test_data, 2, 50);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x0000020B;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8;
  htim1.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIAC_CONTROL_Pin|TEST_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TRIAC_CONTROL_Pin TEST_LED_Pin */
  GPIO_InitStruct.Pin = TRIAC_CONTROL_Pin|TEST_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
