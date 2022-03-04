/* USER CODE BEGIN Header */
/**
 * Snake game by Alireza Tabatabaee, 2022. https://github.com/alireza-tabatabaee
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4_pcd8544.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	UP		1
#define LEFT	2
#define RIGHT	3
#define DOWN	4
#define FOOD	5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
uint8_t grid[21][12];	// game grid
int8_t hx, hy;			// x and y of head
uint8_t tx, ty;			// x and y of tail
int8_t dx, dy;			// direction of snake's movement
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t lfsr;
uint16_t lfsrrand()
{
	lfsr ^= lfsr>>7;
	lfsr ^= lfsr<<9;
	lfsr ^= lfsr>>13;
	return lfsr;
}
_Bool foodflag = 0;
void newGameStart(void)
{
	// clean out the game grid
	for(int i=0; i<21; i++)
	{
		for(int j=0; j<12; j++)
			grid[i][j] = 0;
	}
	// putting the snake with length 3 between (4,6) and (6,6)
	grid[4][6] = RIGHT;
	grid[5][6] = RIGHT;
	grid[6][6] = RIGHT;
	// (6,6) is the initial position for snake's head
	hx = 6;
	hy = 6;
	// (4,6) is the initial position for snake's tail
	tx = 4;
	ty = 6;
	// initial moving direction of snake is towards right
	dx = 1;
	dy = 0;
	// giving the LFSR pseudo-random number generator a seed from the timer
	lfsr = TIM1->CNT;
	//
	foodflag = 0;
}

uint8_t checkSnakeCrash(void)
{
	return ((hx+dx)==-1||(hx+dx)==22||(hy+dy)==13||((hy+dy)==-1))||(grid[hx+dx][hy+dy] > 0 && grid[hx+dx][hy+dy]<5);
}

void updateLCD(void)
{
	for(int i=0; i<21; i++)
	{
		for(int j=0; j<12; j++)
		{
			if (grid[i][j] == 0)
			{
				for(int k=0; k<4; k++)
				{
					for(int l=0; l<4; l++)
						PCD8544_DrawPixel(4*i+k, 4*j+l, 0);
				}
			}
			else if(grid[i][j] >0 && grid[i][j]<5)
			{
				for(int k=0; k<4; k++)
				{
					for(int l=0; l<4; l++)
						PCD8544_DrawPixel(4*i+k, 4*j+l, 1);
				}
			}
			else if(grid[i][j] == 5)
			{
				PCD8544_DrawPixel(4*i+1, 4*j, 1);
				PCD8544_DrawPixel(4*i+2, 4*j, 1);
				PCD8544_DrawPixel(4*i+1, 4*j+3, 1);
				PCD8544_DrawPixel(4*i+2, 4*j+3, 1);
				PCD8544_DrawPixel(4*i, 4*j+1, 1);
				PCD8544_DrawPixel(4*i, 4*j+2, 1);
				PCD8544_DrawPixel(4*i+3, 4*j+1, 1);
				PCD8544_DrawPixel(4*i+3, 4*j+2, 1);
			}
		}
	}
	// drawing borders on LCD to make it clearer
	PCD8544_DrawLine(0, 0, 0, 47, PCD8544_Pixel_Set);
	PCD8544_DrawLine(83, 0, 83, 47, PCD8544_Pixel_Set);
	PCD8544_DrawLine(0, 0, 83, 0, PCD8544_Pixel_Set);
	PCD8544_DrawLine(0, 47, 83, 47, PCD8544_Pixel_Set);

	PCD8544_Refresh();
}

void loseDialog()
{
	PCD8544_GotoXY(20, 20);
	PCD8544_Puts("You Lose!", PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
	PCD8544_Refresh();
	HAL_TIM_Base_Stop_IT(&htim1);
}

char keyPressed = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_8) //A,B,C,D
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin)==1){
			GPIOD->ODR |= (0xF << 12);
			keyPressed = 'D';
			HAL_Delay(100);
			return;
		}
		//
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin)==1){
			GPIOD->ODR |= (0xF << 12);
			keyPressed = 'C';
			HAL_Delay(100);
			return;
		}
		//
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin)==1){
			GPIOD->ODR |= (0xF << 12);
			keyPressed = 'B';
			HAL_Delay(100);
			return;
		}
		//
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin)==1){
			GPIOD->ODR |= (0xF << 12);
			keyPressed = 'A';
			HAL_Delay(100);
			return;
		}
	}
	else if(GPIO_Pin==GPIO_PIN_9) //3,6,9,#
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin)==1){
			GPIOD->ODR |= (0xF << 12);
			keyPressed = '#';
			HAL_Delay(100);
			return;
		}
		//
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin)==1){
			GPIOD->ODR |= (0xF << 12);
			keyPressed = '9';
			HAL_Delay(100);
			return;
		}
		//
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin)==1){
			GPIOD->ODR |= (0xF << 12);
			keyPressed = '6';
			HAL_Delay(100);
			return;
		}
		//
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin)==1){
			GPIOD->ODR |= (0xF << 12);
			keyPressed = '3';
			HAL_Delay(100);
			return;
		}
	}
	else if(GPIO_Pin==GPIO_PIN_10) //2,5,8,0
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin)==1){
			GPIOD->ODR |= (0xF << 12);
			keyPressed = '0';
			HAL_Delay(100);
			return;
		}
		//
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin)==1){
			GPIOD->ODR |= (0xF << 12);
			keyPressed = '8';
			HAL_Delay(100);
			return;
		}
		//
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin)==1){
			GPIOD->ODR |= (0xF << 12);
			keyPressed = '5';
			HAL_Delay(100);
			return;
		}
		//
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin)==1){
			GPIOD->ODR |= (0xF << 12);
			keyPressed = '2';
			HAL_Delay(100);
			return;
		}
	}
	else if(GPIO_Pin==GPIO_PIN_11) //1,4,7,*
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin)==1){
			GPIOD->ODR |= (0xF << 12);
			keyPressed = '*';
			HAL_Delay(100);
			return;
		}
		//
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin)==1){
			GPIOD->ODR |= (0xF << 12);
			keyPressed = '7';
			HAL_Delay(100);
			return;
		}
		//
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin)==1){
			GPIOD->ODR |= (0xF << 12);
			keyPressed = '4';
			HAL_Delay(100);
			return;
		}
		//
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin)==1){
			GPIOD->ODR |= (0xF << 12);
			keyPressed = '1';
			HAL_Delay(100);
			return;
		}
	}
	GPIOD->ODR |= (0xF << 12);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  //HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
  if(keyPressed == '2' && dy!=1)
  {
	  dx = 0, dy = -1;
	  grid[hx][hy] = UP;
	  keyPressed = 0;	//clean the key flag so it wouldn't be registered twice during the next iteration
  }
  else if(keyPressed == '4' && dx!=1)
  {
	  dx = -1, dy = 0;
	  grid[hx][hy] = LEFT;
	  keyPressed = 0;
  }
  else if(keyPressed == '6' && dx!=-1)
  {
	  dx = 1, dy = 0;
	  grid[hx][hy] = RIGHT;
	  keyPressed = 0;
  }
  else if(keyPressed == '8' && dy!=-1)
  {
	  dx = 0, dy = 1;
	  grid[hx][hy] = DOWN;
	  keyPressed = 0;
  }


  if( checkSnakeCrash() )
	  loseDialog();
  else
  {
	  uint8_t temp = grid[tx][ty];

	  if(grid[hx+dx][hy+dy] != FOOD) {
		  // clean the former tail if no food was eaten
		  grid[tx][ty] = 0;
		  // update the location of the tail
		  if(temp==UP)
			  ty--;
		  else if(temp==LEFT)
			  tx--;
		  else if(temp==RIGHT)
			  tx++;
		  else if(temp==DOWN)
			  ty++;
	  }
	  else
		  foodflag = 0;

	  // advance snake's head for one cell
	  temp = grid[hx][hy];
	  hx += dx;
	  hy += dy;
	  grid[hx][hy] = temp;
  //}

	  if(!foodflag)
	  {
		  uint8_t foodx, foody;
		  do
		  {
			  foodx = lfsrrand() % 21;
			  foody = lfsrrand() % 12;
		  }
		  while(grid[foodx][foody] != 0);
		  grid[foodx][foody] = FOOD;
		  foodflag = 1;
	  }

	  updateLCD();	//finally, show the results on LCD
  }
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
  MX_SPI3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htim1);
  PCD8544_Init(0x38);
  newGameStart();
  //PCD8544_Puts("Hello World!", PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
  //PCD8544_Refresh();
  while (1)
  {
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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 11999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE8 PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15
                           PD3 PD5 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
