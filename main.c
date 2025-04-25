/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body Final Project SP2025
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
#include "seg7.h"

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
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char ramp = 0;
char RED_BRT = 0;
char GREEN_BRT = 0;
char BLUE_BRT = 0;
char RED_STEP = 1;
char GREEN_STEP = 2;
char BLUE_STEP = 3;
char DIM_Enable = 0;
char Music_ON = 0;
int TONE = 0;
int COUNT = 0;
int INDEX = 0;
int Note = 0;
int Save_Note = 0;
int Vibrato_Depth = 1;
int Vibrato_Rate = 40;
int Vibrato_Count = 0;
char Animate_On = 0;
char Message_Length = 0;
char *Message_Pointer;
char *Save_Pointer;
int Delay_msec = 0;
int Delay_counter = 0;


/* HELLO ECE-330L */
char Message[] =
		{SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,
		CHAR_H,CHAR_E,CHAR_L,CHAR_L,CHAR_O,SPACE,CHAR_E,CHAR_C,CHAR_E,DASH,CHAR_3,CHAR_3,CHAR_0,CHAR_L,
		SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE};

/* Declare array for Song */
Music Song[100];
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
  MX_TIM7_Init();
  //MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /********************************************************************
   * PWR->CR |= ???;  //Enable Real Time Clock (RTC) Register Access  *
   * RCC->BDCR |= ???;  //Set clock source for RTC                    *
   * RCC->BDCR |= ???; //Enable RTC									  *
   ********************************************************************/

  /*** Configure GPIOs ***/
  GPIOD->MODER = 0x55555555; // set all Port D pins to outputs
  GPIOA->MODER |= 0x000000FF; // Port A mode register - make A0 to A3 analog pins
  GPIOE->MODER |= 0x55555555; // Port E mode register - make E0 to E15 outputs
  GPIOC->MODER |= 0x0; // Port C mode register - all inputs
  GPIOE->ODR = 0xFFFF; // Set all Port E pins high

  /*** Configure ADC1 ***/
  RCC->APB2ENR |= 1<<8;  // Turn on ADC1 clock by forcing bit 8 to 1 while keeping other bits unchanged
  ADC1->SMPR2 |= 1; // 15 clock cycles per sample
  ADC1->CR2 |= 1;        // Turn on ADC1 by forcing bit 0 to 1 while keeping other bits unchanged

  /*** Configure Clock ***/
  PWR->CR |= 1<<8;
  RCC->BDCR |= 2<<8;
  RCC->BDCR |= 1<<15;

  /*****************************************************************************************************
  These commands are handled as part of the MX_TIM7_Init() function and don't need to be enabled
  RCC->AHB1ENR |= 1<<5; // Enable clock for timer 7
  __enable_irq(); // Enable interrupts
  NVIC_EnableIRQ(TIM7_IRQn); // Enable Timer 7 Interrupt in the NVIC controller
  *******************************************************************************************************/

  TIM7->PSC = 199; //250Khz timer clock prescaler value, 250Khz = 50Mhz / 200
  TIM7->ARR = 1; // Count to 1 then generate interrupt (divide by 2), 125Khz interrupt rate to increment byte counter for 78Hz PWM
  TIM7->DIER |= 1; // Enable timer 7 interrupt
  TIM7->CR1 |= 1; // Enable timer counting

  /* Jeopardy Song */
  Song[0].note = A4;
  Song[0].size = quarter;
  Song[0].tempo = 1400;
  Song[0].space = 10;
  Song[0].end = 0;

  Song[1].note = D5;
  Song[1].size = quarter;
  Song[1].tempo = 1400;
  Song[1].space = 10;
  Song[1].end = 0;

  Song[2].note = A4;
  Song[2].size = quarter;
  Song[2].tempo = 1400;
  Song[2].space = 10;
  Song[2].end = 0;


  static int initial_pos = 0;

  // Set Initial LED
  GPIOD->ODR = (1 >> 0);

  while (1)
  {
	/**************************
	 * Show Current Clock Time
	 **************************/
	  if ((GPIOC->IDR & 0x3) == 0x0){

		  Seven_Segment_Digit(7, CHAR_C, 0);
		  Seven_Segment_Digit(6, CHAR_L, 0);
		  Seven_Segment_Digit(5, CHAR_C, 0);
		  Seven_Segment_Digit(4, SPACE, 0);

		  /*** Hour Set up ***/
		  int hours = (RTC->TR >> 16) & 0x3F;

		  // Getting ones spot for hours
		  int hours_tens = (hours >> 4) & 0x7;
		  Seven_Segment_Digit(3,hours_tens,0);

		  // Getting ones spot for hours
		  int hours_ones = hours & 0xF;
		  Seven_Segment_Digit(2, hours_ones, 1);


		  /*** Minute Set up ***/
		  int minute = ((RTC->TR >> 8) & 0x7F);

		  // Getting tens spot for minutes
		  int minute_tens = (minute >> 4) & 0x7;
		  Seven_Segment_Digit(1,minute_tens,0);

		  // Getting ones spot for minutes
		  int minute_ones = minute & 0xF;
		  Seven_Segment_Digit(0, minute_ones, 0);
		  HAL_Delay(1);
	  }

	/*****************************
	 * Update Clock Configuration
	 *****************************/
	  else if ((GPIOC->IDR & 0x3) == 0x2){
		  int hours_tenths = 0;
		  int hours_ones = 0;
		  int minutes_tenths = 0;
		  int minutes_ones = 0;

		  Seven_Segment_Digit(7, CHAR_A, 0);
		  Seven_Segment_Digit(6, CHAR_L, 0);
		  Seven_Segment_Digit(5, CHAR_T, 0);
		  Seven_Segment_Digit(4, SPACE, 0);
		  Seven_Segment_Digit(3, CHAR_C, 0);
		  Seven_Segment_Digit(2, CHAR_L, 0);
		  Seven_Segment_Digit(1, CHAR_O, 0);
		  Seven_Segment_Digit(0, CHAR_C, 0);

		  HAL_Delay(2000);

		  Seven_Segment_Digit(7, CHAR_A, 0);
		  Seven_Segment_Digit(6, CHAR_L, 0);
		  Seven_Segment_Digit(5, CHAR_T, 0);
		  Seven_Segment_Digit(4, SPACE, 0);
		  Seven_Segment_Digit(3, CHAR_H, 0);
		  Seven_Segment_Digit(2, CHAR_H, 1);
		  Seven_Segment_Digit(1, CHAR_M, 0);
		  Seven_Segment_Digit(0, CHAR_M, 0);

		  while((GPIOC->IDR & 0x3) == 0x2){
			  // Grabs the value of the new hex from switches 15-12
			  int newHexVal = (GPIOC->IDR >> 12) & 0xF;

			  // Grabs the PC11 value
			  int addValButton = (GPIOC->IDR >> 11) & 0x1;

			  // Grabs the PC10 value
			  int switchPossitionButton = (GPIOC->IDR >> 10) & 0x1;

			  // When PC10 is pushed
			  if(switchPossitionButton == 0){

				  // Change position of index pointer
				  initial_pos = (initial_pos + 1) % 5;

				  // Change position of LED index indicator
				  GPIOD->ODR = (1 << initial_pos);

				  // Wait for button release to avoid multiple triggers
				  while (((GPIOC->IDR >> 10) & 0x1) == 0) {
					  // Just wait
				  }
			  }

			  // Updates based on new position
			  if(addValButton == 0 && initial_pos == 3){
				  hours_tenths = newHexVal;
				  Seven_Segment_Digit(initial_pos, hours_tenths, 0);
			  } else if (addValButton == 0 && initial_pos == 2){
				  hours_ones = newHexVal;
				  Seven_Segment_Digit(initial_pos, hours_ones, 1);
			  } else if (addValButton == 0 && initial_pos == 1){
				  minutes_tenths = newHexVal;
				  Seven_Segment_Digit(initial_pos, minutes_tenths, 0);
			  } else if (addValButton == 0 && initial_pos == 0){
				  minutes_ones = newHexVal;
				  Seven_Segment_Digit(initial_pos, minutes_ones, 0);
			  } else if(addValButton == 0 && initial_pos == 4){

				  // Notify the user that we are updating the time
				  GPIOD->ODR = 0xFFFF;

				  // Allows for precise timing
				  RTC->PRER = 0x102;
				  RTC->PRER |= 0x007F0000;

				  // Enable Write Privilege
				  RTC->WPR = 0xCA;
				  RTC->WPR = 0x53;

				  // Put in INIT mode to update clock
				  RTC->ISR |= 1<<7;

				  // Wait for update
				  HAL_Delay(500);

				  // 24 Hour Time
				  RTC->CR &= ~(1 << 6);

				  // Actual Clock Configurations
				  RTC->TR = ((hours_tenths) << 20)  |  // Hours tens digit
				            ((hours_ones) << 16)    |  // Hours units digit
				            ((minutes_tenths) << 12)|  // Minutes tens digit
				            ((minutes_ones) << 8)   |  // Minutes units digit
				            0x00;                      // Seconds set to 0

				  // Clear RSF flag
				  RTC->ISR &= ~1<<7;
				  HAL_Delay(500);
				  GPIOD->ODR = 1<<initial_pos;

			  }

		  }
	  }

	/**********************
	 * Show Current Date
	 **********************/
	  else if ((GPIOC->IDR & 0x3) == 0x1){

		  /*** Day of Week Display ***/
		  int weekday = (RTC->DR >> 13) & 0x7;
		  Seven_Segment_Digit(7, weekday, 0);
		  Seven_Segment_Digit(6, SPACE, 0);

		  /*** Month       Display ***/

		  int month = (RTC->DR >> 8) & 0x1F;

		  // Getting tens position of month
		  int month_tens = (month >> 4) & 0x1;
		  Seven_Segment_Digit(5, month_tens, 0);

		  // Getting ones position of month
		  int month_ones = month & 0xF;
		  Seven_Segment_Digit(4, month_ones, 1);  // Decimal point on

		  /*** Day         Display ***/

		  int day = (RTC->DR) & 0x3F;

		  // Getting tens position of day
		  int days_tens = (day >> 4) & 0x3;
		  Seven_Segment_Digit(3, days_tens, 0);

		  // Getting ones position of day
		  int days_ones = day & 0xF;
		  Seven_Segment_Digit(2, days_ones, 1);  // Decimal point on

		  /*** Year        Display ***/

		  int year = (RTC->DR >> 16) & 0xFF;

		  // Getting tens position of year
		  int years_tens = (year >> 4) & 0xF;
		  Seven_Segment_Digit(1, years_tens, 0);

		  // Getting ones position of year
		  int years_ones = year & 0xF;
		  Seven_Segment_Digit(0, years_ones, 0);  // Decimal point on

		  HAL_Delay(1);
	  }

	/*********************
	 * Date Configuration
	 *********************/
	  else if ((GPIOC->IDR & 0x3) == 0x3){
		  int month_tens = 0;
		  int month_ones = 0;
		  int day_tens = 0;
		  int day_ones = 0;
		  int year_tens = 0;
		  int year_ones = 0;
		  int weekday = 0;

		  /*** Notify that user is updating date ***/
		  Seven_Segment_Digit(7, CHAR_A, 0);
		  Seven_Segment_Digit(6, CHAR_L, 0);
		  Seven_Segment_Digit(5, CHAR_T, 0);
		  Seven_Segment_Digit(4, SPACE, 0);
		  Seven_Segment_Digit(3, CHAR_D, 0);
		  Seven_Segment_Digit(2, CHAR_A, 0);
		  Seven_Segment_Digit(1, CHAR_T, 0);
		  Seven_Segment_Digit(0, CHAR_E, 0);

		  HAL_Delay(2000);

		  /*** Show user the format ***/
		  Seven_Segment_Digit(7, CHAR_D, 0);
		  Seven_Segment_Digit(6, SPACE, 0);
		  Seven_Segment_Digit(5, CHAR_M, 0);
		  Seven_Segment_Digit(4, CHAR_M, 1);
		  Seven_Segment_Digit(3, CHAR_D, 0);
		  Seven_Segment_Digit(2, CHAR_D, 1);
		  Seven_Segment_Digit(1, CHAR_Y, 0);
		  Seven_Segment_Digit(0, CHAR_Y, 0);

		  while((GPIOC->IDR & 0x3) == 0x3){
			  // Grabs the value of the new hex from switches 15-12
			  int newHexVal = (GPIOC->IDR >> 12) & 0xF;

			  // Grabs the PC11 value
			  int addValButton = (GPIOC->IDR >> 11) & 0x1;

			  // Grabs the PC10 value
			  int switchPossitionButton = (GPIOC->IDR >> 10) & 0x1;

			  // When PC10 is pushed
			  if(switchPossitionButton == 0){

				  // Change position of index pointer
				  initial_pos = (initial_pos + 1) % 8;

				  // Change position of LED index indicator
				  GPIOD->ODR = (1 << initial_pos);

				  // Wait for button release to avoid multiple triggers
				  while (((GPIOC->IDR >> 10) & 0x1) == 0) {
					  // Just wait
				  }
			  }

			  // Updates based on new position

			  /*** Weekday Update ***/
			  if(addValButton == 0 && initial_pos == 6){
				  weekday = newHexVal;
				  Seven_Segment_Digit(initial_pos + 1, weekday, 0);

			  /*** Month Update ***/
		  	  } else if(addValButton == 0 && initial_pos == 5){
				  month_tens = newHexVal;
				  Seven_Segment_Digit(initial_pos, month_tens, 0);

			  } else if (addValButton == 0 && initial_pos == 4){
				  month_ones = newHexVal;
				  Seven_Segment_Digit(initial_pos, month_ones, 1);

		      /*** Day Update ***/
			  } else if (addValButton == 0 && initial_pos == 3){
				  day_tens = newHexVal;
				  Seven_Segment_Digit(initial_pos, day_tens, 0);
			  } else if (addValButton == 0 && initial_pos == 2){
				  day_ones = newHexVal;
				  Seven_Segment_Digit(initial_pos, day_ones, 1);

			  /*** Year Update ***/
			  } else if (addValButton == 0 && initial_pos == 1){
				  year_tens = newHexVal;
				  Seven_Segment_Digit(initial_pos, year_tens, 0);
			  } else if (addValButton == 0 && initial_pos == 0){
				  year_ones = newHexVal;
				  Seven_Segment_Digit(initial_pos, year_ones, 0);

			  /*** Confirm Changes ***/
			  } else if(addValButton == 0 && initial_pos == 7){

				  // Notify the user that we are updating the time
				  GPIOD->ODR = 0xFFFF;

				  // Allows for precise timing
				  RTC->PRER = 0x102;
				  RTC->PRER |= 0x007F0000;

				  // Enable Write Privilege
				  RTC->WPR = 0xCA;
				  RTC->WPR = 0x53;

				  // Put in INIT mode to update clock
				  RTC->ISR |= 1<<7;

				  // Wait for update
				  HAL_Delay(500);

				  // 24 Hour Time
				  RTC->CR &= ~(1 << 6);

				  // Actual Clock Configurations
				  RTC->DR = ((year_tens) << 20) |  // Year tens digit
				            ((year_ones) << 16) |  // Year units digit
				            ((month_tens) << 12)|  // Month tens digit
				            ((month_ones) << 8) |  // Month units digit
				            ((day_tens) << 4)   |  // Day tens digit
				            day_ones            |  // Day units digit
							((weekday) << 13);     // Weekday 1-7

				  // Clear RSF flag
				  RTC->ISR &= ~1<<7;
				  HAL_Delay(500);
				  GPIOD->ODR = 1<<initial_pos;

			  }
		  }
	  }

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}






static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

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
