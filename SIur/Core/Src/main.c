/* USER CODE BEGIN Header */
/**
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
#include "kurczak_wav.h"
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

/* USER CODE BEGIN PV */
volatile uint16_t delay_ms_value = 0;
volatile uint32_t pianie_index_spokoj = 0;
volatile uint32_t pianie_index_wkurw = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay_ms(uint16_t delay)
{
	delay_ms_value = delay;
	while(delay_ms_value);
}

void scierwo_set(uint16_t setting)
{
	LL_TIM_OC_SetCompareCH1(TIM1, setting);
}

void wibruj_kogucie()
{
	LL_TIM_OC_SetCompareCH2(TIM1, 749);
}
void kogucie_wibracje_stop()
{
	LL_TIM_OC_SetCompareCH2(TIM1, 0);
}

void chujowe_oko_mam()
{
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13);
}

void chujowe_oko_zamykam()
{
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_13);
}

void kurczak_piej(uint8_t ryk)
{
	LL_TIM_OC_SetCompareCH1(TIM4, ryk);
}

uint8_t czy_ma_nakurwiac()
{
	return !LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_12);
}

uint8_t czy_kurczak_wkurwiony()
{
	return LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_9);
}


void TIM1_TRG_COM_TIM11_IRQHandler()
{
	if(LL_TIM_IsActiveFlag_UPDATE(TIM11))
	{
		if(delay_ms_value > 0)
			delay_ms_value--;
		LL_TIM_ClearFlag_UPDATE(TIM11);
	}

}

void cisza_przed_wkurwem()
{
	delay_ms(1000);
	chujowe_oko_mam();
	delay_ms(200);
	chujowe_oko_zamykam();
	delay_ms(200);
	chujowe_oko_mam();
	delay_ms(200);
	chujowe_oko_zamykam();
	delay_ms(500);
	chujowe_oko_mam();
	delay_ms(666);
}

volatile uint8_t tik_oka = 0;
volatile uint8_t mrugaj_flaga = 0;
void TIM1_UP_TIM10_IRQHandler()
{
	if(LL_TIM_IsActiveFlag_UPDATE(TIM10))
	{
		if(czy_ma_nakurwiac())
		{
			if(czy_kurczak_wkurwiony())
			{
				if(tik_oka == 0)
				{
					mrugaj_flaga = 1;
					tik_oka = 1;
				}
				if(mrugaj_flaga == 0)
				{
					wibruj_kogucie();
					//chujowe_oko_mam();
					kurczak_piej(ryk_wkurw[pianie_index_wkurw]);
					pianie_index_wkurw = (pianie_index_wkurw + 1) % NUM_ELEMENTS_ANGRY;
				}
			}
			else
			{
				tik_oka = 0;
				kogucie_wibracje_stop();
				chujowe_oko_zamykam();
				kurczak_piej(ryk_spokoj[pianie_index_spokoj]);
				pianie_index_spokoj = (pianie_index_spokoj + 1) % NUM_ELEMENTS_TRANQUIL;
			}
		}
		else
		{
			kurczak_piej(0);
			kogucie_wibracje_stop();
			chujowe_oko_zamykam();
		}
		LL_TIM_ClearFlag_UPDATE(TIM10);
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

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
 // MX_TIM1_Init();
 // MX_TIM4_Init();
 // MX_TIM10_Init();
 // MX_GPIO_Init();
 // MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  //--------------------------TIM1-----------------------------------
  LL_GPIO_InitTypeDef gpio_init = {0};

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  LL_TIM_DeInit(TIM1);
  LL_TIM_SetPrescaler(TIM1, 1279);
  LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);
  LL_TIM_SetAutoReload(TIM1, 999);
  LL_TIM_SetClockDivision(TIM1, LL_TIM_CLOCKDIVISION_DIV1);
  LL_TIM_GenerateEvent_UPDATE(TIM1);

  LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_SetCompareCH1(TIM1, 0);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);

  LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_SetCompareCH2(TIM1, 0);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);

  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_EnableCounter(TIM1);

  gpio_init.Pin = LL_GPIO_PIN_8;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_LOW;
  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Alternate = LL_GPIO_AF_1;

  LL_GPIO_Init(GPIOA, &gpio_init);

  gpio_init.Pin = LL_GPIO_PIN_9;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_LOW;
  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Alternate = LL_GPIO_AF_1;

  LL_GPIO_Init(GPIOA, &gpio_init);

//--------------------------TIM4-----------------------------------
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  LL_TIM_DeInit(TIM4);
  LL_TIM_SetPrescaler(TIM4, 0);
  LL_TIM_SetCounterMode(TIM4, LL_TIM_COUNTERMODE_UP);
  LL_TIM_SetAutoReload(TIM4, 255);
  LL_TIM_SetClockDivision(TIM4, LL_TIM_CLOCKDIVISION_DIV1);
  LL_TIM_GenerateEvent_UPDATE(TIM4);
  LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_SetCompareCH1(TIM4, 0);
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableAllOutputs(TIM4);
  LL_TIM_EnableCounter(TIM4);

  gpio_init.Pin = LL_GPIO_PIN_6;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Alternate = LL_GPIO_AF_2;

  LL_GPIO_Init(GPIOB, &gpio_init);

//-------------------TIM11 + TIM10------------------------------------------
  LL_TIM_InitTypeDef tim_init = {0};
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM10);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM11);

  tim_init.Prescaler = 7;
  tim_init.Autoreload = 999;
  tim_init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;

  LL_TIM_Init(TIM10, &tim_init);
  tim_init.Prescaler = 63;
  tim_init.Autoreload = 999;
  tim_init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
  LL_TIM_Init(TIM11, &tim_init);

  LL_TIM_EnableIT_UPDATE(TIM10);
  NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 8);
  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

  LL_TIM_EnableIT_UPDATE(TIM11);
  NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 8);
  NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);

  LL_TIM_EnableCounter(TIM10);
  LL_TIM_EnableCounter(TIM11);
//----------------PB12 + PB9 + PB13----------------------------
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  gpio_init.Mode = LL_GPIO_MODE_INPUT;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_init.Pin = LL_GPIO_PIN_9;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_LOW;

  LL_GPIO_Init(GPIOB, &gpio_init);

  gpio_init.Pin = LL_GPIO_PIN_12;
  gpio_init.Pull = LL_GPIO_PULL_UP;

  LL_GPIO_Init(GPIOB, &gpio_init);

  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.Pin = LL_GPIO_PIN_13;

  LL_GPIO_Init(GPIOB, &gpio_init);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t przerwa_w_fapaniu = 15;
  uint8_t solidry_nakurw = przerwa_w_fapaniu / 2;
  while (1)
  {
	uint16_t setting = 50;

	while(setting < 80)
	{
		if(mrugaj_flaga)
		{
			cisza_przed_wkurwem();
			mrugaj_flaga = 0;
		}
		if(czy_ma_nakurwiac())
		{
			scierwo_set(setting);
			setting += 1;
		}

		if(czy_kurczak_wkurwiony())
			delay_ms(solidry_nakurw);
		else
			delay_ms(przerwa_w_fapaniu);
	}
	while(setting > 50)
	{
		if(mrugaj_flaga)
		{
			cisza_przed_wkurwem();
			mrugaj_flaga = 0;
		}
		if(czy_ma_nakurwiac())
		{
			scierwo_set(setting);
			setting -= 1;
		}
		if(czy_kurczak_wkurwiony())
			delay_ms(solidry_nakurw);
		else
			delay_ms(przerwa_w_fapaniu);
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 64, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(64000000);
  LL_SetSystemCoreClock(64000000);
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}


