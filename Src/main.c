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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"

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

/* USER CODE BEGIN PV */

uint16_t result;
char znak[7];
uint16_t SensorValue[8];
int ActualRight = 0;
int ActualLeft = 0;
float ActualSensorValue = 0;
float Error;
int SetPoint = 0;
int Kp = 10000;
int ControlVariable = 0;
uint16_t max = 49999;
uint16_t min = 10000;
float NewSensor[8];
float Value;
float Sensor;
float Sum;
float ValueOfSensor;
float Integral = 0;
float Ki = 1;
float Derivative = 0;
int Kd = 1000;
int limit = 6; //6
int IntegralLimit = 1000;
float LastError = 0;
uint8_t rx_buff[10];
int switch_on = 0;
int a = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */





	void SpeedEngine(int RightValue, int LeftValue)
	{
		if (RightValue > max)	RightValue = max;
		if (LeftValue > max)	LeftValue = max;


		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, RightValue);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, LeftValue);

		ActualRight = RightValue;
		ActualLeft = LeftValue;
	}







	void HAL_Values()
	{
		  for (int i = 0; i<=7; i++)
		  {

			  	ValueOfSensor = SensorValue[i];

			  	if (ValueOfSensor <= 500)
			  		ValueOfSensor = 0;
			  	else if (ValueOfSensor < 3500 && ValueOfSensor > 500)
			  		{
			  		ValueOfSensor = ValueOfSensor / 3500;

			  		ValueOfSensor = ValueOfSensor * 100;
			  		ValueOfSensor = round(ValueOfSensor);
			  		ValueOfSensor = ValueOfSensor / 100;
			  		}
			  	else
			  		ValueOfSensor = 1;



			 NewSensor[i] = ValueOfSensor;

		  }
	}

	void HAL_Licz (float Sensor_LS, float Sensor_L1, float Sensor_L2, float Sensor_S1, float Sensor_S2, float Sensor_R1, float Sensor_R2, float Sensor_RS)
	{
		if (Sensor_RS > 0 && Sensor_LS == 0)
		{
			Sum = Sensor_RS * 8 + Sensor_R2 * 4;
			Value = Sum / (Sensor_RS + Sensor_R2);
		}

		else if (Sensor_LS > 0 && Sensor_RS == 0)
		{
			Sum = Sensor_LS * (-8) + Sensor_L1 * (-4);
			Value = Sum / (Sensor_LS + Sensor_L1);
		}

		else if (Sensor_R2 > 0 && Sensor_L1 == 0)
		{
			Sum = Sensor_R2 * 4 + Sensor_R1 * 2;
			Value = Sum / (Sensor_R2 + Sensor_R1);
		}

		else if (Sensor_L1 > 0 && Sensor_R2 == 0)
		{
			Sum = Sensor_L1 * (-4) + Sensor_L2 * (-2);
			Value = Sum / (Sensor_L1 + Sensor_L2);
		}

		else if (Sensor_R1 > 0)
		{
			Sum = Sensor_R1 * 2 + Sensor_S2;
			Value = Sum / (Sensor_R1 + Sensor_S2);
		}

		else if (Sensor_L2 > 0)
		{
			Sum = Sensor_L2 * (-2) - Sensor_S1;
			Value = Sum / (Sensor_L2 + Sensor_S1);
		}

		else
		{
			Sum = Sensor_S1 * (-1) + Sensor_S2;
			Value = Sum / (Sensor_S1 + Sensor_S2);
		}


		Value = Value * 100;
		Value = round(Value);
		Value = Value / 100;

		ActualSensorValue = Value;


	}



	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //przerwanie od licznika co 100ms
	{

		if (switch_on == 1)
		{
		// P
		Error = SetPoint + ActualSensorValue; //uchyb

		//I
		if (Integral < IntegralLimit && Integral > (IntegralLimit * (-1)))
		Integral = Integral + Error;
		else if (Integral > IntegralLimit)				Integral = IntegralLimit;
		else if (Integral < (IntegralLimit * (-1)))		Integral = IntegralLimit * (-1);

		//D
		Derivative = Error - LastError;
		LastError = Error;
		}

		else {}

		ControlVariable = round(Kp * Error) + round(Ki * Integral) + round(Kd * Derivative);

		if (ControlVariable < 0)
		 			ControlVariable = ControlVariable * (-1);
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART2_IRDA_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)SensorValue, 8);

  HAL_GPIO_WritePin(R_SILNIK1_GPIO_Port, R_SILNIK1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(R_SILNIK2_GPIO_Port, R_SILNIK2_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(L_SILNIK1_GPIO_Port, L_SILNIK1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(L_SILNIK2_GPIO_Port, L_SILNIK2_Pin, GPIO_PIN_RESET);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); //R_PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //L_PWM

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 10000);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 10000);


  HAL_TIM_Base_Start_IT(&htim10);

  HAL_IRDA_Receive_IT(&hirda2, rx_buff, 10);

  HAL_GPIO_WritePin(RGB_BLUE_GPIO_Port, RGB_BLUE_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (switch_on == 1)
	{

		  HAL_Values();

if (NewSensor[0] > 0 || NewSensor[1] > 0 || NewSensor[2] > 0 || NewSensor[3] > 0 || NewSensor[4] > 0 || NewSensor[5] > 0 || NewSensor[6] > 0 || NewSensor[7] > 0)
{



	HAL_Licz(NewSensor[0], NewSensor[1], NewSensor[2], NewSensor[3], NewSensor[4], NewSensor[5], NewSensor[6], NewSensor[7]);




	//if (ControlVariable < 0)
 	//		ControlVariable = ControlVariable * (-1);

	// Predkosci

	if (Error > limit)		SpeedEngine(0, max);

	else if (Error < (-limit)) SpeedEngine(max,0);

	else if (Error > 0 && Error < limit) //Prawo
	{

		SpeedEngine(max - ControlVariable, max);
		if (ActualRight < min) SpeedEngine(min, ActualLeft);

	}

	else if (Error < 0 && Error > (-limit))
	{
		SpeedEngine(max, max - ControlVariable);
		if (ActualLeft < min) SpeedEngine(ActualRight, min);
	}

	else
	{
		ActualRight = ((ActualRight + ActualLeft) / 2) + 5000;
		ActualLeft = ActualRight;
		SpeedEngine(ActualRight, ActualLeft);
	}





}
else	// zjecha³ na bia³y
	{
		//SpeedEngine(0,0);
	}



  }

	  else {SpeedEngine(0,0);}

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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if (a < 3)
	{
		switch_on = 1;
		 HAL_GPIO_WritePin(RGB_BLUE_GPIO_Port, RGB_BLUE_Pin, GPIO_PIN_RESET);
	}

	else
	{
		switch_on = 0;
		a = 0;
		 HAL_GPIO_WritePin(RGB_BLUE_GPIO_Port, RGB_BLUE_Pin, GPIO_PIN_SET);
	}

	a = a + 1;

}

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
