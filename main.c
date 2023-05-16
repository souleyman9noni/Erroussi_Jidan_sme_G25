/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lib_lcd.h"
#include <stdio.h>
static rgb_lcd lcd_data;
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim2,0);
	while(__HAL_TIM_GET_COUNTER (&htim2) < time);

}
uint8_t charTransmit[1];
char *cmdData= "AT";
char cmdDataTmp[40];
sprintf(cmdDataTmp,"%s\r\n",cmdData);

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
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  lcd_init(&hi2c1,&lcd_data charTransmit,1,1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_SET );  // pull the TRIG pin HIGH
  	  	  	  delay_us(10);  // wait for 2 us
  	  	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_RESET );  // pull the TRIG pin HIGH
  	  	  	  HAL_Delay(500);

  	  	  	  HAL_GPIO_EXTI_Callback(GPIO_PIN_8);

  	  /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */

  	  /*commence la communication avec le capteur*/

  	  HAL_Delay(3000);
  	  Data_Output(GPIOA, GPIO_PIN_11); //info vers le capteur
  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
  	  DWT_Delay_us(1200); //signal de commande
  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
  	  DWT_Delay_us(30); //signal de commande
  	  Data_Input(GPIOA, GPIO_PIN_11); //info vers le microcontroleur

  	  /*commence la reception de donnees*/

  	  while(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)));

  	  for (k=0;k<1000;k++)
  	  {
  		  if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET)
  		  {
  	  	  	break;
  	  	  }
  	  }

  	  while(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)));
  	  DWT_Delay_us(40);

  	  Read_data(&dataH1); //dans la library HT.c
  	  Read_data(&dataH2);
  	  Read_data(&dataT1);
  	  Read_data(&dataT2);
  	  Read_data(&SUM);

  	  check = dataH1 + dataH2 + dataT1 + dataT2; //pour verifier la lecture dans le IDE

  	  RH = (dataH1<<8) | dataH2;
  	  TEMP = (dataT1<<8) | dataT2;

  	  Humidite = RH / 10.0;
  	  Temperature = TEMP / 10.0;

  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); //pour la prochaine lecture

  	  /*commence transmission vers LCD*/
  	  clearlcd();

  	sprintf(bufT, "Temp.: %.1f C", Temperature);
  	  lcd_position(&hi2c1,0,0);
  	  lcd_print(&hi2c1,bufT);

  	  HAL_Delay(1000);

  	sprintf(bufRH,"Humidite: %.1f", Humidite);

  	  	  lcd_position(&hi2c1,0,0);
  	  	  lcd_print(&hi2c1,bufRH);
  	  	  lcd_print(&hi2c1,"%");



  	  	reglagecouleur(0,0,255);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{


	float distance=0;
	char chaine [50];
	__HAL_TIM_SET_COUNTER(&htim2,0);
	while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)==GPIO_PIN_SET){

		distance=__HAL_TIM_GET_COUNTER (&htim2)/58;



	}
		 sprintf(chaine,"distance=%2.f cm",distance);
		 lcd_position(&hi2c1,0,1);
		 lcd_print(&hi2c1,chaine);



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
