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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "app_bluenrg_ms.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_i2c.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define timer_freq 80.0  //timer clock freq in MHz
#define T0H 0.35  //each different clone can have their own timings
#define T1H 0.9  //timing here are in us
#define T0L 0.9
#define T1L 0.35
#define Treset 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
ADC_HandleTypeDef hadc1, hadc2;
//UART_HandleTypeDef huart2;

struct lcd_disp disp;					//comment
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	// variables needed for LEDs
	uint8_t LED_data[24]; //number of LEDs*3
	uint16_t pos;
	uint8_t mask = 0B10000000;
	uint8_t lastbit;
	long double period;
	uint16_t low_CCR1, low_ARR, high_CCR1, high_ARR, treset_ARR;


void Neopixel_setup(void){

	//calculate all the timings.
	period = 1 / timer_freq;
	low_CCR1 = round(T0H / period);
	low_ARR = round((T0H + T0L) / period);
	high_CCR1 = round(T1H / period);
	high_ARR = round((T1H + T1L) / period);
	treset_ARR = ceil(Treset / period);

	RCC->AHB1ENR |= RCC_AHB2ENR_GPIODEN; //enable port D clock
	GPIOD->MODER |= GPIO_MODER_MODER12_1; //setup pin 12 on port d to AF mode
	GPIOD->AFR[1] = (GPIOD->AFR[1] & (0b1111<<(4*(12-8))) | 0b0010<<(4*(12-8))); //setup pin 12 on port D to AF timer 2-5

	RCC->AHB1ENR |= RCC_APB1ENR1_TIM4EN; //enable the timer4 clock
	TIM4->PSC = 0;   //set prescale to zero as timer has to go as fast as posible
	TIM4->CCMR1 = (TIM4->CCMR1 & ~(0b110<<4)) | (0b110<<4); //set PWM mode 110
	TIM4->CCR1 = 0; //set to zero so that the pin stay low until transmission
	TIM4->ARR = treset_ARR; //set to timing for reset LEDs
	TIM4->CCER |= TIM_CCER_CC1E; //enable output to pin.
	TIM4->CR1 |= TIM_CR1_CEN; //Disable channel 1. This bit is used to start and stop transmission.
	TIM4->CR1 |= TIM_CR1_ARPE; //buffer ARR
	TIM4->CCMR1 |= TIM_CCMR1_OC1PE; //buffer CCR1
	TIM4->DIER &= ~TIM_DIER_UIE; // ensure we are not enabling interrupt flag to be generated this bit is used to start/stop transmission
	TIM4->CR1 |= TIM_CR1_CEN; //enable channel 1.

	NVIC_EnableIRQ(TIM4_IRQn); // Enable interrupt(NVIC level)
}

void show_neopixels(){
	pos = 0; //set the interrupt to start at first byte
	lastbit = 0;
	mask = 0B10000000; //set the interrupt to start at second bit

	TIM4->SR &= ~TIM_SR_UIF; // clear UIF flag
	TIM4->DIER |= TIM_DIER_UIE; //enable interrupt flag to be generated to start transmission
}

void set_colour(int x, int y){
	if(x==1){ //white
		for (uint8_t i = 0; i < 24; i+=1){
			LED_data[i] = y;
		}
	}
	if(x==2){ //red
		for (uint8_t i = 0; i < 24; i+=3){
			LED_data[i] = 0;
			LED_data[i+1] = y;
			LED_data[i+2] = 0;
		}
	}
	if(x==3){ //yellow
		for (uint8_t i = 0; i < 24; i+=3){
			LED_data[i] = y;
			LED_data[i+1] = y;
			LED_data[i+2] = 0;
		}
	}
	if(x==4){ //green
		for (uint8_t i = 0; i < 24; i+=3){
			LED_data[i] = y;
			LED_data[i+1] = 0;
			LED_data[i+2] = 0;
		}
	}
	if(x==5){ //turquoise
		for (uint8_t i = 0; i < 24; i+=3){
			LED_data[i] = y;
			LED_data[i+1] = 0;
			LED_data[i+2] = y;
		}
	}
	if(x==6){ //blue
			for (uint8_t i = 0; i < 24; i+=3){
				LED_data[i] = 0;
				LED_data[i+1] = 0;
				LED_data[i+2] = y;
			}
	}
	if(x==7){ //pink
				for (uint8_t i = 0; i < 24; i+=3){
					LED_data[i] = 0;
					LED_data[i+1] = y;
					LED_data[i+2] = y;
				}
	}
	if(x==8){ //LEDs off
			for (uint8_t i = 0; i < 24; i+=1){
				LED_data[i] = 0;
			}
		}
	show_neopixels(); //send data to LEDs
}

void print_measurement(uint16_t raw, uint16_t raw2, char msg[]){ //for now prints raws
	double Temp;
	uint16_t Temp0, moc;
	char msg_light[6];

	Temp = log(((10240000/raw) - 10000));
	Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))*	Temp );
	Temp = Temp - 273.15; //temperature in Celsius degrees
	Temp0 = Temp;
	moc = 5*5*1000/raw + 5*5*1000/raw2;

   if(raw2 > 1000){
	   strcpy(msg_light,"dark");
   }else{
	   strcpy(msg_light,"light");
   }

	sprintf((char *)disp.f_line, "T:%huC L:%s",Temp0, msg_light);
	sprintf((char *)disp.s_line, "P:%humW",moc);
	lcd_display(&disp);

//	HAL_UART_Transmit()//


}

void light_correctness(uint16_t raw2, int j){
	if(raw2>1000){
			set_colour(j, 150); //party mode when it's dark
	}
	if(raw2<1000){
	  		 set_colour(1, 70); //white light when it's bright
	}
}


uint8_t neopoxel_transmitting(){
	return (TIM4->DIER & TIM_DIER_UIE) && 1;
}

void measurement_light(void){ //red light while printing measurement
	for(int z=0; z<70; z++){
		set_colour(2,z);
		HAL_Delay(15);
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
	//moje adc
	uint16_t raw, raw2;
	char msg[32];
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
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_ADC2_Init();
  MX_BlueNRG_MS_Init();
  /* USER CODE BEGIN 2 */
  Neopixel_setup(); //start for LEDs
  disp.addr = (0x27 << 1);
  disp.bl = true;
  lcd_init(&disp);
  sprintf((char *)disp.f_line, "Press button to");
  sprintf((char *)disp.s_line, "see measurements");
  lcd_display(&disp);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  for(int j=2; j<8; j++){ //loop for changing colors when it's dark

	  // Test: set gpio pin high - start timing
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET );

	  // get adc value
	  HAL_ADC_Start(&hadc1);
	  if(HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
	  {
	  	  raw = HAL_ADC_GetValue(&hadc1);

	  }
	  // test: set GPIO pin low - stop timing
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET );

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET );//------------------
	  // get adc value
	  HAL_ADC_Start(&hadc2);
	  if(HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY) == HAL_OK)
	  {
	  	  raw2 = HAL_ADC_GetValue(&hadc2);
	  }
  	  light_correctness(raw2, j); //white light when it's bright and party mode when it's dark

	  // test: set GPIO pin low - stop timing
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET );

	  while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)==0){
		  print_measurement(raw, raw2, msg); //prints measurements
		  measurement_light(); // red light
	  }
	  HAL_Delay(400); //delay needed for proper change of colors in party mode
	  }//end for

    /* USER CODE END WHILE */

  MX_BlueNRG_MS_Process();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void TIM4_IRQHandler(void){

	TIM4->SR &= ~TIM_SR_UIF; // clear UIF flag

		if(pos<sizeof(LED_data)){
			if(LED_data[pos] & mask){
				TIM4->CCR1 = high_CCR1;
				TIM4->ARR = high_ARR;
			}else{
				TIM4->CCR1 = low_CCR1;
				TIM4->ARR = low_ARR;
			}
			if(mask==1){
				mask = 0B10000000;
				pos+=1;
			}else mask = mask >> 1;
		}else{
			TIM4->CCR1 = 0; //set to zero so that pin stays low
			TIM4->ARR = treset_ARR; //set to timing for reset LEDs
			TIM4->DIER &= ~TIM_DIER_UIE; //disable interrupt flag to end transmission.
		}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
