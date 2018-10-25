
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "PS2_device.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

typedef struct {
	uint8_t report_id;
	uint8_t modifier;
	uint8_t reserved;
	uint8_t keycode[8];
} keyboard_report_t;


struct keysMapHID_t {
	uint8_t report_id;
	uint8_t keys[10];
};
struct mouseHID_t mouseHID;
static keyboard_report_t keyboardHID;
static keyboard_report_t newkeyboardHID;
struct keysMapHID_t keyMapsHid;

uint8_t channel = 0;
uint8_t adc[2];
uint8_t OutBuffer[10];
uint8_t isDataReady;
uint16_t Joystick[2];
uint32_t firsted = 0;
uint32_t second = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM7_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void saveKeySettings();
void readDataFromFlash();
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
bool shouldSend();
void writeButton(uint8_t number, GPIO_PinState pinState);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USB_DEVICE_Init();
  MX_TIM16_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	PS2_Initialize();

	//readDataFromFlash();
	HAL_TIM_Base_Start_IT(&htim16);
	//HAL_ADC_Start_IT(&hadc1);
	keyboardHID.report_id = 0x01;
	keyboardHID.keycode[0]= 4;
	newkeyboardHID.report_id = 0x01;
	mouseHID.reportId = 0x02;
	keyMapsHid.report_id = 0x03;
	isDataReady = 0;
	HAL_ADC_Start_DMA(&hadc1,&Joystick[0],2);
	readDataFromFlash();
	//HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		//keyboardHID.keycode[0]= 4;

		//USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &keyboardHID,sizeof(keyboard_report_t));
		if(isDataReady == 1){
		 if(OutBuffer[0] == 0x00){
			 USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &keyMapsHid, sizeof(struct keysMapHID_t));

		 }else{
		 for(int i = 0; i<10; i++){
		 keyMapsHid.keys[i] = OutBuffer[i];
		 }
		 //keyMapsHid.keys = OutBuffer;

		 saveKeySettings();
		 }
		 isDataReady = 0;

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 11;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 159;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 9999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Clock_host_Pin|Data_value_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(User_LED_GPIO_Port, User_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Clock_device_Pin Button4_Pin Button1_Pin Button2_Pin 
                           Button3_Pin Button5_Pin Button6_Pin */
  GPIO_InitStruct.Pin = Clock_device_Pin|Button4_Pin|Button1_Pin|Button2_Pin 
                          |Button3_Pin|Button5_Pin|Button6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Clock_host_Pin Data_value_Pin */
  GPIO_InitStruct.Pin = Clock_host_Pin|Data_value_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : User_LED_Pin */
  GPIO_InitStruct.Pin = User_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(User_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void saveKeySettings() {

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(
			FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR);

	static FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = 64;
	EraseInitStruct.NbPages = 1;
	EraseInitStruct.Banks = FLASH_BANK_1;
	static uint32_t SectorError;
	HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
	CLEAR_BIT(FLASH->SR, (FLASH_SR_EOP));
	uint32_t startaddres = 0x08020000;
	uint32_t endaddres = 0x0802000A;
	int i = 0;
	uint64_t DW[2];
	DW[0] = 0;
	DW[1] = 0;
	for (int k = 0; k < 8; k++) {
		DW[0] = DW[0] | ((uint64_t) (keyMapsHid.keys[k])) << (k * 8);
	}
	DW[1] = keyMapsHid.keys[8] | (keyMapsHid.keys[9] << 8);

	while (startaddres < endaddres) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, startaddres, DW[i])
				== HAL_OK) {
			startaddres += 8;
			i += 1;

		} else {

			break;
		}
	}
	HAL_FLASH_Lock();

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc == &hadc1){

	//	adc[channel] = HAL_ADC_GetValue(&hadc1);
		//channel++;
		//if(channel == 2){
			//channel = 0;
			if(Joystick[0]<65){
				newkeyboardHID.keycode[6] = keyMapsHid.keys[6];//26;
			}else if(Joystick[0]>150){
				newkeyboardHID.keycode[6] = keyMapsHid.keys[7];//22;
			}else{
				newkeyboardHID.keycode[6] = 0x00;
			}

			if(Joystick[1]<65){
				newkeyboardHID.keycode[7] = keyMapsHid.keys[8];//4;
			}else if(Joystick[1]>150){
				newkeyboardHID.keycode[7] = keyMapsHid.keys[9];//7;
			}else{
				newkeyboardHID.keycode[7] = 0x00;
			}
			if(shouldSend()){
				USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &keyboardHID,
										sizeof( keyboard_report_t));
			}
		//}

	}
}

void readDataFromFlash() {

	uint32_t startaddress = 0x08020000;
	uint32_t endaddress = 0x0802000A;
	int i = 0;
	while (startaddress < endaddress) {
		keyMapsHid.keys[i] = *(__IO uint8_t *) startaddress;
		startaddress += 1;
		i += 1;

	}

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	GPIO_PinState pinState = HAL_GPIO_ReadPin(GPIOA, GPIO_Pin);

	if (GPIO_Pin == Clock_device_Pin && preSendingState == false) {
		if (pinState == GPIO_PIN_RESET && Sending == false) {

			PS2_Handle_Reciving();
			++firsted;
		} else if (pinState == GPIO_PIN_RESET && Sending == true) {
			PS2_Handle_Sending();

		}
	}else if(GPIO_Pin != 1) {

		switch (GPIO_Pin) {
		case Button1_Pin:
			writeButton(1, pinState);
			break;
		case Button2_Pin:
			writeButton(2, pinState);
			break;
		case Button3_Pin:
			writeButton(3, pinState);
			break;
		case Button4_Pin:
			writeButton(4, pinState);
			break;
		case Button5_Pin:
			writeButton(5, pinState);
			break;
		case Button6_Pin:
			writeButton(6, pinState);
			break;


			}
		if (shouldSend()) {
						keyboardHID = newkeyboardHID;
						USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &keyboardHID,
								sizeof( keyboard_report_t));
		}

	}
}

bool shouldSend() {
	for (int i = 0; i < sizeof(keyboardHID.keycode); i++) {
		if (newkeyboardHID.keycode[i] != keyboardHID.keycode[i] || keyboardHID.modifier != newkeyboardHID.modifier) {
			keyboardHID = newkeyboardHID;
			return true;
		}
	}
	return false;
}

void writeButton(uint8_t number, GPIO_PinState pinState) {

	if (pinState == GPIO_PIN_RESET) {
		if(keyMapsHid.keys[number-1] > 0x65){
			newkeyboardHID.modifier = newkeyboardHID.modifier| ((1<<keyMapsHid.keys[(number-1)]-0x66));
		}else
		newkeyboardHID.keycode[number-1] = keyMapsHid.keys[number-1];
	} else {
		if(keyMapsHid.keys[number-1] > 0x65){
					newkeyboardHID.modifier = newkeyboardHID.modifier^ ((1<<keyMapsHid.keys[(number-1)]-0x66));
		}else{
		newkeyboardHID.keycode[number-1] = 0x00;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim == &htim7) {

		HAL_TIM_Base_Stop_IT(htim);
		//end = HAL_GetTick();
		//diff = end - start;
		HAL_GPIO_WritePin(Data_value_GPIO_Port, Data_value_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Clock_host_GPIO_Port, Clock_host_Pin, GPIO_PIN_SET);
		preSendingState = false;
		Sending = true;

	} else if (htim == &htim16) {
		if (first == true) {
			uint8_t toSend = 0xFF;
			PS2_SendPackage(&toSend);
			first = false;
		} else if (StreamMode == false && Sending == false && Reciving == false) {
			uint8_t toSend;
			if (Recived) {
				PS2_Handle_Recived();
			} else if (intellimouseState > 10) {
				toSend = 0xF4;
				PS2_SendPackage(&toSend);

			} else {
				switch (intellimouseState++) {
				case 1:
					toSend = 0xC8;
					break;
				case 3:
					toSend = 0x64;
					break;
				case 5:
					toSend = 0x50;
					break;
				case 6:
					toSend = 0xE8;
					break;
				case 7:
					toSend = 0x03;
					break;
				case 8:
					toSend = 0xE7;
					break;

				case 10:
					toSend = 40;
					break;
				default:
					toSend = 0xF3;
					break;
				}
				PS2_SendPackage(&toSend);
			}
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
