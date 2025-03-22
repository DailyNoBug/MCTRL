/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;
float ctrl[4];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for AdcValueGetTask */
osThreadId_t AdcValueGetTaskHandle;
const osThreadAttr_t AdcValueGetTask_attributes = {
  .name = "AdcValueGetTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4
};
/* Definitions for dataTrans */
osThreadId_t dataTransHandle;
const osThreadAttr_t dataTrans_attributes = {
  .name = "dataTrans",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint16_t Read_ADC_Channel(uint32_t channel);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void AdcValueGet(void *argument);
void dataTransTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of AdcValueGetTask */
  AdcValueGetTaskHandle = osThreadNew(AdcValueGet, NULL, &AdcValueGetTask_attributes);

  /* creation of dataTrans */
  dataTransHandle = osThreadNew(dataTransTask, NULL, &dataTrans_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  static uint8_t shine_led_state = 0;
  for(;;)
  {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, shine_led_state);
      shine_led_state ^= 1;
      printf("enter default task\r\n");
      osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_AdcValueGet */
/**
* @brief Function implementing the AdcValueGetTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AdcValueGet */
void AdcValueGet(void *argument)
{
  /* USER CODE BEGIN AdcValueGet */
  /* Infinite loop */
    uint16_t adc_values[4];
    printf("enter getAdcValueTask\r\n");
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    for(;;)
    {
        adc_values[0] = Read_ADC_Channel(ADC_CHANNEL_6);  // IN6
        adc_values[1] = Read_ADC_Channel(ADC_CHANNEL_7);  // IN7
        adc_values[2] = Read_ADC_Channel(ADC_CHANNEL_8);  // IN8
        adc_values[3] = Read_ADC_Channel(ADC_CHANNEL_9);  // IN9

        ctrl[0] = (adc_values[0] - 1000) / (2700.0 - 1000.0) * 100.0;
        ctrl[1] = (adc_values[1] - 350)  / (3900.0 - 350.0)  * 100.0;
        ctrl[2] = (3800 - adc_values[2]) / (3800.0 - 280.0)  * 100.0;
        ctrl[3] = (3800 - adc_values[3]) / (3800.0 - 200.0)  * 100.0;

        printf("channel1: %.2f, channel2: %.2f, channel3: %.2f, channel4: %.2f\r\n",
               ctrl[0], ctrl[1], ctrl[2], ctrl[3]);
        osDelay(10);
    }
  /* USER CODE END AdcValueGet */
}

/* USER CODE BEGIN Header_dataTransTask */
/**
* @brief Function implementing the dataTrans thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dataTransTask */
void dataTransTask(void *argument)
{
  /* USER CODE BEGIN dataTransTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END dataTransTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

uint16_t Read_ADC_Channel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};

    // 配置目标通道
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;       // 单通道固定为Rank1
    sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig); // 重新配置通道

    HAL_ADC_Start(&hadc1);                            // 启动转换
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // 阻塞等待转换完成
    return HAL_ADC_GetValue(&hadc1);                  // 返回转换结果
}

/* USER CODE END Application */

