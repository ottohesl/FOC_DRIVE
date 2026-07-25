/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FOC_Math.h"
#include "usb_device.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for TouchGFX */
osThreadId_t TouchGFXHandle;
const osThreadAttr_t TouchGFX_attributes = {
  .name = "TouchGFX",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 4000 * 4
};
/* Definitions for FOC */
osThreadId_t FOCHandle;
const osThreadAttr_t FOC_attributes = {
  .name = "FOC",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 2000 * 4
};
/* Definitions for DATA */
osThreadId_t DATAHandle;
const osThreadAttr_t DATA_attributes = {
  .name = "DATA",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 1280 * 4
};
/* Definitions for VOFA */
osMessageQueueId_t VOFAHandle;
const osMessageQueueAttr_t VOFA_attributes = {
  .name = "VOFA"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void TouchGFX_Task(void *argument);
void FOC_Task(void *argument);
void DATA_Task(void *argument);

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

  /* Create the queue(s) */
  /* creation of VOFA */
  VOFAHandle = osMessageQueueNew (16, sizeof(SVPWM), &VOFA_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TouchGFX */
  TouchGFXHandle = osThreadNew(TouchGFX_Task, NULL, &TouchGFX_attributes);

  /* creation of FOC */
  FOCHandle = osThreadNew(FOC_Task, NULL, &FOC_attributes);

  /* creation of DATA */
  DATAHandle = osThreadNew(DATA_Task, NULL, &DATA_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_TouchGFX_Task */
/**
  * @brief  Function implementing the TouchGFX thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_TouchGFX_Task */
__weak void TouchGFX_Task(void *argument)
{
  /* init code for USB_Device */
  MX_USB_Device_Init();
  /* USER CODE BEGIN TouchGFX_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TouchGFX_Task */
}

/* USER CODE BEGIN Header_FOC_Task */
/**
* @brief Function implementing the FOC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FOC_Task */
__weak void FOC_Task(void *argument)
{
  /* USER CODE BEGIN FOC_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END FOC_Task */
}

/* USER CODE BEGIN Header_DATA_Task */
/**
* @brief Function implementing the DATA thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DATA_Task */
__weak void DATA_Task(void *argument)
{
  /* USER CODE BEGIN DATA_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DATA_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

