/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "tim.h"
#include "mpu6050.h"
#include "vl6180x_api.h"
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
osThreadId defaultTaskHandle;
osThreadId myTaskMotorHandle;
osThreadId myTaskEncoderHandle;
osThreadId myTaskAttitudeHandle;
osThreadId myTaskDistanceHandle;
osMutexId myMutexPrintfHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTaskMotor(void const * argument);
void StartTaskEncoder(void const * argument);
void StartTaskAttitude(void const * argument);
void StartTaskDistance(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of myMutexPrintf */
  osMutexDef(myMutexPrintf);
  myMutexPrintfHandle = osMutexCreate(osMutex(myMutexPrintf));

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTaskMotor */
  osThreadDef(myTaskMotor, StartTaskMotor, osPriorityIdle, 0, 128);
  myTaskMotorHandle = osThreadCreate(osThread(myTaskMotor), NULL);

  /* definition and creation of myTaskEncoder */
  osThreadDef(myTaskEncoder, StartTaskEncoder, osPriorityIdle, 0, 128);
  myTaskEncoderHandle = osThreadCreate(osThread(myTaskEncoder), NULL);

  /* definition and creation of myTaskAttitude */
  osThreadDef(myTaskAttitude, StartTaskAttitude, osPriorityIdle, 0, 128);
  myTaskAttitudeHandle = osThreadCreate(osThread(myTaskAttitude), NULL);

  /* definition and creation of myTaskDistance */
  osThreadDef(myTaskDistance, StartTaskDistance, osPriorityIdle, 0, 128);
  myTaskDistanceHandle = osThreadCreate(osThread(myTaskDistance), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
    printf("hello\r\n");
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskMotor */
/**
* @brief Function implementing the myTaskMotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMotor */
void StartTaskMotor(void const * argument)
{
  /* USER CODE BEGIN StartTaskMotor */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // 启动通道1的PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // 启动通道2的PWM
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 900 - 1);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 900 - 1);
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_WritePin(Motor1Dir1_GPIO_Port, Motor1Dir1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motor1Dir2_GPIO_Port, Motor1Dir2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor2Dir1_GPIO_Port, Motor2Dir1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motor2Dir2_GPIO_Port, Motor2Dir2_Pin, GPIO_PIN_RESET);
    osDelay(1000);
  }
  /* USER CODE END StartTaskMotor */
}

/* USER CODE BEGIN Header_StartTaskEncoder */
/**
* @brief Function implementing the myTaskEncoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskEncoder */
void StartTaskEncoder(void const * argument)
{
  /* USER CODE BEGIN StartTaskEncoder */
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1 | TIM_CHANNEL_2);
  /* Infinite loop */
  for(;;)
  {
    int Direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
    int CaptureNumber = (short)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_GET_COUNTER(&htim2) = 0;
    printf("Direction1 is %d \r\n", Direction);
    printf("CaptureNumber1 is %d \r\n", CaptureNumber);
    int Direction2 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
    int CaptureNumber2 = (short)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_GET_COUNTER(&htim3) = 0;
    printf("Direction2 is %d \r\n", Direction2);
    printf("CaptureNumber2 is %d \r\n", CaptureNumber2);
    HAL_Delay(1000);
  }
  /* USER CODE END StartTaskEncoder */
}

/* USER CODE BEGIN Header_StartTaskAttitude */
/**
* @brief Function implementing the myTaskAttitude thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskAttitude */
void StartTaskAttitude(void const * argument)
{
  /* USER CODE BEGIN StartTaskAttitude */
  MPU6050_t MPU6050;
  while (MPU6050_Init(&hi2c1) == 1);
  /* Infinite loop */
  for(;;)
  {
    MPU6050_Read_All(&hi2c1, &MPU6050);
    printf("KalmanAngleX %f \r\n",MPU6050.KalmanAngleX);
    printf("KalmanAngleY %f \r\n",MPU6050.KalmanAngleY);
    osDelay(1000);
  }
  /* USER CODE END StartTaskAttitude */
}

/* USER CODE BEGIN Header_StartTaskDistance */
/**
* @brief Function implementing the myTaskDistance thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskDistance */
void StartTaskDistance(void const * argument)
{
  /* USER CODE BEGIN StartTaskDistance */
	VL6180xDev_t myDev = 0x52;
	VL6180x_RangeData_t Range;

	VL6180x_InitData(myDev);
	VL6180x_Prepare(myDev);
  /* Infinite loop */
  for(;;)
  {
    VL6180x_RangePollMeasurement(myDev, &Range);
    if (Range.errorStatus == 0)
      printf("range in mm:%ld \r\n", Range.range_mm);
    else
      printf("error code:%ld \r\n", Range.errorStatus);
    osDelay(1000);
  }
  /* USER CODE END StartTaskDistance */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

