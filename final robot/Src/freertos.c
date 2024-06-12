/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

#include "stdlib.h"
#include <string.h>

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
extern UART_HandleTypeDef huart1;

char Tx_buf[70];
char temp_Tx_buf[70];
float dataTx[10];
float data[10] = {34.67, -34.67, 4.78, -4.78, 89.2, -89.2, 38, -38, 0.45, -0.45};
float dataFloat;
int dataInt = -56;
float dataTx[10];
uint8_t for_index = 0;
char temp_char [6];



extern uint8_t miniPC_data[69];
uint8_t miniPC_data_conv[69];
int RX_index = 0;
char temp_miniPC_RX[6];
float miniPC_dataFloat[10];



/* USER CODE END Variables */
osThreadId ImuTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId gimbalTaskHandle;
osThreadId shootingTaskHandle;
osThreadId REFEREEHandle;
osThreadId DetectHandle;
osThreadId USART_MiniPCHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void INS_task(void const * argument);
void chassis_task(void const * argument);
void gimbal_task(void const * argument);
void shooting_task(void const * argument);
void referee_usart_task(void const * argument);
void detect_task(void const * argument);
void Start_USART_MiniPC(void const * argument);

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
  /* definition and creation of ImuTask */
  osThreadDef(ImuTask, INS_task, osPriorityRealtime, 0, 1024);
  ImuTaskHandle = osThreadCreate(osThread(ImuTask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, chassis_task, osPriorityAboveNormal, 0, 1024);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* definition and creation of gimbalTask */
  osThreadDef(gimbalTask, gimbal_task, osPriorityHigh, 0, 1024);
  gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);

  /* definition and creation of shootingTask */
  osThreadDef(shootingTask, shooting_task, osPriorityHigh, 0, 512);
  shootingTaskHandle = osThreadCreate(osThread(shootingTask), NULL);

  /* definition and creation of REFEREE */
  osThreadDef(REFEREE, referee_usart_task, osPriorityNormal, 0, 128);
  REFEREEHandle = osThreadCreate(osThread(REFEREE), NULL);

//  /* definition and creation of Detect */
//  osThreadDef(Detect, detect_task, osPriorityBelowNormal, 0, 128);
//  DetectHandle = osThreadCreate(osThread(Detect), NULL);

  /* definition and creation of USART_MiniPC */
  osThreadDef(USART_MiniPC, Start_USART_MiniPC, osPriorityNormal, 0, 128);
  USART_MiniPCHandle = osThreadCreate(osThread(USART_MiniPC), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

///* USER CODE BEGIN Header_INS_task */
/////**
////  * @brief  Function implementing the ImuTask thread.
////  * @param  argument: Not used
////  * @retval None
////  */
///* USER CODE END Header_INS_task */
//void INS_task(void const * argument)
//{
//  /* USER CODE BEGIN INS_task */
////  /* Infinite loop */
////  for(;;)
////  {
////    osDelay(1);
////  }
//  /* USER CODE END INS_task */
//}

/* USER CODE BEGIN Header_chassis_task */
///**
//* @brief Function implementing the ChassisTask thread.
//* @param argument: Not used
//* @retval None
////*/
///* USER CODE END Header_chassis_task */
//void chassis_task(void const * argument)
//{
//  /* USER CODE BEGIN chassis_task */
////  /* Infinite loop */
////  for(;;)
////  {
////    osDelay(1);
////  }
//  /* USER CODE END chassis_task */
//}

///* USER CODE BEGIN Header_gimbal_task */
/////**
////* @brief Function implementing the gimbalTask thread.
////* @param argument: Not used
////* @retval None
////*/
///* USER CODE END Header_gimbal_task */
//void gimbal_task(void const * argument)
//{
//  /* USER CODE BEGIN gimbal_task */
////  /* Infinite loop */
////  for(;;)
////  {
////    osDelay(1);
////  }
//  /* USER CODE END gimbal_task */
//}

///* USER CODE BEGIN Header_shooting_task */
/////**
////* @brief Function implementing the shootingTask thread.
////* @param argument: Not used
////* @retval None
////*/
///* USER CODE END Header_shooting_task */
//void shooting_task(void const * argument)
//{
//  /* USER CODE BEGIN shooting_task */
////  /* Infinite loop */
////  for(;;)
////  {
////    osDelay(1);
////  }
//  /* USER CODE END shooting_task */
//}

/* USER CODE BEGIN Header_referee_usart_task */
///**
//* @brief Function implementing the REFEREE thread.
//* @param argument: Not used
//* @retval None
//*/
///* USER CODE END Header_referee_usart_task */
//void referee_usart_task(void const * argument)
//{
//  /* USER CODE BEGIN referee_usart_task */
////  /* Infinite loop */
////  for(;;)
////  {
////    osDelay(1);
////  }
//  /* USER CODE END referee_usart_task */
//}

///* USER CODE BEGIN Header_detect_task */
///**
//* @brief Function implementing the Detect thread.
//* @param argument: Not used
//* @retval None
//*/
///* USER CODE END Header_detect_task */
//void detect_task(void const * argument)
//{
//  /* USER CODE BEGIN detect_task */
//  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
//  /* USER CODE END detect_task */
//}

/* USER CODE BEGIN Header_Start_USART_MiniPC */
/**
* @brief Function implementing the USART_MiniPC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_USART_MiniPC */
void Start_USART_MiniPC(void const * argument)
{
  /* USER CODE BEGIN Start_USART_MiniPC */
  /* Infinite loop */
  for(;;)
  {
		// Demodulating data received on the STM32. miniPC_data comes from the AI_receive.c function. This part is before the transmission because it was giving some problems
		memcpy(miniPC_data_conv, miniPC_data, 69*sizeof(char));	
		for (RX_index = 0 ; RX_index < 10 ; RX_index++){
				memcpy(temp_miniPC_RX, &miniPC_data_conv[RX_index*7], 6*sizeof(char));
				miniPC_dataFloat[RX_index] = atof(temp_miniPC_RX);
		}

// Transmitting data to MiniPC		
		memcpy(dataTx, data, 10*sizeof(float));
		dataFloat = (float)dataInt;
		dataTx[9] = dataFloat;
		for (for_index = 0 ; for_index < 10 ; for_index++){
				memcpy(temp_char, (char [6]){0,0,0,0,0,0}, 6*sizeof(char));
				sprintf(temp_char, "%f", dataTx[for_index]);
				memcpy(&temp_Tx_buf[1 + for_index*7], temp_char, 6*sizeof(char));
				memcpy(&temp_Tx_buf[for_index*7+7], " ", sizeof(char));
		}

		memcpy(temp_Tx_buf, "%", sizeof(char));
		memcpy(Tx_buf, temp_Tx_buf, 70*sizeof(char));
		HAL_UART_Transmit_IT(&huart1, (uint8_t *) Tx_buf, 70);
		//HAL_UART_Transmit_IT(&huart1, (uint8_t *) "ciao", sizeof("ciao"));
		
    osDelay(10);
  }
  /* USER CODE END Start_USART_MiniPC */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
