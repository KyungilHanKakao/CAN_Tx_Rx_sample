/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include "veml7700.h"
#include "w25q.h"
#include "Z_FLASH_W25QXXX.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VEML7700_I2C_ADDRESS    0x10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern I2C_HandleTypeDef hi2c1;
extern CAN_HandleTypeDef hcan;
extern SPI_HandleTypeDef hspi1;

CAN_RxHeaderTypeDef rxHeader; //CAN Bus Transmit Header
CAN_TxHeaderTypeDef txHeader; //CAN Bus Receive Header
uint8_t canRX[8] = {0,0,0,0,0,0,0,0};  //CAN Bus Receive Buffer
CAN_FilterTypeDef canfil; //CAN Bus Filter
uint32_t canMailbox; //CAN Bus Mail box variable

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityLow, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityBelowNormal, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

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
  /* USER CODE BEGIN StartDefaultTask */

  canfil.FilterBank = 0;
  canfil.FilterMode = CAN_FILTERMODE_IDMASK;
  canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfil.FilterIdHigh = 0;
  canfil.FilterIdLow = 0;
  canfil.FilterMaskIdHigh = 0;
  canfil.FilterMaskIdLow = 0;
  canfil.FilterScale = CAN_FILTERSCALE_32BIT;
  canfil.FilterActivation = ENABLE;
  canfil.SlaveStartFilterBank = 14;

  txHeader.DLC = 8;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.StdId = 0x050;
  //txHeader.ExtId = 0x02;
  txHeader.TransmitGlobalTime = DISABLE;

  HAL_CAN_ConfigFilter(&hcan,&canfil);
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);

  /* Infinite loop */
  for(;;)
  {
	  uint8_t csend[] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
	  HAL_CAN_AddTxMessage(&hcan,&txHeader,csend,&canMailbox);

	  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	  osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */

	  // Init / power on sensor
	  veml7700 veml;
	  veml7700_init(&veml, &hi2c1, VEML7700_I2C_ADDRESS);
	  veml7700_power_on(&veml);
	  veml7700_set_als_gain(&veml,REG_ALS_CONF_GAIN_1_4 );
	  veml7700_set_als_integration_time(&veml,REG_ALS_CONF_IT800 );



	  while(1){
		  uint16_t als = veml7700_read_als(&veml);
		  uint16_t white = veml7700_read_white(&veml);
		  uint16_t id= veml7700_getID(&veml);
		  uint16_t cal_als= als * (veml.resolution);



		  printf("cal_als %d\n", cal_als);
		  printf("white %d\n", white);
		  printf("id %x\n", id);

		  if(cal_als <100){
			  veml7700_set_als_gain(&veml,REG_ALS_CONF_GAIN_2 );
			  veml7700_set_als_integration_time(&veml,REG_ALS_CONF_IT800 );
			  printf("Change : g2,it800\n");
		  }
		  else if(cal_als > 1000 ){
			  veml7700_set_als_gain(&veml,REG_ALS_CONF_GAIN_1_8 );
			  veml7700_set_als_integration_time(&veml,REG_ALS_CONF_IT800 );
			  printf("Change : g1_8,it800\n");
		  }
		  else{
			  veml7700_set_als_gain(&veml,REG_ALS_CONF_GAIN_1_4 );
			  veml7700_set_als_integration_time(&veml,REG_ALS_CONF_IT800 );
			  printf("Change : g1_4,it800\n");
		  }

		  osDelay(1000);
	  }

	  // VEML7700 constantly measuring values, so turn it off to save power
	  veml7700_shutdown(&veml);

  /* Infinite loop */

  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
	uint8_t uniqueID[8];
	uint8_t writeData[8]={00,01,02,03,04,05,06,07};
	Flash_Init();
	/* Infinite loop */
  for(;;)
  {
	  Flash_Reset();
	  Flash_ReadUniqueID(uniqueID);
	  printf("unique id = ");
	  for(uint8_t i=0; i<8;i++){
		  printf(" %x ",uniqueID[i]);
	  }
	  printf("\n");

	  printf("power-down id = %04x\n",Flash_ReadDevID());
	  printf("id = %04x\n",Flash_ReadManufactutrerAndDevID());
	  printf("jedec id = %04lx\n",Flash_ReadJedecID());

	  Flash_Write(0x00,writeData,8 );
	  Flash_Read(0x00,uniqueID,8);
	  printf("Flash_Read = ");
	  for(uint8_t i=0; i<8;i++){
		  printf(" %x ",uniqueID[i]);
	  }
	  printf("\n");
	  osDelay(1500);
  }
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, canRX);
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);

	if (rxHeader.StdId == 0x34) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}


}
/* USER CODE END Application */

