/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

typedef enum MainStates_e{
	INITIAL_STATE,
	REGULAR_STATE,
	HOUR_SETTINGS_STATE,
	MIN_SETTINGS_STATE,
#ifdef DAC_SEC
	SEC_SETTINGS_STATE,
#endif
	TWERK_SETTINGS_STATE,
	FIN_SETTINGS_STATE
}MainStates_e;

typedef struct BtnStruct_t{
	uint8_t btn_led_set;
	uint8_t btn_menu;
}BtnStruct_t;

struct led_t{
	uint8_t status;
	uint8_t blinking;
};

typedef struct led_status_t{
	struct led_t led_hour;
	struct led_t led_min;
	struct led_t led_sec;
}led_status_t;

typedef struct time_data_t{
	uint8_t hours;
	uint8_t mins;
	uint8_t sec;
	uint8_t twerking;
}time_data_t;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ClockDealer */
osThreadId_t ClockDealerHandle;
const osThreadAttr_t ClockDealer_attributes = {
  .name = "ClockDealer",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for DacDealer */
osThreadId_t DacDealerHandle;
const osThreadAttr_t DacDealer_attributes = {
  .name = "DacDealer",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BtnHandler */
osThreadId_t BtnHandlerHandle;
const osThreadAttr_t BtnHandler_attributes = {
  .name = "BtnHandler",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LedDealer */
osThreadId_t LedDealerHandle;
const osThreadAttr_t LedDealer_attributes = {
  .name = "LedDealer",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MainTaskName */
osThreadId_t MainTaskNameHandle;
const osThreadAttr_t MainTaskName_attributes = {
  .name = "MainTaskName",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ClockDataMutex */
osMutexId_t ClockDataMutexHandle;
const osMutexAttr_t ClockDataMutex_attributes = {
  .name = "ClockDataMutex"
};
/* Definitions for LedsSem */
osSemaphoreId_t LedsSemHandle;
const osSemaphoreAttr_t LedsSem_attributes = {
  .name = "LedsSem"
};
/* Definitions for SaveEvent */
osEventFlagsId_t SaveEventHandle;
const osEventFlagsAttr_t SaveEvent_attributes = {
  .name = "SaveEvent"
};
/* USER CODE BEGIN PV */
osMessageQueueId_t BtnQueue;
led_status_t g_led_data;
time_data_t *g_dac_current_time;
time_data_t g_editable_time, g_readed_time;

float g_h_delta, g_m_delta, g_s_delta;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void *argument);
void ClockTask(void *argument);
void DacTask(void *argument);
void BtnTask(void *argument);
void LedTask(void *argument);
void MainTask(void *argument);

/* USER CODE BEGIN PFP */

//dec cntrl

float time_hours_to_voltage(time_data_t *time){

	static float r = 0.0f;


	if (time->twerking){
		r += 0.01;
		if (r > 0.08f)
			r = -0.08f;
	}else{
		r = 0.0f;
	}

	int data_fitter = 0;
	if (time->hours > 12)
		data_fitter = 12;
	return (float)(r + (float)(g_h_delta * (float)(time->hours - data_fitter)) * VOLTAGE_CORR_COEFF);
}

float time_mins_to_voltage(time_data_t *time){

	static float r = 0.0f;

	if (time->twerking){
		r += 0.01;
		if (r > 0.08f)
			r = -0.08f;
	}
	else{
		r = 0.0f;
	}

	return (float)(r + (float)(g_m_delta * (float)(time->mins)));
}

#ifdef DAC_SEC
float time_secs_to_voltage(time_data_t *time){
	return (float)(g_s_delta * (float)(time->sec));
}
#endif

//time controls

void time_hour_increment(time_data_t *time){
	time->hours++;
	if (time->hours == 13){
		time->hours = 1;
	}
}

void time_min_increment(time_data_t *time){
	time->mins++;
	if (time->mins == 60){
		time->mins = 0;
	}
}

#ifdef DAC_SEC
void time_sec_increment(time_data_t *time){
	time->sec++;
	if (time->sec == 60){
		time->sec = 0;
	}
}
#endif

void time_switch_to_local(void){
	g_dac_current_time = &g_editable_time;
}

void time_switch_to_global(void){
	g_dac_current_time = &g_readed_time;
}

void time_set_readed(time_data_t *time){
	g_readed_time = *time;
}

void time_get_data(time_data_t *time){
	*time = *g_dac_current_time;
}

void time_set_local(time_data_t *time){
	g_editable_time = *time;
}

void time_init(time_data_t *time){
	time->hours = 0;
	time->mins = 0;
	time->sec = 0;
	time->twerking = 0;
}

//led controls

void get_leds_data(led_status_t* led_data){
	*led_data = g_led_data;
}

void set_leds_data(led_status_t* led_data){
	g_led_data = *led_data;
}

void toggle_leds(led_status_t* led_data){
	if (led_data->led_hour.status == 1){
		led_data->led_hour.status = 0;
	}else{
		led_data->led_hour.status = 1;
	}
	if (led_data->led_min.status == 1){
		led_data->led_min.status = 0;
	}else{
		led_data->led_min.status = 1;
	}
#ifdef DAC_SEC
	if (led_data->led_sec.status == 1){
		led_data->led_sec.status = 0;
	}else{
		led_data->led_sec.status = 1;
	}
#endif
}

void init_leds_data(void){
	g_led_data.led_hour.blinking = 0;
	g_led_data.led_hour.status = 0;
	g_led_data.led_min.blinking = 0;
	g_led_data.led_min.status = 0;
#ifdef DAC_SEC
	g_led_data.led_sec.blinking = 0;
	g_led_data.led_sec.status = 0;
#endif
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  init_leds_data();
  time_init(&g_readed_time);
  time_init(&g_editable_time);

  //get time to data from ds1302
  time_switch_to_global();

  //calc deltas for time-to-voltage functions
  g_h_delta = (float)MAX_VOLTAGE_TO_SHOW/(float)12.0f;
  g_m_delta = (float)MAX_VOLTAGE_TO_SHOW/(float)60.0f;
#ifdef DAC_SEC
  g_s_delta = (float)MAX_VOLTAGE_TO_SHOW/(float)60.0f;
#endif
  HAL_IWDG_Refresh(&hiwdg);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of ClockDataMutex */
  ClockDataMutexHandle = osMutexNew(&ClockDataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of LedsSem */
  LedsSemHandle = osSemaphoreNew(1, 1, &LedsSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  BtnQueue = osMessageQueueNew(4, sizeof(BtnStruct_t), NULL);
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ClockDealer */
  ClockDealerHandle = osThreadNew(ClockTask, NULL, &ClockDealer_attributes);

  /* creation of DacDealer */
  DacDealerHandle = osThreadNew(DacTask, NULL, &DacDealer_attributes);

  /* creation of BtnHandler */
  BtnHandlerHandle = osThreadNew(BtnTask, NULL, &BtnHandler_attributes);

  /* creation of LedDealer */
  LedDealerHandle = osThreadNew(LedTask, NULL, &LedDealer_attributes);

  /* creation of MainTaskName */
  MainTaskNameHandle = osThreadNew(MainTask, NULL, &MainTaskName_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of SaveEvent */
  SaveEventHandle = osEventFlagsNew(&SaveEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CLOCK_CLK_Pin|CLOCK_nRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_MIN_Pin|LED_HOUR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_SEC_GPIO_Port, LED_SEC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CLOCK_CLK_Pin CLOCK_nRST_Pin */
  GPIO_InitStruct.Pin = CLOCK_CLK_Pin|CLOCK_nRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CLOCK_DATA_Pin */
  GPIO_InitStruct.Pin = CLOCK_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CLOCK_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_MIN_Pin LED_HOUR_Pin */
  GPIO_InitStruct.Pin = LED_MIN_Pin|LED_HOUR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_SEC_Pin */
  GPIO_InitStruct.Pin = LED_SEC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_SEC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_SET_LED_INP_Pin BTN_MENU_INP_Pin */
  GPIO_InitStruct.Pin = BTN_SET_LED_INP_Pin|BTN_MENU_INP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	//wdg refresh
	HAL_IWDG_Refresh(&hiwdg);
    osDelay(5);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ClockTask */
/**
* @brief Function implementing the ClockDealer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ClockTask */
void ClockTask(void *argument)
{
  /* USER CODE BEGIN ClockTask */
	osStatus_t status;
	uint32_t flag = 0;
	time_data_t l_time_data;
	Time_Struct time;
	led_status_t leds;

	//init ds1302
	DS1302_Init();

	//check saved leds status
	uint8_t ram_data = 0;
	ram_data = DS1302_RamRead(0);

	if ((ram_data & 0x01) == 1){
		leds.led_hour.status = 1;
		leds.led_min.status = 1;
		leds.led_sec.status = 1;
	}
	else if ((ram_data & 0x01) == 0){
		leds.led_hour.status = 0;
		leds.led_min.status = 0;
		leds.led_sec.status = 0;
	}

	//init local time data
	time_init(&l_time_data);

	if ((ram_data >> 1) == 1){
		l_time_data.twerking = 1;
	}
	else if ((ram_data >> 1) == 0){
		l_time_data.twerking = 0;
	}


	status = osMutexAcquire(LedsSemHandle, 0);
	if (status == osOK){
		set_leds_data(&leds);
		osMutexRelease(LedsSemHandle);
	}

	time = DS1302_ReadTime_Struct();

	//if default settings - needs to initialize
	if ((time.hour == 0) && (time.sec == 0) && (time.min == 0) && (time.year == 2000)){
		time.year = 2002;
		time.sec = 5;
		time.isHourClock24 = 0;
		DS1302_SetTime_Struct(&time);
	}



	status = osMutexAcquire(ClockDataMutexHandle, 0);
	if (status == osOK){
		time_set_readed(&l_time_data);
		osMutexRelease(ClockDataMutexHandle);
	}
	uint32_t timeout = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  timeout += 500;
	//save time to ds1302 if save event happened
	flag = osEventFlagsGet(SaveEventHandle);
	if (flag == 0x01){
		osEventFlagsClear(SaveEventHandle, flag);
		status = osMutexAcquire(ClockDataMutexHandle, 0);
		if (status == osOK){
			time_get_data(&l_time_data);
			time_set_readed(&l_time_data);
			time.hour = l_time_data.hours;
			time.min = l_time_data.mins;

			status = osMutexAcquire(LedsSemHandle, 0);
			if (status == osOK){
				get_leds_data(&leds);
				DS1302_RamSave(0, ((leds.led_hour.status) | (l_time_data.twerking << 1)));
				osMutexRelease(LedsSemHandle);
			}

			DS1302_SetTime_Struct(&time);
			time_switch_to_global();
			osMutexRelease(ClockDataMutexHandle);
		}
	}
	else if (flag == 0x02){
		status = osMutexAcquire(ClockDataMutexHandle, 0);
		if (status == osOK){
			time_get_data(&l_time_data);
			osMutexRelease(ClockDataMutexHandle);
		}
		osEventFlagsClear(SaveEventHandle, flag);
		status = osMutexAcquire(LedsSemHandle, 0);
		if (status == osOK){
			get_leds_data(&leds);
			DS1302_RamSave(0, ((leds.led_hour.status) | (l_time_data.twerking << 1)));
			osMutexRelease(LedsSemHandle);
		}
	}

	//read time from ds1302 and store it to global time data
	time = DS1302_ReadTime_Struct();
	l_time_data.hours = time.hour;
	l_time_data.mins = time.min;
	status = osMutexAcquire(ClockDataMutexHandle, 0);
	if (status == osOK){
		time_set_readed(&l_time_data);
		osMutexRelease(ClockDataMutexHandle);
	}
	osDelayUntil(timeout);
  }
  /* USER CODE END ClockTask */
}

/* USER CODE BEGIN Header_DacTask */
/**
* @brief Function implementing the DacDealer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DacTask */
void DacTask(void *argument)
{
  /* USER CODE BEGIN DacTask */
	MCP4725 l_hours_dac, l_mins_dac, l_secs_dac;
	osStatus_t mutex_request = 0;
	time_data_t l_time_data;

	time_init(&l_time_data);

	//init hour and min DACs
	osKernelLock();
	l_hours_dac = MCP4725_init(&hi2c1, 0x60, 3.3f);
	l_mins_dac  = MCP4725_init(&hi2c1, 0x61, 3.3f);
#ifdef DAC_SEC
	l_secs_dac  = MCP4725_init(&hi2c1, 0x56, 3.3f);
#endif
	osKernelUnlock();
  /* Infinite loop */
  for(;;)
  {

	//try get global time data and update DACs
	mutex_request = osMutexAcquire(ClockDataMutexHandle, 300);
	if (mutex_request == osOK){
		time_get_data(&l_time_data);
		osMutexRelease(ClockDataMutexHandle);
		osKernelLock();
		MCP4725_setVoltage(&l_hours_dac, time_hours_to_voltage(&l_time_data), MCP4725_FAST_MODE, MCP4725_POWER_DOWN_OFF);
		MCP4725_setVoltage(&l_mins_dac, time_mins_to_voltage(&l_time_data), MCP4725_FAST_MODE, MCP4725_POWER_DOWN_OFF);
#ifdef DAC_SEC
		MCP4725_setVoltage(&l_secs_dac, time_secs_to_voltage(&l_time_data), MCP4725_FAST_MODE, MCP4725_POWER_DOWN_OFF);
#endif
		osKernelUnlock();
	}
	osDelay(10);
  }
  /* USER CODE END DacTask */
}

/* USER CODE BEGIN Header_BtnTask */
/**
* @brief Function implementing the BtnHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BtnTask */
void BtnTask(void *argument)
{
  /* USER CODE BEGIN BtnTask */

	//Reading data from btns with denouncer. Send data to queue if press-action fully completed

	int btn_led_set_cntr = 0, btn_menu_cntr = 0;
	uint8_t btn_menu_hi_state = 0, btn_led_set_hi_state = 0;
	BtnStruct_t l_btn_state = {.btn_led_set = 0, .btn_menu = 0};
	osStatus_t status;
  /* Infinite loop */
  for(;;)
  {
	if (HAL_GPIO_ReadPin(BTN_SET_LED_INP_GPIO_Port, BTN_SET_LED_INP_Pin)){
		if (btn_led_set_hi_state == 0){
			btn_led_set_cntr++;
			if (btn_led_set_cntr > BTN_DEBOUNCER_MS){
				btn_led_set_hi_state = 1;
			}
		}
	}
	else{
		if (btn_led_set_cntr == 0){
			if (btn_led_set_hi_state == 1){
				btn_led_set_hi_state = 0;
				l_btn_state.btn_led_set = 1;
				status = osMessageQueuePut(BtnQueue, &l_btn_state, 0, 0);
				if (status == osOK){
					l_btn_state.btn_led_set = 0;
				}
			}
		}
		else{
			btn_led_set_cntr--;
		}
	}


	if (HAL_GPIO_ReadPin(BTN_MENU_INP_GPIO_Port, BTN_MENU_INP_Pin)){
		if (btn_menu_hi_state == 0){
			btn_menu_cntr++;
			if (btn_menu_cntr > BTN_DEBOUNCER_MS){
				btn_menu_hi_state = 1;
			}
		}
	}
	else{
		if (btn_menu_cntr == 0){
			if (btn_menu_hi_state == 1){
				btn_menu_hi_state = 0;
				l_btn_state.btn_menu = 1;
				status = osMessageQueuePut(BtnQueue, &l_btn_state, 0, 0);
				if (status == osOK){
					l_btn_state.btn_menu = 0;
				}
			}
		}
		else{
			btn_menu_cntr--;
		}
	}
    osDelay(1);
  }
  /* USER CODE END BtnTask */
}

/* USER CODE BEGIN Header_LedTask */
/**
* @brief Function implementing the LedDealer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LedTask */
void LedTask(void *argument)
{
  /* USER CODE BEGIN LedTask */

	//Checks leds global statuses and change GPIO states, if needed/ Also could blinks

	led_status_t leds;
	osStatus_t status;
	uint32_t blink_timeout = 0, blink_event = 0, toggle_status = 0;

	uint32_t timeout = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	timeout += 10;

	status = osSemaphoreAcquire(LedsSemHandle, 300);
	if (status == osOK){
		get_leds_data(&leds);
		osSemaphoreRelease(LedsSemHandle);
	}

	if (blink_timeout > LED_BLINK_TICK_MS_TIMES_10){
		if (toggle_status){
			toggle_status = 0;
		}else
		{
			toggle_status = 1;
		}
		blink_event = 1;
		blink_timeout = 0;
	}
	else{
		blink_event = 0;
		blink_timeout++;
	}


	if (leds.led_hour.status == 1){
		if (leds.led_hour.blinking == 1){
			if (blink_event){
				if (toggle_status){
					HAL_GPIO_WritePin(LED_HOUR_GPIO_Port, LED_HOUR_Pin, GPIO_PIN_SET);
				}
				else{
					HAL_GPIO_WritePin(LED_HOUR_GPIO_Port, LED_HOUR_Pin, GPIO_PIN_RESET);
				}
			}
		}else{
			HAL_GPIO_WritePin(LED_HOUR_GPIO_Port, LED_HOUR_Pin, GPIO_PIN_SET);
		}
	}else{
		HAL_GPIO_WritePin(LED_HOUR_GPIO_Port, LED_HOUR_Pin, GPIO_PIN_RESET);
	}


	if (leds.led_min.status == 1){
		if (leds.led_min.blinking == 1){
			if (blink_event){
				if (toggle_status){
					HAL_GPIO_WritePin(LED_MIN_GPIO_Port, LED_MIN_Pin, GPIO_PIN_SET);
				}
				else{
					HAL_GPIO_WritePin(LED_MIN_GPIO_Port, LED_MIN_Pin, GPIO_PIN_RESET);
				}
			}
		}else{
			HAL_GPIO_WritePin(LED_MIN_GPIO_Port, LED_MIN_Pin, GPIO_PIN_SET);
		}
	}else{
		HAL_GPIO_WritePin(LED_MIN_GPIO_Port, LED_MIN_Pin, GPIO_PIN_RESET);
	}


#ifdef DAC_SEC
	if (leds.led_sec.status == 1){
		if (leds.led_sec.blinking == 1){
			if (sec_timeout < osKernelSysTick()){
				HAL_GPIO_TogglePin(LED_SEC_GPIO_Port, LED_SEC_Pin);
				sec_timeout = osKernelSysTick() + LED_BLINK_TICK;
			}
		}else{
			HAL_GPIO_WritePin(LED_SEC_GPIO_Port, LED_SEC_Pin, GPIO_PIN_SET);
		}
	}else{
		HAL_GPIO_WritePin(LED_SEC_GPIO_Port, LED_SEC_Pin, GPIO_PIN_RESET);
	}
#endif
    osDelayUntil(timeout);
  }
  /* USER CODE END LedTask */
}

/* USER CODE BEGIN Header_MainTask */
/**
* @brief Function implementing the MainTaskName thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MainTask */
void MainTask(void *argument)
{
  /* USER CODE BEGIN MainTask */

	//Main logic of program
	//TODO: save and restore illumination state settings

	MainStates_e state;
	time_data_t l_time_data;
	led_status_t l_leds, pre_settings_leds;
	BtnStruct_t l_btn_state;

	state = INITIAL_STATE;
	time_init(&l_time_data);

  /* Infinite loop */
  for(;;)
  {
	switch (state){
	case INITIAL_STATE:
	{
		get_leds_data(&l_leds);
		get_leds_data(&pre_settings_leds);
		pre_settings_leds.led_hour.blinking = 1;
		pre_settings_leds.led_min.blinking = 1;
		pre_settings_leds.led_sec.blinking = 1;

		state = REGULAR_STATE;
		break;
	}
	case REGULAR_STATE:
	{
		//Checks btn-messages. if a message has arrived, it checks which one. When a message from the menu button - goes to the settings
		osStatus_t status;
		status = osMessageQueueGet(BtnQueue, &l_btn_state, 0, 300);
		if (status == osOK){

			if (l_btn_state.btn_menu){
				pre_settings_leds.led_hour.status = 1;
				pre_settings_leds.led_hour.blinking = 1;
				pre_settings_leds.led_min.status = 1;
				pre_settings_leds.led_min.blinking = 0;
				status = osSemaphoreAcquire(LedsSemHandle, 0);
				if (status == osOK){
					set_leds_data(&pre_settings_leds);
					osSemaphoreRelease(LedsSemHandle);
				}
				status = osMutexAcquire(ClockDataMutexHandle, 0);
				if (status == osOK){
					//During the time settings, only the local time changes (a copy of the exact time), which is then(FIN_SETTINGS_STATE) written to the ds1302 memory and changes back to the global one (from ds1302)
					time_get_data(&l_time_data);
					time_switch_to_local();
					time_set_local(&l_time_data);
					osMutexRelease(ClockDataMutexHandle);
				}
				state = HOUR_SETTINGS_STATE;

			}else if(l_btn_state.btn_led_set){
				//if led btn pressed, then change leds status and save it to RAM-space at ds1302
				toggle_leds(&l_leds);
				status = osSemaphoreAcquire(LedsSemHandle, 0);
				if (status == osOK){
					osEventFlagsSet(SaveEventHandle, 0x02); //sending flag to store new state at RAM
					set_leds_data(&l_leds);
					osSemaphoreRelease(LedsSemHandle);
				}
			}

		}
		break;
	}
	case HOUR_SETTINGS_STATE:
	{
		//The behavior of the hour, minute and second settings is the same. Checking the buttons and either changing the time or saving it. Also LED status control
		osStatus_t status;
		status = osMessageQueueGet(BtnQueue, &l_btn_state, 0, 300);
		if (status == osOK){
			if (l_btn_state.btn_led_set){
				time_hour_increment(&l_time_data);
				status = osMutexAcquire(ClockDataMutexHandle, 0);
				if (status == osOK){
					time_set_local(&l_time_data);
					osMutexRelease(ClockDataMutexHandle);
				}
			}
			else if (l_btn_state.btn_menu){
				pre_settings_leds.led_hour.status = 1;
				pre_settings_leds.led_hour.blinking = 0;
				pre_settings_leds.led_min.status = 1;
				pre_settings_leds.led_min.blinking = 1;

				status = osSemaphoreAcquire(LedsSemHandle, 0);
				if (status == osOK){
					set_leds_data(&pre_settings_leds);
					osSemaphoreRelease(LedsSemHandle);
				}
				state = MIN_SETTINGS_STATE;
			}
		}
		break;
	}
	case MIN_SETTINGS_STATE:
	{
		osStatus_t status;
		status = osMessageQueueGet(BtnQueue, &l_btn_state, 0, 300);
		if (status == osOK){
			if (l_btn_state.btn_led_set){
				time_min_increment(&l_time_data);
				status = osMutexAcquire(ClockDataMutexHandle, 0);
				if (status == osOK){
					time_set_local(&l_time_data);
					osMutexRelease(ClockDataMutexHandle);
				}
			}
			else if (l_btn_state.btn_menu){

				//reset prev states of leds and lets toggle from ON status
				pre_settings_leds.led_min.status = 1;
				pre_settings_leds.led_hour.status = 1;
				pre_settings_leds.led_min.blinking = 1;
				pre_settings_leds.led_hour.blinking = 1;
#ifdef DAC_SEC
				pre_settings_leds.led_min.status = 0;
				pre_settings_leds.led_hour.status = 0;
				pre_settings_leds.led_sec.status = 1;
				state = SEC_SETTINGS_STATE;
#else
				status = osSemaphoreAcquire(LedsSemHandle, 0);
				if (status == osOK){
					set_leds_data(&pre_settings_leds);
					osSemaphoreRelease(LedsSemHandle);
				}
				state = TWERK_SETTINGS_STATE;
#endif
			}
		}
		break;
	}
#ifdef DAC_SEC
	case SEC_SETTINGS_STATE:
	{
		osStatus_t status;
		status = osMessageQueueGet(BtnQueue, &l_btn_state, 0, 300);
		if (status == osOK){
			if (l_btn_state.btn_led_set){
				time_sec_increment(&l_time_data);
				status = osMutexAcquire(ClockDataMutexHandle, 0);
				if (status == osOK){
					time_set_local(&l_time_data);
					osMutexRelease(ClockDataMutexHandle);
				}
			}
			else if (l_btn_state.btn_menu){
				pre_settings_leds.led_min.status = 1;
				pre_settings_leds.led_hour.status = 1;
				pre_settings_leds.led_sec.status = 1;
				pre_settings_leds.led_min.blinking = 1;
				pre_settings_leds.led_hour.blinking = 1;
				pre_settings_leds.led_sec.blinking = 1;
				status = osSemaphoreAcquire(LedsSemHandle, 0);
				if (status == osOK){
					set_leds_data(&pre_settings_leds);
					osSemaphoreRelease(LedsSemHandle);
				}
				state = TWERK_SETTINGS_STATE;
			}
		}
		break;
	}
#endif
	case TWERK_SETTINGS_STATE:
	{
		osStatus_t status;
		status = osMessageQueueGet(BtnQueue, &l_btn_state, 0, 300);
		if (status == osOK){
			if (l_btn_state.btn_led_set){
				if (l_time_data.twerking){
					l_time_data.twerking = 0;
				}
				else{
					l_time_data.twerking = 1;
				}
				status = osMutexAcquire(ClockDataMutexHandle, 0);
				if (status == osOK){
					time_set_local(&l_time_data);
					osMutexRelease(ClockDataMutexHandle);
				}
			}
			else if (l_btn_state.btn_menu){
				pre_settings_leds.led_min.status = 0;
				pre_settings_leds.led_hour.status = 0;
				pre_settings_leds.led_sec.status = 0;
				status = osSemaphoreAcquire(LedsSemHandle, 0);
				if (status == osOK){
					set_leds_data(&pre_settings_leds);
					osSemaphoreRelease(LedsSemHandle);
				}
				status = osMutexAcquire(ClockDataMutexHandle, 0);
				if (status == osOK){
					time_set_local(&l_time_data);
					osMutexRelease(ClockDataMutexHandle);
				}
				state = FIN_SETTINGS_STATE;
			}
		}
		break;
	}
	case FIN_SETTINGS_STATE:
	{
		//End of settings. The illumination state is restored. The save settings event is set.
		osStatus_t status;
		status = osSemaphoreAcquire(LedsSemHandle, 0);
		if (status == osOK){
			set_leds_data(&l_leds);
			osSemaphoreRelease(LedsSemHandle);
		}

		osEventFlagsSet(SaveEventHandle, 0x01);
		state = REGULAR_STATE;
		break;
	}
	}
    osDelay(10);
  }
  /* USER CODE END MainTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
