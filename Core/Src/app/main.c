/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Weather Station - Step 1 Implementation
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "ssd1306.h"
#include "fonts.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "bmp280.h"
#include "sd_functions.h"

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ===== TIMING CONSTANTS =====
#define SENSOR_UPDATE_PERIOD_MS     500
#define DISPLAY_UPDATE_PERIOD_MS    500
#define UART_LOG_PERIOD_MS          1000
#define SD_LOG_PERIOD_MS            1000

// ===== BUFFER SIZES =====
#define UART_BUFFER_SIZE            128
#define LOG_BUFFER_SIZE             64
#define DISPLAY_LINE_SIZE           32

// ===== SENSOR VALIDATION =====
#define MIN_TEMPERATURE_C           -40.0f
#define MAX_TEMPERATURE_C           85.0f
#define MIN_PRESSURE_HPA            300.0f
#define MAX_PRESSURE_HPA            1100.0f

// ===== TASK STACK SIZES =====
#define DEFAULT_STACK_SIZE          (128 * 4)
#define SENSOR_STACK_SIZE           (512 * 4)
#define DISPLAY_STACK_SIZE          (512 * 4)
#define UART_STACK_SIZE             (384 * 4)
#define SD_STACK_SIZE               (512 * 4)

// ===== DISPLAY POSITIONS =====
#define DISPLAY_LINE1_X             2
#define DISPLAY_LINE1_Y             2
#define DISPLAY_LINE2_X             2
#define DISPLAY_LINE2_Y             16

// ===== SYSTEM DELAYS =====
#define DISPLAY_INIT_DELAY_MS       50
#define DEFAULT_TASK_DELAY_MS       1
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart2;

/* RTOS Task Handles ---------------------------------------------------------*/
osThreadId_t defaultTaskHandle;
osThreadId_t sensorTaskHandle;
osThreadId_t displayTaskHandle;
osThreadId_t uartTaskHandle;
osThreadId_t sdLogTaskHandle;

/* RTOS Sync Objects ---------------------------------------------------------*/
osMutexId_t i2cMutexHandle;
osMutexId_t uartMutexHandle;
osMutexId_t sdMutexHandle;
osSemaphoreId_t sdMountedSemHandle;
osMessageQueueId_t displayQueueHandle;
osMessageQueueId_t uartQueueHandle;
osMessageQueueId_t sdQueueHandle;

/* USER CODE BEGIN PV */
/* ===== DATA STRUCTURES ===== */
typedef struct {
    float temp;
    float press;
    uint32_t tick;
} BmpSample_t;

/* ===== GLOBAL VARIABLES ===== */
static BMP280_t bmp280 = {0};          // Sensor state

/* ===== TASK ATTRIBUTES ===== */
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = DEFAULT_STACK_SIZE,
    .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t sensorTask_attributes = {
    .name = "Sensor",
    .stack_size = SENSOR_STACK_SIZE,
    .priority = osPriorityNormal
};

const osThreadAttr_t displayTask_attributes = {
    .name = "Display",
    .stack_size = DISPLAY_STACK_SIZE,
    .priority = osPriorityBelowNormal
};

const osThreadAttr_t uartTask_attributes = {
    .name = "UART",
    .stack_size = UART_STACK_SIZE,
    .priority = osPriorityLow
};

const osThreadAttr_t sdTask_attributes = {
    .name = "SDLog",
    .stack_size = SD_STACK_SIZE,
    .priority = osPriorityLow
};

/* ===== MUTEX ATTRIBUTES ===== */
const osMutexAttr_t i2cMutex_attributes = {
    .name = "i2cMutex"
};

const osMutexAttr_t dataMutex_attributes = {
    .name = "dataMutex"
};

const osMutexAttr_t uartMutex_attributes = {
    .name = "uartMutex"
};

const osMutexAttr_t sdMutex_attributes = {
    .name = "sdMutex"
};

/* ===== SEMAPHORE & QUEUE ATTRIBUTES ===== */
const osSemaphoreAttr_t sdMountedSem_attributes = {
    .name = "sdMountedSem"
};

const osMessageQueueAttr_t displayQueue_attributes = {
    .name = "displayQ"
};

const osMessageQueueAttr_t uartQueue_attributes = {
    .name = "uartQ"
};

const osMessageQueueAttr_t sdQueue_attributes = {
    .name = "sdQ"
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void SensorTask(void *argument);
static void DisplayTask(void *argument);
static void UartTask(void *argument);
static void SDLogTask(void *argument);
static void uart_printf(const char *fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Thread-safe UART printf function
 * @param fmt: Format string
 */
static void uart_printf(const char *fmt, ...)
{
    char buf[UART_BUFFER_SIZE];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    osMutexAcquire(uartMutexHandle, osWaitForever);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
    osMutexRelease(uartMutexHandle);
}

/**
 * @brief Sensor reading task → sends samples via queues
 */
static void SensorTask(void *argument)
{
    osMutexAcquire(i2cMutexHandle, osWaitForever);
    BMP280_Init(&bmp280, &hi2c1);
    BMP280_ReadCalibrationData(&bmp280);
    osMutexRelease(i2cMutexHandle);

    for (;;)
    {
        BmpSample_t sample;

        osMutexAcquire(i2cMutexHandle, osWaitForever);
        sample.temp = BMP280_ReadTemperature(&bmp280);
        sample.press = BMP280_ReadPressure(&bmp280);
        osMutexRelease(i2cMutexHandle);
        sample.tick = HAL_GetTick();

        (void)osMessageQueuePut(displayQueueHandle, &sample, 0, 0);
        (void)osMessageQueuePut(uartQueueHandle, &sample, 0, 0);
        (void)osMessageQueuePut(sdQueueHandle, &sample, 0, 0);

        osDelay(SENSOR_UPDATE_PERIOD_MS);
    }
}

/**
 * @brief Display update task
 * @param argument: Not used
 */
static void DisplayTask(void *argument)
{
    // Initialize SSD1306 display under I2C mutex protection
    osMutexAcquire(i2cMutexHandle, osWaitForever);
    SSD1306_Init();
    HAL_Delay(DISPLAY_INIT_DELAY_MS);
    osMutexRelease(i2cMutexHandle);

    BmpSample_t sample;
    char line1[DISPLAY_LINE_SIZE];
    char line2[DISPLAY_LINE_SIZE];

    for (;;)
    {
        if (osMessageQueueGet(displayQueueHandle, &sample, NULL, osWaitForever) != osOK) {
            continue;
        }

        // Format display strings
        snprintf(line1, sizeof(line1), "T: %.2f C", sample.temp);
        snprintf(line2, sizeof(line2), "P: %.2f hPa", sample.press);

        // Update display under I2C mutex protection
        osMutexAcquire(i2cMutexHandle, osWaitForever);
        SSD1306_Fill(0);
        SSD1306_GotoXY(DISPLAY_LINE1_X, DISPLAY_LINE1_Y);
        SSD1306_Puts(line1, &Font_7x10, 1);
        SSD1306_GotoXY(DISPLAY_LINE2_X, DISPLAY_LINE2_Y);
        SSD1306_Puts(line2, &Font_7x10, 1);
        SSD1306_UpdateScreen();
        osMutexRelease(i2cMutexHandle);

        osDelay(DISPLAY_UPDATE_PERIOD_MS);
    }
}

/**
 * @brief UART logging task
 * @param argument: Not used
 */
static void UartTask(void *argument)
{
    for (;;)
    {
        BmpSample_t sample;
        if (osMessageQueueGet(uartQueueHandle, &sample, NULL, osWaitForever) != osOK) {
            continue;
        }
        uart_printf("%lu,%.2f,%.2f\r\n", sample.tick, sample.temp, sample.press);
    }
}

/**
 * @brief SD card logging task
 * @param argument: Not used
 */
static void SDLogTask(void *argument)
{
    if (sd_mount() != FR_OK) {
        uart_printf("SD mount failed!\r\n");
        vTaskDelete(NULL);
    }

    osSemaphoreRelease(sdMountedSemHandle);

    char log_buffer[LOG_BUFFER_SIZE];
    BmpSample_t sample;

    for (;;)
    {
        if (osMessageQueueGet(sdQueueHandle, &sample, NULL, osWaitForever) != osOK) {
            continue;
        }

        snprintf(log_buffer, sizeof(log_buffer), "%lu,%.2f,%.2f\r\n",
                sample.tick, sample.temp, sample.press);

        osMutexAcquire(sdMutexHandle, osWaitForever);
        sd_append_file("log.txt", log_buffer);
        osMutexRelease(sdMutexHandle);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* Create mutexes */
  i2cMutexHandle = osMutexNew(&i2cMutex_attributes);
  uartMutexHandle = osMutexNew(&uartMutex_attributes);
  sdMutexHandle = osMutexNew(&sdMutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* Create semaphore */
  sdMountedSemHandle = osSemaphoreNew(1, 0, &sdMountedSem_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* Create queues (depth 8) */
  displayQueueHandle = osMessageQueueNew(8, sizeof(BmpSample_t), &displayQueue_attributes);
  uartQueueHandle    = osMessageQueueNew(8, sizeof(BmpSample_t), &uartQueue_attributes);
  sdQueueHandle      = osMessageQueueNew(8, sizeof(BmpSample_t), &sdQueue_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* Create application tasks */
  sensorTaskHandle = osThreadNew(SensorTask, NULL, &sensorTask_attributes);
  displayTaskHandle = osThreadNew(DisplayTask, NULL, &displayTask_attributes);
  uartTaskHandle = osThreadNew(UartTask, NULL, &uartTask_attributes);
  sdLogTaskHandle = osThreadNew(SDLogTask, NULL, &sdTask_attributes);
  /* USER CODE END RTOS_THREADS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);
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
    osDelay(DEFAULT_TASK_DELAY_MS);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9)
  {
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

#ifdef USE_FULL_ASSERT
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
