/* USER CODE BEGIN Header */
/*
 * The following code was made to control a 240x320 TFT screen which uses the ILI9341 controller for the display,
 * and the XPT2046 controller for the touch screen.
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "tft_display.h"
#include "tft_touch.h"
#include "tft_fonts.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
SemaphoreHandle_t mutex;
QueueHandle_t queue;

#define MAIN_COLOR			COLOR_YELLOW
#define BACKGROUND_COLOR	COLOR_BLACK

static void Display(void *pvParameters){

	TFT_TOUCH_COORDS coords;
	coords.x = 0;
	coords.y = 0;

	char value_x[] = "000";
	char value_y[] = "000";

	xSemaphoreTake(mutex, portMAX_DELAY);

	TFT_Display_FillScreen(BACKGROUND_COLOR);
	TFT_Display_DrawRectangle(MAIN_COLOR,5, 5, TFT_Display_WIDTH-6, TFT_Display_HEIGHT-6);

	TFT_Display_WriteString(100, 50, "X =", Font_16x26, MAIN_COLOR, BACKGROUND_COLOR);
	TFT_Display_WriteString(100, 120, "Y =", Font_16x26, MAIN_COLOR, BACKGROUND_COLOR);
	TFT_Display_WriteString(70, 200, "Touch the screen", Font_11x18, MAIN_COLOR, BACKGROUND_COLOR);

	xSemaphoreGive(mutex);

	while(1){

		// Get the pressed coordinates from the queue, after timeout reset the coordinates
		if ( xQueueReceive(queue,&coords,100) != pdPASS ){
			coords.x = 0;
			coords.y = 0;
		}

		// Formatting
		if(coords.x < 10){
			sprintf(value_x, "00%i", coords.x);
		}
		else{
			if(coords.x < 100){
				sprintf(value_x, "0%i", coords.x);
			}
			else{
				sprintf(value_x, "%i", coords.x);
			}
		}

		if(coords.y < 10){
			sprintf(value_y, "00%i", coords.y);
		}
		else{
			if(coords.y < 100){
				sprintf(value_y, "0%i", coords.y);
			}
			else{
				sprintf(value_y, "%i", coords.y);
			}
		}


		xSemaphoreTake(mutex, portMAX_DELAY);

		TFT_Display_WriteString(160, 50, value_x, Font_16x26, MAIN_COLOR, BACKGROUND_COLOR);
		TFT_Display_WriteString(160, 120, value_y, Font_16x26, MAIN_COLOR, BACKGROUND_COLOR);

		xSemaphoreGive(mutex);
	}
}

static void Touch(void *pvParameters){

	TFT_TOUCH_COORDS coords;
	coords.x = 0;
	coords.y = 0;

	while(1){

		if( TFT_Touch_ScreenPressed() ){

			xSemaphoreTake(mutex, portMAX_DELAY);

			// Reduce SPI speed to give time to the touch controller (only necessary if using same SPI for both touch and display controllers)
			while(HAL_SPI_DeInit(&TFT_TOUCH_SPI_PORT) != HAL_OK);
			TFT_TOUCH_SPI_PORT.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; // 562K bits/s
			while(HAL_SPI_Init(&TFT_TOUCH_SPI_PORT) != HAL_OK);

			// Get pressed coordinates
			TFT_Touch_GetCoordinates(&coords.y, &coords.x);

			// Revert SPI speed value for appropriate communication with the display controller
			while(HAL_SPI_DeInit(&TFT_TOUCH_SPI_PORT) != HAL_OK);
			TFT_TOUCH_SPI_PORT.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; // 18M bits/s
			while(HAL_SPI_Init(&TFT_TOUCH_SPI_PORT) != HAL_OK);

			xSemaphoreGive(mutex);

			xQueueSendToBack(queue,&coords,portMAX_DELAY);

			vTaskDelay(100/portTICK_PERIOD_MS);
		}

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  TFT_Touch_Unselect();
  TFT_Display_Unselect();
  TFT_Display_Init();

  mutex = xSemaphoreCreateMutex();
  queue = xQueueCreate(1,sizeof(TFT_TOUCH_COORDS));

  xTaskCreate(Display,
		  	  	(char *) "",
         		configMINIMAL_STACK_SIZE,
				NULL,
				(tskIDLE_PRIORITY + 1UL),
         		NULL);

  xTaskCreate(Touch,
		  	  	(char *) "",
     	    	configMINIMAL_STACK_SIZE,
				NULL,
				(tskIDLE_PRIORITY + 1UL),
     	    	NULL);

  vTaskStartScheduler();
  /* USER CODE END 2 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, DISPLAY_LED_Pin|DISPLAY_DC_Pin|DISPLAY_RESET_Pin|DISPLAY_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DISPLAY_LED_Pin DISPLAY_DC_Pin DISPLAY_RESET_Pin DISPLAY_CS_Pin */
  GPIO_InitStruct.Pin = DISPLAY_LED_Pin|DISPLAY_DC_Pin|DISPLAY_RESET_Pin|DISPLAY_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_CS_Pin */
  GPIO_InitStruct.Pin = TOUCH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TOUCH_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_IRQ_Pin */
  GPIO_InitStruct.Pin = TOUCH_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOUCH_IRQ_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
