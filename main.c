/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "lcd_i2cModule.h"
#include "sht3x.h"
#include <stdio.h>
#include <stdint.h>

static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_GPIO_Init(void);

/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define ADC_SUCHA 3350
#define ADC_MOKRA 2000
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
uint16_t PomiarADC=5;
float temperature, humidity, wilgotnosc_gleby;
char tekst[10];
volatile uint8_t przerwanie_aktyne = 0;
GPIO_PinState poprzedni_stan_B5;

osSemaphoreId ButtonSemaphoreHandle;
const osSemaphoreAttr_t ButtonSemaphore_attributes = {
    .name = "ButtonSemaphore"
};
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
sht3x_handle_t handle = {
    .i2c_handle = &hi2c2,
    .device_address = SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW
};
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */



void SensorTask(void *argument)
{
    HAL_ADC_Start(&hadc2);

    LCD_i2cDeviceCheck();
    LCD_Init();
    LCD_BackLight(LCD_BL_ON);
    LCD_Clear();

    for (;;)
    {
        if (HAL_ADC_PollForConversion(&hadc2, 1000) == HAL_OK)
        {
            PomiarADC = HAL_ADC_GetValue(&hadc2);
        }

        // Odczyt czujnika SHT3x
        sht3x_read_temperature_and_humidity(&handle, &temperature, &humidity);

        wilgotnosc_gleby = (ADC_SUCHA - PomiarADC) * 100.0 / (ADC_SUCHA - ADC_MOKRA);
        if (wilgotnosc_gleby > 100) wilgotnosc_gleby = 100;
        if (wilgotnosc_gleby < 0) wilgotnosc_gleby = 0;



        LCD_Clear();
        LCD_SetCursor(1, 1);
        LCD_Print("TEMP[DEG]: %.1f", temperature);
        LCD_SetCursor(2, 1);
        LCD_Print("HUM[PERC]: %.1f", humidity);

        HAL_Delay(1500);

        LCD_Clear();
        LCD_SetCursor(1, 1);
        LCD_Print("SOIL HUM : %.1f%%", wilgotnosc_gleby);

        HAL_Delay(1500);


        if (przerwanie_aktyne == 0)
        {
            if (wilgotnosc_gleby > 45)
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
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
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_ADC2_Init();

    osKernelInitialize();

    ButtonSemaphoreHandle = osSemaphoreNew(1, 0, &ButtonSemaphore_attributes);

    osThreadNew(SensorTask, NULL, NULL);


    osKernelStart();

    while (1) {}
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

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                                      |RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

/**
  * @brief ADC2 Initialization Function
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function (LCD)
  */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x2000090E;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

/**
  * @brief I2C2 Initialization Function (SHT3x)
  */
static void MX_I2C2_Init(void)
{
    hi2c2.Instance = I2C2;
    hi2c2.Init.Timing = 0x2000090E;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c2);
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;  // REAKCJA NA OBA ZBOCZA
  GPIO_InitStruct.Pull = GPIO_NOPULL;                  // lub ewentualnie GPIO_PULLDOWN / GPIO_PULLUP, w zależności od układu
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;                   // OK, jeśli potrzebujesz
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        GPIO_PinState przycisk = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);


        if (przycisk == GPIO_PIN_SET) // przycisk wciśnięty
        {
            przerwanie_aktyne = 1;

            // Sprawdź aktualny stan GPIOB5 i odwróć go
            poprzedni_stan_B5 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);

            if (poprzedni_stan_B5 == GPIO_PIN_RESET)
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // wyłącz podlewanie
            }
            else
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // włącz podlewanie
            }
        }
        else // przycisk puszczony
        {
            przerwanie_aktyne = 0;

            // Przywróć poprzedni stan GPIOB5
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, poprzedni_stan_B5);
        }
    }
}


/* USER CODE END 4 */

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
