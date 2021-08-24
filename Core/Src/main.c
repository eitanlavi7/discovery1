/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
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
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef tim4Handle;

typedef enum
{
    USER_LED_CONFIG_DEFAULT = 0,
    USER_LED_CONFIG_FOUR_SECONDS_PERIOD
}USER_LED_CONFIG;

typedef struct T_ButtonStatus_StructType
{
    GPIO_PinState current;
    GPIO_PinState previous;
} T_ButtonStatus_Struct;

/* USER CODE BEGIN PV */
static uint8_t changed_case_data[50];
static uint8_t src8;
T_RxData_Struct received_data = {0}; /* Global received buffer */
 
 const uint8_t crc_max_look_up_table[256] =
 {
     0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
     0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
     0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
     0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
     0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
     0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
     0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
     0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
     0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
     0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
     0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
     0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
     0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
     0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
     0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
     0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
     0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
     0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
     0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
     0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
     0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
     0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
     0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
     0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
     0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
     0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
     0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
     0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
     0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
     0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
     0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
     0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
 };
  
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM_Init(void);
static void processReceivedData(uint8_t *Buf);
static uint8_t * stringChangeCase(uint8_t * string);
static uint8_t getCRC(uint8_t * string);

/* USER CODE BEGIN PFP */

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
 static T_ButtonStatus_Struct user_button_state = {0};
 static uint8_t period_4_seconds = 0;
 
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* Timer Init with Default: 0.25 s period */  
  MX_TIM_Init(); 
  
  /* Start Timer with CC Interrupt */
  HAL_TIM_OC_Start_IT(&tim4Handle, TIM_CHANNEL_1);
  
  /* Set Timer 4 Priority to highest */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  
  /* Enable TIM4 Interrupt */
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    /* Process any received data if new data received */
    if (RECEIVED_DATA_STATUS_RECEIVED == received_data.status)
    {
        /* Process data */
        processReceivedData((received_data.data));
                                
        /* Reset Status */
        received_data.status = RECEIVED_DATA_STATUS_NOT_RECEIVED;        
    }
    
    /* Read User Button */
    user_button_state.current = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    
     /* 5s delay to debounce button state */
    HAL_Delay(5);
    
    /* If Button was pressed and then released, change LED Period */
    if ((GPIO_PIN_SET == user_button_state.previous) && (GPIO_PIN_RESET == user_button_state.current))
    {
        /* Toggle 4 seconds period flag */
        period_4_seconds = !period_4_seconds;
        
        if (period_4_seconds)
        {
            /* Update Prescaler to achieve 4 seconds period */
            __HAL_TIM_SET_PRESCALER(&tim4Handle, 48000-1);
        }
        else
        {
            /* Update Prescaler to Change to Default value of 0.25 seconds period */
            __HAL_TIM_SET_PRESCALER(&tim4Handle, 3000-1);
        }
    }
    
    /* Update button previous state if different from current */
    if (user_button_state.previous != user_button_state.current)
    {
        user_button_state.previous = user_button_state.current;
    }
    
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  GPIO_InitTypeDef GPIO_GreenLEDInitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD3_Pin|LD5_Pin|LD6_Pin
                           |Audio_RST_Pin, GPIO_PIN_RESET);

  /* Init Green LED GPIO PD12 to Timer 4 Channel 1 */
  GPIO_GreenLEDInitStruct.Pin = LD4_Pin;                    /* PD12 */
  GPIO_GreenLEDInitStruct.Pull = GPIO_NOPULL;                /* No Pull */
  GPIO_GreenLEDInitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;      /* Medium Frequency */
  GPIO_GreenLEDInitStruct.Mode = GPIO_MODE_AF_PP;           /* Alternate Function */
  GPIO_GreenLEDInitStruct.Alternate = GPIO_AF2_TIM4;        /* Timer 4 Channel 1 - From UM1472 */
  HAL_GPIO_Init(GPIOD, &GPIO_GreenLEDInitStruct);  
  
  /*Configure GPIO pin : DATA_Ready_Pin */
  GPIO_InitStruct.Pin = DATA_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD5_Pin|LD6_Pin
                            |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief Changes case for the input string
  * @retval Pointer to char (string with changed case).
  */
static uint8_t * stringChangeCase(uint8_t * string)
{
    uint16_t index;
    uint8_t ch;
    static uint8_t output[50];
    
    index = 0;
    while (string[index] != '\0')
    {
        /* Change one character at a time */
        ch = string[index];
        if (ch >= 'A' && ch <= 'Z')
        {
            /* Upper case to lower case */
            output[index] = ch + (uint8_t)32;           
        }
        else if (ch >= 'a' && ch <= 'z')
        {
            /* Lower case to upper case */
            output[index] = ch - (uint8_t)32;
        }
        else
        {
            /* Keep the rest of the characters as is */
            output[index] = ch;
        }
        
        index++;        
    }
    output[index] = '\0';
    return output;
}

/**
  * @brief Gets CRC8-MAX of a string. Mode of computation is by using crc8-max loop up table
  * @retval uint8 crc_value
  */
static uint8_t getCRC(uint8_t * string)
{
    uint16_t index = 0;
    uint8_t crc = 0x00U; 

    if (string != NULL)
    {
        crc &= 0xFFU;
        while (string[index] != '\0' && string[index] != '\n' && string[index] != '\r')
        {
            /* Look for crc vaue in look up table */
            crc = crc_max_look_up_table[(uint8_t)(string[index] ^ crc)];
            index++;
        }
    }
    
    return crc; 
}

/**
  * @brief Processes received data: 1.Case is changed, 2. CRC8 is calculated and appended. 3. New Message is transmitted.
  * @retval None
  */
static void processReceivedData(uint8_t *Buf)
{    
    /* Clean Changed Case string */
    memset((char *)changed_case_data, '\0', sizeof(changed_case_data));
  
    /* Update changed case string */
    strcpy((char *)changed_case_data,(const char *)stringChangeCase(Buf));

    /* Calculate CRC (Max*/
    src8 = getCRC(changed_case_data);

    /* Append Checksum to String to be sent over */
    strncat((char *)changed_case_data, (const char *)&src8, (size_t)1);

    /* Send the new string */
    CDC_Transmit_FS(changed_case_data, (uint16_t)strlen((char *)changed_case_data));
    
    /* Wait for transmission*/
    HAL_Delay(200);
}

/**
  * @brief Timer Configuration
  * @retval None
  */
static void MX_TIM_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    /* Enable TIM4 Clock */
    __HAL_RCC_TIM4_CLK_ENABLE();
    
    /* Timer 4 Basic Timer Initialization 
     * CK_INT = APB1Timer Clock = (SYSCLK/4)*2 = 96 MHz/2 = 48 MHz
     * Prescaler = 3000 - 1. TIM Input Freq = 48MHz/3K = 16000. Input Period = 6.25us
     */
    tim4Handle.Instance = TIM4;
    tim4Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim4Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim4Handle.Init.Prescaler = 3000 - 1;    

    /* Desider Period = 0.25s */
    /* ARR + 1 = (Desired Period)/(TIM Input Period) = 250ms/0.0625ms = 4000 */
    tim4Handle.Init.Period = 4000 - 1;

    if (HAL_TIM_Base_Init(&tim4Handle) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* Clock Source Configuration */
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&tim4Handle, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* OC Initialization */
    if (HAL_TIM_OC_Init(&tim4Handle) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* Master Configuration */
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&tim4Handle, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* OC Channel Configuration */
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;

    /* For 25% Duty Cycle Start pulse at 3/4 of period */
    sConfigOC.Pulse = (uint32_t)(((tim4Handle.Init.Period) * 3) / 4);
    
    if (HAL_TIM_OC_ConfigChannel(&tim4Handle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }    
}

/* USER CODE END 4 */

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
