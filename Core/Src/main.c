/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "app_fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "fatfs_sd.h"
uint16_t address = 0;
FATFS fs;
FIL fil;
FRESULT fresult;
UINT br, bw;  // File read/write count
/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

#define MAX_FILENAME_LENGTH 12

#define RD_LEN  (8 * 14 * 2 *2)
uint8_t Rec_Data[RD_LEN];  // 512/64 = 8 -> 8*14*2 (half transfer)
volatile uint16_t Rec_Data_Ri = 0;
volatile uint16_t Rec_Data_Ti = 0;

char buffer[512 + 1] = { };  //+1 because of null string termination
uint16_t buffer_i = 0;

volatile uint32_t size = 0;

struct {
	int16_t x;
	int16_t y;
	int16_t z;
} acc;

int16_t temp;
double temp_d;

struct {
	int16_t x;
	int16_t y;
	int16_t z;
} gyro;
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
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */
I2C_HandleTypeDef * const imu_i2c = &hi2c2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

	//FATFS
	fresult = f_mount(&fs, "/", 1);
	if (fresult != FR_OK)
		Error_Handler();

	/*************** Card capacity details ********************/

	/* Check free space */
	f_getfree("", &fre_clust, &pfs);
	total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);

	char filename[MAX_FILENAME_LENGTH];
	uint16_t file_count = 0;
	while (f_stat(filename, NULL) == FR_OK && file_count < 1000) {
		file_count++;
	}
	sprintf(filename, "OUT%03d.TXT", file_count);

	f_open(&fil, filename, FA_OPEN_ALWAYS | FA_WRITE);

	f_puts("start", &fil);
	f_sync(&fil);

	//scan i2c
	while (address == 0) {
		for (uint16_t i = 1; i < 128; i++) {
			if (HAL_I2C_IsDeviceReady(imu_i2c, (uint16_t) (i << 1), 3, 10)
					== HAL_OK)
				address = i << 1;
		}
	}

	uint8_t data;
	HAL_I2C_Mem_Read(imu_i2c, address, 0x75, 1, &data, 1, 10);  //who-I-am check
	if (data != 0x68 && data != 0x72)
		Error_Handler();

	HAL_Delay(50);

	data = 0;
	HAL_I2C_Mem_Write(imu_i2c, address, 0x6b, 1, &data, 1, 10);  //pow conf
	HAL_I2C_Mem_Write(imu_i2c, address, 0x1b, 1, &data, 1, 10);  //gyr conf
	HAL_I2C_Mem_Write(imu_i2c, address, 0x1c, 1, &data, 1, 10);  //acc conf
	data = 16 - 1;  //data rate 8kHz prescaller; could be adjusted 32-1 (250 Hz) usually works without errors
	HAL_I2C_Mem_Write(imu_i2c, address, 0x19, 1, &data, 1, 10);  //data rate

	data = 1 << 4;
	HAL_I2C_Mem_Write(imu_i2c, address, 0x37, 1, &data, 1, 10); //irr clear when read
	data = 1 << 0;
	HAL_I2C_Mem_Write(imu_i2c, address, 0x38, 1, &data, 1, 10);  //irr enable
	data = 0x7;
	HAL_I2C_Mem_Write(imu_i2c, address, 0x68, 1, &data, 1, 10);  //reset
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	size = 20 * 16 * 1024 / 64;  //1MB
	//size = 1024 * free_space /64;
	while (size) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/* poll for data by reading the status register, where bit 0 is DATA_RDY_INT
		 * the register will clear automatically after its read */
		HAL_I2C_Mem_Read(imu_i2c, address, 0x3A, 1, &data, 1, 2);
		if (data & 0x01) {
			HAL_I2C_Mem_Read(imu_i2c, address, 0x3b, 1, &(Rec_Data[Rec_Data_Ri]), 14,2);//TODO DMA
			Rec_Data_Ri += 14;


			if(Rec_Data_Ri == Rec_Data_Ti) {//overrun
				//error_counter += 1;
				Rec_Data_Ri -= 14;
			}

			if(Rec_Data_Ri == (RD_LEN)) Rec_Data_Ri = 0;

			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		}


		if (Rec_Data_Ri != Rec_Data_Ti) {
			--size;
			acc.x = (int16_t)(Rec_Data[Rec_Data_Ti+0]<<8 | Rec_Data[Rec_Data_Ti+1]);
			acc.y = (int16_t)(Rec_Data[Rec_Data_Ti+2]<<8 | Rec_Data[Rec_Data_Ti+3]);
			acc.z = (int16_t)(Rec_Data[Rec_Data_Ti+4]<<8 | Rec_Data[Rec_Data_Ti+5]);

			temp = (int16_t)(Rec_Data[Rec_Data_Ti+6]<<8 | Rec_Data[Rec_Data_Ti+7]);
			temp_d = temp/340 + 36.53;

			gyro.x = (int16_t)(Rec_Data[Rec_Data_Ti+8]<<8 | Rec_Data[Rec_Data_Ti+9]);
			gyro.y = (int16_t)(Rec_Data[Rec_Data_Ti+10]<<8 | Rec_Data[Rec_Data_Ti+11]);
			gyro.z = (int16_t)(Rec_Data[Rec_Data_Ti+12]<<8 | Rec_Data[Rec_Data_Ti+13]);

			Rec_Data_Ti += 14;
			if (Rec_Data_Ti == (RD_LEN)){
				Rec_Data_Ti = 0;
			}

			buffer_i += sprintf(&(buffer[buffer_i]),"%08x,%08x,%08x;%08.4f;%08x,%08x,%08x\r\n",acc.x,acc.y,acc.z,temp_d,gyro.x,gyro.y,gyro.z);
			if(buffer_i & 0x3f) Error_Handler();//not proper length
			if(buffer_i == 512){
				f_puts(buffer, &fil);
				buffer_i = 0;
			}
		}
	}
	if(buffer_i){//SEND THE REST
		buffer[buffer_i]=0;
		f_puts(buffer, &fil);
		buffer_i = 0;
	}
	f_close(&fil);
	while(1)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(1000);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00602173;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Ch4_5_DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_5_DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_5_DMAMUX1_OVR_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB9 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRR0_Pin */
  GPIO_InitStruct.Pin = IRR0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(IRR0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {

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
	while (1) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(100);
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
