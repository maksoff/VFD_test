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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vfd.h"
#include "nrf24l01p.h"
#include "string.h"

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

SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void nrf24l01p_spi_ss(nrf24l01p_spi_ss_level_t level)
{
	// we will transmit data to nRF, MSB FIRST
	if (!level)
		hspi2.Instance->CR1 &= ~(SPI_CR1_LSBFIRST);
	HAL_GPIO_WritePin(SPI2_nRF_CSn_GPIO_Port, SPI2_nRF_CSn_Pin, level);
	// we will transmit data to VFD, LSB FIRST
	if (level)
		hspi2.Instance->CR1 |= SPI_CR1_LSBFIRST;
}

uint8_t nrf24l01p_spi_rw(uint8_t value)
{
	uint8_t data;
	HAL_SPI_TransmitReceive(&hspi2, &value, &data, 1, 100);
	return data;
}

void vfd_update(void) {
	uint8_t data = 0b11000000; // command 3, set address to 0
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
	HAL_SPI_Transmit(&hspi2, vfd.arr1, sizeof(vfd.arr1), 0xffffffff);
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1);
}

void do_microrl(void)
{
	return;
}


void do_vfd_init(void)
{
#define FULL_DEMO (0)
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1); // put CS high

	HAL_Delay(10);
	HAL_GPIO_WritePin(HV_EN_GPIO_Port, HV_EN_Pin, 1);

	for (int i = 0; i < sizeof(vfd.arr1); i++) {
		vfd.arr1[i] = 0xFF;
	}
	uint8_t data;

	data = 0b01000001; // command 2, write to LED port
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
	HAL_Delay(10);

	data = 0b1111; // disable LEDs

	HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1);
	HAL_Delay(10);

	//write twice, some strange problem with SPI init

	data = 0b01000001; // command 2, write to LED port
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
	HAL_Delay(10);

	data = 0b1111; // disable LEDs

	HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1);
	HAL_Delay(10);

	data = 0b01000000; // command 2, write to Display port
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1);
	HAL_Delay(10);
	vfd_update();
	HAL_Delay(10);
	// init display, 11 digits 17 segments
	data = 0b00000111; // command 1, 11 digits 17 segments
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1);
	HAL_Delay(10);
	data = 0b10000000; // command 4
	data |= 1 << 3; // enable/disable display
	data |= 0b111; // set brightness
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1);

	if(FULL_DEMO)
	{

		for (uint8_t i = 0; i <= 0b111; i++) {
			data = 0b10000000; // command 4
			data |= 1 << 3; // enable/disable display
			data |= i; // set brightness
			HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 0);
			HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
			HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1);
			HAL_Delay(250);
			do_microrl();
		}

		for (int i = 0; i < 11; i++) {
			for (int b = 0; b < 3; b++) // erasing from right to left
			{
				vfd.arr2[i][b] = 0;
			}
			vfd_update();
			HAL_Delay(150);
			do_microrl();
		}
		HAL_Delay(500);
		do_microrl();
	}


	//erase everything... just in case
	clr_vfd();

	data = 0b10000000; // command 4
	data |= 1 << 3; // enable/disable display
	data |= 0b111; // set max brightness
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1);

	// fill everything
	for (int j = 1; j < 15; j++) {
		uint32_t temp = 1 << j;
		for (int i = 1; i < 11; i++) {
			for (int b = 0; b < 3; b++) {
				vfd.arr2[i][b] |= (temp >> (b << 3)) & 0xFF;
			}
		}
		vfd_update();
		HAL_Delay(100);
		do_microrl();
	}

	const uint8_t arr[][2] = {
			{ 6, 0 },
			{ 0, 0 },
			{ 0, 1 },
			{ 0, 4 },
			{ 0, 3 },
			{ 0, 5 },
			{ 0, 2 },
			{ 0, 6 },
			{ 1, 16 },
			{ 1, 15 },
			{ 2, 16 },
			{ 2, 15 },
			{ 3, 16 },
			{ 3, 15 },
			{ 4, 16 },
			{ 4, 15 },
			{ 5, 16 },
			{ 5, 15 },
			{ 6, 16 },
			{ 6, 15 },
			{ 8, 16 },
			{ 8, 15 },
			{ 9, 16 },
			{ 10, 16 },
			{ 10, 15 },
	};

	for (int j = 0; j < sizeof(arr) / 2; j++) {
		for (int b = 0; b < 3; b++)
			vfd.arr2[arr[j][0]][b] |= ((1 << arr[j][1]) >> (b << 3)) & 0xFF;
		vfd_update();
		HAL_Delay(50);
		do_microrl();
	}

	HAL_Delay(300);

	str2vfd("TEST BOARD");

	vfd_update();
}

void vfd_leds(uint8_t leds)
{
	uint8_t data = 0b01000001; // command 2, write to LED port
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);

	data = (~leds)&0b1111;

	HAL_SPI_Transmit(&hspi2, &data, 1, 0xffffffff);
	HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, 1);
}

void do_leds(void)
{
	static uint32_t last_time = 0;
	static uint8_t tick_counter = 0;
	if (HAL_GetTick() - last_time < 500)
		return;
	last_time = HAL_GetTick();
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	bool PB1 = HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin);
	bool PB2 = HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin);
	if (PB1 || PB2)
	{
		vfd_leds((1<<((tick_counter >> 1)&0b11)));
		if (PB1)
		{
			tick_counter++;
			clr_vfd();
			str2vfd("PB1 OKAY");
			vfd_update();
		}
		else
		{
			tick_counter--;
			clr_vfd();
			str2vfd("PB2 OKAY");
			vfd_update();
		}
	}
	else
	{
		//vfd_leds(0);
	}
}

void do_fram_test(void)
{
	bool read(void)
	{
		uint8_t temp;
		HAL_I2C_Mem_Read(&hi2c1, 0b10100000, 0x42, 1, &temp, 1, 10);
		if (temp != 0x42)
			return false;
		HAL_I2C_Mem_Read(&hi2c1, 0b10100010, 0x5A, 1, &temp, 1, 10);
		if (temp != 0xA5)
			return false;
		return true;
	}
	void write(void)
	{
		uint8_t temp = 0x42;
		HAL_I2C_Mem_Write(&hi2c1, 0b10100000, 0x42, 1, &temp, 1, 10);
		temp = 0xA5;
		HAL_I2C_Mem_Write(&hi2c1, 0b10100010, 0x5A, 1, &temp, 1, 10);
	}
	static uint32_t last_time = 0;
	if (HAL_GetTick() - last_time < 200)
		return;
	bool PB1 = HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin);
	bool PB2 = HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin);
	if (PB1 && PB2)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0); // turn led on
		vfd_leds(0b1111);
		HAL_Delay(500);
		if (read())
		{
			vfd_leds(0b0100);
			clr_vfd();
			str2vfd("FRAM FOUND");
			vfd_update();
		}
		else
		{
			vfd_leds(0b0001);
			write();
			HAL_Delay(500);
			if (read())
			{
				vfd_leds(0b0011);
				clr_vfd();
				str2vfd("FRAM OKAY");
				vfd_update();
			}
			else
			{
				vfd_leds(0b1000);
				clr_vfd();
				str2vfd("NO FRAM!");
				vfd_update();
			}
		}

		while(HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin));
		if (HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin))
		{
			// PB2 still pressed, erase RAM
			vfd_leds(0b1001);
			uint8_t zero[256] = {0};
			//first half
			HAL_I2C_Mem_Write(&hi2c1, 0b10100000, 0, 1, zero, sizeof(zero), 200);
			//second half
			HAL_I2C_Mem_Write(&hi2c1, 0b10100010, 0, 1, zero, sizeof(zero), 200);
			HAL_Delay(500);
			vfd_leds(0b1010);
			while (HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin));
		}
	}
	last_time = HAL_GetTick();
}


enum
{
    NRF_CHANNEL = 123,
    NRF_POWER_UP_DELAY = 2,
    NRF_PAYLOAD_LENGTH = 10,
    NRF_RETRANSMITS = 0,

    #if RF_PAYLOAD_LENGTH <= 18
        NRF_RETRANSMIT_DELAY = 250
    #else
        NRF_RETRANSMIT_DELAY = 500
    #endif
};

uint8_t address[5] = { 0x31, 0x41, 0x59, 0x26, 0x56 };

void delay(uint32_t delay)
{
	HAL_Delay(delay);
}

void nrf_init_tx(uint8_t *address)
{
    nrf24l01p_get_clear_irq_flags();
    nrf24l01p_close_pipe(NRF24L01P_ALL);
    nrf24l01p_open_pipe(NRF24L01P_TX, false);
    nrf24l01p_set_crc_mode(NRF24L01P_CRC_16BIT);
    nrf24l01p_set_address_width(NRF24L01P_AW_5BYTES);
    nrf24l01p_set_address(NRF24L01P_TX, address);
    nrf24l01p_set_operation_mode(NRF24L01P_PTX);
    nrf24l01p_set_rf_channel(NRF_CHANNEL);

    nrf24l01p_set_power_mode(NRF24L01P_PWR_UP);
    delay(NRF_POWER_UP_DELAY);
}

void nrf_init_rx(uint8_t *address)
{
    nrf24l01p_get_clear_irq_flags();
    nrf24l01p_close_pipe(NRF24L01P_ALL);
    nrf24l01p_open_pipe(NRF24L01P_PIPE0, false);
    nrf24l01p_set_crc_mode(NRF24L01P_CRC_16BIT);
    nrf24l01p_set_address_width(NRF24L01P_AW_5BYTES);
    nrf24l01p_set_address(NRF24L01P_PIPE0, address);
    nrf24l01p_set_operation_mode(NRF24L01P_PRX);
    nrf24l01p_set_rx_payload_width(NRF24L01P_PIPE0, NRF_PAYLOAD_LENGTH);
    nrf24l01p_set_rf_channel(NRF_CHANNEL);

    nrf24l01p_set_power_mode(NRF24L01P_PWR_UP);
    delay(NRF_POWER_UP_DELAY);
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
  MX_SPI2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  __HAL_SPI_ENABLE(&hspi2);

  uint8_t test;
  nrf24l01p_spi_ss(NRF24L01P_SPI_SS_HIGH);

	// hold PB1 to enable tx
	bool rx = !HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin);
	if (!rx)
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);

  do_vfd_init();
  test = nrf24l01p_nop();
  if (test == 0b1110)
  {
	  str2vfd("NRF24L01+");
	  vfd_update();
  }

  if (rx)
  {
	  nrf_init_rx(address);
	  // enable read
	  HAL_GPIO_WritePin(nRF_CE_GPIO_Port, nRF_CE_Pin, 1);
  }
  else
	  nrf_init_tx(address);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  do_leds();
	  do_fram_test();

	  if (HAL_GetTick() > 10*60*1000)
		  HAL_GPIO_WritePin(HV_EN_GPIO_Port, HV_EN_Pin, 0);

	  static uint32_t last_time = 0;
	  if (HAL_GetTick() - last_time > (rx?110:100))
	  {
		  if (rx)
		  {
			  if (nrf24l01p_get_irq_flags() & (1 << NRF24L01P_IRQ_RX_DR))
			  {
				  nrf24l01p_clear_irq_flag(NRF24L01P_IRQ_RX_DR);

				  uint8_t payload[NRF_PAYLOAD_LENGTH];

				  while (!nrf24l01p_rx_fifo_empty())
					  nrf24l01p_read_rx_payload(payload);

				 if (payload[0] == 1)
				 {
					 vfd_leds(0b0100);
					 str2vfd("RX PB1");
					 vfd_update();
				 }
				 else
				 {
					 vfd_leds(0b0010);
					 str2vfd("RX PB2");
					 vfd_update();
				 }

			  }
			  else
			  {
				  //vfd_leds(0b0000);
			  }
		  }
		  else
		  {
			bool PB1 = HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin);
			bool PB2 = HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin);
			if (PB1 || PB2)
			{
				static uint8_t payload[NRF_PAYLOAD_LENGTH];
				memset(payload, 0x44, sizeof(payload));
				payload[0] = PB1;
				nrf24l01p_write_tx_payload(payload, sizeof(payload));

				HAL_GPIO_WritePin(nRF_CE_GPIO_Port, nRF_CE_Pin, 1);
				HAL_Delay(1);
				HAL_GPIO_WritePin(nRF_CE_GPIO_Port, nRF_CE_Pin, 0);

				do {} while (!(nrf24l01p_get_irq_flags() & (1 << NRF24L01P_IRQ_TX_DS)));
				nrf24l01p_clear_irq_flag(NRF24L01P_IRQ_TX_DS);
			}

		  }
		  last_time = HAL_GetTick();
	  }
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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, nRF_CE_Pin|SPI2_nRF_CSn_Pin|HV_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PU_GPIO_Port, USB_PU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PT6315_STB_GPIO_Port, PT6315_STB_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : nRF_CE_Pin SPI2_nRF_CSn_Pin HV_EN_Pin PT6315_STB_Pin */
  GPIO_InitStruct.Pin = nRF_CE_Pin|SPI2_nRF_CSn_Pin|HV_EN_Pin|PT6315_STB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1_Pin PB2_Pin */
  GPIO_InitStruct.Pin = PB1_Pin|PB2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PU_Pin */
  GPIO_InitStruct.Pin = USB_PU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PU_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
