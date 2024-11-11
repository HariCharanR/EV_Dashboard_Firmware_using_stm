/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef txh;
CAN_RxHeaderTypeDef rxh;
uint8_t txData[8] = {0,0,0,0,0,0,0,0};
uint8_t rxData[8];
uint32_t mailbox1;
char msg[200];
struct batteryData{
	uint8_t soc;
	uint16_t current;
	uint8_t gather_vol;
	uint8_t cum_vol;
	uint8_t cell_vol[2];
	uint8_t cell_temp[2];
	uint8_t charging_status;
	uint16_t rem_ah;
}bat;

struct frame_flags{
	uint8_t soc_frame;
	uint8_t vol_frame;
	uint8_t temp_frame;
	uint8_t ah_frame;
	uint8_t motor_frame;
}flags;
struct motorData {
	uint8_t temp;
	uint8_t freq;
}motor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void filter_config(void);
void led_indicate(uint8_t no);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void sendRequest(uint8_t no);
void uart_tx(uint16_t address , uint16_t data);
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_CAN1_Init();
//  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  filter_config();
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_BUSOFF | CAN_IT_RX_FIFO0_MSG_PENDING);

  if(HAL_CAN_Start(&hcan1) != HAL_OK){
	  Error_Handler();
  }
  	  flags.soc_frame = 0;
  	  flags.ah_frame = 0;
  	  flags.temp_frame = 0;
  	  flags.vol_frame = 0;
  	  flags.motor_frame = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */
	  if(flags.motor_frame == 1){
		  uint16_t speed = ((3*3.14*0.2413) / 25) * (motor.freq * 60);
		  uart_tx(0x1800, motor.temp);
		  uart_tx(0x1900 , speed / 3.86);
//		  sprintf(msg , "MOTOR PARAMETERS:\nFREQ:%d\nTEMP:%d\n",  motor.temp , motor.freq);
//		  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
		  led_indicate(1);
		  flags.motor_frame = 0;

	  }else{
	  sendRequest(5);
	  while(flags.motor_frame != 1);
	  }

	  	  if(flags.soc_frame == 1){
	  		  uint16_t power = 0;
	  		  if(bat.charging_status == 0x2 || bat.charging_status == 0x0){
	  			  power = bat.cum_vol * bat.current;
	  		  }
	  		  uart_tx(0x1000, bat.soc);
	  		  uart_tx(0x1100 , bat.current);
	  		  uart_tx(0x1200 , bat.cum_vol);
	  		  uart_tx(0x1300 , power);
//	  		  sprintf(msg , "Battery Parameters:\nsoc:%d\ncurrent:%d\nvoltage:%d\npower:%d\n\n\n", bat.soc , bat.current , bat.cum_vol , power);
//	  		  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	  		  flags.soc_frame = 0;
	  	  }else{
	  sendRequest(1);
	  	  	while(flags.soc_frame != 1);
	  	  }
	  	  if(flags.vol_frame == 1){
	  		  uint16_t avg_vol = (bat.cell_vol[0] + bat.cell_vol[1]) / 2;
	  		  uart_tx(0x1400 , avg_vol);
//	  		  sprintf(msg , "Avg Cell Voltage : %d" , avg_vol);
//	  		  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	  		  flags.vol_frame = 0;
	  	  }else{

	  	  sendRequest(2);
	  	  while(flags.vol_frame != 1);
	  	  }
	  	  if(flags.temp_frame == 1){
	  		  uint16_t avg_temp = (bat.cell_temp[0] + bat.cell_temp[1]) / 2;
	  		  uart_tx(0x1500 ,avg_temp);
//	  		  sprintf(msg , "Avg Cell Temp : %d" , avg_temp);
//	  		  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	  		  flags.temp_frame = 0;
	  	  }else{
	  	  sendRequest(3);
	  	  while(flags.temp_frame != 1);
	  	  }
	  	  if(flags.ah_frame==1){
	  		  uart_tx(0x1600 , bat.charging_status);
	  		  uart_tx(0x1700 , bat.rem_ah);
//	  		  sprintf(msg , "Charging_status:%d\nAH:%d\n" , bat.charging_status , bat.rem_ah);
//	  		  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	  		  flags.ah_frame = 0;
	  	  }else{

	  	  sendRequest(4);
	  	  while(flags.ah_frame != 1);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led1_Pin|led4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, led3_Pin|led2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : led1_Pin led4_Pin */
  GPIO_InitStruct.Pin = led1_Pin|led4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : led3_Pin led2_Pin */
  GPIO_InitStruct.Pin = led3_Pin|led2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void filter_config(void){
	CAN_FilterTypeDef fil1;
	fil1.FilterActivation = ENABLE;
	fil1.FilterBank = 0;
	fil1.FilterFIFOAssignment = CAN_RX_FIFO0;
	fil1.FilterIdHigh = 0x0000;
	fil1.FilterIdLow = 0x0000;
	fil1.FilterMaskIdHigh = 0x0000;
	fil1.FilterMaskIdLow = 0x0000;
	fil1.FilterMode = CAN_FILTERMODE_IDMASK;
	fil1.FilterScale = CAN_FILTERSCALE_32BIT;

	if(HAL_CAN_ConfigFilter(&hcan1,&fil1) != HAL_OK){
		Error_Handler();
	}
}
void led_indicate(uint8_t no){
	switch(no){
	case 1:
		HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
		HAL_Delay(500);					//on board led
		HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
		break;
	case 2:
		HAL_GPIO_TogglePin(led2_GPIO_Port, led2_Pin);
		HAL_Delay(500);		// A0 led
		HAL_GPIO_TogglePin(led2_GPIO_Port, led2_Pin);
		break;
	case 3:
		HAL_GPIO_TogglePin(led3_GPIO_Port, led3_Pin);
		HAL_Delay(500);			// A1 led
		HAL_GPIO_TogglePin(led3_GPIO_Port, led3_Pin);
		break;
	case 4:
		HAL_GPIO_TogglePin(led4_GPIO_Port, led4_Pin);
		HAL_Delay(500);			// A2 led
		HAL_GPIO_TogglePin(led4_GPIO_Port, led4_Pin);
		break;
	}

}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){

	if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0, &rxh, rxData) != HAL_OK){
		Error_Handler();
	}

	if(rxh.ExtId == 0xDCBA){
		motor.temp = rxData[0];
		motor.freq = rxData[1];
		flags.motor_frame = 1;
	}

	else if(rxh.ExtId == 0x18904001){
		bat.cum_vol = ((rxData[0] << 8) | rxData[1])*0.1;
		bat.gather_vol = ((rxData[2]<<8) | rxData[3])*0.1;
		bat.current = (((rxData[4] << 8 )| rxData[5])-30000)*0.1;
		bat.soc = ((rxData[6]<<8) | rxData[7])*0.1;
		flags.soc_frame =1;
	}
	else if(rxh.ExtId == 0x18914001){
		 bat.cell_vol[0] = ((rxData[0] << 8) | rxData[1])/1000;
		 bat.cell_vol[1] = ((rxData[3]<<8) | rxData[4])/1000;
		 flags.vol_frame =1;
	}
	else if(rxh.ExtId == 0x18924001){
		bat.cell_temp[0] = rxData[0] - 40;
		bat.cell_temp[1] = rxData[2] - 40;
		flags.temp_frame = 1;
	}
	else if(rxh.ExtId == 0x18934001){
		bat.charging_status = rxData[0];
		bat.rem_ah =  ((rxData[4] << 24)|(rxData[5] << 16)|(rxData[6]<< 8)|rxData[7])/1000;
		flags.ah_frame = 1;
	}



}

void sendRequest(uint8_t no){
	txh.DLC =8;
	txh.IDE = CAN_ID_EXT;
	txh.RTR = CAN_RTR_DATA;
	switch(no){
		case 1:
			txh.ExtId = 0x18900140;
			break;
		case 2:
			txh.ExtId = 0x18910140;

			break;
		case 3:
			txh.ExtId = 0x18920140;
			break;
		case 4:
			txh.ExtId = 0x18930140;
			break;
		case 5:
			txh.ExtId = 0xABCD;
			break;
		}

	if(HAL_CAN_AddTxMessage(&hcan1, &txh, txData, &mailbox1) != HAL_OK){
		Error_Handler();
	}else{
		led_indicate(1);
	}

}
void uart_tx(uint16_t address , uint16_t data){

	uint8_t uart_frame[8] = {0x5A ,0xA5, 0x05 , 0x82,0x00 ,0x00  ,0x00,0x00};
	uart_frame[4] = ((address>>8) & 0xff);
	uart_frame[5] = (address & 0xff);
	uart_frame[6] = ((data>>8) & 255);
	uart_frame[7] = (data & 255);
	if(HAL_UART_Transmit(&huart1,uart_frame , 8 , HAL_MAX_DELAY) != HAL_OK){
		Error_Handler();
	}else{
//		led_indicate(1);
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
