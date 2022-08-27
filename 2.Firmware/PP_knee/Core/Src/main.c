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
#include "can.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
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

/* USER CODE BEGIN PV */
static CAN_TxHeaderTypeDef        TxMessage;    //CAN���͵���Ϣ����Ϣͷ
static CAN_RxHeaderTypeDef        RxMessage;    //CAN���յ���Ϣ����Ϣͷ

volatile float pitch,roll,yaw;//ŷ����
uint8_t uart_buf_imu[6];//���ڷ�����������������

uint8_t canOrderBuffer[8] = {0xA1, 0xA2, 0xA3, 0xA4};//���������͵�����

uint8_t collect_start = 0;//�ɼ���ʼ��־λ
uint8_t collect_over = 0;//�ɼ�������־λ

uint8_t send_data = 0;//�������ݱ�־λ

uint8_t flag_send_over = 0;//�����Ͳɼ�������־�ı�־λ
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void CANFilter_Config(void);
void CAN_Send_Order(uint8_t orderD);
void CAN_Send_Data(uint8_t aData[]);
void MPU6050_Get_Data(void);
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
  MX_CAN_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	/* 1. CAN Filter Config */
	CANFilter_Config();
	/* 2. CAN Start */
	if(HAL_CAN_Start(&hcan) != HAL_OK) 
	{
		Error_Handler();
	}
  /* 3. Enable CAN RX Interrupt */
	if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) 
	{
		Error_Handler();
	}
	
	MPU_Init();//��ʼ��MPU6050
	while(mpu_dmp_init())
	{
	}//�ȴ�mpu_dmp��ʼ��
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(collect_start)
		{
			MPU6050_Get_Data();
			collect_start = 0;
		}
		
		if(collect_over)
		{
			CAN_Send_Order(canOrderBuffer[3]);
			collect_over = 0;
		}
		
		if(send_data)
		{
			CAN_Send_Data(uart_buf_imu);
			send_data = 0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief CAN���������ú���������ȫΪ�㣬�����й��ˣ�
  * @retval None
  */
static void CANFilter_Config(void)
{
    CAN_FilterTypeDef  sFilterConfig;
		uint32_t StdId_mast = 0x111;//������ַ
    
    sFilterConfig.FilterBank = 0;                       //CAN��������ţ�������0
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;   //CAN������ģʽ���б�ģʽ
    sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;  //CAN�������߶ȣ�16λ
    sFilterConfig.FilterIdHigh = StdId_mast << 5;			  //16λ�£��洢������ID
    sFilterConfig.FilterIdLow = 0x0000;					        
    sFilterConfig.FilterMaskIdHigh = 0x0000;			      
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = 0;				      //����ͨ����������ƥ��󣬴洢���ĸ�FIFO
    sFilterConfig.FilterActivation = ENABLE;    		    //���������
    sFilterConfig.SlaveStartFilterBank = 0;
    
    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief CAN���������
  * @retval None
  */
void CAN_Send_Order(uint8_t orderD)
{
	uint8_t dataO[1];
	dataO[0] = orderD;

	uint32_t TxMailbox;
	TxMessage.IDE = CAN_ID_STD;     //����ID���ͣ���׼ģʽ
	TxMessage.StdId = 0x122;        //����ID�ţ���ʶ�����ɽ��й���
	TxMessage.RTR = CAN_RTR_DATA;   //���ô�������֡
	TxMessage.DLC = 1;              //�������ݳ���

	if (HAL_CAN_AddTxMessage(&hcan, &TxMessage, dataO, &TxMailbox) != HAL_OK) 
	{
		Error_Handler();
  }
}

/**
  * @brief  CAN �������ݺ���
  * @param  aData: ���͵�����
  * @param  len: ���ݵĳ���
  * @retval None
  */
void CAN_Send_Data(uint8_t aData[])
{
	uint32_t TxMailbox;
	
	TxMessage.IDE = CAN_ID_STD;     //����ID����
	TxMessage.StdId = 0x122;        //����ID�ţ���ʶ�����ɽ��й���
	TxMessage.RTR = CAN_RTR_DATA;   //���ô�������֡
	TxMessage.DLC = 6;              //�������ݳ���

	if (HAL_CAN_AddTxMessage(&hcan, &TxMessage, aData, &TxMailbox) != HAL_OK) 
	{
		Error_Handler();
	}
}

/**
  * @brief CAN�����жϴ�����
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t  data[8];
	
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, data) == HAL_OK)
	{	
		switch(data[0])
		{
			case 0x02:
				CAN_Send_Order(canOrderBuffer[1]);
				break;
			case 0xB2:
				send_data = 1;
				break;
			case 0xC1:
				collect_start = 1;
				break;
			case 0xC2:
				collect_start = 1;
				flag_send_over = 1;
				break;
			case 0xC3://�ò���û�ã�ֻ��Ϊ�˴ӻ�����ͳһ
				collect_start = 1;
				break;
		}
	}
}

/**
  * @brief ��ȡIMU��Ϣ
  * @retval None
  */
void MPU6050_Get_Data(void)
{
	while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){ }
	
	uart_buf_imu[0]=(short)pitch>>8;
	uart_buf_imu[1]=(short)pitch&0XFF;
	uart_buf_imu[2]=(short)roll>>8;
	uart_buf_imu[3]=(short)roll&0XFF;
	uart_buf_imu[4]=(short)yaw>>8;
	uart_buf_imu[5]=(short)yaw&0XFF;
		
	if(flag_send_over)//��������Ͳɼ�������־
	{
		collect_over = 1;
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

