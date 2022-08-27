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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
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
volatile float pitch,roll,yaw;//ŷ����
uint8_t uart_buf_imu[6*4];//���ڷ�����������������

uint16_t Battery;//��ص�ѹֵ
uint8_t uart_buf_bat[2];//���ڷ��͵�ص�ѹ����

uint16_t ADC_ConvertedValue[4];//DMA�洢���ݵ�����
uint16_t Voltage;//���д�����������
uint16_t VoltageS[12][4];//���д�����������
uint8_t Row_list[4] = {0, 1, 2, 3};//��˳��
//******************{6,7,5,8,4,9,3,10,2,11,1,12}
//******************{6,7,5,8,4,9,1,11,2,10,3,12}
//******************{5,6,4,7,3,8,0,10,1,9, 2,11}
uint8_t Row_A[12] = {1,0,0,1,1,0,0,0, 1,1, 0, 1};
uint8_t Row_B[12] = {0,1,0,1,1,0,0,1, 0,0, 1, 1};
uint8_t Row_C[12] = {1,1,1,1,0,0,0,0, 0,0, 0, 0};
uint8_t Row_D[12] = {0,0,0,0,0,1,0,1, 0,1, 0, 1};//1~12
uint8_t uart_buf_v[96];//���ڷ������е�ѹ����

static CAN_TxHeaderTypeDef        TxMessage;    //CAN���͵���Ϣ����Ϣͷ
static CAN_RxHeaderTypeDef        RxMessage;    //CAN���յ���Ϣ����Ϣͷ

extern uint8_t RxBuffer[8];//������λ���������ݵĻ�����

uint8_t OrderBuffer[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};//�豸����λ��֮�������
uint8_t canOrderBuffer[9] = {0x01, 0x02, 0x03, 0xB1, 0xB2, 0xB3, 0xC1, 0xC2, 0xC3};//������ӻ�֮�������

uint8_t linkOK = 0;//��������ͬ���־λ
uint8_t Modes = 0;//��������ģʽ��0�����/ֹͣ���ͣ�2�����+��أ�3���Ƕ�+��أ�4�����+�Ƕ�+���

uint8_t uart_send_buf[130];//���ݷ��ͻ�����

uint8_t up_bat_ones = 1;//��ȡ������ݼ��ʱ������λ
uint8_t times_bat;//��ȡ������ݼ��ʱ������λ
uint8_t times_up_bat;//��ȡ������ݼ��ʱ������λ
uint8_t times_req;//ѭ����������ʱ��������λ

uint8_t toe_slave = 0;//���ӻ��ڲ��ڵı�־λ
uint8_t knee_slave = 0;//���ӻ��ڲ��ڵı�־λ
uint8_t hip_slave = 0;//���ӻ��ڲ��ڵı�־λ

uint8_t slavers_mode = 0;//���ӻ��ڲ��ڵı�־λ

uint8_t order_or_data = 0;//�������յ������������

uint8_t imu_collect_over = 0;//�ӻ����ݲɼ�������־

uint8_t receive_data_over = 0;//�ӻ����ݲɼ�������־

uint8_t data_test[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void usart1_niming_report(uint8_t fun, uint8_t *data, uint8_t len);
void mpu6050_send_data(short roll, short pitch, short yaw);
void Get_Battery(void);
void Get_DMA_ADC1(void);
void delay_us(uint32_t us);
void MPU6050_Get_Data(void);
static void CANFilter_Config(void);
void CAN_Send(uint8_t orderD);
void Upload_Battery(uint8_t *data, uint8_t len);
void Upload_Order(uint8_t data);
void Upload_Data(uint8_t *data, uint8_t len);
void ask_Slavers();
void ask_IPC_Link();
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
	HAL_Delay(500);//����ϵ���ʱ������ESP�ϵ�ʱ�Դ����жϵ�Ӱ��
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
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
	
	ask_Slavers();//��ѯ�ӻ����������
	
	ask_IPC_Link();//����λ������������������
	
	Get_Battery();//��ȡ��ص�ѹ
	
	uint8_t i;//����forѭ������
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		while(!linkOK){times_bat = 0;}//�ȴ�����ͬ��
			
		if(up_bat_ones)//ͬ�����Ӻ���һ�ε�ص�ѹ
		{
			Upload_Battery(uart_buf_bat, 2);
			up_bat_ones = 0;
		}

		switch(Modes)//ѡ����ģʽ
		{
			case 2:
				Get_DMA_ADC1();
				Upload_Data(uart_buf_v, 96);
				break;
			case 3:
				if(toe_slave)//����ӻ�1��
				{
					CAN_Send(canOrderBuffer[6]);//�ôӻ�1���Ͳɼ�������־
				}
				else if(knee_slave)//����ӻ�1���ڣ��ӻ�2��
				{
					CAN_Send(canOrderBuffer[7]);//�ôӻ�2���Ͳɼ�������־
				}
				else if(hip_slave)//����ӻ�1��2���ڣ��ӻ�3��
				{
					CAN_Send(canOrderBuffer[8]);//�ôӻ�3���Ͳɼ�������־
				}
				else//���û�дӻ�����
				{
					receive_data_over = 1;
				}
				MPU6050_Get_Data();
//				while(!receive_data_over){}//�ȴ��ӻ����ݴ������
				Upload_Data(uart_buf_imu, 6*4);
				receive_data_over = 0;
				break;
			case 4:
				Get_DMA_ADC1();
				if(toe_slave)//����ӻ�1��
				{
					CAN_Send(canOrderBuffer[6]);//�ôӻ�1���Ͳɼ�������־
				}
				else if(knee_slave)//����ӻ�1���ڣ��ӻ�2��
				{
					CAN_Send(canOrderBuffer[7]);//�ôӻ�2���Ͳɼ�������־
				}
				else if(hip_slave)//����ӻ�1��2���ڣ��ӻ�3��
				{
					CAN_Send(canOrderBuffer[8]);//�ôӻ�3���Ͳɼ�������־
				}
				else//���û�дӻ�����
				{
					receive_data_over = 1;
				}
				MPU6050_Get_Data();
				while(!receive_data_over){}//�ȴ��ӻ����ݴ������
				for(i=0; i<96; i++)
				{
					uart_send_buf[i] = uart_buf_v[i];
				}
				for(i=0; i<6*4; i++)
				{
					uart_send_buf[96+i] = uart_buf_imu[i];
				}
				Upload_Data(uart_send_buf, 120);
				break;
		}
		
		if(times_up_bat == 6)//ÿ��һ�����ϴ�һ�ε�ص�ѹ
		{
			Battery = (uint16_t)(Battery/6.0);//��һ�����ڲɼ������ε�ѹȡƽ��ֵ
			uart_buf_bat[0] = Battery>>8;//���ݸ�λ
			uart_buf_bat[1] = Battery&0xff;//���ݵ�λ
			Upload_Battery(uart_buf_bat, 2);
			times_up_bat = 0;
		}
		
//		Upload_Order(OrderBuffer[0]);
//		HAL_Delay(100);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief �������ݸ�����������λ�����(V2.6�汾)
  * @retval None
	* @fun ������. 0XA0~0XAF
	* @data ���ݻ�����,���28�ֽ�!!
	* @len data����Ч���ݸ���
	*/
void usart1_niming_report(uint8_t fun, uint8_t *data, uint8_t len)
{
	uint8_t send_buf[32];
	uint8_t imu_data[1];
	uint8_t i;
	if(len>28)return;	//���28�ֽ����� 
	send_buf[len+3]=0;	//У��������
	send_buf[0]=0X88;	//֡ͷ
	send_buf[1]=fun;	//������
	send_buf[2]=len;	//���ݳ���
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//��������
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//����У���	
	for(i=0;i<len+4;i++)
	{
		imu_data[0] = send_buf[i];
		HAL_UART_Transmit(&huart1, imu_data, 1, 2);
	}
}

/**
	*@brief ͨ������1�ϱ���������̬���ݸ�����
	*@roll �����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
	*@pitch ������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
	*@yaw �����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
	*/
void mpu6050_send_data(short roll, short pitch, short yaw)
{
	uint8_t tbuf[6]; 
	
	tbuf[0]=(roll>>8)&0XFF;
	tbuf[1]=roll&0XFF;
	tbuf[2]=(pitch>>8)&0XFF;
	tbuf[3]=pitch&0XFF;
	tbuf[4]=(yaw>>8)&0XFF;
	tbuf[5]=yaw&0XFF; 

	usart1_niming_report(0XA1,tbuf,6);//�Զ���֡,0XA1
}	

/**
	*@brief ͨ��ADC2��ȡ��ص�ѹֵ
  * @retval None
	*/
void Get_Battery(void)
{
	uint16_t Battery_buf = 0;//��ص�ѹֵ
	HAL_ADC_Start(&hadc2);//����ADCת��
	HAL_ADC_PollForConversion(&hadc2, 1);//�ȴ�ת����ɣ��ڶ���������ʾ��ʱʱ�䣬��λms.
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2), HAL_ADC_STATE_REG_EOC))
	{
		Battery_buf = HAL_ADC_GetValue(&hadc2);//��ȡADCת�����ݣ�����Ϊ12λ
		Battery_buf = (float)Battery_buf*(3.310/4096)*1000;
		Battery_buf = Battery_buf*4.0+200;
		
		uart_buf_bat[0] = Battery_buf>>8;//���ݸ�λ
		uart_buf_bat[1] = Battery_buf&0xff;//���ݵ�λ
	}
}

/**
	*@brief ͨ��ADC1��ȡ��ȡDMA_ADֵ
  * @retval None
	*/
void Get_DMA_ADC1(void)
{
	uint8_t r,i,m;
	
	for(r=0; r<12; r++)//��
	{
		HAL_GPIO_WritePin(GPIOA, R_A_Pin, Row_A[r]);
		HAL_GPIO_WritePin(GPIOA, R_B_Pin, Row_B[r]);
		HAL_GPIO_WritePin(GPIOA, R_C_Pin, Row_C[r]);
		HAL_GPIO_WritePin(GPIOA, R_D_Pin, Row_D[r]);
		delay_us(50);
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_ConvertedValue, 4);
		delay_us(50);
		for(i=0;i<4;i++)
		{
			m = Row_list[i];//��˳��
			Voltage = (uint16_t)((float)ADC_ConvertedValue[i]/4096*3.310*1000)%10000;
			VoltageS[r][m] = Voltage;
			
			uart_buf_v[r*8+2*m] = Voltage>>8;//���ݸ�λ
			uart_buf_v[r*8+2*m+1] = Voltage&0xff;//���ݵ�λ
		}
	}
}

/**
	*@brief ��ʱ��������λus��
  * @retval None
	*/
void delay_us(uint32_t us)
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
    while (delay--)
	{
		;
	}
}

/**
	*@brief ���������ݶ�ȡ
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
}

/**
  * @brief CAN���������ú���������ȫΪ�㣬�����й��ˣ�
  * @retval None
  */
static void CANFilter_Config(void)
{
    CAN_FilterTypeDef  sFilterConfig;
		uint32_t StdId1 = 0x121;//�ӻ�1��ַ
	  uint32_t StdId2 = 0x122;//�ӻ�2��ַ
	  uint32_t StdId3 = 0x123;//�ӻ�3��ַ
    
    sFilterConfig.FilterBank = 0;                       //CAN��������ţ�������0
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;   //CAN������ģʽ���б�ģʽ
    sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;  //CAN�������߶ȣ�16λ
    sFilterConfig.FilterIdHigh = StdId1 << 5;			      //16λ�£��洢��һ���ӻ���ID
    sFilterConfig.FilterIdLow = StdId2 << 5;					  //16λ�£��洢�ڶ����ӻ���ID
    sFilterConfig.FilterMaskIdHigh = StdId3 << 5;			  //16λ�£��洢�������ӻ���ID
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = 0;				      //����ͨ����������ƥ��󣬴洢���ĸ�FIFO
    sFilterConfig.FilterActivation = ENABLE;    		    //���������
    sFilterConfig.SlaveStartFilterBank = 0;
    
    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief CAN�������ݺ���
  * @retval None
  */
void CAN_Send(uint8_t orderD)
{
	uint8_t dataO[1];
	dataO[0] = orderD;

	uint32_t TxMailbox;
	TxMessage.IDE = CAN_ID_STD;     //����ID���ͣ���׼ģʽ
	TxMessage.StdId = 0x111;        //����ID�ţ���ʶ�����ɽ��й���
	TxMessage.RTR = CAN_RTR_DATA;   //���ô�������֡
	TxMessage.DLC = 1;              //�������ݳ���

	if (HAL_CAN_AddTxMessage(&hcan, &TxMessage, dataO, &TxMailbox) != HAL_OK) 
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
	uint8_t i;
	uint8_t  data[8];
	uint8_t  data_uart[1];
	
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, data) == HAL_OK)
	{
		if(!order_or_data)
		{
			switch(data[0])
			{
				case 0xA1:
					toe_slave = 1;
					break;
				case 0xA2:
					knee_slave = 1;
					break;
				case 0xA3:
					hip_slave = 1;
					break;
				case 0xA4:
					imu_collect_over = 1;
					if(toe_slave)//����ӻ�1��
					{
						CAN_Send(canOrderBuffer[3]);//�ôӻ�1��������
					}
					else if(knee_slave)//����ӻ�1���ڣ��ӻ�2��
					{
						CAN_Send(canOrderBuffer[4]);//�ôӻ�2��������
					}
					else//����ӻ�1��2���ڣ��ӻ�3��
					{
						CAN_Send(canOrderBuffer[5]);//�ôӻ�3��������
					}
					order_or_data = 1;//��������
					break;
			}
		}
		else
		{
			switch(RxMessage.StdId)
			{
				case 0x121://�ӻ�1�����
					for(i=0; i<6; i++)
					{
						uart_buf_imu[6+i] = data[i];
					}
					if(knee_slave)//�ӻ�2��
					{
						CAN_Send(canOrderBuffer[4]);//�ôӻ�2��������
					}
					else if(hip_slave)//�ӻ�2���ڣ��ӻ�3����
					{
						CAN_Send(canOrderBuffer[5]);//�ôӻ�3��������
					}
					else//�ӻ�2��3���ڣ������ݴ������
					{
						receive_data_over = 1;//�ӻ����ݴ��������־λ��1
						order_or_data = 0;//���ݴ������
					}
					break;
				case 0x122://�ӻ�2��ϥ�ؽ�
					for(i=0; i<6; i++)
					{
						uart_buf_imu[6*2+i] = data[i];
					}
					if(hip_slave)//����ӻ�3����
					{
						CAN_Send(canOrderBuffer[5]);//�ôӻ�3��������
					}
					else//�ӻ�3���ڣ������ݴ������
					{
						receive_data_over = 1;//�ӻ����ݴ��������־λ��1
						order_or_data = 0;//���ݴ������
					}
					break;
				case 0x123://�ӻ�3���Źؽ�
					for(i=0; i<6; i++)
					{
						uart_buf_imu[6*3+i] = data[i];
					}
					receive_data_over = 1;//�ӻ����ݴ��������־λ��1
					order_or_data = 0;//���ݴ������
					break;
			}
		}
	}
}

/**
  * @brief ���ڽ����жϴ�����
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	if(huart -> Instance == USART1) 
	{
		if(RxBuffer[0] == 0XF1)//���յ�ͬ����������
		{
			linkOK = 1;
		}
		if(RxBuffer[0] == 0XF2)//���յ��������+�����������
		{
			Modes = 2;
		}
		if(RxBuffer[0] == 0XF3)//���յ����ͽǶ�+�����������
		{
			Modes = 3;
		}
		if(RxBuffer[0] == 0XF4)//���յ��������+�Ƕ�+�����������
		{
			Modes = 4;
		}
		if(RxBuffer[0] == 0XF5)//���յ�ֹͣ��������
		{
			Modes = 0;
		}
	}
	HAL_UART_Receive_IT(&huart1, RxBuffer, 1);
}

/**
  * @brief �ϴ���ص�ѹ
  * @retval None
	* @data ���ݻ�����
	* @len data����Ч���ݸ���
	*/
void Upload_Battery(uint8_t *data, uint8_t len)
{
	uint8_t send_buf[8] = {0};
	uint8_t send_data[2] = {0};
	uint8_t i;

	send_buf[6]=0; //У��������

	send_buf[0]=0X88;	 //֡ͷ*************0
	send_buf[1]=0XFF;	 //֡ͷ*************1
	send_buf[2]=0XA1;	 //������***********2
	send_buf[3]=2;	   //���ݳ���*********3

	for(i=0;i<len;i++)send_buf[4+i]=data[i];			    //��������
	for(i=0;i<len+4;i++)send_buf[len+4]+=send_buf[i];	//����У���	
	
	for(i=0;i<(len+6)/2;i++)
	{
		send_data[0] = send_buf[2*i];
		send_data[1] = send_buf[2*i+1];
		HAL_UART_Transmit(&huart1, send_data, 2, 1);
	}
}

/**
  * @brief �����ϴ���λ��
  * @retval None
	* @data ���ݻ�����
	* @len data����Ч���ݸ���
	*/
void Upload_Order(uint8_t data)
{
	uint8_t send_buf[10] = {0};
	uint8_t i;

	send_buf[5]=0; //У��������
	
	send_buf[0]=0X88;	 //֡ͷ*************0
	send_buf[1]=0XFF;	 //֡ͷ*************1
	send_buf[2]=0XA0;	 //������***********2
	send_buf[3]=1;	   //���ݳ���*********3

	send_buf[4]=data;			    //��������
	for(i=0;i<5;i++)send_buf[5]+=send_buf[i];	//����У���

	HAL_UART_Transmit(&huart1, send_buf, 6, 1);
}

/**
  * @brief �����ϴ���λ��
  * @retval None
	* @data ���ݻ�����
	* @len data����Ч���ݸ���
	*/
void Upload_Data(uint8_t *data, uint8_t len)
{
	uint8_t send_buf[130] = {0};
	uint8_t send_data[2] = {0};
	uint8_t i;

	send_buf[0]=0X88;	 //֡ͷ*************0
	send_buf[1]=0XFF;	 //֡ͷ*************1
	send_buf[2]=0XAF;	 //������***********2
	send_buf[3]=len;	 //���ݳ���*********3

	for(i=0;i<len;i++)send_buf[4+i]=data[i];			    //��������
	for(i=0;i<len+4;i++)send_buf[len+4]+=send_buf[i];	//����У���	
	
	for(i=0;i<(len+6)/2;i++)
	{
		send_data[0] = send_buf[2*i];
		send_data[1] = send_buf[2*i+1];
		HAL_UART_Transmit(&huart1, send_data, 2, 1);
	}
}

/**
  * @brief ��ʱ���жϴ�����
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1)
	{
		times_bat++;
		
		if(times_bat == 10)//ÿ10s��ȡһ�ε�ص�ѹ
		{
			uint16_t Battery_buf_tim = 0;//��ص�ѹֵ
			times_up_bat++;
			
			HAL_ADC_Start(&hadc2);//����ADCװ��
			HAL_ADC_PollForConversion(&hadc2, 1);//�ȴ�ת����ɣ��ڶ���������ʾ��ʱʱ�䣬��λms.
			if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2), HAL_ADC_STATE_REG_EOC))
			{
				Battery_buf_tim = HAL_ADC_GetValue(&hadc2);//��ȡADCת�����ݣ�����Ϊ12λ
				Battery_buf_tim = (float)Battery_buf_tim*(3.310/4096)*1000;
				Battery_buf_tim = Battery_buf_tim*4.0+200;
				
				Battery = Battery+Battery_buf_tim;
			}
			times_bat = 0;
		}
	}
	
	if(htim == &htim2)
	{
		times_req++;
		
		if(linkOK)
		{
			HAL_TIM_Base_Stop_IT(&htim2);
		}
		
		if((times_req==2)&(!linkOK))//ÿ2s����һ������
		{
			ask_IPC_Link();//����������������
			times_req = 0;
		}
	}
}


/**
  * @brief ��ѯ�ӻ�������������м����ӻ�����,�ӻ�������Ϸ�ʽ
  * @retval None
  */
void ask_Slavers()
{
	CAN_Send(canOrderBuffer[0]);//ѯ�ʴӻ�1
	HAL_Delay(10);//�ȴ��ӻ�1�Ļش�
	CAN_Send(canOrderBuffer[1]);//ѯ�ʴӻ�2
	HAL_Delay(10);//�ȴ��ӻ�2�Ļش�
	CAN_Send(canOrderBuffer[2]);//ѯ�ʴӻ�3
	HAL_Delay(10);//�ȴ��ӻ�3�Ļش�
	
	slavers_mode = toe_slave + knee_slave*10 + hip_slave*100;//����ӻ��������
}

/**
  * @brief ����λ������������������
  * @retval None
  */
void ask_IPC_Link()
{
	switch(slavers_mode)
	{
		case 0:
			Upload_Order(OrderBuffer[0]);
			break;
		case 1:
			Upload_Order(OrderBuffer[1]);
			break;
		case 10:
			Upload_Order(OrderBuffer[5]);
			break;
		case 11:
			Upload_Order(OrderBuffer[2]);
			break;
		case 100:
			Upload_Order(OrderBuffer[7]);
			break;
		case 101:
			Upload_Order(OrderBuffer[3]);
			break;
		case 110:
			Upload_Order(OrderBuffer[6]);
			break;
		case 111:
			Upload_Order(OrderBuffer[4]);
			break;
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

