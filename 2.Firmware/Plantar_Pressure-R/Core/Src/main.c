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
volatile float pitch,roll,yaw;//欧拉角
uint8_t uart_buf_imu[6*4];//串口发送陀螺仪数据容器

uint16_t Battery;//电池电压值
uint8_t uart_buf_bat[2];//串口发送电池电压容器

uint16_t ADC_ConvertedValue[4];//DMA存储数据的容器
uint16_t Voltage;//阵列传感器的数据
uint16_t VoltageS[12][4];//阵列传感器的数据
uint8_t Row_list[4] = {0, 1, 2, 3};//列顺序
//******************{6,7,5,8,4,9,3,10,2,11,1,12}
//******************{6,7,5,8,4,9,1,11,2,10,3,12}
//******************{5,6,4,7,3,8,0,10,1,9, 2,11}
uint8_t Row_A[12] = {1,0,0,1,1,0,0,0, 1,1, 0, 1};
uint8_t Row_B[12] = {0,1,0,1,1,0,0,1, 0,0, 1, 1};
uint8_t Row_C[12] = {1,1,1,1,0,0,0,0, 0,0, 0, 0};
uint8_t Row_D[12] = {0,0,0,0,0,1,0,1, 0,1, 0, 1};//1~12
uint8_t uart_buf_v[96];//串口发送阵列电压容器

static CAN_TxHeaderTypeDef        TxMessage;    //CAN发送的消息的消息头
static CAN_RxHeaderTypeDef        RxMessage;    //CAN接收的消息的消息头

extern uint8_t RxBuffer[8];//接收上位机串口数据的缓冲器

uint8_t OrderBuffer[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};//设备与上位机之间的命令
uint8_t canOrderBuffer[9] = {0x01, 0x02, 0x03, 0xB1, 0xB2, 0xB3, 0xC1, 0xC2, 0xC3};//主机与从机之间的命令

uint8_t linkOK = 0;//请求连接同意标志位
uint8_t Modes = 0;//发送数据模式，0：电池/停止发送，2：足底+电池，3：角度+电池，4：足底+角度+电池

uint8_t uart_send_buf[130];//数据发送缓冲器

uint8_t up_bat_ones = 1;//读取电池数据间隔时间设置位
uint8_t times_bat;//读取电池数据间隔时间设置位
uint8_t times_up_bat;//读取电池数据间隔时间设置位
uint8_t times_req;//循环发送请求时间间隔设置位

uint8_t toe_slave = 0;//足尖从机在不在的标志位
uint8_t knee_slave = 0;//足尖从机在不在的标志位
uint8_t hip_slave = 0;//足尖从机在不在的标志位

uint8_t slavers_mode = 0;//足尖从机在不在的标志位

uint8_t order_or_data = 0;//主机接收的是命令还是数据

uint8_t imu_collect_over = 0;//从机数据采集结束标志

uint8_t receive_data_over = 0;//从机数据采集结束标志

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
	HAL_Delay(500);//软件上电延时，避免ESP上电时对串口中断的影响
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
	
	MPU_Init();//初始化MPU6050
	while(mpu_dmp_init())
	{
	}//等待mpu_dmp初始化
	
	ask_Slavers();//查询从机的连接情况
	
	ask_IPC_Link();//向上位机发送请求连接命令
	
	Get_Battery();//读取电池电压
	
	uint8_t i;//定义for循环变量
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		while(!linkOK){times_bat = 0;}//等待连接同意
			
		if(up_bat_ones)//同意连接后发送一次电池电压
		{
			Upload_Battery(uart_buf_bat, 2);
			up_bat_ones = 0;
		}

		switch(Modes)//选择发送模式
		{
			case 2:
				Get_DMA_ADC1();
				Upload_Data(uart_buf_v, 96);
				break;
			case 3:
				if(toe_slave)//如果从机1在
				{
					CAN_Send(canOrderBuffer[6]);//让从机1发送采集结束标志
				}
				else if(knee_slave)//如果从机1不在，从机2在
				{
					CAN_Send(canOrderBuffer[7]);//让从机2发送采集结束标志
				}
				else if(hip_slave)//如果从机1、2不在，从机3在
				{
					CAN_Send(canOrderBuffer[8]);//让从机3发送采集结束标志
				}
				else//如果没有从机连接
				{
					receive_data_over = 1;
				}
				MPU6050_Get_Data();
//				while(!receive_data_over){}//等待从机数据传输完成
				Upload_Data(uart_buf_imu, 6*4);
				receive_data_over = 0;
				break;
			case 4:
				Get_DMA_ADC1();
				if(toe_slave)//如果从机1在
				{
					CAN_Send(canOrderBuffer[6]);//让从机1发送采集结束标志
				}
				else if(knee_slave)//如果从机1不在，从机2在
				{
					CAN_Send(canOrderBuffer[7]);//让从机2发送采集结束标志
				}
				else if(hip_slave)//如果从机1、2不在，从机3在
				{
					CAN_Send(canOrderBuffer[8]);//让从机3发送采集结束标志
				}
				else//如果没有从机连接
				{
					receive_data_over = 1;
				}
				MPU6050_Get_Data();
				while(!receive_data_over){}//等待从机数据传输完成
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
		
		if(times_up_bat == 6)//每隔一分钟上传一次电池电压
		{
			Battery = (uint16_t)(Battery/6.0);//对一分钟内采集的六次电压取平均值
			uart_buf_bat[0] = Battery>>8;//数据高位
			uart_buf_bat[1] = Battery&0xff;//数据低位
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
  * @brief 传送数据给匿名四轴上位机软件(V2.6版本)
  * @retval None
	* @fun 功能字. 0XA0~0XAF
	* @data 数据缓存区,最多28字节!!
	* @len data区有效数据个数
	*/
void usart1_niming_report(uint8_t fun, uint8_t *data, uint8_t len)
{
	uint8_t send_buf[32];
	uint8_t imu_data[1];
	uint8_t i;
	if(len>28)return;	//最多28字节数据 
	send_buf[len+3]=0;	//校验数置零
	send_buf[0]=0X88;	//帧头
	send_buf[1]=fun;	//功能字
	send_buf[2]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//复制数据
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和	
	for(i=0;i<len+4;i++)
	{
		imu_data[0] = send_buf[i];
		HAL_UART_Transmit(&huart1, imu_data, 1, 2);
	}
}

/**
	*@brief 通过串口1上报结算后的姿态数据给电脑
	*@roll 横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
	*@pitch 俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
	*@yaw 航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
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

	usart1_niming_report(0XA1,tbuf,6);//自定义帧,0XA1
}	

/**
	*@brief 通过ADC2读取电池电压值
  * @retval None
	*/
void Get_Battery(void)
{
	uint16_t Battery_buf = 0;//电池电压值
	HAL_ADC_Start(&hadc2);//启动ADC转换
	HAL_ADC_PollForConversion(&hadc2, 1);//等待转换完成，第二个参数表示超时时间，单位ms.
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2), HAL_ADC_STATE_REG_EOC))
	{
		Battery_buf = HAL_ADC_GetValue(&hadc2);//读取ADC转换数据，数据为12位
		Battery_buf = (float)Battery_buf*(3.310/4096)*1000;
		Battery_buf = Battery_buf*4.0+200;
		
		uart_buf_bat[0] = Battery_buf>>8;//数据高位
		uart_buf_bat[1] = Battery_buf&0xff;//数据低位
	}
}

/**
	*@brief 通过ADC1读取读取DMA_AD值
  * @retval None
	*/
void Get_DMA_ADC1(void)
{
	uint8_t r,i,m;
	
	for(r=0; r<12; r++)//行
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
			m = Row_list[i];//列顺序
			Voltage = (uint16_t)((float)ADC_ConvertedValue[i]/4096*3.310*1000)%10000;
			VoltageS[r][m] = Voltage;
			
			uart_buf_v[r*8+2*m] = Voltage>>8;//数据高位
			uart_buf_v[r*8+2*m+1] = Voltage&0xff;//数据低位
		}
	}
}

/**
	*@brief 延时函数（单位us）
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
	*@brief 陀螺仪数据读取
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
  * @brief CAN过滤器配置函数（掩码全为零，不进行过滤）
  * @retval None
  */
static void CANFilter_Config(void)
{
    CAN_FilterTypeDef  sFilterConfig;
		uint32_t StdId1 = 0x121;//从机1地址
	  uint32_t StdId2 = 0x122;//从机2地址
	  uint32_t StdId3 = 0x123;//从机3地址
    
    sFilterConfig.FilterBank = 0;                       //CAN过滤器编号，过滤器0
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;   //CAN过滤器模式，列表模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;  //CAN过滤器尺度，16位
    sFilterConfig.FilterIdHigh = StdId1 << 5;			      //16位下，存储第一个从机的ID
    sFilterConfig.FilterIdLow = StdId2 << 5;					  //16位下，存储第二个从机的ID
    sFilterConfig.FilterMaskIdHigh = StdId3 << 5;			  //16位下，存储第三个从机的ID
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = 0;				      //报文通过过滤器的匹配后，存储到哪个FIFO
    sFilterConfig.FilterActivation = ENABLE;    		    //激活过滤器
    sFilterConfig.SlaveStartFilterBank = 0;
    
    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief CAN发送数据函数
  * @retval None
  */
void CAN_Send(uint8_t orderD)
{
	uint8_t dataO[1];
	dataO[0] = orderD;

	uint32_t TxMailbox;
	TxMessage.IDE = CAN_ID_STD;     //设置ID类型，标准模式
	TxMessage.StdId = 0x111;        //设置ID号，标识符，可进行过滤
	TxMessage.RTR = CAN_RTR_DATA;   //设置传送数据帧
	TxMessage.DLC = 1;              //设置数据长度

	if (HAL_CAN_AddTxMessage(&hcan, &TxMessage, dataO, &TxMailbox) != HAL_OK) 
	{
		Error_Handler();
  }
}

/**
  * @brief CAN接收中断处理函数
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
					if(toe_slave)//如果从机1在
					{
						CAN_Send(canOrderBuffer[3]);//让从机1发送数据
					}
					else if(knee_slave)//如果从机1不在，从机2在
					{
						CAN_Send(canOrderBuffer[4]);//让从机2发送数据
					}
					else//如果从机1、2不在，从机3在
					{
						CAN_Send(canOrderBuffer[5]);//让从机3发送数据
					}
					order_or_data = 1;//命令传输结束
					break;
			}
		}
		else
		{
			switch(RxMessage.StdId)
			{
				case 0x121://从机1，足尖
					for(i=0; i<6; i++)
					{
						uart_buf_imu[6+i] = data[i];
					}
					if(knee_slave)//从机2在
					{
						CAN_Send(canOrderBuffer[4]);//让从机2发送数据
					}
					else if(hip_slave)//从机2不在，从机3存在
					{
						CAN_Send(canOrderBuffer[5]);//让从机3发送数据
					}
					else//从机2、3不在，则数据传输结束
					{
						receive_data_over = 1;//从机数据传输结束标志位置1
						order_or_data = 0;//数据传输结束
					}
					break;
				case 0x122://从机2，膝关节
					for(i=0; i<6; i++)
					{
						uart_buf_imu[6*2+i] = data[i];
					}
					if(hip_slave)//如果从机3存在
					{
						CAN_Send(canOrderBuffer[5]);//让从机3发送数据
					}
					else//从机3不在，则数据传输结束
					{
						receive_data_over = 1;//从机数据传输结束标志位置1
						order_or_data = 0;//数据传输结束
					}
					break;
				case 0x123://从机3，髋关节
					for(i=0; i<6; i++)
					{
						uart_buf_imu[6*3+i] = data[i];
					}
					receive_data_over = 1;//从机数据传输结束标志位置1
					order_or_data = 0;//数据传输结束
					break;
			}
		}
	}
}

/**
  * @brief 串口接收中断处理函数
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	if(huart -> Instance == USART1) 
	{
		if(RxBuffer[0] == 0XF1)//接收到同意连接命令
		{
			linkOK = 1;
		}
		if(RxBuffer[0] == 0XF2)//接收到发送足底+电池数据命令
		{
			Modes = 2;
		}
		if(RxBuffer[0] == 0XF3)//接收到发送角度+电池数据命令
		{
			Modes = 3;
		}
		if(RxBuffer[0] == 0XF4)//接收到发送足底+角度+电池数据命令
		{
			Modes = 4;
		}
		if(RxBuffer[0] == 0XF5)//接收到停止发送命令
		{
			Modes = 0;
		}
	}
	HAL_UART_Receive_IT(&huart1, RxBuffer, 1);
}

/**
  * @brief 上传电池电压
  * @retval None
	* @data 数据缓存区
	* @len data区有效数据个数
	*/
void Upload_Battery(uint8_t *data, uint8_t len)
{
	uint8_t send_buf[8] = {0};
	uint8_t send_data[2] = {0};
	uint8_t i;

	send_buf[6]=0; //校验数置零

	send_buf[0]=0X88;	 //帧头*************0
	send_buf[1]=0XFF;	 //帧头*************1
	send_buf[2]=0XA1;	 //功能字***********2
	send_buf[3]=2;	   //数据长度*********3

	for(i=0;i<len;i++)send_buf[4+i]=data[i];			    //复制数据
	for(i=0;i<len+4;i++)send_buf[len+4]+=send_buf[i];	//计算校验和	
	
	for(i=0;i<(len+6)/2;i++)
	{
		send_data[0] = send_buf[2*i];
		send_data[1] = send_buf[2*i+1];
		HAL_UART_Transmit(&huart1, send_data, 2, 1);
	}
}

/**
  * @brief 命令上传上位机
  * @retval None
	* @data 数据缓存区
	* @len data区有效数据个数
	*/
void Upload_Order(uint8_t data)
{
	uint8_t send_buf[10] = {0};
	uint8_t i;

	send_buf[5]=0; //校验数置零
	
	send_buf[0]=0X88;	 //帧头*************0
	send_buf[1]=0XFF;	 //帧头*************1
	send_buf[2]=0XA0;	 //功能字***********2
	send_buf[3]=1;	   //数据长度*********3

	send_buf[4]=data;			    //复制数据
	for(i=0;i<5;i++)send_buf[5]+=send_buf[i];	//计算校验和

	HAL_UART_Transmit(&huart1, send_buf, 6, 1);
}

/**
  * @brief 数据上传上位机
  * @retval None
	* @data 数据缓存区
	* @len data区有效数据个数
	*/
void Upload_Data(uint8_t *data, uint8_t len)
{
	uint8_t send_buf[130] = {0};
	uint8_t send_data[2] = {0};
	uint8_t i;

	send_buf[0]=0X88;	 //帧头*************0
	send_buf[1]=0XFF;	 //帧头*************1
	send_buf[2]=0XAF;	 //功能字***********2
	send_buf[3]=len;	 //数据长度*********3

	for(i=0;i<len;i++)send_buf[4+i]=data[i];			    //复制数据
	for(i=0;i<len+4;i++)send_buf[len+4]+=send_buf[i];	//计算校验和	
	
	for(i=0;i<(len+6)/2;i++)
	{
		send_data[0] = send_buf[2*i];
		send_data[1] = send_buf[2*i+1];
		HAL_UART_Transmit(&huart1, send_data, 2, 1);
	}
}

/**
  * @brief 定时器中断处理函数
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1)
	{
		times_bat++;
		
		if(times_bat == 10)//每10s读取一次电池电压
		{
			uint16_t Battery_buf_tim = 0;//电池电压值
			times_up_bat++;
			
			HAL_ADC_Start(&hadc2);//启动ADC装换
			HAL_ADC_PollForConversion(&hadc2, 1);//等待转换完成，第二个参数表示超时时间，单位ms.
			if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2), HAL_ADC_STATE_REG_EOC))
			{
				Battery_buf_tim = HAL_ADC_GetValue(&hadc2);//读取ADC转换数据，数据为12位
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
		
		if((times_req==2)&(!linkOK))//每2s发送一次请求
		{
			ask_IPC_Link();//发送请求连接命令
			times_req = 0;
		}
	}
}


/**
  * @brief 查询从机的连接情况，有几个从机连接,从机连接组合方式
  * @retval None
  */
void ask_Slavers()
{
	CAN_Send(canOrderBuffer[0]);//询问从机1
	HAL_Delay(10);//等待从机1的回答
	CAN_Send(canOrderBuffer[1]);//询问从机2
	HAL_Delay(10);//等待从机2的回答
	CAN_Send(canOrderBuffer[2]);//询问从机3
	HAL_Delay(10);//等待从机3的回答
	
	slavers_mode = toe_slave + knee_slave*10 + hip_slave*100;//计算从机连接情况
}

/**
  * @brief 向上位机发送请求连接命令
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

