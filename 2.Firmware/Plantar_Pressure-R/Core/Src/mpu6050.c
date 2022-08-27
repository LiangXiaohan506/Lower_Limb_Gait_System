#include "mpu6050.h"
#include "stdio.h"
#include "usart.h"
#include "i2c.h"

/***********
*@brief:IICдһ���ֽ�
*@param:reg-�Ĵ�����ַ��data-����
*@return:0������
************/
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c2,MPU_WRITE,reg,I2C_MEMADD_SIZE_8BIT,&data,1,0xfff);
	return 0;
}

/***********
*@brief:IIC��ȡһ���ֽ�
*@param:reg-�Ĵ�����ַ
*@return:��ȡ��ֵ
************/
uint8_t MPU_Read_Byte(uint8_t reg)
{
	unsigned char R_Data = 0;
	
	HAL_I2C_Mem_Read(&hi2c2,MPU_READ,reg,I2C_MEMADD_SIZE_8BIT,&R_Data,1,0xfff);
	return R_Data;
}

/***********
*@brief:IIC����д
*@param:reg-��ַ��len-Ҫ��ȡ�ĳ���,buf-��ȡ�������ݴ洢��
*@return:
************/
uint8_t MPU_Write_Len(uint8_t reg,uint8_t len,uint8_t *buf)
{
  HAL_I2C_Mem_Write(&hi2c2, MPU_WRITE, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
  return 0;
}

/***********
*@brief:IIC������
*@param:reg-��ַ��len-Ҫ��ȡ�ĳ���,buf-��ȡ�������ݴ洢��
*@return:����ֵ0,����
************/
uint8_t MPU_Read_Len(uint8_t reg,uint8_t len,uint8_t *buf)
{ 
  HAL_I2C_Mem_Read(&hi2c2, MPU_READ, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
  return 0;	
}

/***********
*@brief:����mpu6050�����Ǵ����������̷�Χ
*@param:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
*@return:����0�����óɹ�
************/
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3); //���������������̷�Χ,�� 3��4λ����
}

/***********
*@brief:����mpu6050���ٶȼ������̷�Χ
*@param:fsr:0,��2g;1,��4g;2,��8g;3,��16g
*@return:����0�����óɹ�
************/
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3); //���ü��ٶȼ������̷�Χ,�� 3��4λ����
}

/***********
*@brief:����mpu6050��ͨ�˲���
*@param:���ֵ�ͨ�˲���Ƶ�ʣ�hz)
*@return:����0���ɹ�
************/
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf >= 188)
		data = 1;
	else if(lpf >= 98)
		data = 2;
	else if(lpf >= 42)
		data = 3;
	else if(lpf >= 20)
		data = 4;
	else if(lpf >= 10)
		data = 5;
	else
		data = 6;
	return MPU_Write_Byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���
}

/***********
*@brief:����MPU6050������(Fs=1KHz)
*@param:rate-4-1000(Hz)
*@return:0-�ɹ�
************/
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate > 1000)
		rate = 1000;
	if(rate < 4)
		rate = 4;
	data = 1000/rate-1;
	data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);
	return MPU_Set_LPF(rate/2);  //Ĭ������LPFΪ�����ʵ�1/2
}

/***********
*@brief:��ȡ�¶���Ϣ
*@param:void
*@return:�¶�ֵ
************/
float MPU_Get_Temperature(void)
{
	unsigned char buf[2];
	short raw;
	float temp;
	
	MPU_Read_Len(MPU_TEMP_OUTH_REG,2,buf);
	raw = (buf[0]<<8 | buf[1]);
	
	temp = (36.53+(double)raw)/340*100;
	
	return temp/100.0f;
}

/***********
*@brief:��ȡ����������(ԭʼֵ)
*@param:������xyz(������)
*@return:����0�ɹ�
************/
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
	uint8_t buf[6],res;
	
	res = MPU_Read_Len(MPU_GYRO_XOUTH_REG,6,buf);
	if(res == 0)
	{
		*gx = ((uint16_t)buf[0]<<8) | buf[1];
		*gy = ((uint16_t)buf[2]<<8) | buf[3];
		*gz = ((uint16_t)buf[4]<<8) | buf[5];
	}
	return res;
}

/***********
*@brief:��ȡ���ٶ���Ϣ(ԭʼֵ)
*@param:xyz(������)
*@return:����0�ɹ�
************/
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
	uint8_t buf[6],res;
	res = MPU_Read_Len(0x3B,6,buf);
	if(res == 0)
	{
		*ax = ((uint16_t)buf[0]<<8) | buf[1];
		*ay = ((uint16_t)buf[2]<<8) | buf[3];
		*az = ((uint16_t)buf[4]<<8) | buf[5];
	}
	return res;
}

/***********
*@brief:
*@param:
*@return:
************/
uint8_t MPU_Init(void)
{
	uint8_t res;
	
	HAL_I2C_Init(&hi2c2); //��ʼ��IIC
	
	/*�ϵ�������һ����ʱ����֤����׼ȷ*/
	HAL_Delay(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0x80);//��λMPU6050,д��1000 0000
	HAL_Delay(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0x00);//sleepд��0�����ֻ���״̬
	
	MPU_Set_Gyro_Fsr(3);  //�����������̡�2000
	MPU_Set_Accel_Fsr(0); //���ٶȼ������̡�2g
	MPU_Set_Rate(50);     //���ò�����=50Hz
	
	MPU_Write_Byte(MPU_INT_EN_REG,0x00);//�ر������ж�
	MPU_Write_Byte(MPU_USER_CTRL_REG,0x00); //�رմ�iic
	MPU_Write_Byte(MPU_FIFO_EN_REG,0x00);   //�ر�FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0x80);  //INT���ŵ͵�ƽ��Ч
	
	res = MPU_Read_Byte(MPU_DEVICE_ID_REG);  //��ȡ��ַ
//	UsartPrintf(&huart1,"\r\nMPU6050:0x%2x\r\n",res);
//	UsartPrintf(&huart1,"\r\nMPU6050:0x%2x\r\n",MPU_ADDR);
	if(res == MPU_ADDR) //�жϵ�ַ�Ƿ���ȷ
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0x01); //001,pll��x��Ϊ�ο�ϵͳʱ��Դ
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0x00); //����ʹ�ô���ģʽ
		MPU_Set_Rate(50);
	}
	else 
		return 1;
	
	return 0;
}
