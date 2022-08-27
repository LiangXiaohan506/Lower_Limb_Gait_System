#include "mpu6050.h"
#include "stdio.h"
#include "usart.h"
#include "i2c.h"

/***********
*@brief:IIC写一个字节
*@param:reg-寄存器地址，data-数据
*@return:0，正常
************/
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c2,MPU_WRITE,reg,I2C_MEMADD_SIZE_8BIT,&data,1,0xfff);
	return 0;
}

/***********
*@brief:IIC读取一个字节
*@param:reg-寄存器地址
*@return:读取的值
************/
uint8_t MPU_Read_Byte(uint8_t reg)
{
	unsigned char R_Data = 0;
	
	HAL_I2C_Mem_Read(&hi2c2,MPU_READ,reg,I2C_MEMADD_SIZE_8BIT,&R_Data,1,0xfff);
	return R_Data;
}

/***********
*@brief:IIC连续写
*@param:reg-地址，len-要读取的长度,buf-读取到的数据存储区
*@return:
************/
uint8_t MPU_Write_Len(uint8_t reg,uint8_t len,uint8_t *buf)
{
  HAL_I2C_Mem_Write(&hi2c2, MPU_WRITE, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
  return 0;
}

/***********
*@brief:IIC连续读
*@param:reg-地址，len-要读取的长度,buf-读取到的数据存储区
*@return:返回值0,正常
************/
uint8_t MPU_Read_Len(uint8_t reg,uint8_t len,uint8_t *buf)
{ 
  HAL_I2C_Mem_Read(&hi2c2, MPU_READ, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
  return 0;	
}

/***********
*@brief:设置mpu6050陀螺仪传感器满量程范围
*@param:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
*@return:返回0，设置成功
************/
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3); //设置陀螺仪满量程范围,第 3、4位控制
}

/***********
*@brief:设置mpu6050加速度计满量程范围
*@param:fsr:0,±2g;1,±4g;2,±8g;3,±16g
*@return:返回0，设置成功
************/
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3); //设置加速度计满量程范围,第 3、4位控制
}

/***********
*@brief:设置mpu6050低通滤波器
*@param:数字低通滤波器频率（hz)
*@return:返回0，成功
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
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器
}

/***********
*@brief:设置MPU6050采样率(Fs=1KHz)
*@param:rate-4-1000(Hz)
*@return:0-成功
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
	return MPU_Set_LPF(rate/2);  //默认设置LPF为采样率的1/2
}

/***********
*@brief:读取温度信息
*@param:void
*@return:温度值
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
*@brief:获取陀螺仪数据(原始值)
*@param:陀螺仪xyz(带符号)
*@return:返回0成功
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
*@brief:获取加速度信息(原始值)
*@param:xyz(带符号)
*@return:返回0成功
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
	
	HAL_I2C_Init(&hi2c2); //初始化IIC
	
	/*上电后最好有一定延时，保证数据准确*/
	HAL_Delay(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0x80);//复位MPU6050,写入1000 0000
	HAL_Delay(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0x00);//sleep写入0，保持唤醒状态
	
	MPU_Set_Gyro_Fsr(3);  //陀螺仪满量程±2000
	MPU_Set_Accel_Fsr(0); //加速度计满量程±2g
	MPU_Set_Rate(50);     //设置采样率=50Hz
	
	MPU_Write_Byte(MPU_INT_EN_REG,0x00);//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG,0x00); //关闭从iic
	MPU_Write_Byte(MPU_FIFO_EN_REG,0x00);   //关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0x80);  //INT引脚低电平有效
	
	res = MPU_Read_Byte(MPU_DEVICE_ID_REG);  //读取地址
//	UsartPrintf(&huart1,"\r\nMPU6050:0x%2x\r\n",res);
//	UsartPrintf(&huart1,"\r\nMPU6050:0x%2x\r\n",MPU_ADDR);
	if(res == MPU_ADDR) //判断地址是否正确
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0x01); //001,pll，x轴为参考系统时钟源
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0x00); //都不使用待机模式
		MPU_Set_Rate(50);
	}
	else 
		return 1;
	
	return 0;
}
