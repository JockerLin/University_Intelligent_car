#ifndef _IIC_Sim_h_
#define _IIC_Sim_h_

#include "common.h"
#include "include.h"

//引脚输入输出方向
#define SCL__OUT gpio_ddr (I2C0_SCL_PIN, GPO)
#define SCL__IN gpio_ddr (I2C0_SCL_PIN, GPI)
#define SDA__OUT gpio_ddr (I2C0_SDA_PIN, GPO)
#define SDA__IN gpio_ddr (I2C0_SDA_PIN, GPI)

//引脚高低电平
#define SCL_1 gpio_set(I2C0_SCL_PIN, 1)
#define SCL_0 gpio_set(I2C0_SCL_PIN, 0)
#define SDA_1 gpio_set(I2C0_SDA_PIN, 1)
#define SDA_0 gpio_set(I2C0_SDA_PIN, 0)
#define WAIT  DELAY_US(2)
#define SDA_S gpio_get(I2C0_SDA_PIN)

void IIC_Start(void);//start condition
void IIC_Stop(void);//stop conditoin
void IIC_Ack(void);
void IIC_NAck(void);
int16 IIC_Wait_Ack(void);
void IIC_Send_Byte(uint8 txd);//uint16
int16 IIC_Read_Byte(unsigned char ack);//uint16
int16 mpu6050_read(uint8 reg);
void mpu6050_write(uint8 reg,uint8 data);

#endif