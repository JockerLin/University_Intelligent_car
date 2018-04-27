#include "common.h"
#include "include.h"

//void IIC_Start(i2cn);

void IIC_Start(void)//start condition
{
  SCL__OUT;
  SDA__OUT;
  SCL_1;
  WAIT;
  SDA_1;
  WAIT;
  SDA_0;
  SCL_0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}

void IIC_Stop(void)//stop conditoin
{
  SCL__OUT;
  SDA__OUT;
  SCL_1;
  WAIT;
  SDA_0;
  WAIT;
  SDA_1;
}

//����ACKӦ��
void IIC_Ack(void)
{
  SCL_0;
  SDA__OUT;
  SDA_0;
  WAIT;
  SCL_1;
  WAIT;
  SCL_0;
}

//������ACKӦ��		    
void IIC_NAck(void)
{
  SCL_0;
  SDA_0;
  SDA_1;
  WAIT;
  SCL_1;
  WAIT;
  SCL_0;
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
int16 IIC_Wait_Ack(void)
{
	uint8 ucErrTime=0;
        SDA__OUT;  //���ùܽ�Ϊ���������ʱgpio_set()����ʹ��
	SDA_1;
        WAIT;	   
	SCL_1;
        WAIT;
        SDA__IN;
	while(SDA_S)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
                        printf("yingdashibai\n");
			return 1;
		}
	}
	SCL_0;//ʱ�����0 	   
	return 0;  
}

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(uint8 txd)//uint16
{                        
    int t;
    SDA__OUT;
    SCL_0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {                   
		if((txd<<t)&0x80)
			SDA_1;
		else
			SDA_0; 
		WAIT;   //��������ʱ���Ǳ����
		SCL_1;
		WAIT;
		SCL_0;	
		WAIT;
    }	 
		
}

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
int16 IIC_Read_Byte(unsigned char ack)//uint16
{
      unsigned char i;
        int16 receive=0;
        SDA__IN;
      for(i=0;i<8;i++ )
	{
         SCL_0; 
         WAIT;
         SCL_1;        
         receive = (receive<<1)|SDA_S;  //����1
         //receive<<=1;                 //����2
         //if(SDA_S)receive++;   
		WAIT; 
		SCL_0;
		WAIT; 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}

int16 mpu6050_read(uint8 reg)
{
  int16 a;
  IIC_Start();
  IIC_Send_Byte(0xD0);
  IIC_Wait_Ack();
  IIC_Send_Byte(reg);
  IIC_Wait_Ack();
  IIC_Start();
  IIC_Send_Byte(0xD1);
  IIC_Wait_Ack();
  a=IIC_Read_Byte(0);  //0��ʾ����nack
  IIC_Stop();
  return a;
}

void mpu6050_write(uint8 reg,uint8 data)
{
  IIC_Start();
  IIC_Send_Byte(0xD0);
  IIC_Wait_Ack();
  IIC_Send_Byte(reg);
  IIC_Wait_Ack();
  IIC_Send_Byte(data);
  IIC_Wait_Ack();
  IIC_Stop();
  DELAY_US(1000);
}
/*
void mpu6050_init()
{
  mpu6050_write(SMPLRT_DIV,0x07);
  mpu6050_write(CONFIG,0x06);
  mpu6050_write(AUX_VDDIO,0x80);
  mpu6050_write(GYRO_CONFIG,0x18);
  mpu6050_write(ACCEL_CONFIG,0x00);
  mpu6050_write(PWR_MGMT_1,0x00);  //�������޷����мĴ�����дֵ����ֵ
}
*/