#ifndef __8700_2100_H__
#define __8700_2100_H__ 

#define	SlaveAddress2100	      0x20	//0010 0000//IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ   ������8451�ĳ���
#define CTRL_REG1_2100                0x13     //0001 0011

#define	SlaveAddress8700	      0x1e	//0001 1110//IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ   ������8451�ĳ���
#define CTRL_REG1_8700                0x2a      // 0010 1010


#define OUT_X_MSB_REG         0x01
#define OUT_X_LSB_REG         0x02
#define OUT_Y_MSB_REG         0x03
#define OUT_Y_LSB_REG         0x04
#define OUT_Z_MSB_REG         0x05
#define OUT_Z_LSB_REG         0x06


void Init2100();
void Init8700();
void IIC_Read_Acc(unsigned char slave,unsigned char * p,short * q);
void IIC_Read_Gyro(unsigned char slave,unsigned char * p,short * q);
void IIC_Read(unsigned char slave,unsigned char * p,short * q);
void IIC_Read_Acc_16bit(unsigned char slave,unsigned char * p, short * q);//�����ٶ�
void IIC_Read_Gyro_16bit(unsigned char slave,unsigned char * p, short * q);//��������

#endif