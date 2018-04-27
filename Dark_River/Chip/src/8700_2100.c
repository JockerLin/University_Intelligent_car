#include "include.h"
#include "common.h"



    void Init2100()
{
        i2c_write_reg(I2C0,SlaveAddress2100,0x0d,0x02);
        //向从机SlaveAddress2100（0x20）的寄存器0x0d写入数据0x02

        Pause();

        i2c_write_reg(I2C0,SlaveAddress2100,CTRL_REG1_2100,0x02);
        //向从机SlaveAddress2100（0x20）的寄存器CTRL_REG1_2100（0x13）写入数据0x02

        Pause();
}

    void Init8700()
{

     //  I2C_WriteAddr(I2C1,SlaveAddress8700,0x0f,0x33);

     //   Pause();

        i2c_write_reg(I2C0,SlaveAddress8700,CTRL_REG1_8700,0x05);
        //向从机 SlaveAddress8700 （0x1e）的寄存器CTRL_REG1_8700（0x2a）写入数据0x05

        Pause();
}

//官方给的原始程序
void IIC_Read(unsigned char slave,unsigned char * p, short * q)
{
  int MUP_Zero=0x0000;
  
  p[0]  =    i2c_read_reg(I2C0,slave,OUT_X_MSB_REG);
  
  Pause();
  p[1]  =    i2c_read_reg(I2C0,slave,OUT_X_LSB_REG);
  Pause();
  
  p[2]  =    i2c_read_reg(I2C0,slave,OUT_Y_MSB_REG);
  Pause();
  p[3]  =    i2c_read_reg(I2C0,slave,OUT_Y_LSB_REG);
  Pause();
  
  p[4]  =    i2c_read_reg(I2C0,slave,OUT_Z_MSB_REG);
  Pause();
  p[5]  =    i2c_read_reg(I2C0,slave,OUT_Z_LSB_REG);
  Pause();


  q[0] = (((MUP_Zero | p[0])<<8)|p[1]);
  q[1] = (((MUP_Zero | p[2])<<8)|p[3]);
  q[2] = (((MUP_Zero | p[4])<<8)|p[5]);

}
void IIC_Read_Gyro(unsigned char slave,unsigned char * p, short * q)//读陀螺仪
{
  int MUP_Zero=0x0000;
  
  p[0]  =    i2c_read_reg(I2C0,slave,OUT_X_MSB_REG);
  Pause();
  p[1]  =    i2c_read_reg(I2C0,slave,OUT_X_LSB_REG);
  Pause();
  
  p[2]  =    i2c_read_reg(I2C0,slave,OUT_Y_MSB_REG);
  Pause();
  p[3]  =    i2c_read_reg(I2C0,slave,OUT_Y_LSB_REG);
  Pause();

  q[0] = (((MUP_Zero | p[0])<<8)|p[1]);
  q[1] = (((MUP_Zero | p[2])<<8)|p[3]);
  //q[2] = (((MUP_Zero | p[4])<<8)|p[5]);

}
void IIC_Read_Acc(unsigned char slave,unsigned char * p, short * q)//读加速度
{
  int MUP_Zero=0x0000;

  p[4]  =    i2c_read_reg(I2C0,slave,OUT_Z_MSB_REG);
  //从SlaveAddress8700(0x1e)地址读取OUT_X_MSB_REG(0x05)寄存器的值
  Pause();
  
  p[5]  =    i2c_read_reg(I2C0,slave,OUT_Z_LSB_REG);
  //从SlaveAddress8700(0x1e)地址读取OUT_X_MSB_REG(0x06)寄存器的值
  Pause();

  //q[0] = (((MUP_Zero | p[0])<<8)|p[1]);
  //q[1] = (((MUP_Zero | p[2])<<8)|p[3]);
  q[2] = ((((int16)p[4])<<8)|p[5]);
  
  //q[1]=i2c_read_reg_16bit(I2C0,slave,OUT_Z_MSB_REG);

}

//8位+8位连续读出16位加速度Z轴数值
void IIC_Read_Acc_16bit(unsigned char slave,unsigned char * p, short * q)//读加速度
{

  q[2]=i2c_read_reg_16bit(I2C0,slave,OUT_Z_MSB_REG);//加速度Z轴

}

void IIC_Read_Gyro_16bit(unsigned char slave,unsigned char * p, short * q)//读陀螺仪
{

  q[0]=i2c_read_reg_16bit(I2C0,slave,OUT_X_MSB_REG);//读陀螺仪X
  Pause();
  
  q[1]=i2c_read_reg_16bit(I2C0,slave,OUT_Y_MSB_REG);//读陀螺仪X
  

}


