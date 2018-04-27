#include "common.h"
#include "include.h"
#include "VCAN_LCD_CH.h"
#include "math.h"
#include "core_cm4.h"


uint8 imgbuff[1200],imgbuff_trans[CAMERA_SIZE],img[CAMERA_H][CAMERA_W],//定义存储接收图像的数组
      Depart=0,qipao = 1,Live_Watch_Mark=0,
      Gyro_x_h,Gyro_x_l,Gyro_z_h,Gyro_z_l,Accle_z_h,Accle_z_l,bit8_data[6]; //定义高八位低八位 存低八位的陀螺仪或加速度值
uint16 podao=0,II=0;
uint32 Duoji_Duty = 0,Mada_Duty=0,Start_Time_Count=0,Start_Time=0;
unsigned short send_data[8];
int16 Gyro_X,Gyro_Y,Gyro_Z,Acc_X,Acc_Y,Acc_Z,Acc_Z_Test,bit16_data16[3]; //存放十六位的陀螺仪或加速度值;
short count=0;//改成short
float Picture_detal=0,Dir_detal=0,Picture_detal_Old=0,Picture_detal_detal=0,fLeftVal,fRightVal;

extern uint8 Mid_Line[60];
extern uint16 Get_Picture_Count,Angle_Caculate_Count,Angle_Output_Count,
               Speed_Control_Count,Speed_Output_Count,Dir_Count,Dir_Output_Count,
               Stop_Check_Begin_Count,Charge_Stop_Car_Count,Stop_Check_Count,
               Road_Kind,Road_Kind_Last,O_Road_Num,Stand_Inplace_Count,
               DIP_Number,ML_Black_Spot_count,//拨码开关状态读取
               DIP_Number,Number_3,Number_2,Number_1;
extern int16 LB_Lose_Count_Last,RB_Lose_Count_Last;
extern int32 Left_Motor,Right_Motor;
extern float Duoji_P,Duoji_I,Duoji_D,Stand_Speed,Picture_detal_Printf,
              Angle_Acc,Car_Gyro_X,Gyro_Speed_X,Gyro_Speed_X_Ratio,Angle_Mix,Angle_Gyro,Angle_Now,
              g_fCarSpeedSetL,//1200,//600,800~35.7s  1000~28.64~165m稳定测试 
              g_fCarSpeedSetR,//1200;//600;//120~123s
              g_fCarSpeedSet,
              g_fCarSpeedSet_Start,
              g_fCarSpeedSet_Max;
extern double fGyroscopeAngleIntegral,g_fCarAngle ,g_fGyroscopeAngleSpeed ,g_fGravityAngle,fDeltaValue ;
typedef unsigned char byte;//bute 能代替unsigned char 这个数据类型
typedef unsigned int word;//word 能代替unsigned int 这个数据类型

//函数声明
void main_scan();
void LCD_Init(void); //oled 初始化
void System_Init(void);
void PORTA_IRQHandler(void);
void DMA0_IRQHandler(void);
void PIT0_IRQHandler(void);
void Get_Middle_Line(void);
void Img_Printf_OLED(void);
void sendimg(void *imgaddr, uint32 imgsize);
extern void LCD_P6x8Str(byte x,byte y,byte ch[]);//显示字符串
extern void LCD_P14x16Str(byte x,byte y,byte ch[]);//显示汉子
extern void LCD_Print(byte x, byte y, byte ch[]);//显示汉字和字符

/*
函数名称：主函数 
最后调试时间：2016-1112
作者：厦理创新实验室智能车队-暗涌 
库函数：山外
前瞻62.5cm
有疑问致电邮：13950986716，lczq@vip.qq.com
**************************************************************************************************/
void  main(void)
{
    DisableInterrupts;   //禁止总中断
    System_Init();//系统初始化 所有乱七八糟的初始化都在里面 嘿嘿嘿
    set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //设置 PORTA 的中断服务函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);      //设置 DMA0 的中断服务函数为 PORTA_IRQHandler
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //设置PIT0的中断复位函数为 PIT0_IRQHandler
    set_irq_priority(PIT0_IRQn, 1);//中断优先级设置
    set_irq_priority(DMA0_IRQn, 0);//DMA中断等级更高
    //pit_init_ms(PIT0, 1);     //初始化PIT0，定时时间为
    EnableInterrupts;//使能中断开
    while(1)
    {
      if(qipao == 1)
      //while(1)
         {
           //gpio_set (PTE12,0);
           disable_irq (PIT0_IRQn);  //关中断
           ftm_pwm_duty(FTM0,FTM_CH3,0); //3>6 左 后
           ftm_pwm_duty(FTM0,FTM_CH6,0);//
           ftm_pwm_duty(FTM0,FTM_CH4,0);//4>5 右 后
           ftm_pwm_duty(FTM0,FTM_CH5,0);//
           DIP_status();
           plan_Chose();
           main_scan();
           g_fCarSpeedSet_Start=g_fCarSpeedSet;
           g_fCarSpeedSet_Max=g_fCarSpeedSet_Start+20;
         }
      //enable_irq(PIT0_IRQn);     //使能PIT0中断
      II=600; 
      while(II--) {imgbuff_trans[II]=imgbuff[II];}
      camera_get_img();//11XXXus——摄像头获取图像（一维数组）
      //Img_Printf_OLED();//2ms
      //gpio_set (PTA15,1);
    }
}

/*
函数名称：中断服务程序 
最后调试时间：20161126
作者：厦理创新实验室智能车队-暗涌
库函数：山外
时间：8ms 一个基准 
有疑问致电邮：13950986716，lczq@vip.qq.com
**************************************************************************************************/
void PIT0_IRQHandler(void)
{
      Get_Acc_And_Gyro();//788us——获取加速度陀螺仪数值
      Balance_Filter(Acc_Z,Gyro_Z);//60us——互补滤波角度计算
      Angle_All_Control();//16us——角度环
      Send_Data_Up();//1818us 跑的时候关
      MotorOutput();//12us——电机输出
      Depart=Depart%3;
      switch(Depart)
      {
      case 0://速度控制 进来5次 算1次
        {
          Get_Speed();//12us——获取编码器的值（每次间隔时间一定要一样，否则会过弯道一卡一卡的）
          Speed_Output_Count++;
          Speed_Loop_Output();//11us——速度输出
          Speed_Output_Count=Speed_Output_Count%5;
          if(Speed_Output_Count == 0)
           {
             Speed_Loop_Control();//25us——速度计算
             Speed_Output_Count=0;
           }
          break;
        }
      case 1://采图像啊 
        {
          camera_get_img();//11XXXus——摄像头获取图像（一维数组）
          img_extract((uint8 *)img,(uint8 *)imgbuff,CAMERA_SIZE);//247us——解压为灰度图像（二维数组）
          //vcan_sendimg(img, sizeof(img));//图像发送到上位机，步兵高清
          break;
        }
      case 2://方向跟随  进来2次 算1次 与获取图像同一步骤
        {
          Dir_Loop_Control();//4400us——图像处理，方向环计算
          Dir_Loop_Output();//14us——方向环输出
          break;
        }
      }
      //vcan_sendimg(img, sizeof(img));//图像发送到上位机 带中线
      Depart++;
      Stop_Check();//角度变了或检测到终点了 停车！
      PIT_Flag_Clear(PIT0);//清中断标志位     
      disable_irq (PIT0_IRQn);  //关中断
      /*计算时间
      pit_time_start  (PIT1);
      count= pit_time_get_us(PIT1);
      */
}

/*
函数名称：中断服务函数，运用场中断 
最后调试时间：20170501
作者：厦理创新实验室智能车队-暗涌
库函数：山外
有疑问致电邮：13950986716，lczq@vip.qq.com
**************************************************************************************************/
void PORTA_IRQHandler()
{
    uint8  n;    //引脚号
    uint32 flag;
    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位
    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
    if(!qipao)
    {
      
      //if(Live_Watch_Mark)//动态参数显示——按键选择开关——比赛关
      /*
      {
        DIP_status();//获取拨码开关数值
        LCD_Print(2,0,"Road_Kind:");//查看路况种类
        LCD_PrintU16(80,0,Road_Kind);
        LCD_Print(2,1,"ML_Black:");//中间行数黑点数
        LCD_PrintU16(80,1,ML_Black_Spot_count);
        LCD_Print(2,2,"O_Dir:");//圆环方向设置
        if(Number_1==1)
        {
          LCD_Print(50,2,"R");
        }
        else if(Number_1==0)
        {
          LCD_Print(50,2,"L ");
        }
        DisplayFloat((int16)(Dir_detal*1000),2,3);//方向偏差
        DisplayFloat((int16)(Picture_detal_detal*1000),2,4);//方向偏差的偏差
        LCD_Print(2,5,"O_Times:");//
        LCD_PrintU16(80,5,O_Road_Num);
        LCD_Print(2,6,"LB_Lose:");//
        LCD_PrintU16(80,6,LB_Lose_Count_Last);
        LCD_Print(2,7,"RB_Lose:");//
        LCD_PrintU16(80,7,RB_Lose_Count_Last);
      }
      */
      //DIP_status();
      //DELAY_MS(3);
      //pit_time_start(PIT1);
      Get_Acc_And_Gyro();//788us——获取加速度陀螺仪数值
      Balance_Filter(Acc_Z,Gyro_Z);//60us——互补滤波角度计算
      Angle_All_Control();//16us——角度环
      Get_Speed();//12us——获取编码器的值（每次间隔时间一定要一样，否则会过弯道一卡一卡的）
      Speed_Output_Count++;
      Speed_Loop_Output();//11us——速度输出
      Speed_Output_Count=Speed_Output_Count%10;
      if(Speed_Output_Count == 0)
       {
         Speed_Loop_Control();//25us——速度计算
         Speed_Output_Count=0;
       }
      //Img_Printf_OLED();
      Dir_Loop_Control();//4400us——图像处理，方向环计算
      Dir_Loop_Output();//14us——方向环输出
      Stop_Check();//出借 停车！
      MotorOutput();//12us——电机输出
      //Send_Data_Up();
      img_extract((uint8 *)img,(uint8 *)imgbuff_trans,CAMERA_SIZE);//247us——解压为灰度图像（二维数组）
      /*
      led_change_count++;
      led_change_count=led_change_count%100;
      if(led_change_count==0)
        led_turn(LED0);
      */
      //led_turn(LED0);
      //vcan_sendimg(img, sizeof(img));
      //count= pit_time_get_us(PIT1);
      //*(int*)0=0;//向0x00地址写0
      //printf()
      //puts(ok?)
      //while(1)
      //{
      //  uart_putstr (UART0,"6666");
      //}
    }
}

/*
函数名称：DMA中断服务函数 
最后调试时间：20160928
作者：厦理创新实验室智能车队-暗涌 
库函数：山外
有疑问致电邮：13950986716，lczq@vip.qq.com
**************************************************************************************************/
void DMA0_IRQHandler()
{
    camera_dma();
}

void System_Init(void)
{
  LCD_Init();//OLED屏幕初始化
  Key_Init();//键盘初始化
  camera_init(imgbuff); //摄像头初始化
  pll_init(PLL180);
  i2c_init(I2C0,400*1000);//初始化I2C,设置波特率400K
  uart_init (UART0, 115200);//初始化蓝牙波特率
  Init8700();//加速度计 3轴
  Init2100();//陀螺仪   3轴
  //led_init(LED0);
  ftm_pwm_init(FTM0, FTM_CH3,10000, 0);//初始化电机输出
  ftm_pwm_init(FTM0, FTM_CH4,10000, 0);//初始化电机输出
  ftm_pwm_init(FTM0, FTM_CH5,10000, 0);//初始化电机输出
  ftm_pwm_init(FTM0, FTM_CH6,10000, 0);//初始化电机输出
  ftm_quad_init(FTM1);//初始化编码器通道
  ftm_quad_init(FTM2);//初始化编码器通道
  gpio_init(PTA15, GPO, 0);//蜂鸣器
  //拨码开关从左到右 分别是：①圆环方向；②斑马线停车；③坡道开关；④模糊
  gpio_init(PTC9, GPI, 0);//拨码开关
  gpio_init(PTC11, GPI, 0);//拨码开关
  gpio_init(PTC13, GPI, 0);//拨码开关
  gpio_init(PTC15, GPI, 0);//拨码开关
  gpio_init(PTD0, GPI, 0);//拨码开关
  gpio_init(PTD2, GPI, 0);//拨码开关
  gpio_init(PTD11, GPI, 0);//拨码开关
  gpio_init(PTD12, GPI, 0);//拨码开关
}