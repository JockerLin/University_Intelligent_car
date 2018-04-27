/*
函数名称：Car_Control.c
最后修改时间：2016-12-10
作者：厦理创新实验室-暗涌队
有疑问致电邮：13950986716，lczq@vip.qq.com
数据记录分析：1、抬高摄像头角度，入弯出弯会抬头低头，应该协调好直立环和方向环，
              2、600速度能比较稳定地跑1~2圈。可是U弯道后面偏内道，易冲出,
              3、十字弯偏右问题仍然未解决.16-1119 解决了直道偏移问题，可是只是简单地在十字丢线处划中线，并没有采集那四个拐点啊,
              4、要根据拐点画出黑线啊,
              5、车子启动时间慢，一碰到路肩速度降为0(暂未解决)
              6、 加入斜入十字和小S 但是不行用，还在调试中 先解决小S吧 快成功了
速度记录：16-11-13 1、600~1.31
          16-11-18 2、800~1.54
          16-11-18 3、1000~1.92m~30s  165m稳定测试 
          16-11-18 4、1010~2.0m~27.0s
          16-11-26 5、900~27.0s
          16-11-27 6、850 分段 28.4
          16-11-28 7、850 分段 涂油 最好27.2，不涂油不稳定，待加强。
          16-12-01 8、900 分段 不涂油 最好26.7 不碰到露肩 一般27.1+ 
          16-12-04 9、1000 分段 26+
          16-12-05 9、1000 不分段 26+ 前瞻很重要 艹你奶奶的 调半死！
          16-12-05 10、分段啊 不稳定
          16-12-10 11、1050 25.8s 
          17-02-26 12、拐弯卡顿
          17-03-03 未解决问题：1、误入圆环：十字前+上坡 2、进出圆环不流畅
          17-03-10 斜入十字要开连续滤波，拐大弯会误判到斜入十字，会跑外道去，条件不够强
           55m 65m 47.73m 70.3m
*/
#include "Car_Control.h"
#include "math.h"
#include "include.h"
#include "common.h"
/**************************PID参数**************************/
/*太后往后，太前往前*/
/************角度环************/
/*内环角速度*/
float P_Angle_Speed_Init=0.09,//0.06,//0.095,//0.0900,//0.08//0.055 //大跑道用0.07+低速度
       P_Angle_Speed_Now=0.09,
       I_Angle_Speed=0,//0.01,//0.05,//0.010,
       D_Angle_Speed=0.01;//0.012;//0.03//0.025;//0.02;
//老方法 P~D 0.62~0.075  0.28~0.035
//NEW    P~D 
/*外环角度*/
float  P_Angle=1.0//1.05
       ,I_Angle=0
       ,D_Angle=0.00
       ;

/************速度环************/
float g_fSpeedControlP=1.2,//1.2,//2.02,//2.7,//1.95,
       g_fSpeedControlP_Init=1.2,
       g_fSpeedControlI=0,//0.025,//1.3,
       g_fSpeedControlD=0.6;//-0.6;//-0.6;//-1
       
float g_fCarSpeedSetL=0,//1200,//600,800~35.7s  1000~28.64~165m稳定测试 
      g_fCarSpeedSetR=0,//1200;//600;//120~123s
      g_fCarSpeedSet=2700,//2200,//
      g_fCarSpeedSet_Go=2700,//2200,
      g_fCarSpeedSet_Start=2700,//2200,
      g_fCarSpeedSet_Max=0;//2200;
float fLeft, fRight;
//2500 能跑2.6+

/************方向环************/
//P加大的话十字处理不好会偏离啊 囧
//前瞻距离与P
//看脚下 -P9  一个S弯道-
float P_Direction_Control=43,//44,//9.0,//7.8,//20,//7.75,//7.85,//4.5,//4.5~600,//3.4,//3.5//0.25 1000~6.3
       P_Direction_Control_now=43,//44,//9.0//7.8,//20,//7.75,//7.85,
       D_Direction_Control=0.033,//30;//30;//54;//54.5;//56;//;//25;//0.35;//0.32
       D_Direction_Control_now=0.033; //26 17
        // D阻碍拐弯
       // P D组合 6+30  9+30 7+20 9+25 11+30 9+31 2000+7+23 8.25+28 
       // 新PD组合 1800+30+0.02 2000+36+0.02+2.1m 39+0.022 1800+36+0.025 
        // 2100+34+0.021 2200+34+0.021
       //14s 2600+42+0.02
       //170508：39+22+20
       //0627 36+33 26+17
//抬高摄像头 缩减行数



/**************************角度环参数**************************/ 

#define One_Pro_Period   0.001216//0.0069
#define GRAVITY_OFFSET   -30//-150//300//加速度零偏值，如何确定
#define GYROSCOPE_OFFSET 65 //陀螺仪零偏值 OK
#define GYROSCOPE_ANGLE_SIGMA_FREQUENCY	 125//150//125//85//200//15//8 ？？//互补滤波没用到
#define GRAVITY_ADJUST_TIME_CONSTANT	 2//0.5//1//2//1.2//0.05//主要这个影响加速度比重很大，互补滤波没用到
#define GYROSCOPE_ANGLE_RATIO		23.4//23//29//30.5//太小不行//陀螺仪角速度放大倍数互补滤波调节这两个
#define GRAVITY_ANGLE_RATIO             1.0//加速度放大倍数，会让图像偏移，互补滤波调节这两个
#define Filter_Pass_Ratio               0.994//0.994//0.995//0.995//0.99//越小越跟着加速度

/************数据采集************/
extern int16 Gyro_X,Gyro_Y,Gyro_Z,Acc_X,Acc_Y,Acc_Z,Acc_Z_Test;
int16  Gyro_Test=0;
float Angle_Acc,Car_Gyro_X,Gyro_Speed_X,Gyro_Speed_X_Ratio,Angle_Mix,Angle_Gyro;//gg
float Stand_Speed,Speed_L,Speed_R,Speed_L_Last,Speed_R_Last;  //站立速度，左右轮速度
float Angle_Control_Out,Speed_Control_Out,Direction_Control_Out,Direction_Control_Out_Last; //这行暂时没用到

/************角度外环************/
float Gravity_Angle_Error,
      Gravity_Angle_Error_Error,
      Gravity_Angle_Error_Last,
      Gravity_Angle_Integral,
      Stand_Speed_Out_Out,
      Gravity_Angle_I
      ;
uint16 Gravity_Angle_Integral_Start_Time=0,
       Gravity_Angle_Integral_Start_Mark=0,
       Gravity_Angle_Index=1
       ;
/************角度内环************/
float Angle_Error,
      Angle_Speed_Error,
      Angle_Speed_Expect,//期望角速度，由加速度计算而来。
      Angle_Speed_Integral,
      Angle_Speed_Error_Error,
      Angle_Speed_Error_Last,
      Stand_Speed_Part_In_Now,
      Stand_Speed_Part_In_Old,
      Stand_Speed_Part_In
      ;
uint16 Angle_Speed_Integral_Start_Time=0,
       Angle_Speed_Integral_Start_Mark=0,
       Angle_Speed_Index=1 //积分分离标志
       ;
/************角度环总************/// real heavy 4500 10g 2450  3300  5150
float Angle_Standard=3600,Angle_Min=-9750,Angle_Max=2450;//重心 
double fGyroscopeAngleIntegral=0, fDeltaValue, g_fCarAngle, g_fGyroscopeAngleSpeed, g_fGravityAngle;
float Angle_Car_Now,Angle_Car_Now_2,g_fAngleControlOut,g_fAngleControlOut_Last,g_fAngleandSpeed;

uint16 Time_Count=0,Start_Continue_Flag=0;
uint16 Stop_Check_Count=0,Charge_Stop_Car_Count=0;
extern int i,j,Flag_Right,Flag_Left;        //定义行变量i=0~60，列变量j=0~80
extern uint8 img[CAMERA_H][CAMERA_W];      //拓展定义图像存储函数img
       
/*

*/
/**************************速度环参数**************************/
float fPL,fPR,fDL,fDR,fIL,fIR;
float Speed_Left,Speed_Right,Speed_Right_Last,Speed_Left_Last,Speed_Left_Max;
float g_fSpeedDeltaL,g_fSpeedDeltaR,g_fSpeedDeltaPreL,g_fSpeedDeltaPreR;
float g_fSpeedControlIntegralL,g_fSpeedControlIntegralR;
float g_fSpeedControlOutOldL,g_fSpeedControlOutNewL,g_fSpeedControlOutOldR,g_fSpeedControlOutNewR; 
float g_fSpeedControlDeltaL,g_fSpeedControlDeltaR,g_fSpeedControlOutL,g_fSpeedControlOutR,g_fSpeedControlOutL_Max=-4000;
extern float fLeftVal,fRightVal;
uint16 Stop_Car=0;//车子角度过大停车
int32 Left_Motor,Right_Motor,Left_Motor_Last,Right_Motor_Last;
uint16 Angle_Caculate_Count=0,
       Angle_Output_Count=0,
       Speed_Control_Count=0,
       Speed_Output_Count=0,
       Dir_Count=0,
       Dir_Output_Count=0;
uint16 Stop_Count=0;
extern int16 Left_Black_Add,
              Left_Black_Average,
              Right_Black_Add,
              Right_Black_Average,
              Left_Black_Variance,
              Right_Black_Variance,
              All_Variance;
extern uint8 qipao ;
uint16 Stand_Inplace_Count=0,Outside=0;
uint16 Stop_Check_Begin_Count=0,Stand_Check_Count=0;

/**************************方向环参数**************************/
extern int32 Left_Black[60],Right_Black[60];
float g_fDirectionControlOut,Picture_detal_All;
extern float Picture_detal,Dir_detal,Picture_detal_Old,Picture_detal_detal;
extern int32 Direction_Control; 
int32 Direction_Control_Max=600,Direction_Control_Min=-600;//2.25 嘿嘿
uint16 Get_Picture_Count=0;
uint16 DIP_Number=0,Number_0=0,Number_1=0,Number_2=0,Number_3=0,Number_4=0,Number_5=0,Number_6=0,Number_7=0;
extern uint16 Road_Kind,ML_Black_Spot_count,Zebra_Mark,Zebra_Mark_Rea,Zebra_Mark_Rea_Timer,Zebra_Mark_Rea_Timer_Set;
extern int16 Right_Black_sixty_lose, //60
              Left_Black_sixty_lose,
              Right_Black_fourtyfive_lose,//45
              Left_Black_fourtyfive_lose,
              Right_Black_thrity_lose,//30
              Left_Black_thrity_lose,
              Right_Black_fiveteen_lose,//15
              Left_Black_fiveteen_lose,
              Right_Black_zero_lose,//0
              Left_Black_zero_lose;
extern uint16 Finish_Check_Count,Jump_Change_Point_Count;
int8 High_Road_Left=0,High_Road_Right=0,High_Road_Left_i,High_Road_Left_j,
     High_Road_Right_i,High_Road_Right_j,High_Road_Flag,High_Road_Time=0;
extern int8  LB_Lose_Count,RB_Lose_Count;
extern uint16 podao,Fuck_Up_Mark;
extern int16 Block_Right,Block_Left;
extern int16 Ramp_Flag,Ramp_Timer,Ramp_Timer_Set,Ramp_Next_Delay;//坡道
extern int16 Detal_Column,FS_Spd,Po_B,O_Add;
extern float Dir_Spd_Fuzzy_constant;
/**************************电机参数**************************/
int32 g_dLeftMotorPulseSigma,	  //编码器左轮脉冲计数
      g_dRightMotorPulseSigma;  //编码器右轮脉冲计数
float Motor_Left_Out,
      Motor_Right_Out
      ;
#define MOTOR_DEAD_VAL_L     0//20  //死区电压
#define MOTOR_DEAD_VAL_R     0//20

/**************************数据采集和发送数组**************************/
extern uint8 bit8_data[6];               //存低八位的陀螺仪或加速度值
extern int16 bit16_data16[3];           //存放十六位的陀螺仪或加速度值
extern unsigned short send_data[8];
extern uint16 count;


/*
测时间：开始：pit_time_start(PIT1);
        结束：count= pit_time_get_us(PIT1);
*/
/*
函数名称：采集加速度和陀螺仪的位置信息 
最后修改时间：2016-11-01
作者：厦理创新实验室智能车队-暗涌
参考：龙邱九轴
程序时间：采集6个数据：4.011ms，采集2个数据：4.010ms
有疑问致电邮：13950986716，lczq@vip.qq.com
**************************************************************************************************/
void Get_Acc_And_Gyro()
{
  IIC_Read_Acc_16bit(SlaveAddress8700,bit8_data,bit16_data16);//I2C读取加速度值
  //Acc_X= bit16_data16[0];
  //Acc_Y= bit16_data16[1];
  Acc_Z= bit16_data16[2];
  IIC_Read_Gyro_16bit(SlaveAddress2100,bit8_data,bit16_data16);//I2C读取陀螺仪值
  Gyro_X= bit16_data16[0];
  Gyro_Z= bit16_data16[1];
  //Gyro_Y= bit16_data16[2];
  
  /*
  if(Acc_Z>32767)
    Acc_Z=32767;
  else if(Acc_Z<-32767)
    Acc_Z=-32767;
  */
  //Acc_Z=(Acc_Z>32767?32767:Acc_Z);
  //Acc_Z=(Acc_Z<-32767?-32767:Acc_Z);
  //Gyro_X=(Gyro_X>32767?32767:Gyro_X);
  //Gyro_X=(Gyro_X<-32767?-32767:Gyro_X);
  //Gyro_Z=(Gyro_Z>32767?32767:Gyro_Z);
  //Gyro_Z=(Gyro_Z<-32767?-32767:Gyro_Z);
}

/*
函数名称：数据发送至上位机
最后修改时间：2016-11-26
作者：厦理创新实验室智能车队-暗涌
参考：匿名
程序时间：1.81ms
有疑问致电邮：13950986716，lczq@vip.qq.com
**************************************************************************************************/
void Send_Data_Up()
{
  send_data[0]=(int16)fGyroscopeAngleIntegral;//fGyroscopeAngleIntegral;//Acc_X;
  send_data[1]=Acc_Z;//Gravity_Angle_Integral;//Stand_Speed;//g_fGravityAngle;//Acc_Y;
  send_data[2]=Gyro_Z;//-250--1000//Gravity_Angle_Error;//Gravity_Angle_Error_Error;
  send_data[3]=Gyro_Test;//fGyroscopeAngleIntegral;//Gyro_X;//g_fGyroscopeAngleSpeed;
  //send_data[4]=Angle_Car_Now_2;//Angle_Now;//Gyro_Y;
  //send_data[5]=Angle_Car_Now;//fLeftVal,fRightVal
  /*数据记录
   设定2200 最小速度差值-1050*/
  Data_Send(UART0,send_data);//通过串口输出数据到上位机显示波形
}

/*
函数名称：直立角度计算 
最后修改时间：2016-11-12
作者：厦理创新实验室智能车队-暗涌
参考：清华滤波
调试记录：161023-：归一化没做好。
有疑问致电邮：13950986716，lczq@vip.qq.com

**************************************************************************************************/
void AngleCalculate(float Acc_Z,float Gyro_Y)
{
  
  fDeltaValue=0;
  g_fGravityAngle = (Acc_Z - GRAVITY_OFFSET) * GRAVITY_ANGLE_RATIO;  //加速度
  g_fGyroscopeAngleSpeed = (Gyro_Y/*Gyro_Z */- GYROSCOPE_OFFSET ) * GYROSCOPE_ANGLE_RATIO;   //gyro角速度
  g_fCarAngle = fGyroscopeAngleIntegral;
  fDeltaValue =(g_fGravityAngle - g_fCarAngle) / GRAVITY_ADJUST_TIME_CONSTANT;      //补偿量
    // 加速度偏差值 =（采回来的加速度  - 上次融合后的加速度）/2
  fGyroscopeAngleIntegral +=(g_fGyroscopeAngleSpeed + fDeltaValue) / GYROSCOPE_ANGLE_SIGMA_FREQUENCY;
  Angle_Car_Now=fGyroscopeAngleIntegral/180;
  //为什么采回来的值是平缓的，消抖在哪？管TM的，我有互补滤波了
}

/*
函数名称：直立角度计算 
最后修改时间：2016-11-26
作者：厦理创新实验室-暗涌队
参考：MIT互补滤波
调试：测试成功，还需调参数
数据记录：见dox
有疑问致电邮：13950986716，lczq@vip.qq.com
**************************************************************************************************/
void Balance_Filter(float Acc_Z,float Gyro_Z)
{
  g_fGravityAngle =        (Acc_Z - GRAVITY_OFFSET) * GRAVITY_ANGLE_RATIO;  
  //滤出零偏值后的加速度 =(原始加速度Z轴值-零偏值) *  加速度比例
  
  g_fGyroscopeAngleSpeed = (/*Gyro_Y*/Gyro_Z - GYROSCOPE_OFFSET ) * GYROSCOPE_ANGLE_RATIO; 
  //滤出零偏值后的gyro角速度=(原始加速度Z轴值-零偏值)             * 陀螺仪比例
  
  fGyroscopeAngleIntegral=Filter_Pass_Ratio*(fGyroscopeAngleIntegral+g_fGyroscopeAngleSpeed*One_Pro_Period)
                          +(1-Filter_Pass_Ratio)*g_fGravityAngle;//互补滤波核心
  //融合角度            =滤波器比例*(融合角度+角速度*一个程序周期)+(1-滤波器比例)*加速度
  
  //0.996  0.98  0.996
   if(fGyroscopeAngleIntegral>17000)fGyroscopeAngleIntegral=17000;//限幅
   if(fGyroscopeAngleIntegral<-17000)fGyroscopeAngleIntegral=-17000;//限幅
  Angle_Car_Now_2=asin(fGyroscopeAngleIntegral/17000)/3.1415*180;//+19.3;
  Angle_Car_Now=fGyroscopeAngleIntegral/180;
  /*if(Angle_Car_Now>90 || Angle_Car_Now<50)
  {
    Stop_Car=1;
  }*/
  
  Gyro_Test+=Gyro_Z - GYROSCOPE_OFFSET;
}
/*
函数名称：直立角度计算 
最后修改时间：2016-11-11
作者：厦理创新实验室智能车队-暗涌
参考：网传互补滤波,TMB太不可靠了我艹，只能靠自己
有疑问致电邮：13950986716，lczq@vip.qq.com
**************************************************************************************************/
void Angle_Control()//没试验成功
{
  
    Angle_Acc=180/PI*(float)(atan2(Acc_Z,Acc_Y));  //求Acc_z/Acc_y的反正切
    /*法二：Angle_Acc=180/PI*(float)(asin(Acc_z/8000));  求Acc_z/8000的反正弦*/
    
    /*if(Angle_Acc>90)
      Angle_Acc=90;
    if(Angle_Acc<-90)
      Angle_Acc=-90;*/
    
    int16 Gyro_X_Zero=28;//静止时X轴角速度零偏值
    float Gyro_Speed_X_Ratio=0.0002;//陀螺仪积分系数，调整使得陀螺仪角度积分趋近于加速度计算的角度值
    
    Car_Gyro_X=(float)(Gyro_X*1.0-Gyro_X_Zero);//X轴陀螺仪角速度-零偏值
    Gyro_Speed_X=(float)(Car_Gyro_X*Gyro_Speed_X_Ratio);//用于积分的陀螺仪速度=X轴角速度*积分系数
    Angle_Gyro=(float)(Angle_Gyro+Gyro_Speed_X);
    Angle_Mix=Angle_Gyro+(Angle_Acc-Angle_Gyro)*0.425;//互补滤波
    
}



/*
函数名称：速度获取 
最后修改时间：2016-11-26
作者：厦理创新实验室智能车队-暗涌
参考：未测试
有疑问致电邮：13950986716，lczq@vip.qq.com
**************************************************************************************************/

void Get_Speed()
{
  
  //Left_Motor = 0 , Right_Motor = 0;
  Left_Motor_Last=Left_Motor;
  Right_Motor_Last=Right_Motor;
  
  Right_Motor=-ftm_quad_get(FTM1);
  Left_Motor=ftm_quad_get(FTM2); //通过编码器获取速度
  
  
  ftm_quad_clean(FTM1);     //清正交解码脉冲数
  ftm_quad_clean(FTM2);
  
  /*
  if(g_fCarSpeedSet_Start>0)
  {
    if(Left_Motor-Left_Motor_Last>20)
      Left_Motor=Left_Motor_Last+20;
    else if(Left_Motor-Left_Motor_Last<-20)
      Left_Motor=Left_Motor_Last-20;
    if(Right_Motor-Right_Motor_Last>20)
      Right_Motor=Right_Motor_Last+20;
    else if(Right_Motor-Right_Motor_Last<-20)
      Right_Motor=Right_Motor_Last-20;
  }
  */
  
  g_dLeftMotorPulseSigma+=Left_Motor;//将得到的速度给电机输出计算
  g_dRightMotorPulseSigma+=Right_Motor;
    
}

/*
函数名称：直立速度外环计算 
最后修改时间：2016-11-26
作者：厦理创新实验室智能车队-暗涌
参考：pixhawk飞控串级PID+施泓
有疑问致电邮：13950986716，lczq@vip.qq.com
**************************************************************************************************/
void Angle_Out_Loop_Control()
{
  Gravity_Angle_Error=fGyroscopeAngleIntegral;
  
  if(Gravity_Angle_Integral_Start_Time>200)
  {
    Gravity_Angle_Integral_Start_Mark=1;//积分开始标志位
  }
  else
  {
    Gravity_Angle_Integral_Start_Time++;//积分开始标志计时位
  }
  
  if(Gravity_Angle_Integral_Start_Mark)
  {
    /*
    if(abs((uint32)Gravity_Angle_Error)>200)
    {
      Gravity_Angle_Index=0;
    }
    else
    {
      Gravity_Angle_Index=1;
    }
    */
    Gravity_Angle_Index=1;
    Gravity_Angle_Integral+=Gravity_Angle_Error;//积分项，还需积分分离和抗积分饱和、变积分处理！！！！暂放 后面有时间拓展
  }
  
  Gravity_Angle_I=Gravity_Angle_Integral*I_Angle;
    
    if(Gravity_Angle_I<-80)//积分限幅 1%
      Gravity_Angle_I=-80;
    if(Gravity_Angle_I>80)//积分限幅 1%
      Gravity_Angle_I=80;
    
  
  //Gravity_Angle_Error_Error=Gravity_Angle_Error-Gravity_Angle_Error_Last;//加速度偏差的偏差
  //Gravity_Angle_Error_Last=Gravity_Angle_Error;//上个偏差
  
  Stand_Speed_Out_Out=Gravity_Angle_Error*P_Angle
                    + Gravity_Angle_Index*Gravity_Angle_I
                    + g_fGyroscopeAngleSpeed*D_Angle;
  //保持直立的外环角度输出，更新速度小于内环，1:5
  
}

/*
函数名称：直立速度内环计算 
最后修改时间：2016-11-26
作者：厦理创新实验室智能车队-暗涌
参考：pixhawk飞控串级PID+施泓
调试：输出是外环为主，内环相当于D的作用
有疑问致电邮：13950986716，lczq@vip.qq.com
**************************************************************************************************/
void Angle_In_Loop_Control()
{
  
  Angle_Error=(Angle_Standard-Stand_Speed_Out_Out);//正常跑的时候此值为正 *P_Angle;//角度差//不用归一，直接求误差，精度更大
  
  Angle_Speed_Expect=Angle_Error/180/One_Pro_Period;//短时间内相当于求导，角度的变化/时间=角速度
  /*
  if(Angle_Speed_Expect>500)
  {
    Angle_Speed_Expect=500;
  }
  if(Angle_Speed_Expect<-500)
  {
    Angle_Speed_Expect=-500;
  }
  */
  //由角度计算的角度差与陀螺仪求差。，静止时相差必须为0才准确。
  
  Angle_Speed_Error=Angle_Speed_Expect-Gyro_Z;//g_fGyroscopeAngleSpeed;//当前角速度偏差
  
  if(Angle_Speed_Integral_Start_Time>1000)
  {
    Angle_Speed_Integral_Start_Mark=1;//积分开始标志位
  }
  else
  {
    Angle_Speed_Integral_Start_Time++;//积分开始标志计时位    
  }
  
  if(Angle_Speed_Integral_Start_Mark)//进行积分运算
  {
    if(abs((uint32)Angle_Speed_Error)>200)
    {
      Angle_Speed_Index=0;
    }
    else
    {
      Angle_Speed_Index=1;
    }
    Angle_Speed_Integral+=Angle_Speed_Error;//积分项，还需积分分离和抗积分饱和、变积分处理！！！！暂放 后面有时间拓展
    
    if(Angle_Speed_Integral<-5000)//积分限幅
      Angle_Speed_Integral=-5000;
    if(Angle_Speed_Integral>5000)//积分限幅
      Angle_Speed_Integral=5000;
  }
  Angle_Speed_Error_Error=Angle_Speed_Error-Angle_Speed_Error_Last;//角速度偏差的偏差
  Angle_Speed_Error_Last=Angle_Speed_Error;//上个偏差
  
  Stand_Speed_Part_In_Old=Stand_Speed_Part_In;
  
  Stand_Speed_Part_In=Angle_Speed_Error*P_Angle_Speed_Now 
                   + Angle_Speed_Integral*I_Angle_Speed*Angle_Speed_Index 
                    + Angle_Speed_Error_Error*D_Angle_Speed;
  /*
  Stand_Speed_Part_In=Angle_Error*P_Angle_Speed 
                    + Angle_Speed_Integral*I_Angle_Speed*Angle_Speed_Index 
                    + Angle_Speed_Error*D_Angle_Speed;
  */
  //直立速度内环输出 5倍于外环 
  
  Stand_Speed_Part_In_Now=Stand_Speed_Part_In;//(Stand_Speed_Part_In_Old+Stand_Speed_Part_In)/2;
}

/*
函数名称：直立速度总计算 
最后修改时间：2016-11-26
作者：厦理创新实验室智能车队-暗涌
参考：手把手教你套路——老周
有疑问致电邮：13950986716，lczq@vip.qq.com
**************************************************************************************************/
void Angle_All_Control()
{
  Time_Count++;
  if(Time_Count==5)
  {
    Angle_Out_Loop_Control();//角度外环 5个周期更新一次
    Time_Count=0;
  }
  Angle_In_Loop_Control();//角速度内环 1个周期更新一次
  
  Stand_Speed=Stand_Speed_Part_In_Now;//+Stand_Speed_Part1;//直立速度包含两部分，角度外环和角速度内环
  g_fAngleControlOut_Last=g_fAngleControlOut;
  g_fAngleControlOut=Stand_Speed;
  //g_fAngleControlOut=(g_fAngleControlOut-g_fAngleControlOut_Last>300?g_fAngleControlOut_Last+300:g_fAngleControlOut);
  //g_fAngleControlOut=(g_fAngleControlOut-g_fAngleControlOut_Last<-300?g_fAngleControlOut_Last-300:g_fAngleControlOut);
}


/*
函数名称：速度环 
最后修改时间：2016-11-26
作者：厦理创新实验室智能车队-暗涌
参考：振林、施泓、自福
有疑问致电邮：13950986716，lczq@vip.qq.com
**************************************************************************************************/
void Speed_Loop_Control()
{
  //Speed_Left=g_dLeftMotorPulseSigma/Speed_Output_Count;
  //Speed_Right=g_dRightMotorPulseSigma/Speed_Output_Count;
  Speed_Right_Last=Speed_Right;
  Speed_Left_Last=Speed_Left;
  
  //g_dLeftMotorPulseSigma_Last=g_dLeftMotorPulseSigma;
  //g_dRightMotorPulseSigma_Last=g_dRightMotorPulseSigma;
  
  Speed_Right=(g_dLeftMotorPulseSigma+g_dRightMotorPulseSigma)/2;///Speed_Output_Count;
  Speed_Left=(g_dLeftMotorPulseSigma+g_dRightMotorPulseSigma)/2;///Speed_Output_Count;
  
  g_dLeftMotorPulseSigma=0,g_dRightMotorPulseSigma=0;
  
  if(Speed_Left-Speed_Left_Last>250 && g_fCarSpeedSet)Speed_Left=Speed_Left_Last+250;//速度反馈的变化幅度限幅
  if(Speed_Left-Speed_Left_Last<-250 && g_fCarSpeedSet)Speed_Left=Speed_Left_Last-250;//速度反馈的变化幅度限幅
  if(Speed_Right-Speed_Right_Last>250 && g_fCarSpeedSet)Speed_Right=Speed_Right_Last+250;//速度反馈的变化幅度限幅
  if(Speed_Right-Speed_Right_Last<-250 && g_fCarSpeedSet)Speed_Right=Speed_Right_Last-250;//速度反馈的变化幅度限幅
  /*
  if(Speed_Left>1050)
  {
    gpio_set (PTA14,0);
    
  }
  else
    gpio_set (PTA14,1);
  */
  /* 
  if(g_fCarSpeedSet_Start>0)
  {
    Speed_Right=(Speed_Right>1.2*g_fCarSpeedSet_Start?1.2*g_fCarSpeedSet_Start:Speed_Right);//速度反馈量限幅
    Speed_Left=(Speed_Left>1.2*g_fCarSpeedSet_Start?1.2*g_fCarSpeedSet_Start:Speed_Left);//速度反馈量限幅
  }
  */
  //g_fSpeedDeltaL=(g_fSpeedDeltaL>-1200?-1200:g_fSpeedDeltaL);//速度最大偏差限幅
  //g_fSpeedDeltaR=(g_fSpeedDeltaR>-1200?-1200:g_fSpeedDeltaR);//速度最大偏差限幅
  
  g_fSpeedDeltaPreL = g_fSpeedDeltaL;//上一个速度detail
  g_fSpeedDeltaPreR = g_fSpeedDeltaR;//上一个速度detail
  
  

  if(Zebra_Mark_Rea_Timer>0 && Zebra_Mark_Rea_Timer<80)//冲刺重点后紧急停车
  {
    g_fCarSpeedSet=0;
  }
  
  
  //g_fSpeedDeltaL = Speed_Left  - g_fCarSpeedSet  ;//本次速度detail
  //g_fSpeedDeltaR = Speed_Right - g_fCarSpeedSet  ;//本次速度detail
  
  g_fSpeedDeltaL = g_fCarSpeedSet - Speed_Left;//本次速度detail
  g_fSpeedDeltaR = g_fCarSpeedSet - Speed_Right ;//本次速度detail
  
  if(Speed_Left>Speed_Left_Max)
  {
    Speed_Left_Max=Speed_Left;
  }
  
  //g_fSpeedDeltaL = g_fCarSpeedSet-Speed_Left;//本次速度detail
  //g_fSpeedDeltaR = g_fCarSpeedSet-Speed_Right;//本次速度detail
  
  //if(g_fSpeedDeltaL-g_fSpeedDeltaPreL>150 && g_fCarSpeedSet)g_fSpeedDeltaL=g_fSpeedDeltaPreL+150;//每周期最大速度误差限幅
  //if(g_fSpeedDeltaL-g_fSpeedDeltaPreL<-150 && g_fCarSpeedSet)g_fSpeedDeltaL=g_fSpeedDeltaPreL-150;//每周期最大速度误差限幅  
  //if(g_fSpeedDeltaR-g_fSpeedDeltaPreR>150 && g_fCarSpeedSet)g_fSpeedDeltaR=g_fSpeedDeltaPreR+150;//每周期最大速度误差限幅
  //if(g_fSpeedDeltaR-g_fSpeedDeltaPreR<-150 && g_fCarSpeedSet)g_fSpeedDeltaR=g_fSpeedDeltaPreR-150;//每周期最大速度误差限幅
  
  fPL = g_fSpeedDeltaL * g_fSpeedControlP;//计算速度P
  fPR = g_fSpeedDeltaR * g_fSpeedControlP;
  fDL = (g_fSpeedDeltaL - g_fSpeedDeltaPreL) * g_fSpeedControlD;//计算速度D
  fDR = (g_fSpeedDeltaR - g_fSpeedDeltaPreR) * g_fSpeedControlD;
  
  //if(Speed_Right && Speed_Left)
  {
  fIL = fIL+g_fSpeedDeltaL * g_fSpeedControlI;//计算左轮速度I 对于I有必要积分分离
  if(fIL > 100)
      fIL = 100;
  if(fIL < -100)
      fIL = -100;
  
  g_fSpeedControlIntegralL = (fIL);//g_fSpeedControlIntegralL + 
   
  fIR =fIR+ g_fSpeedDeltaR * g_fSpeedControlI;//计算右轮速度I
  if(fIR > 100)
     fIR = 100;
  if(fIR < -100)
     fIR = -100;
  g_fSpeedControlIntegralR = (fIR);//g_fSpeedControlIntegralR + 
  }
  /*
  g_fSpeedControlOutOldL = g_fSpeedControlOutNewL;
  g_fSpeedControlOutOldR = g_fSpeedControlOutNewR;
  g_fSpeedControlOutNewL = fPL + g_fSpeedControlIntegralL +fDL;
  g_fSpeedControlOutNewR = fPR + g_fSpeedControlIntegralR + fDR;
  
  g_fSpeedControlDeltaL  = (g_fSpeedControlOutNewL - g_fSpeedControlOutOldL);///g_nSpeedControlCount;//平滑输出
  g_fSpeedControlDeltaR  = (g_fSpeedControlOutNewR - g_fSpeedControlOutOldR);///g_nSpeedControlCount;//
  
  g_fSpeedControlOutOldL = (g_fSpeedControlOutOldL + g_fSpeedControlDeltaL);
  g_fSpeedControlOutOldR = (g_fSpeedControlOutOldR + g_fSpeedControlDeltaR);
  
  g_fSpeedControlOutL = g_fSpeedControlOutOldL;
  g_fSpeedControlOutR = g_fSpeedControlOutOldR;*/
  
  /*
  if(g_fSpeedControlOutL>500)
    g_fSpeedControlOutL=500;
  if(g_fSpeedControlOutL<-500)
    g_fSpeedControlOutL=-500;
  if(g_fSpeedControlOutR>500)
    g_fSpeedControlOutR=500;
  if(g_fSpeedControlOutR<-500)
    g_fSpeedControlOutR=-500;
  */
  
  g_fSpeedControlOutOldL = g_fSpeedControlOutNewL;
  g_fSpeedControlOutOldR = g_fSpeedControlOutNewR;
  g_fSpeedControlOutNewL = fPL + g_fSpeedControlIntegralL +fDL;
  g_fSpeedControlOutNewR = fPR + g_fSpeedControlIntegralR + fDR;
  
  g_fSpeedControlDeltaL  = (g_fSpeedControlOutNewL - g_fSpeedControlOutOldL);//这次与上次速度输出PWM之差
  g_fSpeedControlDeltaR  = (g_fSpeedControlOutNewR - g_fSpeedControlOutOldR);//这次与上次速度输出PWM之差
  
}
void Speed_Loop_Output()
{
  //g_fSpeedControlOutOldL =g_fSpeedControlDeltaL*Speed_Output_Count/10;
  //g_fSpeedControlOutOldR =g_fSpeedControlDeltaR*Speed_Output_Count/10;
  
  g_fSpeedControlOutL = g_fSpeedControlOutOldL+g_fSpeedControlDeltaL*Speed_Output_Count/10;
  g_fSpeedControlOutR = g_fSpeedControlOutOldR+g_fSpeedControlDeltaR*Speed_Output_Count/10;
  /*
  if(g_fSpeedControlOutL>-1300 && abs(Dir_detal)<10 && g_fCarSpeedSet!=0)
    g_fSpeedControlOutL=-1300;
  if(g_fSpeedControlOutR>-1300 && abs(Dir_detal)<10 && g_fCarSpeedSet!=0)
    g_fSpeedControlOutR=-1300;
  */
  /*
  if(abs(g_fSpeedControlOutL)==1215)
    qipao=1;
  else
    qipao=0;
  */
  if(g_fSpeedControlOutL>g_fSpeedControlOutL_Max && g_fSpeedControlOutL!=0 && Stop_Check_Begin_Count>100)//记录输出速度pwm最大
  {
    g_fSpeedControlOutL_Max=g_fSpeedControlOutL;
  }
  //g_fSpeedControlOutL=(g_fSpeedControlOutL>400?400:g_fSpeedControlOutL);//速度输出限幅 有用？！
  //g_fSpeedControlOutR=(g_fSpeedControlOutR>400?400:g_fSpeedControlOutR);//速度输出限幅 没卵用吧 草拟日本鬼子
}
/*
函数名称：方向环 
最后修改时间：2016-11-26
作者：厦理创新实验室智能车队-暗涌
参考：《林青春的调车日常》
有疑问致电邮：13950986716，lczq@vip.qq.com
**************************************************************************************************/
void Dir_Loop_Control()
{
  //pit_time_start  (PIT1);
  Get_Middle_Line();
  //count=pit_time_get_us(PIT1);
  //Finish_Check();//起跑线检测
  Direction_Detal_Caculate();//计算出 平均每行误差Picture_detal
  //Picture_detal_All=Picture_detal_All+Picture_detal;
}

/*
函数名称：方向环 
最后修改时间：2016-11-26
作者：厦理创新实验室智能车队-暗涌
参考：《林青春的调车日常》
修改：可以用陀螺仪为KD
有疑问致电邮：13950986716，lczq@vip.qq.com
**************************************************************************************************/
void Dir_Loop_Output()
{
  Dir_detal=Picture_detal;
  Picture_detal_detal=Picture_detal-Picture_detal_Old; //偏差的偏差
  Picture_detal_Old=Picture_detal; //现在偏差存起来，以便计算D
  //Part_Road();//分段处理 包含坡道
  if(Number_3)
  {
    Fuzzy_Apply(Dir_detal,Picture_detal_detal);//模糊PD
  }
  
  //方案1：用偏差的偏差做D
  //Direction_Control_Out=(Dir_detal*P_Direction_Control_now)+((Picture_detal_detal)*D_Direction_Control_now);
  
  //方案2：用陀螺仪X轴做D 
    /*
  if(ML_Black_Spot_count<8 && Fuck_Up_Mark)
  {
    P_Direction_Control_now=P_Direction_Control_now*0.1;
  }
    */
  Direction_Control_Out_Last=Direction_Control_Out;
  Direction_Control_Out=(Dir_detal*P_Direction_Control_now)+((Gyro_X-50)*D_Direction_Control_now);
  /*
  if(Direction_Control_Out>Direction_Control_Max)    //速度限制
    Direction_Control_Out=Direction_Control_Max;
  if(Direction_Control_Out<Direction_Control_Min)    //速度限制
    Direction_Control_Out=Direction_Control_Min;
  */
  Picture_detal=0;    //一帧图像偏差清零，继续算下一个。
  Direction_Control_Out=(Direction_Control_Out-Direction_Control_Out_Last>100?Direction_Control_Out=Direction_Control_Out_Last+100:Direction_Control_Out);
  Direction_Control_Out=(Direction_Control_Out-Direction_Control_Out_Last<-100?Direction_Control_Out=Direction_Control_Out_Last-100:Direction_Control_Out);
  //100待定 要采集数据分析
  g_fDirectionControlOut=Direction_Control_Out;
  g_fDirectionControlOut=(g_fDirectionControlOut>300?300:g_fDirectionControlOut);
  g_fDirectionControlOut=(g_fDirectionControlOut<-300?-300:g_fDirectionControlOut);//400待定

}

/*
函数名称：最终速度输出 
最后修改时间：2016-11-26
作者：厦理创新实验室智能车队-暗涌
参考：振林、施泓、自福
有疑问致电邮：13950986716，lczq@vip.qq.com
**************************************************************************************************/
void MotorOutput(void)
{
  //if(g_fDirectionControlOut)
  
  if(Zebra_Mark_Rea_Timer>0)
  {
    
    Zebra_Mark_Rea_Timer--;
    if(Zebra_Mark_Rea_Timer<=80)
    {
      g_fAngleControlOut=0;
    }
    
    if(Zebra_Mark_Rea_Timer<=70)
    {
      g_fSpeedControlOutL=0;
      g_fSpeedControlOutR=0;
    }
    
    if(Zebra_Mark_Rea_Timer==50)
    {
      //g_fSpeedControlOutL=0;
      //g_fSpeedControlOutR=0;
      qipao=1;
    }
  }
  
  g_fAngleandSpeed=g_fAngleControlOut- g_fSpeedControlOutL;
  
  //fLeft=g_fSpeedControlOutL;
  //fRight=g_fSpeedControlOutR;
  
  //gpio_set (PTA15,1);
  if(Ramp_Flag)
  {
    //gpio_set (PTA15,1);
    Ramp_Timer--;
    g_fAngleControlOut=(g_fAngleControlOut>3500?3500:g_fAngleControlOut);
    g_fAngleControlOut=(g_fAngleControlOut<1000?1000:g_fAngleControlOut);
    
    g_fSpeedControlOutL=(g_fSpeedControlOutL>4000?4000:g_fSpeedControlOutL);
    g_fSpeedControlOutL=(g_fSpeedControlOutL<1000?1000:g_fSpeedControlOutL);
    
    g_fAngleandSpeed=g_fAngleControlOut- g_fSpeedControlOutL;
    
    g_fAngleandSpeed=(g_fAngleandSpeed>350?350:g_fAngleandSpeed);
    g_fAngleandSpeed=(g_fAngleandSpeed<-350?-350:g_fAngleandSpeed);
  }

  if(Ramp_Timer==1)
  {
    //gpio_set (PTA15,0);
    //P_Angle_Speed_Now=1.0*P_Angle_Speed_Init;
    //g_fSpeedControlP=1.0*g_fSpeedControlP_Init;
    Ramp_Next_Delay=200;
    Ramp_Flag=0;
    Ramp_Timer=Ramp_Timer_Set;
  }
  
  if(Ramp_Next_Delay>1)
  {
    if(Ramp_Next_Delay>50)
    {
      g_fAngleandSpeed=(g_fAngleandSpeed-350)*(200-Ramp_Next_Delay)/150;
    }
    Ramp_Next_Delay--;
  }
  
  //fLeft  = g_fAngleControlOut;//- g_fSpeedControlOutL+ g_fDirectionControlOut;
  //fRight = g_fAngleControlOut;//- g_fSpeedControlOutR- g_fDirectionControlOut;
  
  //fLeft  = g_fAngleandSpeed + g_fDirectionControlOut;
  //fRight = g_fAngleandSpeed - g_fDirectionControlOut;
  
  if(g_fDirectionControlOut<0)
  {
    fLeft  = g_fAngleandSpeed;//+ 1*g_fDirectionControlOut;
    fRight = g_fAngleandSpeed- 2*g_fDirectionControlOut; 
  }
  else if(g_fDirectionControlOut>0)
  {
    fLeft  = g_fAngleandSpeed+ 2*g_fDirectionControlOut;   
    fRight = g_fAngleandSpeed;//- 1*g_fDirectionControlOut; 
  }
  
  //左边负偏差 右边正偏差
  /*
  if(g_fDirectionControlOut<0)
  {
    fLeft  = g_fAngleControlOut- g_fSpeedControlOutL;//+ 1*g_fDirectionControlOut;
    fRight = g_fAngleControlOut- g_fSpeedControlOutR- 2*g_fDirectionControlOut; 
  }
  else if(g_fDirectionControlOut>0)
  {
    fLeft  = g_fAngleControlOut- g_fSpeedControlOutL+ 2*g_fDirectionControlOut;   
    fRight = g_fAngleControlOut- g_fSpeedControlOutR;//- 1*g_fDirectionControlOut; 
  }
  */
  
  fLeftVal=fLeft;
  fRightVal=fRight;
  
  if(fLeftVal > 900)  fLeftVal = 900;
  if(fLeftVal < -900)  fLeftVal = -900;
  if(fRightVal > 900)  fRightVal = 900;
  if(fRightVal < -900)  fRightVal = -900;
  
  if(Stop_Car==1)
  {
    fLeftVal=0;
    fRightVal=0;
    while(1);
  }
  
  SetMotorVoltage(fLeftVal, fRightVal);
}

/*
函数名称：速度通过PWM输出 
最后修改时间：2016-11-26
作者：厦理创新实验室智能车队-暗涌
参考：振林、施泓、自福
有疑问致电邮：13950986716，lczq@vip.qq.com
**************************************************************************************************/
void SetMotorVoltage(float fLeftVal, float fRightVal)
{
  if(fLeftVal > 0)//左轮
    {
      Motor_Left_Out=(fLeftVal + MOTOR_DEAD_VAL_L );
      ftm_pwm_duty(FTM0,FTM_CH3,0);
      ftm_pwm_duty(FTM0,FTM_CH6,(uint32)(Motor_Left_Out)); 
    }
   else
    {
      Motor_Left_Out=(fLeftVal - MOTOR_DEAD_VAL_L );
      ftm_pwm_duty(FTM0,FTM_CH6,0);
      ftm_pwm_duty(FTM0,FTM_CH3,(uint32)(-Motor_Left_Out));  
    }
  if(fRightVal > 0)//右轮
    {
      Motor_Right_Out=(fRightVal + MOTOR_DEAD_VAL_L );
      ftm_pwm_duty(FTM0,FTM_CH4,0);
      ftm_pwm_duty(FTM0,FTM_CH5,(uint32)(Motor_Right_Out)); 
    }
   else
    {
      Motor_Right_Out=(fRightVal - MOTOR_DEAD_VAL_L );
      ftm_pwm_duty(FTM0,FTM_CH5,0);
      ftm_pwm_duty(FTM0,FTM_CH4,(uint32)(-Motor_Right_Out));  
    }
}

/*
函数名称：紧急停车（防止磨齿轮
最后修改时间：20161210
作者：厦理创新实验室智能车队-暗涌
有疑问致电邮：13950986716，lczq@vip.qq.com
未解决问题：
**************************************************************************************************/

void Stop_Check(void)
{
  if(Stop_Check_Begin_Count<1000)
  {
    Stop_Check_Begin_Count++;
  }
  
  else if(Stop_Check_Begin_Count>500)
  {
  if(/*((fGyroscopeAngleIntegral>7000 && fGyroscopeAngleIntegral<9500) || 
     (fGyroscopeAngleIntegral<-7000 && fGyroscopeAngleIntegral>-9000)) //MIN -7390 MAX 8900 BLAN 5020
      //||(Jump_Change_Point_Count>7 && Jump_Change_Point_Count<12) 
     ||*/ //Speed_Left>1.2*g_fCarSpeedSet_Start || Speed_Right>1.2*g_fCarSpeedSet_Start||
       (!Zebra_Mark &&
        img[58][38]==0x00 && img[58][39]==0x00 && img[58][40]==0x00 
          && img[58][41]==0x00 && img[58][41]==0x00 
            && img[54][38]==0x00 && img[54][39]==0x00 && img[54][40]==0x00 
              && img[54][41]==0x00 && img[54][41]==0x00
                && img[56][38]==0x00 && img[56][39]==0x00 && img[56][40]==0x00 
                  && img[56][41]==0x00 && img[56][41]==0x00)
     )
  {
    
    Outside++;
    if(Outside==10)
      qipao=1;
  }
  else
    {
      //gpio_set (PTC12, 0);
    }
  if(Number_1==1 && Zebra_Mark /*Jump_Change_Point_Count>7 && Jump_Change_Point_Count<12*/)//斑马线停车
  {
    Zebra_Mark_Rea=1;
    Zebra_Mark_Rea_Timer=Zebra_Mark_Rea_Timer_Set;//检测到斑马线继续前进30个周期再停下车
    //qipao=1;
  }
  }
  
  /*
  if(Block_Left)
  {
    qipao=1;
  }
 */
  /*
  if(Road_Kind==3)
  {
    All_Start();
    qipao=1;
    gpio_set (PTD2, 0);
    gpio_set (PTD3, 0);
  }
  */
  /*
  if(fGyroscopeAngleIntegral>16000 || fGyroscopeAngleIntegral<10000)//通过角度判断车子
  {
    Stop_Check_Count++;
    if(Stop_Check_Count==50)
    {
      Stop_Check_Count=0;
      All_Start();
      qipao=1;
    }
  }
  
  if(g_dLeftMotorPulseSigma<50 || g_dRightMotorPulseSigma<50)
  {
    Stop_Check_Count++;
    if(Stop_Check_Count==50)
    {
      Stop_Check_Count=0;
      All_Start();
      qipao=1;
    }
  }
  */
  
}

void All_Start(void)
{
  Motor_Left_Out=0;
  Motor_Right_Out=0;
}

void Stand_Inplace(void)
{
  g_fCarSpeedSet=0;
}

/*
函数名称：发车前的站立2秒
最后修改时间：20161210
作者：厦理创新实验室智能车队-暗涌
有疑问致电邮：13950986716，lczq@vip.qq.com
未解决问题：
**************************************************************************************************/

float Begin_Stand_Check(void)
{
  if(Stand_Check_Count<100)
  {
    Stand_Check_Count++;
    //g_fCarSpeedSet_Go=0;
    return 0;
  }
  else
  {
    //g_fCarSpeedSet_Go=g_fCarSpeedSet;
    return g_fCarSpeedSet;
  }
}
/*
void Finish_Check(void)
{
  Jump_Change_Point_Count=0;
  if(Finish_Check_Count<500)
  {
    Finish_Check_Count++;
  }
  else
  {
    for(i=15;i>14;i--)
      for(j=5;j<75;j++)
      {
        if(img[i][j-1]==0xff && img[i][j]==0xff && img[i][j+1]==0x00 && img[i][j+2]==0x00)
        {
          Jump_Change_Point_Count++;
        }
      }
    if(Jump_Change_Point_Count>6 && Jump_Change_Point_Count<11)
    {
      //uint32 Guo_Yi=0;
      //while(Guo_Yi<200000)
      //{
      //  Guo_Yi++;
      //}
      DELAY_MS(1000);
      qipao=1;
    }
  }
}
*/


/*
函数名称：道路分段
最后修改时间：20161205
作者：厦理创新实验室智能车队-暗涌
有疑问致电邮：13950986716，lczq@vip.qq.com
未解决问题： 1、直道会摇晃啊老妹儿，不按照我要求啊，
**************************************************************************************************/


void Part_Road() //道路分段处理 有待细分 还有优先级控制
{
  //Caculate_Variance();
  /*
  if(All_Variance<400)
  {
    g_fCarSpeedSet=g_fCarSpeedSet+5;
    if(g_fCarSpeedSet>g_fCarSpeedSet_Max)
      g_fCarSpeedSet=g_fCarSpeedSet_Max;   
  }
  else
  {
    g_fCarSpeedSet=g_fCarSpeedSet_Stand;
    //if(g_fCarSpeedSet<g_fCarSpeedSet_Stand)
    //  g_fCarSpeedSet=g_fCarSpeedSet_Stand; 
  }
  if(g_fSpeedDeltaL>100 || g_fSpeedDeltaR>100)  //速度比设定值大
  {
    P_Direction_Control_now=P_Direction_Control+1;
  }
  else if(g_fSpeedDeltaL<100 || g_fSpeedDeltaR<100)//速度比设定值小了
  {
    P_Direction_Control_now=P_Direction_Control-1.5;
  }
  else
  {
    P_Direction_Control_now=P_Direction_Control;
  }
  
  
  // 450 直道入弯道 
  // 380 直道前面 50cm 入弯道
  // 
  if(All_Variance<450)//小弯道
  {
    P_Direction_Control_now=P_Direction_Control-1;   
  }
  if((g_fSpeedDeltaL>100 || g_fSpeedDeltaR>100) && (All_Variance>500))//过大弯且速度没达到
  {
    P_Direction_Control_now=P_Direction_Control-1;
  }
  if((g_fSpeedDeltaL>100 || g_fSpeedDeltaR>100) && (All_Variance>500) && (All_Variance<750))//
  {
    P_Direction_Control_now=P_Direction_Control_now+1;
  }
  */
  //g_fSpeedDeltaL = Speed_Left - g_fCarSpeedSet  ;//本次detail
  //g_fSpeedDeltaR = Speed_Right - g_fCarSpeedSet  ;
  
  /*
  //S或直道，速度没达到
  if(All_Variance<450 && g_fSpeedDeltaL<100 && g_fSpeedDeltaR<100) 
  {
    g_fCarSpeedSet+=5;

  }
  
  //S或直道，速度达到
  else if(All_Variance<450 && g_fSpeedDeltaL<50 && g_fSpeedDeltaR<50 && g_fSpeedDeltaL>-50 && g_fSpeedDeltaR>-50) 
  {
    //g_fCarSpeedSet+=5;
    P_Direction_Control_now=P_Direction_Control;

  }
  */
  //十字弯道内三个90°All_Variance能达到1000
  //500-900入左转90°
  //550-1000入右转90°
  //
  /*
  //即将入90°弯道，速度没达到
  if(All_Variance>550)//&& g_fSpeedDeltaL<100 && g_fSpeedDeltaR<100)
  {
    g_fCarSpeedSet-=10;
    P_Direction_Control_now=P_Direction_Control+0.5;
  }
  else
  {
    g_fCarSpeedSet=g_fCarSpeedSet_Stand;
    P_Direction_Control_now=P_Direction_Control;
  }
  */

  /*if( ML_Black_Spot_count<20 && (!Right_Black_sixty_lose || !Left_Black_sixty_lose) 
     && !Left_Black_fourtyfive_lose && !Right_Black_fourtyfive_lose && !Right_Black_thrity_lose 
       && !Left_Black_thrity_lose && Road_Kind!=3 && Road_Kind!=1 && Road_Kind!=2
     )*/
  
  //坡道判断 17-03-12
  /*
  if( ML_Black_Spot_count<5 && !Right_Black_sixty_lose && !Left_Black_sixty_lose 
     && !Left_Black_fourtyfive_lose && !Right_Black_fourtyfive_lose && !Right_Black_thrity_lose 
       && !Left_Black_thrity_lose && !Left_Black_fiveteen_lose && !Right_Black_fiveteen_lose 
         && Road_Kind==0 && !Zebra_Mark && abs(Dir_detal)<3 && abs(Gyro_Y)>700 
     )
  {
    High_Road_Flag=1;
  }
  */
    /*
    for(i=0;i<30;i++)
      for(j=20;j<60;j++)
      {
        if(img[i][j]==0xff && img[i][j-1]==0x00 && img[i][j-2]==0x00 && img[i][j-3]==0x00 && 
           img[i+1][j]==0xff && img[i+1][j-1]==0xff && img[i+1][j-2]==0xff && img[i+1][j-3]==0xff &&
             abs(Left_Black[i]-Left_Black[i+1])>=3)
        {
          High_Road_Left=1;
          High_Road_Left_i=i;
          High_Road_Left_j=j;
          break;
        }
        if(img[i][j]==0xff && img[i][j+1]==0x00 && img[i][j+2]==0x00 && img[i][j+3]==0x00 && 
           img[i+1][j]==0xff && img[i+1][j+1]==0xff && img[i+1][j+2]==0xff && img[i+1][j+3]==0xff &&
             abs(Right_Black[i]-Right_Black[i+1])>=3)
        {
          High_Road_Right=1;
          High_Road_Right_i=i;
          High_Road_Right_j=j;
          break;
        }
      }
    */
    /*
    if(High_Road_Left && High_Road_Right && abs(High_Road_Left_i-High_Road_Right_i)<3 && 
       abs(High_Road_Left_j-High_Road_Right_j)<25)
    {
      High_Road_Flag=1;
      //gpio_set (PTC12,1);
    }
    */
    //17-03-14 00:44
  if( //LB_Lose_Count<3 && RB_Lose_Count<3 &&
       !Zebra_Mark && Road_Kind==0 && !podao)//确保在直线上
  {
    /*
    for(i=5;i<55;i++)
    {
      if(!High_Road_Left || !High_Road_Right)
      {
        if(!High_Road_Left && Left_Black[i]-Left_Black[i+1]>3 //&& abs(Left_Black[i]-Left_Black[i+4])==0
           )
        {
          High_Road_Left=1;
          High_Road_Left_i=i;
        }
        if(!High_Road_Right && Right_Black[i]-Right_Black[i+1]<-3 //&& abs(Right_Black[i]-Right_Black[i+4])==0
           )
        {
          High_Road_Right=1;
          High_Road_Right_i=i;
        }
      }
      else
        break;
    }
    
    if(High_Road_Left && High_Road_Right //&& High_Road_Left_i && High_Road_Right_i 
       && abs(High_Road_Left_i-High_Road_Right_i)<3
         )*/
    
      High_Road_Flag=1;
      //gpio_set (PTC12,1);
  }
    else
    {
      High_Road_Left_i=0;
      High_Road_Left_j=0;
      High_Road_Right_i=0;
      High_Road_Right_j=0;
    }
  
    
    
    if(High_Road_Flag)
    {
      P_Angle_Speed_Now=1.3*P_Angle_Speed_Init;
      //g_fCarSpeedSet=0.8*g_fCarSpeedSet_Start;
      High_Road_Time++;
      //g_fCarSpeedSet=g_fCarSpeedSet_Start*0.75;
    }
    else
    {
      P_Angle_Speed_Now=P_Angle_Speed_Init;
      //g_fCarSpeedSet=g_fCarSpeedSet_Start;
      //g_fCarSpeedSet=g_fCarSpeedSet_Start;
    }
    
    if(High_Road_Time==30)
    {
      High_Road_Time=0;
      High_Road_Flag=0;
      High_Road_Left=0;
      High_Road_Right=0;
      High_Road_Left_i=0;
      High_Road_Left_j=0;
      High_Road_Right_i=0;
      High_Road_Right_j=0;
    }
    //gpio_set (PTC12,1);
    //P_Direction_Control_now=0.4*P_Direction_Control;
    //D_Direction_Control_now=0.4*D_Direction_Control;

  /*
  else if(ML_Black_Spot_count<10 && (!Right_Black_sixty_lose || !Left_Black_sixty_lose) 
     && !Left_Black_fourtyfive_lose && !Right_Black_fourtyfive_lose && !Right_Black_thrity_lose 
       && !Left_Black_thrity_lose && Road_Kind!=3 && Road_Kind!=1 && Road_Kind!=2)
  {
    gpio_set (PTC12,1);
    P_Direction_Control_now=0.5*P_Direction_Control;
    D_Direction_Control_now=0.5*D_Direction_Control;
  }
  else if(ML_Black_Spot_count<5 && (!Right_Black_sixty_lose || !Left_Black_sixty_lose) 
     && !Left_Black_fourtyfive_lose && !Right_Black_fourtyfive_lose && !Right_Black_thrity_lose 
       && !Left_Black_thrity_lose && Road_Kind!=3 && Road_Kind!=1 && Road_Kind!=2)
  {
    gpio_set (PTC12,1);
    P_Direction_Control_now=0.3*P_Direction_Control;
    D_Direction_Control_now=0.3*D_Direction_Control;
  }
  */
  
  /*else
  {
    gpio_set (PTC12,0);
    P_Direction_Control_now=P_Direction_Control;
    D_Direction_Control_now=D_Direction_Control;
  }
  */
  
  /*
  if(g_fCarSpeedSet>g_fCarSpeedSet_Max)
      g_fCarSpeedSet=g_fCarSpeedSet_Max;   
  if(g_fCarSpeedSet<g_fCarSpeedSet_Stand)
    g_fCarSpeedSet=g_fCarSpeedSet_Stand;
  */

}

/*
函数名称：拨码开关状态
最后修改时间：20170215
作者：厦理创新实验室智能车队-暗涌
有疑问致电邮：13950986716， lczq@vip.qq.com
未解决问题： 
**************************************************************************************************/
void DIP_status(void)
{
  Number_7=gpio_get (PTD12);
  Number_6=gpio_get (PTD11);
  Number_5=gpio_get (PTD2);
  Number_4=gpio_get (PTD0);
  Number_3=gpio_get (PTC15);
  Number_2=gpio_get (PTC13);
  Number_1=gpio_get (PTC11);
  Number_0=gpio_get (PTC9);
  
  DIP_Number=Number_0*1+Number_1*2+Number_2*4+Number_3*8+Number_4*16+Number_5*32+Number_6*64+Number_7*128;
}

void plan_Chose(void)
{
  if(Number_4==1)//24.5
  {
    g_fCarSpeedSet=3100;
    Detal_Column=32;
    FS_Spd=1100;
    P_Direction_Control=46;
    Dir_Spd_Fuzzy_constant=1.4;
    Po_B=17;
    //O_Add=8;
  }
  else if(Number_5==1)
  {
    g_fCarSpeedSet=3350;
    Detal_Column=28;
    FS_Spd=1200;
    Dir_Spd_Fuzzy_constant=1.6;
    P_Direction_Control=49;
    //g_fSpeedControlP_Init=1.3;
    //g_fSpeedControlP=1.3;
    Po_B=19;
    Ramp_Timer_Set=140;
  }
  else if(Number_6==1)
  {
    g_fCarSpeedSet=3600;
    Detal_Column=26;
    FS_Spd=1250;
    Dir_Spd_Fuzzy_constant=1.7;
    P_Direction_Control=50;
    //g_fSpeedControlP_Init=1.3;
    //g_fSpeedControlP=1.3;
    Po_B=20;
    Ramp_Timer_Set=140;
  }
}
