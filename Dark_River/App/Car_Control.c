/*
�������ƣ�Car_Control.c
����޸�ʱ�䣺2016-12-10
���ߣ�������ʵ����-��ӿ��
�������µ��ʣ�13950986716��lczq@vip.qq.com
���ݼ�¼������1��̧������ͷ�Ƕȣ���������̧ͷ��ͷ��Ӧ��Э����ֱ�����ͷ��򻷣�
              2��600�ٶ��ܱȽ��ȶ�����1~2Ȧ������U�������ƫ�ڵ����׳��,
              3��ʮ����ƫ��������Ȼδ���.16-1119 �����ֱ��ƫ�����⣬����ֻ�Ǽ򵥵���ʮ�ֶ��ߴ������ߣ���û�вɼ����ĸ��յ㰡,
              4��Ҫ���ݹյ㻭�����߰�,
              5����������ʱ������һ����·���ٶȽ�Ϊ0(��δ���)
              6�� ����б��ʮ�ֺ�СS ���ǲ����ã����ڵ����� �Ƚ��СS�� ��ɹ���
�ٶȼ�¼��16-11-13 1��600~1.31
          16-11-18 2��800~1.54
          16-11-18 3��1000~1.92m~30s  165m�ȶ����� 
          16-11-18 4��1010~2.0m~27.0s
          16-11-26 5��900~27.0s
          16-11-27 6��850 �ֶ� 28.4
          16-11-28 7��850 �ֶ� Ϳ�� ���27.2����Ϳ�Ͳ��ȶ�������ǿ��
          16-12-01 8��900 �ֶ� ��Ϳ�� ���26.7 ������¶�� һ��27.1+ 
          16-12-04 9��1000 �ֶ� 26+
          16-12-05 9��1000 ���ֶ� 26+ ǰհ����Ҫ ܳ�����̵� ��������
          16-12-05 10���ֶΰ� ���ȶ�
          16-12-10 11��1050 25.8s 
          17-02-26 12�����俨��
          17-03-03 δ������⣺1������Բ����ʮ��ǰ+���� 2������Բ��������
          17-03-10 б��ʮ��Ҫ�������˲����մ�������е�б��ʮ�֣��������ȥ����������ǿ
           55m 65m 47.73m 70.3m
*/
#include "Car_Control.h"
#include "math.h"
#include "include.h"
#include "common.h"
/**************************PID����**************************/
/*̫������̫ǰ��ǰ*/
/************�ǶȻ�************/
/*�ڻ����ٶ�*/
float P_Angle_Speed_Init=0.09,//0.06,//0.095,//0.0900,//0.08//0.055 //���ܵ���0.07+���ٶ�
       P_Angle_Speed_Now=0.09,
       I_Angle_Speed=0,//0.01,//0.05,//0.010,
       D_Angle_Speed=0.01;//0.012;//0.03//0.025;//0.02;
//�Ϸ��� P~D 0.62~0.075  0.28~0.035
//NEW    P~D 
/*�⻷�Ƕ�*/
float  P_Angle=1.0//1.05
       ,I_Angle=0
       ,D_Angle=0.00
       ;

/************�ٶȻ�************/
float g_fSpeedControlP=1.2,//1.2,//2.02,//2.7,//1.95,
       g_fSpeedControlP_Init=1.2,
       g_fSpeedControlI=0,//0.025,//1.3,
       g_fSpeedControlD=0.6;//-0.6;//-0.6;//-1
       
float g_fCarSpeedSetL=0,//1200,//600,800~35.7s  1000~28.64~165m�ȶ����� 
      g_fCarSpeedSetR=0,//1200;//600;//120~123s
      g_fCarSpeedSet=2700,//2200,//
      g_fCarSpeedSet_Go=2700,//2200,
      g_fCarSpeedSet_Start=2700,//2200,
      g_fCarSpeedSet_Max=0;//2200;
float fLeft, fRight;
//2500 ����2.6+

/************����************/
//P�Ӵ�Ļ�ʮ�ִ����û�ƫ�밡 ��
//ǰհ������P
//������ -P9  һ��S���-
float P_Direction_Control=43,//44,//9.0,//7.8,//20,//7.75,//7.85,//4.5,//4.5~600,//3.4,//3.5//0.25 1000~6.3
       P_Direction_Control_now=43,//44,//9.0//7.8,//20,//7.75,//7.85,
       D_Direction_Control=0.033,//30;//30;//54;//54.5;//56;//;//25;//0.35;//0.32
       D_Direction_Control_now=0.033; //26 17
        // D�谭����
       // P D��� 6+30  9+30 7+20 9+25 11+30 9+31 2000+7+23 8.25+28 
       // ��PD��� 1800+30+0.02 2000+36+0.02+2.1m 39+0.022 1800+36+0.025 
        // 2100+34+0.021 2200+34+0.021
       //14s 2600+42+0.02
       //170508��39+22+20
       //0627 36+33 26+17
//̧������ͷ ��������



/**************************�ǶȻ�����**************************/ 

#define One_Pro_Period   0.001216//0.0069
#define GRAVITY_OFFSET   -30//-150//300//���ٶ���ƫֵ�����ȷ��
#define GYROSCOPE_OFFSET 65 //��������ƫֵ OK
#define GYROSCOPE_ANGLE_SIGMA_FREQUENCY	 125//150//125//85//200//15//8 ����//�����˲�û�õ�
#define GRAVITY_ADJUST_TIME_CONSTANT	 2//0.5//1//2//1.2//0.05//��Ҫ���Ӱ����ٶȱ��غܴ󣬻����˲�û�õ�
#define GYROSCOPE_ANGLE_RATIO		23.4//23//29//30.5//̫С����//�����ǽ��ٶȷŴ��������˲�����������
#define GRAVITY_ANGLE_RATIO             1.0//���ٶȷŴ���������ͼ��ƫ�ƣ������˲�����������
#define Filter_Pass_Ratio               0.994//0.994//0.995//0.995//0.99//ԽСԽ���ż��ٶ�

/************���ݲɼ�************/
extern int16 Gyro_X,Gyro_Y,Gyro_Z,Acc_X,Acc_Y,Acc_Z,Acc_Z_Test;
int16  Gyro_Test=0;
float Angle_Acc,Car_Gyro_X,Gyro_Speed_X,Gyro_Speed_X_Ratio,Angle_Mix,Angle_Gyro;//gg
float Stand_Speed,Speed_L,Speed_R,Speed_L_Last,Speed_R_Last;  //վ���ٶȣ��������ٶ�
float Angle_Control_Out,Speed_Control_Out,Direction_Control_Out,Direction_Control_Out_Last; //������ʱû�õ�

/************�Ƕ��⻷************/
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
/************�Ƕ��ڻ�************/
float Angle_Error,
      Angle_Speed_Error,
      Angle_Speed_Expect,//�������ٶȣ��ɼ��ٶȼ��������
      Angle_Speed_Integral,
      Angle_Speed_Error_Error,
      Angle_Speed_Error_Last,
      Stand_Speed_Part_In_Now,
      Stand_Speed_Part_In_Old,
      Stand_Speed_Part_In
      ;
uint16 Angle_Speed_Integral_Start_Time=0,
       Angle_Speed_Integral_Start_Mark=0,
       Angle_Speed_Index=1 //���ַ����־
       ;
/************�ǶȻ���************/// real heavy 4500 10g 2450  3300  5150
float Angle_Standard=3600,Angle_Min=-9750,Angle_Max=2450;//���� 
double fGyroscopeAngleIntegral=0, fDeltaValue, g_fCarAngle, g_fGyroscopeAngleSpeed, g_fGravityAngle;
float Angle_Car_Now,Angle_Car_Now_2,g_fAngleControlOut,g_fAngleControlOut_Last,g_fAngleandSpeed;

uint16 Time_Count=0,Start_Continue_Flag=0;
uint16 Stop_Check_Count=0,Charge_Stop_Car_Count=0;
extern int i,j,Flag_Right,Flag_Left;        //�����б���i=0~60���б���j=0~80
extern uint8 img[CAMERA_H][CAMERA_W];      //��չ����ͼ��洢����img
       
/*

*/
/**************************�ٶȻ�����**************************/
float fPL,fPR,fDL,fDR,fIL,fIR;
float Speed_Left,Speed_Right,Speed_Right_Last,Speed_Left_Last,Speed_Left_Max;
float g_fSpeedDeltaL,g_fSpeedDeltaR,g_fSpeedDeltaPreL,g_fSpeedDeltaPreR;
float g_fSpeedControlIntegralL,g_fSpeedControlIntegralR;
float g_fSpeedControlOutOldL,g_fSpeedControlOutNewL,g_fSpeedControlOutOldR,g_fSpeedControlOutNewR; 
float g_fSpeedControlDeltaL,g_fSpeedControlDeltaR,g_fSpeedControlOutL,g_fSpeedControlOutR,g_fSpeedControlOutL_Max=-4000;
extern float fLeftVal,fRightVal;
uint16 Stop_Car=0;//���ӽǶȹ���ͣ��
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

/**************************���򻷲���**************************/
extern int32 Left_Black[60],Right_Black[60];
float g_fDirectionControlOut,Picture_detal_All;
extern float Picture_detal,Dir_detal,Picture_detal_Old,Picture_detal_detal;
extern int32 Direction_Control; 
int32 Direction_Control_Max=600,Direction_Control_Min=-600;//2.25 �ٺ�
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
extern int16 Ramp_Flag,Ramp_Timer,Ramp_Timer_Set,Ramp_Next_Delay;//�µ�
extern int16 Detal_Column,FS_Spd,Po_B,O_Add;
extern float Dir_Spd_Fuzzy_constant;
/**************************�������**************************/
int32 g_dLeftMotorPulseSigma,	  //�����������������
      g_dRightMotorPulseSigma;  //�����������������
float Motor_Left_Out,
      Motor_Right_Out
      ;
#define MOTOR_DEAD_VAL_L     0//20  //������ѹ
#define MOTOR_DEAD_VAL_R     0//20

/**************************���ݲɼ��ͷ�������**************************/
extern uint8 bit8_data[6];               //��Ͱ�λ�������ǻ���ٶ�ֵ
extern int16 bit16_data16[3];           //���ʮ��λ�������ǻ���ٶ�ֵ
extern unsigned short send_data[8];
extern uint16 count;


/*
��ʱ�䣺��ʼ��pit_time_start(PIT1);
        ������count= pit_time_get_us(PIT1);
*/
/*
�������ƣ��ɼ����ٶȺ������ǵ�λ����Ϣ 
����޸�ʱ�䣺2016-11-01
���ߣ�������ʵ�������ܳ���-��ӿ
�ο����������
����ʱ�䣺�ɼ�6�����ݣ�4.011ms���ɼ�2�����ݣ�4.010ms
�������µ��ʣ�13950986716��lczq@vip.qq.com
**************************************************************************************************/
void Get_Acc_And_Gyro()
{
  IIC_Read_Acc_16bit(SlaveAddress8700,bit8_data,bit16_data16);//I2C��ȡ���ٶ�ֵ
  //Acc_X= bit16_data16[0];
  //Acc_Y= bit16_data16[1];
  Acc_Z= bit16_data16[2];
  IIC_Read_Gyro_16bit(SlaveAddress2100,bit8_data,bit16_data16);//I2C��ȡ������ֵ
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
�������ƣ����ݷ�������λ��
����޸�ʱ�䣺2016-11-26
���ߣ�������ʵ�������ܳ���-��ӿ
�ο�������
����ʱ�䣺1.81ms
�������µ��ʣ�13950986716��lczq@vip.qq.com
**************************************************************************************************/
void Send_Data_Up()
{
  send_data[0]=(int16)fGyroscopeAngleIntegral;//fGyroscopeAngleIntegral;//Acc_X;
  send_data[1]=Acc_Z;//Gravity_Angle_Integral;//Stand_Speed;//g_fGravityAngle;//Acc_Y;
  send_data[2]=Gyro_Z;//-250--1000//Gravity_Angle_Error;//Gravity_Angle_Error_Error;
  send_data[3]=Gyro_Test;//fGyroscopeAngleIntegral;//Gyro_X;//g_fGyroscopeAngleSpeed;
  //send_data[4]=Angle_Car_Now_2;//Angle_Now;//Gyro_Y;
  //send_data[5]=Angle_Car_Now;//fLeftVal,fRightVal
  /*���ݼ�¼
   �趨2200 ��С�ٶȲ�ֵ-1050*/
  Data_Send(UART0,send_data);//ͨ������������ݵ���λ����ʾ����
}

/*
�������ƣ�ֱ���Ƕȼ��� 
����޸�ʱ�䣺2016-11-12
���ߣ�������ʵ�������ܳ���-��ӿ
�ο����廪�˲�
���Լ�¼��161023-����һ��û���á�
�������µ��ʣ�13950986716��lczq@vip.qq.com

**************************************************************************************************/
void AngleCalculate(float Acc_Z,float Gyro_Y)
{
  
  fDeltaValue=0;
  g_fGravityAngle = (Acc_Z - GRAVITY_OFFSET) * GRAVITY_ANGLE_RATIO;  //���ٶ�
  g_fGyroscopeAngleSpeed = (Gyro_Y/*Gyro_Z */- GYROSCOPE_OFFSET ) * GYROSCOPE_ANGLE_RATIO;   //gyro���ٶ�
  g_fCarAngle = fGyroscopeAngleIntegral;
  fDeltaValue =(g_fGravityAngle - g_fCarAngle) / GRAVITY_ADJUST_TIME_CONSTANT;      //������
    // ���ٶ�ƫ��ֵ =���ɻ����ļ��ٶ�  - �ϴ��ںϺ�ļ��ٶȣ�/2
  fGyroscopeAngleIntegral +=(g_fGyroscopeAngleSpeed + fDeltaValue) / GYROSCOPE_ANGLE_SIGMA_FREQUENCY;
  Angle_Car_Now=fGyroscopeAngleIntegral/180;
  //Ϊʲô�ɻ�����ֵ��ƽ���ģ��������ģ���TM�ģ����л����˲���
}

/*
�������ƣ�ֱ���Ƕȼ��� 
����޸�ʱ�䣺2016-11-26
���ߣ�������ʵ����-��ӿ��
�ο���MIT�����˲�
���ԣ����Գɹ������������
���ݼ�¼����dox
�������µ��ʣ�13950986716��lczq@vip.qq.com
**************************************************************************************************/
void Balance_Filter(float Acc_Z,float Gyro_Z)
{
  g_fGravityAngle =        (Acc_Z - GRAVITY_OFFSET) * GRAVITY_ANGLE_RATIO;  
  //�˳���ƫֵ��ļ��ٶ� =(ԭʼ���ٶ�Z��ֵ-��ƫֵ) *  ���ٶȱ���
  
  g_fGyroscopeAngleSpeed = (/*Gyro_Y*/Gyro_Z - GYROSCOPE_OFFSET ) * GYROSCOPE_ANGLE_RATIO; 
  //�˳���ƫֵ���gyro���ٶ�=(ԭʼ���ٶ�Z��ֵ-��ƫֵ)             * �����Ǳ���
  
  fGyroscopeAngleIntegral=Filter_Pass_Ratio*(fGyroscopeAngleIntegral+g_fGyroscopeAngleSpeed*One_Pro_Period)
                          +(1-Filter_Pass_Ratio)*g_fGravityAngle;//�����˲�����
  //�ںϽǶ�            =�˲�������*(�ںϽǶ�+���ٶ�*һ����������)+(1-�˲�������)*���ٶ�
  
  //0.996  0.98  0.996
   if(fGyroscopeAngleIntegral>17000)fGyroscopeAngleIntegral=17000;//�޷�
   if(fGyroscopeAngleIntegral<-17000)fGyroscopeAngleIntegral=-17000;//�޷�
  Angle_Car_Now_2=asin(fGyroscopeAngleIntegral/17000)/3.1415*180;//+19.3;
  Angle_Car_Now=fGyroscopeAngleIntegral/180;
  /*if(Angle_Car_Now>90 || Angle_Car_Now<50)
  {
    Stop_Car=1;
  }*/
  
  Gyro_Test+=Gyro_Z - GYROSCOPE_OFFSET;
}
/*
�������ƣ�ֱ���Ƕȼ��� 
����޸�ʱ�䣺2016-11-11
���ߣ�������ʵ�������ܳ���-��ӿ
�ο������������˲�,TMB̫���ɿ�����ܳ��ֻ�ܿ��Լ�
�������µ��ʣ�13950986716��lczq@vip.qq.com
**************************************************************************************************/
void Angle_Control()//û����ɹ�
{
  
    Angle_Acc=180/PI*(float)(atan2(Acc_Z,Acc_Y));  //��Acc_z/Acc_y�ķ�����
    /*������Angle_Acc=180/PI*(float)(asin(Acc_z/8000));  ��Acc_z/8000�ķ�����*/
    
    /*if(Angle_Acc>90)
      Angle_Acc=90;
    if(Angle_Acc<-90)
      Angle_Acc=-90;*/
    
    int16 Gyro_X_Zero=28;//��ֹʱX����ٶ���ƫֵ
    float Gyro_Speed_X_Ratio=0.0002;//�����ǻ���ϵ��������ʹ�������ǽǶȻ��������ڼ��ٶȼ���ĽǶ�ֵ
    
    Car_Gyro_X=(float)(Gyro_X*1.0-Gyro_X_Zero);//X�������ǽ��ٶ�-��ƫֵ
    Gyro_Speed_X=(float)(Car_Gyro_X*Gyro_Speed_X_Ratio);//���ڻ��ֵ��������ٶ�=X����ٶ�*����ϵ��
    Angle_Gyro=(float)(Angle_Gyro+Gyro_Speed_X);
    Angle_Mix=Angle_Gyro+(Angle_Acc-Angle_Gyro)*0.425;//�����˲�
    
}



/*
�������ƣ��ٶȻ�ȡ 
����޸�ʱ�䣺2016-11-26
���ߣ�������ʵ�������ܳ���-��ӿ
�ο���δ����
�������µ��ʣ�13950986716��lczq@vip.qq.com
**************************************************************************************************/

void Get_Speed()
{
  
  //Left_Motor = 0 , Right_Motor = 0;
  Left_Motor_Last=Left_Motor;
  Right_Motor_Last=Right_Motor;
  
  Right_Motor=-ftm_quad_get(FTM1);
  Left_Motor=ftm_quad_get(FTM2); //ͨ����������ȡ�ٶ�
  
  
  ftm_quad_clean(FTM1);     //����������������
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
  
  g_dLeftMotorPulseSigma+=Left_Motor;//���õ����ٶȸ�����������
  g_dRightMotorPulseSigma+=Right_Motor;
    
}

/*
�������ƣ�ֱ���ٶ��⻷���� 
����޸�ʱ�䣺2016-11-26
���ߣ�������ʵ�������ܳ���-��ӿ
�ο���pixhawk�ɿش���PID+ʩ��
�������µ��ʣ�13950986716��lczq@vip.qq.com
**************************************************************************************************/
void Angle_Out_Loop_Control()
{
  Gravity_Angle_Error=fGyroscopeAngleIntegral;
  
  if(Gravity_Angle_Integral_Start_Time>200)
  {
    Gravity_Angle_Integral_Start_Mark=1;//���ֿ�ʼ��־λ
  }
  else
  {
    Gravity_Angle_Integral_Start_Time++;//���ֿ�ʼ��־��ʱλ
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
    Gravity_Angle_Integral+=Gravity_Angle_Error;//�����������ַ���Ϳ����ֱ��͡�����ִ����������ݷ� ������ʱ����չ
  }
  
  Gravity_Angle_I=Gravity_Angle_Integral*I_Angle;
    
    if(Gravity_Angle_I<-80)//�����޷� 1%
      Gravity_Angle_I=-80;
    if(Gravity_Angle_I>80)//�����޷� 1%
      Gravity_Angle_I=80;
    
  
  //Gravity_Angle_Error_Error=Gravity_Angle_Error-Gravity_Angle_Error_Last;//���ٶ�ƫ���ƫ��
  //Gravity_Angle_Error_Last=Gravity_Angle_Error;//�ϸ�ƫ��
  
  Stand_Speed_Out_Out=Gravity_Angle_Error*P_Angle
                    + Gravity_Angle_Index*Gravity_Angle_I
                    + g_fGyroscopeAngleSpeed*D_Angle;
  //����ֱ�����⻷�Ƕ�����������ٶ�С���ڻ���1:5
  
}

/*
�������ƣ�ֱ���ٶ��ڻ����� 
����޸�ʱ�䣺2016-11-26
���ߣ�������ʵ�������ܳ���-��ӿ
�ο���pixhawk�ɿش���PID+ʩ��
���ԣ�������⻷Ϊ�����ڻ��൱��D������
�������µ��ʣ�13950986716��lczq@vip.qq.com
**************************************************************************************************/
void Angle_In_Loop_Control()
{
  
  Angle_Error=(Angle_Standard-Stand_Speed_Out_Out);//�����ܵ�ʱ���ֵΪ�� *P_Angle;//�ǶȲ�//���ù�һ��ֱ���������ȸ���
  
  Angle_Speed_Expect=Angle_Error/180/One_Pro_Period;//��ʱ�����൱���󵼣��Ƕȵı仯/ʱ��=���ٶ�
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
  //�ɽǶȼ���ĽǶȲ���������������ֹʱ������Ϊ0��׼ȷ��
  
  Angle_Speed_Error=Angle_Speed_Expect-Gyro_Z;//g_fGyroscopeAngleSpeed;//��ǰ���ٶ�ƫ��
  
  if(Angle_Speed_Integral_Start_Time>1000)
  {
    Angle_Speed_Integral_Start_Mark=1;//���ֿ�ʼ��־λ
  }
  else
  {
    Angle_Speed_Integral_Start_Time++;//���ֿ�ʼ��־��ʱλ    
  }
  
  if(Angle_Speed_Integral_Start_Mark)//���л�������
  {
    if(abs((uint32)Angle_Speed_Error)>200)
    {
      Angle_Speed_Index=0;
    }
    else
    {
      Angle_Speed_Index=1;
    }
    Angle_Speed_Integral+=Angle_Speed_Error;//�����������ַ���Ϳ����ֱ��͡�����ִ����������ݷ� ������ʱ����չ
    
    if(Angle_Speed_Integral<-5000)//�����޷�
      Angle_Speed_Integral=-5000;
    if(Angle_Speed_Integral>5000)//�����޷�
      Angle_Speed_Integral=5000;
  }
  Angle_Speed_Error_Error=Angle_Speed_Error-Angle_Speed_Error_Last;//���ٶ�ƫ���ƫ��
  Angle_Speed_Error_Last=Angle_Speed_Error;//�ϸ�ƫ��
  
  Stand_Speed_Part_In_Old=Stand_Speed_Part_In;
  
  Stand_Speed_Part_In=Angle_Speed_Error*P_Angle_Speed_Now 
                   + Angle_Speed_Integral*I_Angle_Speed*Angle_Speed_Index 
                    + Angle_Speed_Error_Error*D_Angle_Speed;
  /*
  Stand_Speed_Part_In=Angle_Error*P_Angle_Speed 
                    + Angle_Speed_Integral*I_Angle_Speed*Angle_Speed_Index 
                    + Angle_Speed_Error*D_Angle_Speed;
  */
  //ֱ���ٶ��ڻ���� 5�����⻷ 
  
  Stand_Speed_Part_In_Now=Stand_Speed_Part_In;//(Stand_Speed_Part_In_Old+Stand_Speed_Part_In)/2;
}

/*
�������ƣ�ֱ���ٶ��ܼ��� 
����޸�ʱ�䣺2016-11-26
���ߣ�������ʵ�������ܳ���-��ӿ
�ο����ְ��ֽ�����·��������
�������µ��ʣ�13950986716��lczq@vip.qq.com
**************************************************************************************************/
void Angle_All_Control()
{
  Time_Count++;
  if(Time_Count==5)
  {
    Angle_Out_Loop_Control();//�Ƕ��⻷ 5�����ڸ���һ��
    Time_Count=0;
  }
  Angle_In_Loop_Control();//���ٶ��ڻ� 1�����ڸ���һ��
  
  Stand_Speed=Stand_Speed_Part_In_Now;//+Stand_Speed_Part1;//ֱ���ٶȰ��������֣��Ƕ��⻷�ͽ��ٶ��ڻ�
  g_fAngleControlOut_Last=g_fAngleControlOut;
  g_fAngleControlOut=Stand_Speed;
  //g_fAngleControlOut=(g_fAngleControlOut-g_fAngleControlOut_Last>300?g_fAngleControlOut_Last+300:g_fAngleControlOut);
  //g_fAngleControlOut=(g_fAngleControlOut-g_fAngleControlOut_Last<-300?g_fAngleControlOut_Last-300:g_fAngleControlOut);
}


/*
�������ƣ��ٶȻ� 
����޸�ʱ�䣺2016-11-26
���ߣ�������ʵ�������ܳ���-��ӿ
�ο������֡�ʩ�����Ը�
�������µ��ʣ�13950986716��lczq@vip.qq.com
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
  
  if(Speed_Left-Speed_Left_Last>250 && g_fCarSpeedSet)Speed_Left=Speed_Left_Last+250;//�ٶȷ����ı仯�����޷�
  if(Speed_Left-Speed_Left_Last<-250 && g_fCarSpeedSet)Speed_Left=Speed_Left_Last-250;//�ٶȷ����ı仯�����޷�
  if(Speed_Right-Speed_Right_Last>250 && g_fCarSpeedSet)Speed_Right=Speed_Right_Last+250;//�ٶȷ����ı仯�����޷�
  if(Speed_Right-Speed_Right_Last<-250 && g_fCarSpeedSet)Speed_Right=Speed_Right_Last-250;//�ٶȷ����ı仯�����޷�
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
    Speed_Right=(Speed_Right>1.2*g_fCarSpeedSet_Start?1.2*g_fCarSpeedSet_Start:Speed_Right);//�ٶȷ������޷�
    Speed_Left=(Speed_Left>1.2*g_fCarSpeedSet_Start?1.2*g_fCarSpeedSet_Start:Speed_Left);//�ٶȷ������޷�
  }
  */
  //g_fSpeedDeltaL=(g_fSpeedDeltaL>-1200?-1200:g_fSpeedDeltaL);//�ٶ����ƫ���޷�
  //g_fSpeedDeltaR=(g_fSpeedDeltaR>-1200?-1200:g_fSpeedDeltaR);//�ٶ����ƫ���޷�
  
  g_fSpeedDeltaPreL = g_fSpeedDeltaL;//��һ���ٶ�detail
  g_fSpeedDeltaPreR = g_fSpeedDeltaR;//��һ���ٶ�detail
  
  

  if(Zebra_Mark_Rea_Timer>0 && Zebra_Mark_Rea_Timer<80)//����ص�����ͣ��
  {
    g_fCarSpeedSet=0;
  }
  
  
  //g_fSpeedDeltaL = Speed_Left  - g_fCarSpeedSet  ;//�����ٶ�detail
  //g_fSpeedDeltaR = Speed_Right - g_fCarSpeedSet  ;//�����ٶ�detail
  
  g_fSpeedDeltaL = g_fCarSpeedSet - Speed_Left;//�����ٶ�detail
  g_fSpeedDeltaR = g_fCarSpeedSet - Speed_Right ;//�����ٶ�detail
  
  if(Speed_Left>Speed_Left_Max)
  {
    Speed_Left_Max=Speed_Left;
  }
  
  //g_fSpeedDeltaL = g_fCarSpeedSet-Speed_Left;//�����ٶ�detail
  //g_fSpeedDeltaR = g_fCarSpeedSet-Speed_Right;//�����ٶ�detail
  
  //if(g_fSpeedDeltaL-g_fSpeedDeltaPreL>150 && g_fCarSpeedSet)g_fSpeedDeltaL=g_fSpeedDeltaPreL+150;//ÿ��������ٶ�����޷�
  //if(g_fSpeedDeltaL-g_fSpeedDeltaPreL<-150 && g_fCarSpeedSet)g_fSpeedDeltaL=g_fSpeedDeltaPreL-150;//ÿ��������ٶ�����޷�  
  //if(g_fSpeedDeltaR-g_fSpeedDeltaPreR>150 && g_fCarSpeedSet)g_fSpeedDeltaR=g_fSpeedDeltaPreR+150;//ÿ��������ٶ�����޷�
  //if(g_fSpeedDeltaR-g_fSpeedDeltaPreR<-150 && g_fCarSpeedSet)g_fSpeedDeltaR=g_fSpeedDeltaPreR-150;//ÿ��������ٶ�����޷�
  
  fPL = g_fSpeedDeltaL * g_fSpeedControlP;//�����ٶ�P
  fPR = g_fSpeedDeltaR * g_fSpeedControlP;
  fDL = (g_fSpeedDeltaL - g_fSpeedDeltaPreL) * g_fSpeedControlD;//�����ٶ�D
  fDR = (g_fSpeedDeltaR - g_fSpeedDeltaPreR) * g_fSpeedControlD;
  
  //if(Speed_Right && Speed_Left)
  {
  fIL = fIL+g_fSpeedDeltaL * g_fSpeedControlI;//���������ٶ�I ����I�б�Ҫ���ַ���
  if(fIL > 100)
      fIL = 100;
  if(fIL < -100)
      fIL = -100;
  
  g_fSpeedControlIntegralL = (fIL);//g_fSpeedControlIntegralL + 
   
  fIR =fIR+ g_fSpeedDeltaR * g_fSpeedControlI;//���������ٶ�I
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
  
  g_fSpeedControlDeltaL  = (g_fSpeedControlOutNewL - g_fSpeedControlOutOldL);///g_nSpeedControlCount;//ƽ�����
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
  
  g_fSpeedControlDeltaL  = (g_fSpeedControlOutNewL - g_fSpeedControlOutOldL);//������ϴ��ٶ����PWM֮��
  g_fSpeedControlDeltaR  = (g_fSpeedControlOutNewR - g_fSpeedControlOutOldR);//������ϴ��ٶ����PWM֮��
  
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
  if(g_fSpeedControlOutL>g_fSpeedControlOutL_Max && g_fSpeedControlOutL!=0 && Stop_Check_Begin_Count>100)//��¼����ٶ�pwm���
  {
    g_fSpeedControlOutL_Max=g_fSpeedControlOutL;
  }
  //g_fSpeedControlOutL=(g_fSpeedControlOutL>400?400:g_fSpeedControlOutL);//�ٶ�����޷� ���ã���
  //g_fSpeedControlOutR=(g_fSpeedControlOutR>400?400:g_fSpeedControlOutR);//�ٶ�����޷� û���ð� �����ձ�����
}
/*
�������ƣ����� 
����޸�ʱ�䣺2016-11-26
���ߣ�������ʵ�������ܳ���-��ӿ
�ο��������ഺ�ĵ����ճ���
�������µ��ʣ�13950986716��lczq@vip.qq.com
**************************************************************************************************/
void Dir_Loop_Control()
{
  //pit_time_start  (PIT1);
  Get_Middle_Line();
  //count=pit_time_get_us(PIT1);
  //Finish_Check();//�����߼��
  Direction_Detal_Caculate();//����� ƽ��ÿ�����Picture_detal
  //Picture_detal_All=Picture_detal_All+Picture_detal;
}

/*
�������ƣ����� 
����޸�ʱ�䣺2016-11-26
���ߣ�������ʵ�������ܳ���-��ӿ
�ο��������ഺ�ĵ����ճ���
�޸ģ�������������ΪKD
�������µ��ʣ�13950986716��lczq@vip.qq.com
**************************************************************************************************/
void Dir_Loop_Output()
{
  Dir_detal=Picture_detal;
  Picture_detal_detal=Picture_detal-Picture_detal_Old; //ƫ���ƫ��
  Picture_detal_Old=Picture_detal; //����ƫ����������Ա����D
  //Part_Road();//�ֶδ��� �����µ�
  if(Number_3)
  {
    Fuzzy_Apply(Dir_detal,Picture_detal_detal);//ģ��PD
  }
  
  //����1����ƫ���ƫ����D
  //Direction_Control_Out=(Dir_detal*P_Direction_Control_now)+((Picture_detal_detal)*D_Direction_Control_now);
  
  //����2����������X����D 
    /*
  if(ML_Black_Spot_count<8 && Fuck_Up_Mark)
  {
    P_Direction_Control_now=P_Direction_Control_now*0.1;
  }
    */
  Direction_Control_Out_Last=Direction_Control_Out;
  Direction_Control_Out=(Dir_detal*P_Direction_Control_now)+((Gyro_X-50)*D_Direction_Control_now);
  /*
  if(Direction_Control_Out>Direction_Control_Max)    //�ٶ�����
    Direction_Control_Out=Direction_Control_Max;
  if(Direction_Control_Out<Direction_Control_Min)    //�ٶ�����
    Direction_Control_Out=Direction_Control_Min;
  */
  Picture_detal=0;    //һ֡ͼ��ƫ�����㣬��������һ����
  Direction_Control_Out=(Direction_Control_Out-Direction_Control_Out_Last>100?Direction_Control_Out=Direction_Control_Out_Last+100:Direction_Control_Out);
  Direction_Control_Out=(Direction_Control_Out-Direction_Control_Out_Last<-100?Direction_Control_Out=Direction_Control_Out_Last-100:Direction_Control_Out);
  //100���� Ҫ�ɼ����ݷ���
  g_fDirectionControlOut=Direction_Control_Out;
  g_fDirectionControlOut=(g_fDirectionControlOut>300?300:g_fDirectionControlOut);
  g_fDirectionControlOut=(g_fDirectionControlOut<-300?-300:g_fDirectionControlOut);//400����

}

/*
�������ƣ������ٶ���� 
����޸�ʱ�䣺2016-11-26
���ߣ�������ʵ�������ܳ���-��ӿ
�ο������֡�ʩ�����Ը�
�������µ��ʣ�13950986716��lczq@vip.qq.com
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
  
  //��߸�ƫ�� �ұ���ƫ��
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
�������ƣ��ٶ�ͨ��PWM��� 
����޸�ʱ�䣺2016-11-26
���ߣ�������ʵ�������ܳ���-��ӿ
�ο������֡�ʩ�����Ը�
�������µ��ʣ�13950986716��lczq@vip.qq.com
**************************************************************************************************/
void SetMotorVoltage(float fLeftVal, float fRightVal)
{
  if(fLeftVal > 0)//����
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
  if(fRightVal > 0)//����
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
�������ƣ�����ͣ������ֹĥ����
����޸�ʱ�䣺20161210
���ߣ�������ʵ�������ܳ���-��ӿ
�������µ��ʣ�13950986716��lczq@vip.qq.com
δ������⣺
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
  if(Number_1==1 && Zebra_Mark /*Jump_Change_Point_Count>7 && Jump_Change_Point_Count<12*/)//������ͣ��
  {
    Zebra_Mark_Rea=1;
    Zebra_Mark_Rea_Timer=Zebra_Mark_Rea_Timer_Set;//��⵽�����߼���ǰ��30��������ͣ�³�
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
  if(fGyroscopeAngleIntegral>16000 || fGyroscopeAngleIntegral<10000)//ͨ���Ƕ��жϳ���
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
�������ƣ�����ǰ��վ��2��
����޸�ʱ�䣺20161210
���ߣ�������ʵ�������ܳ���-��ӿ
�������µ��ʣ�13950986716��lczq@vip.qq.com
δ������⣺
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
�������ƣ���·�ֶ�
����޸�ʱ�䣺20161205
���ߣ�������ʵ�������ܳ���-��ӿ
�������µ��ʣ�13950986716��lczq@vip.qq.com
δ������⣺ 1��ֱ����ҡ�ΰ����ö�����������Ҫ�󰡣�
**************************************************************************************************/


void Part_Road() //��·�ֶδ��� �д�ϸ�� �������ȼ�����
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
  if(g_fSpeedDeltaL>100 || g_fSpeedDeltaR>100)  //�ٶȱ��趨ֵ��
  {
    P_Direction_Control_now=P_Direction_Control+1;
  }
  else if(g_fSpeedDeltaL<100 || g_fSpeedDeltaR<100)//�ٶȱ��趨ֵС��
  {
    P_Direction_Control_now=P_Direction_Control-1.5;
  }
  else
  {
    P_Direction_Control_now=P_Direction_Control;
  }
  
  
  // 450 ֱ������� 
  // 380 ֱ��ǰ�� 50cm �����
  // 
  if(All_Variance<450)//С���
  {
    P_Direction_Control_now=P_Direction_Control-1;   
  }
  if((g_fSpeedDeltaL>100 || g_fSpeedDeltaR>100) && (All_Variance>500))//���������ٶ�û�ﵽ
  {
    P_Direction_Control_now=P_Direction_Control-1;
  }
  if((g_fSpeedDeltaL>100 || g_fSpeedDeltaR>100) && (All_Variance>500) && (All_Variance<750))//
  {
    P_Direction_Control_now=P_Direction_Control_now+1;
  }
  */
  //g_fSpeedDeltaL = Speed_Left - g_fCarSpeedSet  ;//����detail
  //g_fSpeedDeltaR = Speed_Right - g_fCarSpeedSet  ;
  
  /*
  //S��ֱ�����ٶ�û�ﵽ
  if(All_Variance<450 && g_fSpeedDeltaL<100 && g_fSpeedDeltaR<100) 
  {
    g_fCarSpeedSet+=5;

  }
  
  //S��ֱ�����ٶȴﵽ
  else if(All_Variance<450 && g_fSpeedDeltaL<50 && g_fSpeedDeltaR<50 && g_fSpeedDeltaL>-50 && g_fSpeedDeltaR>-50) 
  {
    //g_fCarSpeedSet+=5;
    P_Direction_Control_now=P_Direction_Control;

  }
  */
  //ʮ�����������90��All_Variance�ܴﵽ1000
  //500-900����ת90��
  //550-1000����ת90��
  //
  /*
  //������90��������ٶ�û�ﵽ
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
  
  //�µ��ж� 17-03-12
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
       !Zebra_Mark && Road_Kind==0 && !podao)//ȷ����ֱ����
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
�������ƣ����뿪��״̬
����޸�ʱ�䣺20170215
���ߣ�������ʵ�������ܳ���-��ӿ
�������µ��ʣ�13950986716�� lczq@vip.qq.com
δ������⣺ 
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
