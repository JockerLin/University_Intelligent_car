#include "common.h"
#include "include.h"
#include "VCAN_LCD_CH.h"
#include "math.h"
#include "core_cm4.h"


uint8 imgbuff[1200],imgbuff_trans[CAMERA_SIZE],img[CAMERA_H][CAMERA_W],//����洢����ͼ�������
      Depart=0,qipao = 1,Live_Watch_Mark=0,
      Gyro_x_h,Gyro_x_l,Gyro_z_h,Gyro_z_l,Accle_z_h,Accle_z_l,bit8_data[6]; //����߰�λ�Ͱ�λ ��Ͱ�λ�������ǻ���ٶ�ֵ
uint16 podao=0,II=0;
uint32 Duoji_Duty = 0,Mada_Duty=0,Start_Time_Count=0,Start_Time=0;
unsigned short send_data[8];
int16 Gyro_X,Gyro_Y,Gyro_Z,Acc_X,Acc_Y,Acc_Z,Acc_Z_Test,bit16_data16[3]; //���ʮ��λ�������ǻ���ٶ�ֵ;
short count=0;//�ĳ�short
float Picture_detal=0,Dir_detal=0,Picture_detal_Old=0,Picture_detal_detal=0,fLeftVal,fRightVal;

extern uint8 Mid_Line[60];
extern uint16 Get_Picture_Count,Angle_Caculate_Count,Angle_Output_Count,
               Speed_Control_Count,Speed_Output_Count,Dir_Count,Dir_Output_Count,
               Stop_Check_Begin_Count,Charge_Stop_Car_Count,Stop_Check_Count,
               Road_Kind,Road_Kind_Last,O_Road_Num,Stand_Inplace_Count,
               DIP_Number,ML_Black_Spot_count,//���뿪��״̬��ȡ
               DIP_Number,Number_3,Number_2,Number_1;
extern int16 LB_Lose_Count_Last,RB_Lose_Count_Last;
extern int32 Left_Motor,Right_Motor;
extern float Duoji_P,Duoji_I,Duoji_D,Stand_Speed,Picture_detal_Printf,
              Angle_Acc,Car_Gyro_X,Gyro_Speed_X,Gyro_Speed_X_Ratio,Angle_Mix,Angle_Gyro,Angle_Now,
              g_fCarSpeedSetL,//1200,//600,800~35.7s  1000~28.64~165m�ȶ����� 
              g_fCarSpeedSetR,//1200;//600;//120~123s
              g_fCarSpeedSet,
              g_fCarSpeedSet_Start,
              g_fCarSpeedSet_Max;
extern double fGyroscopeAngleIntegral,g_fCarAngle ,g_fGyroscopeAngleSpeed ,g_fGravityAngle,fDeltaValue ;
typedef unsigned char byte;//bute �ܴ���unsigned char �����������
typedef unsigned int word;//word �ܴ���unsigned int �����������

//��������
void main_scan();
void LCD_Init(void); //oled ��ʼ��
void System_Init(void);
void PORTA_IRQHandler(void);
void DMA0_IRQHandler(void);
void PIT0_IRQHandler(void);
void Get_Middle_Line(void);
void Img_Printf_OLED(void);
void sendimg(void *imgaddr, uint32 imgsize);
extern void LCD_P6x8Str(byte x,byte y,byte ch[]);//��ʾ�ַ���
extern void LCD_P14x16Str(byte x,byte y,byte ch[]);//��ʾ����
extern void LCD_Print(byte x, byte y, byte ch[]);//��ʾ���ֺ��ַ�

/*
�������ƣ������� 
������ʱ�䣺2016-1112
���ߣ�������ʵ�������ܳ���-��ӿ 
�⺯����ɽ��
ǰհ62.5cm
�������µ��ʣ�13950986716��lczq@vip.qq.com
**************************************************************************************************/
void  main(void)
{
    DisableInterrupts;   //��ֹ���ж�
    System_Init();//ϵͳ��ʼ�� �������߰���ĳ�ʼ���������� �ٺٺ�
    set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //���� PORTA ���жϷ�����Ϊ PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);      //���� DMA0 ���жϷ�����Ϊ PORTA_IRQHandler
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //����PIT0���жϸ�λ����Ϊ PIT0_IRQHandler
    set_irq_priority(PIT0_IRQn, 1);//�ж����ȼ�����
    set_irq_priority(DMA0_IRQn, 0);//DMA�жϵȼ�����
    //pit_init_ms(PIT0, 1);     //��ʼ��PIT0����ʱʱ��Ϊ
    EnableInterrupts;//ʹ���жϿ�
    while(1)
    {
      if(qipao == 1)
      //while(1)
         {
           //gpio_set (PTE12,0);
           disable_irq (PIT0_IRQn);  //���ж�
           ftm_pwm_duty(FTM0,FTM_CH3,0); //3>6 �� ��
           ftm_pwm_duty(FTM0,FTM_CH6,0);//
           ftm_pwm_duty(FTM0,FTM_CH4,0);//4>5 �� ��
           ftm_pwm_duty(FTM0,FTM_CH5,0);//
           DIP_status();
           plan_Chose();
           main_scan();
           g_fCarSpeedSet_Start=g_fCarSpeedSet;
           g_fCarSpeedSet_Max=g_fCarSpeedSet_Start+20;
         }
      //enable_irq(PIT0_IRQn);     //ʹ��PIT0�ж�
      II=600; 
      while(II--) {imgbuff_trans[II]=imgbuff[II];}
      camera_get_img();//11XXXus��������ͷ��ȡͼ��һά���飩
      //Img_Printf_OLED();//2ms
      //gpio_set (PTA15,1);
    }
}

/*
�������ƣ��жϷ������ 
������ʱ�䣺20161126
���ߣ�������ʵ�������ܳ���-��ӿ
�⺯����ɽ��
ʱ�䣺8ms һ����׼ 
�������µ��ʣ�13950986716��lczq@vip.qq.com
**************************************************************************************************/
void PIT0_IRQHandler(void)
{
      Get_Acc_And_Gyro();//788us������ȡ���ٶ���������ֵ
      Balance_Filter(Acc_Z,Gyro_Z);//60us���������˲��Ƕȼ���
      Angle_All_Control();//16us�����ǶȻ�
      Send_Data_Up();//1818us �ܵ�ʱ���
      MotorOutput();//12us����������
      Depart=Depart%3;
      switch(Depart)
      {
      case 0://�ٶȿ��� ����5�� ��1��
        {
          Get_Speed();//12us������ȡ��������ֵ��ÿ�μ��ʱ��һ��Ҫһ�������������һ��һ���ģ�
          Speed_Output_Count++;
          Speed_Loop_Output();//11us�����ٶ����
          Speed_Output_Count=Speed_Output_Count%5;
          if(Speed_Output_Count == 0)
           {
             Speed_Loop_Control();//25us�����ٶȼ���
             Speed_Output_Count=0;
           }
          break;
        }
      case 1://��ͼ�� 
        {
          camera_get_img();//11XXXus��������ͷ��ȡͼ��һά���飩
          img_extract((uint8 *)img,(uint8 *)imgbuff,CAMERA_SIZE);//247us������ѹΪ�Ҷ�ͼ�񣨶�ά���飩
          //vcan_sendimg(img, sizeof(img));//ͼ���͵���λ������������
          break;
        }
      case 2://�������  ����2�� ��1�� ���ȡͼ��ͬһ����
        {
          Dir_Loop_Control();//4400us����ͼ�������򻷼���
          Dir_Loop_Output();//14us�����������
          break;
        }
      }
      //vcan_sendimg(img, sizeof(img));//ͼ���͵���λ�� ������
      Depart++;
      Stop_Check();//�Ƕȱ��˻��⵽�յ��� ͣ����
      PIT_Flag_Clear(PIT0);//���жϱ�־λ     
      disable_irq (PIT0_IRQn);  //���ж�
      /*����ʱ��
      pit_time_start  (PIT1);
      count= pit_time_get_us(PIT1);
      */
}

/*
�������ƣ��жϷ����������ó��ж� 
������ʱ�䣺20170501
���ߣ�������ʵ�������ܳ���-��ӿ
�⺯����ɽ��
�������µ��ʣ�13950986716��lczq@vip.qq.com
**************************************************************************************************/
void PORTA_IRQHandler()
{
    uint8  n;    //���ź�
    uint32 flag;
    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ
    n = 29;                                             //���ж�
    if(flag & (1 << n))                                 //PTA29�����ж�
    {
        camera_vsync();
    }
    if(!qipao)
    {
      
      //if(Live_Watch_Mark)//��̬������ʾ��������ѡ�񿪹ء���������
      /*
      {
        DIP_status();//��ȡ���뿪����ֵ
        LCD_Print(2,0,"Road_Kind:");//�鿴·������
        LCD_PrintU16(80,0,Road_Kind);
        LCD_Print(2,1,"ML_Black:");//�м������ڵ���
        LCD_PrintU16(80,1,ML_Black_Spot_count);
        LCD_Print(2,2,"O_Dir:");//Բ����������
        if(Number_1==1)
        {
          LCD_Print(50,2,"R");
        }
        else if(Number_1==0)
        {
          LCD_Print(50,2,"L ");
        }
        DisplayFloat((int16)(Dir_detal*1000),2,3);//����ƫ��
        DisplayFloat((int16)(Picture_detal_detal*1000),2,4);//����ƫ���ƫ��
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
      Get_Acc_And_Gyro();//788us������ȡ���ٶ���������ֵ
      Balance_Filter(Acc_Z,Gyro_Z);//60us���������˲��Ƕȼ���
      Angle_All_Control();//16us�����ǶȻ�
      Get_Speed();//12us������ȡ��������ֵ��ÿ�μ��ʱ��һ��Ҫһ�������������һ��һ���ģ�
      Speed_Output_Count++;
      Speed_Loop_Output();//11us�����ٶ����
      Speed_Output_Count=Speed_Output_Count%10;
      if(Speed_Output_Count == 0)
       {
         Speed_Loop_Control();//25us�����ٶȼ���
         Speed_Output_Count=0;
       }
      //Img_Printf_OLED();
      Dir_Loop_Control();//4400us����ͼ�������򻷼���
      Dir_Loop_Output();//14us�����������
      Stop_Check();//���� ͣ����
      MotorOutput();//12us����������
      //Send_Data_Up();
      img_extract((uint8 *)img,(uint8 *)imgbuff_trans,CAMERA_SIZE);//247us������ѹΪ�Ҷ�ͼ�񣨶�ά���飩
      /*
      led_change_count++;
      led_change_count=led_change_count%100;
      if(led_change_count==0)
        led_turn(LED0);
      */
      //led_turn(LED0);
      //vcan_sendimg(img, sizeof(img));
      //count= pit_time_get_us(PIT1);
      //*(int*)0=0;//��0x00��ַд0
      //printf()
      //puts(ok?)
      //while(1)
      //{
      //  uart_putstr (UART0,"6666");
      //}
    }
}

/*
�������ƣ�DMA�жϷ����� 
������ʱ�䣺20160928
���ߣ�������ʵ�������ܳ���-��ӿ 
�⺯����ɽ��
�������µ��ʣ�13950986716��lczq@vip.qq.com
**************************************************************************************************/
void DMA0_IRQHandler()
{
    camera_dma();
}

void System_Init(void)
{
  LCD_Init();//OLED��Ļ��ʼ��
  Key_Init();//���̳�ʼ��
  camera_init(imgbuff); //����ͷ��ʼ��
  pll_init(PLL180);
  i2c_init(I2C0,400*1000);//��ʼ��I2C,���ò�����400K
  uart_init (UART0, 115200);//��ʼ������������
  Init8700();//���ٶȼ� 3��
  Init2100();//������   3��
  //led_init(LED0);
  ftm_pwm_init(FTM0, FTM_CH3,10000, 0);//��ʼ��������
  ftm_pwm_init(FTM0, FTM_CH4,10000, 0);//��ʼ��������
  ftm_pwm_init(FTM0, FTM_CH5,10000, 0);//��ʼ��������
  ftm_pwm_init(FTM0, FTM_CH6,10000, 0);//��ʼ��������
  ftm_quad_init(FTM1);//��ʼ��������ͨ��
  ftm_quad_init(FTM2);//��ʼ��������ͨ��
  gpio_init(PTA15, GPO, 0);//������
  //���뿪�ش����� �ֱ��ǣ���Բ�����򣻢ڰ�����ͣ�������µ����أ���ģ��
  gpio_init(PTC9, GPI, 0);//���뿪��
  gpio_init(PTC11, GPI, 0);//���뿪��
  gpio_init(PTC13, GPI, 0);//���뿪��
  gpio_init(PTC15, GPI, 0);//���뿪��
  gpio_init(PTD0, GPI, 0);//���뿪��
  gpio_init(PTD2, GPI, 0);//���뿪��
  gpio_init(PTD11, GPI, 0);//���뿪��
  gpio_init(PTD12, GPI, 0);//���뿪��
}