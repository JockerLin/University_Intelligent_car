/*
函数名称：键盘调节参数
最后修改时间：2016-1113
作者：林青春
有疑问致电邮：13950986716，lczq@vip.qq.com
*/

#include "key.h"
#include "include.h"
#include "VCAN_LED.H"
//#include  "PID.h"
int key_time_count=KEY_PRESS_TIME1;
extern uint32 motor_pwm1,motor_pwm2,motor_pwm3,motor_pwm4;
//      extern uint8 CCD_BUFFLINE[128];
uint8 flag = 1;
extern uint8 qipao ;
extern uint8 flag_t[2];
extern float Duoji_P,Duoji_I,Duoji_D,Duoji_P_Org,Duoji_D_Org;
extern uint32 Mada_Duty,Mada_Duty_Max,Mada_Duty_Min;
extern uint8 imgbuff[CAMERA_SIZE];                             //定义存储接收图像的数组
extern uint8 img[CAMERA_H][CAMERA_W],imgbuff_trans[CAMERA_SIZE];
extern uint16 ii,O_Road_Num;

/**************************PID参数**************************/
/*太后往后，太前往前*/
/************角度环************/
/*内环角速度*/
extern float P_Angle_Speed_Init,//0.08//0.055
              P_Angle_Speed_Now,
              I_Angle_Speed,//0.01,//0.05,//0.010,
              D_Angle_Speed;//0.03//0.025;//0.02;//0.015;//0.0125;//0.0125*2.3//0.02;//0.02//0.03;//0.3;//0.015
/*外环角度*/
extern float  P_Angle,I_Angle,D_Angle;
extern double fGyroscopeAngleIntegral;
/************速度环************/
/*左轮右轮分开*/
extern float g_fSpeedControlP1,//1.85//2.2,//2.35,//2.55,//2.150,//2.250,//2.28,//3.0,//2.35,
              g_fSpeedControlI1,//6,//3.5,//0.05,
              g_fSpeedControlD1,//-0.50,//4.05,//4.85,//4.85,//4.95,//4.25//4.35,//4.2//1.5,//1.2,//2.2,//25,//1.5,
      
              g_fSpeedControlP2,//1.85,//2.2,//2.35,//2.55,//2.150,//2.250,//2.28,//3.0,//2.35,
              g_fSpeedControlI2,//6,//3.5,//0.05,
              g_fSpeedControlD2;//-0.50;//4.05;//4.85;//4.85;//4.95;//4.25//4.35;//4.2//3.9//1.5;//1.2;//25;//1.5;
      
/*左右轮一起算*/
extern float g_fSpeedControlP,
               g_fSpeedControlP_Init,
              g_fSpeedControlD,
              g_fSpeedControlI;

extern float g_fCarSpeedSetL,//600,
              g_fCarSpeedSetR,//600;//120~123s
              g_fCarSpeedSet,
              g_fCarSpeedSet_Start;


extern float fLeft, fRight,Speed_Left_Max,g_fSpeedControlOutL_Max;

/************方向环************/
//P加大的话十字处理不好会偏离啊 囧

extern float P_Direction_Control,//4.5,//4.5~600,//3.4,//3.5//0.25
              P_Direction_Control_now,
              D_Direction_Control,
              D_Direction_Control_now,
              Picture_detal_Printf;//;//25;//0.35;//0.32

extern int16 FS_Spd,Dir_Spd_Fuzzy_Max,Dir_Spd_Fuzzy_Min,Ramp_Timer_Set,O_Add,Po_B;
extern float Dir_Spd_Fuzzy_constant;

uint8 keyvalue=0;
uint8 keypress=0;
uint8 keyzhuangtai=keychaxun;
uint8 buff[56][128]={0};
//extern uint8 CCD_BUFF[TSL1401_MAX*3][TSL1401_SIZE];
uint8 keycurrent =0;
int16 keycount =0;
typedef unsigned char byte;
typedef unsigned int word;

void Img_Printf_OLED();
void LCD_CLS(void);
void LCD_WrCmd(unsigned char cmd);
void LCD_Print(byte x, byte y, byte ch[]);
extern void DisplayFloat(int i,uint8 x,uint8 y);
extern uint16 Road_Kind,Road_Kind_Last;
extern uint8 Live_Watch_Mark;

void oled_display()
{       
  //  PID_DUOJI_init();
    main_scan();
}
void Key_Init()
{
   
    gpio_init(ok_key,GPI,1);
    port_init(ok_key,  ALT1 | PULLUP );
    
    gpio_init(up_key,GPI,1);
    port_init(up_key,   ALT1 | PULLUP );
    
    gpio_init(back_key,GPI,1);
    port_init(back_key,  ALT1| PULLUP ); 
    
    gpio_init(plus_key,GPI,1);
    port_init(plus_key, ALT1 | PULLUP );
    
    gpio_init(minus_key,GPI,1);
    port_init(minus_key,  ALT1 | PULLUP);
        
    gpio_init(down_key,GPI,1);
    port_init(down_key,  ALT1 | PULLUP );
}

 
uint8 getkey(uint8 *keytemp)  //按下返回键值 没按下返回0?1?
{   
    if(gpio_get(ok_key)==0)
    {
        *keytemp=keyOK;
        return key_on;
    }
    if(gpio_get(up_key)==0)//key_check  key_get
    {
        *keytemp=keyUP;
        return key_on;
    }
    if(gpio_get(back_key)==0)
    {
        *keytemp=keyBACK;
        return key_on;
    }
    if(gpio_get(plus_key)==0)
    {
        *keytemp=keyPLUS;
        return key_on;
    }
    if(gpio_get(minus_key)==0)
    {
        *keytemp=keyMINUS;
        return key_on;
    }
    if(gpio_get(down_key)==0)
    {
        *keytemp=keyDOWN;
        return  key_on;
    }
    return key_off;
}

uint8 readkey()    //按键消抖
{
    static int key_count=0;
    static uint8 flag=0;
    
    keyvalue=getkey(&keypress);
    switch(keyzhuangtai)
    {
    case keychaxun://13
      {
          if(keyvalue==key_on)
            keyzhuangtai=keyqueren;//keyzhuangtai=14
          return 0;
      }
      break;
    case keyqueren://14
      {
          if(keyvalue==key_off)                 //这里进行延时确认以消抖
          {
              keyzhuangtai=keychaxun;//keyzhuangtai=13
              return 0;
          }
          keyzhuangtai=keyshifang;//keyzhuangtai=15
          key_count=0;
          flag=0;
          return 0;
      }
      break;
      
    case keyshifang://15
      {
          if(keyvalue==key_off)
          {
              keyzhuangtai=keychaxun;
              if(flag)
                return 0;
              return keypress;
          }
          if(key_count<key_time_count)
            key_count++;
          else
          {
            key_count=0;
            flag=1;
            return keypress;
          }
          return 0;
      }
      break;
    default:
      return 0;
      break;
    }
}

void main_scan(void)    //主函数调用
{ 

  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  windows();
  flag =0;
  while(qipao)
  {
    keycurrent = readkey();   //返回值是按下的键值
    
    if(keycurrent==keyDOWN)
        {
            keycount++;
            //if(keycount==5)
            if(keycount==5)
              keycount=1;    //
        }
    
    if(keycurrent == keyUP)
        {
          keycount--;
         if(keycount < 0)     
            keycount=0;
                 
          if(keycount == 0)
             //keycount =4;
             keycount =6;
        }
    if(keycurrent==keyUP||keycurrent == keyDOWN)  //有变化，刷新页面 |  || 
        {
          switch(keycount)
          {
           case 1:
                LCD_CLS();
                windows1();//P<-
                break;
            case 2:
                LCD_CLS();
                windows2();//D<-
                break;
          case 3:
                LCD_CLS();
                windows3();//SPEED<-
                break;
          case 4:
                LCD_CLS();
                windows4();//IMG<-
                break;
          default:
                break;
          }
        }
    if(keycurrent==keyOK)
        {
            switch(keycount)
            {
              case 1:
                LCD_CLS();
                OLED_Angle_SHOW();
                break;
              case 2:
                LCD_CLS();
                OLED_Speed_SHOW();
                break;
              case 3:
                LCD_CLS();
                OLED_Direction_SHOW();
                break;
              case 4:
                LCD_CLS();
                OLED_IMA_SHOW();
                break;
            default:
                break;
            }
        }
    if(keycurrent == keyPLUS)
    {
      switch(keycount)
      {
      case 1:
        LCD_CLS();
        
      }
    }
    if(keycurrent == keyBACK)
    {
      LCD_CLS();
      qipao = 0; 
      //LCD_WrCmd(0xae);//屏幕不再显示任何东西
    }
    
    //uart_putstr (UART0,"scan");
  }
  LCD_CLS();
}


void windows(void)
{   
    LCD_Print(2,0,"1.Angle");
    LCD_Print(2,1,"2.Speed");
    LCD_Print(2,2,"3.Dir");
    LCD_Print(2,3,"4.IMA");
    LCD_Print(2,4,"O_Times:");//观测圆环个数
    LCD_PrintU16(80,4,O_Road_Num);
    LCD_Print(2,5,"Max_Speed:");//最大编码器采回的速度
    LCD_PrintU16(80,5,(int16)Speed_Left_Max);
    LCD_Print(2,6,"Max_Out:");//速度输出PWM 未与直立叠加之前
    LCD_PrintU16(80,6,(int16)abs((int16)g_fSpeedControlOutL_Max));
}
void windows1(void)
{      
    LCD_Print(2,0,"1.Angle      <-");
    LCD_Print(2,1,"2.Speed");
    LCD_Print(2,2,"3.Dir");
    LCD_Print(2,3,"4.IMA");
}
void windows2(void)
{     
    LCD_Print(2,0,"1.Angle");
    LCD_Print(2,1,"2.Speed      <-");
    LCD_Print(2,2,"3.Dir");
    LCD_Print(2,3,"4.IMA");
}
void windows3(void)
{     
    LCD_Print(2,0,"1.Angle");
    LCD_Print(2,1,"2.Speed");
    LCD_Print(2,2,"3.Dir      <-");
    LCD_Print(2,3,"4.IMA");
}
void windows4(void)
{     
    LCD_Print(2,0,"1.Angle");
    LCD_Print(2,1,"2.Speed");
    LCD_Print(2,2,"3.Dir");
    LCD_Print(2,3,"4.IMA      <-");
    
}

/************************************角度调节*******************************/
void OLED_Angle_SHOW(void)//角度调节
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Angle_IO_PID();
  while(qipao)
  {
   keycurrent=readkey();
   /*
   if(keycurrent == keyPLUS)
   {
      keycount--;
      if(keycount<0)
        keycount=0;
      if(keycount==0)
        keycount=6;
   }
   if(keycurrent == keyMINUS)
   {
      keycount++;
      if(keycount==6)
        keycount = 1;
   }
   switch(keycount)
   {
   case 1:
     {
       LCD_CLS();
       OLED_Angle_IP_SHOW();
     }
   case 2:
     {
       LCD_CLS();
       OLED_Angle_II_SHOW();
     }
   case 3:
     {
       LCD_CLS();
       OLED_Angle_ID_SHOW();
     }
   case 4:
     {
       LCD_CLS();
       OLED_Angle_OP_SHOW();
     }
   case 5:
     {
       LCD_CLS();
       OLED_Angle_OI_SHOW();
     }
   case 6:
     {
       LCD_CLS();
       OLED_Angle_OD_SHOW();
     }
   default:
     break;
   }
   */
   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_Angle_IP_SHOW();

   }
   //if(keycurrent == keyUP)
   //{
   //   LCD_CLS(); 
   //   OLED_Angle_OD_SHOW();

   //}

   if(keycurrent == keyBACK)
   {
      LCD_CLS();
      main_scan();
      break;     
   }

  }
}

void OLED_Angle_IP_SHOW(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Angle_IP();
  while(qipao)
  {
   keycurrent=readkey();
   if(keycurrent == keyPLUS)
   {
      P_Angle_Speed_Init +=0.01; 
      P_Angle_Speed_Now +=0.01; 
      OLED_Angle_IP();
   }
   if(keycurrent == keyMINUS)
   {
      P_Angle_Speed_Init  -=0.01;
      P_Angle_Speed_Now  -=0.01;
      OLED_Angle_IP();
   }
   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_Angle_II_SHOW();
   }
   if(keycurrent == keyUP)
   {
     LCD_CLS();
     OLED_Angle_OD_SHOW();
   }
   if(keycurrent == keyBACK)
   {
     LCD_CLS();
     main_scan();
    break;
   }

  } 
}

void OLED_Angle_II_SHOW(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Angle_II();
  while(qipao)
  {
   keycurrent=readkey();
   if(keycurrent == keyPLUS)
   {
      I_Angle_Speed +=0.1; 
      OLED_Angle_II();
   }
   if(keycurrent == keyMINUS)
   {
      I_Angle_Speed  -=0.1;
      OLED_Angle_II();
   }
   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_Angle_ID_SHOW();
   }
   if(keycurrent == keyUP)
   {
     LCD_CLS();
     OLED_Angle_IP_SHOW();
   }
   if(keycurrent == keyBACK)
   {
     LCD_CLS();
     main_scan();
    break;
   }

  } 
}

void OLED_Angle_ID_SHOW(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Angle_ID();
  while(qipao)
  {
   keycurrent=readkey();
   if(keycurrent == keyPLUS)
   {
      D_Angle_Speed +=0.001;
      OLED_Angle_ID();
   }
   if(keycurrent == keyMINUS)
   {
      D_Angle_Speed  -=0.001;
      OLED_Angle_ID();
   }
   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_Angle_OP_SHOW();
   }
   if(keycurrent == keyUP)
   {
     LCD_CLS();
     OLED_Angle_II_SHOW();
   }
   if(keycurrent == keyBACK)
   {
     LCD_CLS();
     main_scan();
    break;
   }

  } 
}

void OLED_Angle_OP_SHOW(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Angle_OP();
  while(qipao)
  {
   keycurrent=readkey();
   if(keycurrent == keyPLUS)
   {
      P_Angle +=0.1; 
      OLED_Angle_OP();
   }
   if(keycurrent == keyMINUS)
   {
      P_Angle -=0.1;
      OLED_Angle_OP();
   }
   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_Angle_OI_SHOW();
   }
   if(keycurrent == keyUP)
   {
     LCD_CLS();
     OLED_Angle_ID_SHOW();
   }
   if(keycurrent == keyBACK)
   {
     LCD_CLS();
     main_scan();
    break;
   }

  } 
}

void OLED_Angle_OI_SHOW(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Angle_OI();
  while(qipao)
  {
   keycurrent=readkey();
   if(keycurrent == keyPLUS)
   {
      D_Angle +=0.1; 
      OLED_Angle_OI();
   }
   if(keycurrent == keyMINUS)
   {
      D_Angle -=0.1;
      OLED_Angle_OI();
   }
   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_Angle_OD_SHOW();
   }
   if(keycurrent == keyUP)
   {
     LCD_CLS();
     OLED_Angle_OP_SHOW();
   }
   if(keycurrent == keyBACK)
   {
     LCD_CLS();
     main_scan();
    break;
   }

  } 
}

void OLED_Angle_OD_SHOW(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Angle_OD();
  while(qipao)
  {
   keycurrent=readkey();
   if(keycurrent == keyPLUS)
   {
      D_Angle +=0.1; 
      OLED_Angle_OD();
   }
   if(keycurrent == keyMINUS)
   {
      D_Angle -=0.1;
      OLED_Angle_OD();
   }
   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_Angle_IP_SHOW();
   }
   if(keycurrent == keyUP)
   {
     LCD_CLS();
     OLED_Angle_OI_SHOW();
   }
   if(keycurrent == keyBACK)
   {
     LCD_CLS();
     main_scan();
    break;
   }

  } 
}

void OLED_Angle_IO_PID(void)
{
    LCD_Print(2,0,"1.IP");
    LCD_Print(2,1,"2.II");
    LCD_Print(2,2,"3.ID");
    LCD_Print(2,3,"4.OP");
    LCD_Print(2,4,"5.OI");
    LCD_Print(2,5,"6.OD");
    DisplayFloat((int16)(P_Angle_Speed_Init*1000),36,0);
    DisplayFloat((int16)(I_Angle_Speed*1000),36,1);
    DisplayFloat((int16)(D_Angle_Speed*1000),36,2);
    DisplayFloat((int16)(P_Angle*1000),36,3);
    DisplayFloat((int16)(I_Angle*1000),36,4);
    DisplayFloat((int16)(D_Angle*1000),36,5);
}

void OLED_Angle_IP(void)//箭头指向P
{
    LCD_Print(2,0,"1.IP         <-");
    LCD_Print(2,1,"2.II");
    LCD_Print(2,2,"3.ID");
    LCD_Print(2,3,"4.OP");
    LCD_Print(2,4,"5.OI");
    LCD_Print(2,5,"6.OD");
    DisplayFloat((int16)(P_Angle_Speed_Init*1000),36,0);
    DisplayFloat((int16)(I_Angle_Speed*1000),36,1);
    DisplayFloat((int16)(D_Angle_Speed*1000),36,2);
    DisplayFloat((int16)(P_Angle*1000),36,3);
    DisplayFloat((int16)(I_Angle*1000),36,4);
    DisplayFloat((int16)(D_Angle*1000),36,5);
}

void OLED_Angle_II(void)//箭头指向I
{
    LCD_Print(2,0,"1.IP");
    LCD_Print(2,1,"2.II         <-");
    LCD_Print(2,2,"3.ID");
    LCD_Print(2,3,"4.OP");
    LCD_Print(2,4,"5.OI");
    LCD_Print(2,5,"6.OD");
    DisplayFloat((int16)(P_Angle_Speed_Init*1000),36,0);
    DisplayFloat((int16)(I_Angle_Speed*1000),36,1);
    DisplayFloat((int16)(D_Angle_Speed*1000),36,2);
    DisplayFloat((int16)(P_Angle*1000),36,3);
    DisplayFloat((int16)(I_Angle*1000),36,4);
    DisplayFloat((int16)(D_Angle*1000),36,5);
}

void OLED_Angle_ID(void)//箭头指向D
{
    LCD_Print(2,0,"1.IP");
    LCD_Print(2,1,"2.II");
    LCD_Print(2,2,"3.ID         <-");
    LCD_Print(2,3,"4.OP");
    LCD_Print(2,4,"5.OI");
    LCD_Print(2,5,"6.OD");
    DisplayFloat((int16)(P_Angle_Speed_Init*1000),36,0);
    DisplayFloat((int16)(I_Angle_Speed*1000),36,1);
    DisplayFloat((int16)(D_Angle_Speed*1000),36,2);
    DisplayFloat((int16)(P_Angle*1000),36,3);
    DisplayFloat((int16)(I_Angle*1000),36,4);
    DisplayFloat((int16)(D_Angle*1000),36,5);
}

void OLED_Angle_OP(void)//箭头指向外环P
{
    LCD_Print(2,0,"1.IP");
    LCD_Print(2,1,"2.II");
    LCD_Print(2,2,"3.ID");
    LCD_Print(2,3,"4.OP         <-");
    LCD_Print(2,4,"5.OI");
    LCD_Print(2,5,"6.OD");
    DisplayFloat((int16)(P_Angle_Speed_Init*1000),36,0);
    DisplayFloat((int16)(I_Angle_Speed*1000),36,1);
    DisplayFloat((int16)(D_Angle_Speed*1000),36,2);
    DisplayFloat((int16)(P_Angle*1000),36,3);
    DisplayFloat((int16)(I_Angle*1000),36,4);
    DisplayFloat((int16)(D_Angle*1000),36,5);
}

void OLED_Angle_OI(void)//箭头指向外环D
{
    LCD_Print(2,0,"1.IP");
    LCD_Print(2,1,"2.II");
    LCD_Print(2,2,"3.ID");
    LCD_Print(2,3,"4.OP");
    LCD_Print(2,4,"5.OI         <-");
    LCD_Print(2,5,"6.OD");
    DisplayFloat((int16)(P_Angle_Speed_Init*1000),36,0);
    DisplayFloat((int16)(I_Angle_Speed*1000),36,1);
    DisplayFloat((int16)(D_Angle_Speed*1000),36,2);
    DisplayFloat((int16)(P_Angle*1000),36,3);
    DisplayFloat((int16)(I_Angle*1000),36,4);
    DisplayFloat((int16)(D_Angle*1000),36,5);
}

void OLED_Angle_OD(void)//箭头指向外环D
{
    LCD_Print(2,0,"1.IP");
    LCD_Print(2,1,"2.II");
    LCD_Print(2,2,"3.ID");
    LCD_Print(2,3,"4.OP");
    LCD_Print(2,4,"5.OI");
    LCD_Print(2,5,"6.OD         <-");
    DisplayFloat((int16)(P_Angle_Speed_Init*1000),36,0);
    DisplayFloat((int16)(I_Angle_Speed*1000),36,1);
    DisplayFloat((int16)(D_Angle_Speed*1000),36,2);
    DisplayFloat((int16)(P_Angle*1000),36,3);
    DisplayFloat((int16)(I_Angle*1000),36,4);
    DisplayFloat((int16)(D_Angle*1000),36,5);
}


/******************************************速度调节****************************/

void OLED_Speed_SHOW(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Speed_PID();
  while(qipao)
  {
   keycurrent=readkey();
   /*
   if(keycurrent == keyPLUS)
   {
      keycount--;
      if(keycount < 0)          
        keycount=0;
      if(keycount==0)
        keycount=3;
   }
   if(keycurrent == keyMINUS)
   {
      keycount++;
      if(keycount==4)
        keycount = 1;
   }
   switch(keycount)
   {
   case 1:
     {
       LCD_CLS();
       OLED_Speed_P_SHOW();
     }
   case 2:
     {
       LCD_CLS();
       OLED_Speed_I_SHOW();
     }
   case 3:
     {
       LCD_CLS();
       OLED_Speed_D_SHOW();
     }
   case 4:
     {
       LCD_CLS();
       OLED_Speed_Set_SHOW();
     }
   default:
     break;
   }
   */

   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_Speed_P_SHOW();

   }
   if(keycurrent == keyBACK)
   {
      LCD_CLS();
      main_scan();
      break;     
   }

  } 
}

void OLED_Speed_P_SHOW(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Speed_P();
  while(qipao)
  {
   keycurrent=readkey();
   if(keycurrent == keyPLUS)
   {
      g_fSpeedControlP +=0.1; 
      g_fSpeedControlP_Init+=0.1;
      OLED_Speed_P();
   }
   if(keycurrent == keyMINUS)
   {
      g_fSpeedControlP  -=0.1;
      g_fSpeedControlP_Init-=0.1;
      OLED_Speed_P();
   }
   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_Speed_I_SHOW();
   }
   if(keycurrent == keyUP)
   {
     LCD_CLS();
     OLED_Speed_Set_SHOW();
   }
   if(keycurrent == keyBACK)
   {
     LCD_CLS();
     main_scan();
    break;
   }

  } 
}

void OLED_Speed_I_SHOW(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Speed_I();
  while(qipao)
  {
   keycurrent=readkey();
   if(keycurrent == keyPLUS)
   {
      g_fSpeedControlI +=0.005; 
      OLED_Speed_I();
   }
   if(keycurrent == keyMINUS)
   {
      g_fSpeedControlI  -=0.005;
      OLED_Speed_I();
   }
   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_Speed_D_SHOW();
   }
   if(keycurrent == keyUP)
   {
     LCD_CLS();
     OLED_Speed_P_SHOW();
   }
   if(keycurrent == keyBACK)
   {
     LCD_CLS();
     main_scan();
    break;
   }

  } 
}

void OLED_Speed_D_SHOW(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Speed_D();
  while(qipao)
  {
   keycurrent=readkey();
   if(keycurrent == keyPLUS)
   {
      g_fSpeedControlD +=0.1; 
      OLED_Speed_D();
   }
   if(keycurrent == keyMINUS)
   {
      g_fSpeedControlD  -=0.1;
      OLED_Speed_D();
   }
   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_Speed_Set_SHOW();
   }
   if(keycurrent == keyUP)
   {
     LCD_CLS();
     OLED_Speed_I_SHOW();
   }
   if(keycurrent == keyBACK)
   {
     LCD_CLS();
     main_scan();
    break;
   }

  } 
}

void OLED_Speed_Set_SHOW(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Speed_Set();
  while(qipao)
  {
   keycurrent=readkey();
   if(keycurrent == keyPLUS)
   {
      g_fCarSpeedSet +=50; 
      g_fCarSpeedSet_Start +=50;
      OLED_Speed_Set();
   }
   if(keycurrent == keyMINUS)
   {
      g_fCarSpeedSet  -=50;
      g_fCarSpeedSet_Start -=50;
      OLED_Speed_Set();
   }
   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_Speed_P_SHOW();
   }
   if(keycurrent == keyUP)
   {
     LCD_CLS();
     OLED_Speed_D_SHOW();
   }
   if(keycurrent == keyBACK)
   {
     LCD_CLS();
     main_scan();
    break;
   }

  } 
}

void OLED_Speed_PID(void)
{
    LCD_Print(2,0,"1.P");
    LCD_Print(2,1,"2.I");
    LCD_Print(2,2,"3.D");
    LCD_Print(2,3,"4.V");
    DisplayFloat((int32)(g_fSpeedControlP*1000),24,0);
    DisplayFloat((int32)(g_fSpeedControlI*1000),24,1);
    DisplayFloat((int32)(g_fSpeedControlD*1000),24,2);
    DisplayFloat((int32)(g_fCarSpeedSet*10),24,3);
}

void OLED_Speed_P(void)//箭头指向P
{
    LCD_Print(2,0,"1.P         <-");
    LCD_Print(2,1,"2.I");
    LCD_Print(2,2,"3.D");
    LCD_Print(2,3,"4.V");
    DisplayFloat((int32)(g_fSpeedControlP*1000),24,0);
    DisplayFloat((int32)(g_fSpeedControlI*1000),24,1);
    DisplayFloat((int32)(g_fSpeedControlD*1000),24,2);
    DisplayFloat((int32)(g_fCarSpeedSet*10),24,3);
}

void OLED_Speed_I(void)//箭头指向I
{
    LCD_Print(2,0,"1.P");
    LCD_Print(2,1,"2.I         <-");
    LCD_Print(2,2,"3.D");
    LCD_Print(2,3,"4.V");
    DisplayFloat((int32)(g_fSpeedControlP*1000),24,0);
    DisplayFloat((int32)(g_fSpeedControlI*1000),24,1);
    DisplayFloat((int32)(g_fSpeedControlD*1000),24,2);
    DisplayFloat((int32)(g_fCarSpeedSet*10),24,3);
    
}

void OLED_Speed_D(void)//箭头指向D
{
    LCD_Print(2,0,"1.P");
    LCD_Print(2,1,"2.I");
    LCD_Print(2,2,"3.D         <-");
    LCD_Print(2,3,"4.V");
    DisplayFloat((int32)(g_fSpeedControlP*1000),24,0);
    DisplayFloat((int32)(g_fSpeedControlI*1000),24,1);
    DisplayFloat((int32)(g_fSpeedControlD*1000),24,2);
    DisplayFloat((int32)(g_fCarSpeedSet*10),24,3);
    
}

void OLED_Speed_Set(void)//箭头指向设定速度
{
    LCD_Print(2,0,"1.P");
    LCD_Print(2,1,"2.I");
    LCD_Print(2,2,"3.D");
    LCD_Print(2,3,"4.V         <-");
    DisplayFloat((int32)(g_fSpeedControlP*1000),24,0);
    DisplayFloat((int32)(g_fSpeedControlI*1000),24,1);
    DisplayFloat((int32)(g_fSpeedControlD*1000),24,2);
    DisplayFloat((int32)(g_fCarSpeedSet*10),24,3);
    
}





/******************************************方向调节****************************/

void OLED_Direction_SHOW(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Direction_PID();
  while(qipao)
  {
   keycurrent=readkey();
   /*
   if(keycurrent == keyPLUS)
   {
      keycount--;
      if(keycount < 0)          
        keycount=0;
      if(keycount==0)
        keycount=2;
   }
   if(keycurrent == keyMINUS)
   {
      keycount++;
      if(keycount==3)
        keycount = 1;
   }
   switch(keycount)
   {
   case 1:
     {
       LCD_CLS();
       OLED_Direction_P_SHOW();
     }
   case 2:
     {
       LCD_CLS();
       OLED_Direction_D_SHOW();
     }
   default:
     break;
   }
   */

   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_Direction_P_SHOW();

   }
   if(keycurrent == keyBACK)
   {
      LCD_CLS();
      main_scan();
      break;     
   }

  } 
}

void OLED_Direction_P_SHOW(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Direction_P();
  while(qipao)
  {
   keycurrent=readkey();
   if(keycurrent == keyPLUS)
   {
      P_Direction_Control +=1;
      P_Direction_Control_now+=1;
      OLED_Direction_P();
   }
   if(keycurrent == keyMINUS)
   {
      P_Direction_Control  -=1;
      P_Direction_Control_now-=1;
      OLED_Direction_P();
   }
   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_Direction_D_SHOW();
   }
   if(keycurrent == keyUP)
   {
     LCD_CLS();
     OLED_O_Show();
   }
   if(keycurrent == keyBACK)
   {
     LCD_CLS();
     main_scan();
    break;
   }

  } 
}

void OLED_Direction_D_SHOW(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Direction_D();
  while(qipao)
  {
   keycurrent=readkey();
   if(keycurrent == keyPLUS)
   {
      D_Direction_Control +=0.001;
      D_Direction_Control_now +=0.001; 
      OLED_Direction_D();
   }
   if(keycurrent == keyMINUS)
   {
      D_Direction_Control_now  -=0.001;
      D_Direction_Control  -=0.001;
      OLED_Direction_D();
   }
   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_Direction_Po_SHOW();
   }
   if(keycurrent == keyUP)
   {
     LCD_CLS();
     OLED_Direction_P_SHOW();
   }
   if(keycurrent == keyBACK)
   {
     LCD_CLS();
     main_scan();
    break;
   }

  }
}

void OLED_Direction_Po_SHOW(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Direction_Po();
  while(qipao)
  {
   keycurrent=readkey();
   if(keycurrent == keyPLUS)
   {
      Ramp_Timer_Set+=5;
      OLED_Direction_Po();
   }
   if(keycurrent == keyMINUS)
   {
      Ramp_Timer_Set-=5;
      OLED_Direction_Po();
   }
   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_Direction_W_SHOW();
   }
   if(keycurrent == keyUP)
   {
     LCD_CLS();
     OLED_Direction_D_SHOW();
   }
   if(keycurrent == keyBACK)
   {
     LCD_CLS();
     main_scan();
    break;
   }

  }
}

void OLED_Direction_W_SHOW(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Direction_W();
  while(qipao)
  {
   keycurrent=readkey();
   if(keycurrent == keyPLUS)
   {
      Po_B +=1;
      OLED_Direction_W();
   }
   if(keycurrent == keyMINUS)
   {
      Po_B  -=1;
      OLED_Direction_W();
   }
   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_Direction_C_SHOW();
   }
   if(keycurrent == keyUP)
   {
     LCD_CLS();
     OLED_Direction_Po_SHOW();
   }
   if(keycurrent == keyBACK)
   {
     LCD_CLS();
     main_scan();
    break;
   }

  }
}

void OLED_Direction_C_SHOW(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Direction_C();
  while(qipao)
  {
   keycurrent=readkey();
   if(keycurrent == keyPLUS)
   {
      Dir_Spd_Fuzzy_constant +=0.1;
      OLED_Direction_C();
   }
   if(keycurrent == keyMINUS)
   {
      Dir_Spd_Fuzzy_constant  -=0.1;
      OLED_Direction_C();
   }
   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_Direction_Live_Watch_Show();
   }
   if(keycurrent == keyUP)
   {
     LCD_CLS();
     OLED_Direction_W_SHOW();
   }
   if(keycurrent == keyBACK)
   {
     LCD_CLS();
     main_scan();
    break;
   }

  }
}

void OLED_Direction_Live_Watch_Show(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Direction_Live_Watch();
  while(qipao)
  {
   keycurrent=readkey();
   if(keycurrent == keyPLUS)
   {
      Live_Watch_Mark =1;
      OLED_Direction_Live_Watch();
   }
   if(keycurrent == keyMINUS)
   {
      Live_Watch_Mark=0;
      OLED_Direction_Live_Watch();
   }
   if(keycurrent == keyDOWN)
   {
      LCD_CLS(); 
      OLED_O_Show();
   }
   if(keycurrent == keyUP)
   {
     LCD_CLS();
     OLED_Direction_C_SHOW();
   }
   if(keycurrent == keyBACK)
   {
     LCD_CLS();
     main_scan();
    break;
   }

  }
}

void OLED_O_Show(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  OLED_Direction_O();
  while(qipao)
  {
   keycurrent=readkey();
   if(keycurrent == keyPLUS)
   {
      O_Add +=1;
      OLED_Direction_O();
   }
   if(keycurrent == keyMINUS)
   {
      O_Add -=1;
      OLED_Direction_O();
   }
   if(keycurrent == keyDOWN)
   {
      LCD_CLS();
      OLED_Direction_P_SHOW();
   }
   if(keycurrent == keyUP)
   {
     LCD_CLS();
     OLED_Direction_Live_Watch_Show();
   }
   if(keycurrent == keyBACK)
   {
     LCD_CLS();
     main_scan();
    break;
   }

  }
}

void OLED_Direction_PID(void)
{
    LCD_Print(2,0,"1.P");
    LCD_Print(2,1,"2.D");
    LCD_Print(2,2,"3.Po");
    LCD_Print(2,3,"4.W");//速度-方向模糊的基准值
    LCD_Print(2,4,"5.C");//速度-方向模糊的比例常数
    LCD_Print(2,5,"6.L");
    LCD_Print(2,6,"7.O");
    DisplayFloat((int32)(P_Direction_Control*1000),40,0);
    DisplayFloat((int32)(D_Direction_Control*1000),40,1);
    LCD_PrintU16(40,2,Ramp_Timer_Set);
    DisplayFloat(Po_B*1000,40,3);
    DisplayFloat((int32)(Dir_Spd_Fuzzy_constant*1000),40,4);
    DisplayFloat(Live_Watch_Mark*1000,40,5);
    DisplayFloat(O_Add*1000,40,6);
}

void OLED_Direction_P(void)//箭头指向P
{
    LCD_Print(2,0,"1.P         <-");
    LCD_Print(2,1,"2.D");
    LCD_Print(2,2,"3.Po");
    LCD_Print(2,3,"4.W");//速度-方向模糊的基准值
    LCD_Print(2,4,"5.C");//速度-方向模糊的比例常数
    LCD_Print(2,5,"6.L");
    LCD_Print(2,6,"7.O");
    DisplayFloat((int32)(P_Direction_Control*1000),40,0);
    DisplayFloat((int32)(D_Direction_Control*1000),40,1);
    LCD_PrintU16(40,2,Ramp_Timer_Set);
    DisplayFloat(Po_B*1000,40,3);
    DisplayFloat((int32)(Dir_Spd_Fuzzy_constant*1000),40,4);
    DisplayFloat(Live_Watch_Mark*1000,40,5);
    DisplayFloat(O_Add*1000,40,6);
}

void OLED_Direction_D(void)//箭头指向D
{
    LCD_Print(2,0,"1.P");
    LCD_Print(2,1,"2.D         <-");
    LCD_Print(2,2,"3.Po");
    LCD_Print(2,3,"4.W");//速度-方向模糊的基准值
    LCD_Print(2,4,"5.C");//速度-方向模糊的比例常数
    LCD_Print(2,5,"6.L");
    LCD_Print(2,6,"7.O");
    DisplayFloat((int32)(P_Direction_Control*1000),40,0);
    DisplayFloat((int32)(D_Direction_Control*1000),40,1);
    LCD_PrintU16(40,2,Ramp_Timer_Set);
    DisplayFloat(Po_B*1000,40,3);
    DisplayFloat((int32)(Dir_Spd_Fuzzy_constant*1000),40,4);
    DisplayFloat(Live_Watch_Mark*1000,40,5);
    DisplayFloat(O_Add*1000,40,6);
}

void OLED_Direction_Po(void)//箭头指向po
{
    LCD_Print(2,0,"1.P");
    LCD_Print(2,1,"2.D");
    LCD_Print(2,2,"3.Po         <-");
    LCD_Print(2,3,"4.W");//速度-方向模糊的基准值
    LCD_Print(2,4,"5.C");//速度-方向模糊的比例常数
    LCD_Print(2,5,"6.L");
    LCD_Print(2,6,"7.O");
    DisplayFloat((int32)(P_Direction_Control*1000),40,0);
    DisplayFloat((int32)(D_Direction_Control*1000),40,1);
    LCD_PrintU16(40,2,Ramp_Timer_Set);
    DisplayFloat(Po_B*1000,40,3);
    DisplayFloat((int32)(Dir_Spd_Fuzzy_constant*1000),40,4);
    DisplayFloat(Live_Watch_Mark*1000,40,5);
    DisplayFloat(O_Add*1000,40,6);
}

void OLED_Direction_W(void)//箭头指向FS
{
    LCD_Print(2,0,"1.P");
    LCD_Print(2,1,"2.D");
    LCD_Print(2,2,"3.Po");
    LCD_Print(2,3,"4.W         <-");//速度-方向模糊的基准值
    LCD_Print(2,4,"5.C");//速度-方向模糊的比例常数
    LCD_Print(2,5,"6.L");
    LCD_Print(2,6,"7.O");
    DisplayFloat((int32)(P_Direction_Control*1000),40,0);
    DisplayFloat((int32)(D_Direction_Control*1000),40,1);
    LCD_PrintU16(40,2,Ramp_Timer_Set);
    DisplayFloat(Po_B*1000,40,3);
    DisplayFloat((int32)(Dir_Spd_Fuzzy_constant*1000),40,4);
    DisplayFloat(Live_Watch_Mark*1000,40,5);
    DisplayFloat(O_Add*1000,40,6);
}

void OLED_Direction_C(void)//箭头指向C
{
    LCD_Print(2,0,"1.P");
    LCD_Print(2,1,"2.D");
    LCD_Print(2,2,"3.Po");
    LCD_Print(2,3,"4.W");//速度-方向模糊的基准值
    LCD_Print(2,4,"5.C         <-");//速度-方向模糊的比例常数
    LCD_Print(2,5,"6.L");
    LCD_Print(2,6,"7.O");
    DisplayFloat((int32)(P_Direction_Control*1000),40,0);
    DisplayFloat((int32)(D_Direction_Control*1000),40,1);
    LCD_PrintU16(40,2,Ramp_Timer_Set);
    DisplayFloat(Po_B*1000,40,3);
    DisplayFloat((int32)(Dir_Spd_Fuzzy_constant*1000),40,4);
    DisplayFloat(Live_Watch_Mark*1000,40,5);
    DisplayFloat(O_Add*1000,40,6);
}

void OLED_Direction_Live_Watch(void)//箭头指向Live_Watch
{
    LCD_Print(2,0,"1.P");
    LCD_Print(2,1,"2.D");
    LCD_Print(2,2,"3.Po");
    LCD_Print(2,3,"4.W");//速度-方向模糊的基准值
    LCD_Print(2,4,"5.C");//速度-方向模糊的比例常数
    LCD_Print(2,5,"6.L         <-");
    LCD_Print(2,6,"7.O");
    DisplayFloat((int32)(P_Direction_Control*1000),40,0);
    DisplayFloat((int32)(D_Direction_Control*1000),40,1);
    LCD_PrintU16(40,2,Ramp_Timer_Set);
    DisplayFloat(Po_B*1000,40,3);
    DisplayFloat((int32)(Dir_Spd_Fuzzy_constant*1000),40,4);
    DisplayFloat(Live_Watch_Mark*1000,40,5);
    DisplayFloat(O_Add*1000,40,6);
}

void OLED_Direction_O(void)
{
    LCD_Print(2,0,"1.P");
    LCD_Print(2,1,"2.D");
    LCD_Print(2,2,"3.Po");
    LCD_Print(2,3,"4.W");//速度-方向模糊的基准值
    LCD_Print(2,4,"5.C");//速度-方向模糊的比例常数
    LCD_Print(2,5,"6.L");
    LCD_Print(2,6,"7.O         <-");
    DisplayFloat((int32)(P_Direction_Control*1000),40,0);
    DisplayFloat((int32)(D_Direction_Control*1000),40,1);
    LCD_PrintU16(40,2,Ramp_Timer_Set);
    DisplayFloat(Po_B*1000,40,3);
    DisplayFloat((int32)(Dir_Spd_Fuzzy_constant*1000),40,4);
    DisplayFloat(Live_Watch_Mark*1000,40,5);
    DisplayFloat(O_Add*1000,40,6);
}
/*
void OLED_Direction_O(void)
{
    LCD_Print(2,0,"1.P");
    LCD_Print(2,1,"2.D");
    LCD_Print(2,2,"3.Po");
    LCD_Print(2,3,"4.W");//速度-方向模糊的基准值
    LCD_Print(2,4,"5.C");//速度-方向模糊的比例常数
    LCD_Print(2,5,"6.L");
    LCD_Print(2,6,"7.O         <-");
    DisplayFloat((int32)(P_Direction_Control*1000),40,0);
    DisplayFloat((int32)(D_Direction_Control*1000),40,1);
    LCD_PrintU16(40,2,Ramp_Timer_Set);
    DisplayFloat(Po_B*1000,40,3);
    DisplayFloat((int32)(Dir_Spd_Fuzzy_constant*1000),40,4);
    DisplayFloat(Live_Watch_Mark*1000,40,5);
    DisplayFloat(O_Add*1000,40,6);
}*/

/******************************************图像显示****************************/

void OLED_IMA_SHOW(void)
{
  keycount = 0;
  keycurrent = 0;
  LCD_CLS();
  //OLED_Speed_PID();
  while(qipao)
  {
    keycurrent=readkey();
      camera_get_img();                                      //摄像头获取图像 采集和解压共耗时8ms
      img_extract((uint8 *)img,(uint8 *)imgbuff,CAMERA_SIZE);//解压为灰度图像，方便发送到上位机显示
      Finish_Check();
      Get_Middle_Line();
      Direction_Detal_Caculate();
      Dir_Loop_Output();
      Draw_Midline();
      Img_Printf_OLED();
    if(keycurrent == keyBACK)
   {
      LCD_CLS();
      main_scan();
      break;
   }
  }
}

//打印路况种类和偏差


