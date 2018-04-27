/*�ϰ����Ľ� 17-08-03-11:53
�������ƣ�get_midline.c(����������ȡ���� )
����޸�ʱ�䣺20161018-20170803
���ߣ�������ʵ�������ܳ���-��ӿ
�������µ��ʣ�13950986716��lczq@vip.qq.com
���Լ�¼��161009�����Ӳ������飬��·�ֶδ���
        ��161013������ֱ��PD���������ȣ�ֱ�������е㶶�����ƣ�2.25m/s--���ִ�����
        ��161017�����ܵ����ܳɹ���ֱ������ͨ���ֶδ�������СS��ʮ���������ǰհ�ɼӳ�������Ҫ���ӡ�
          161119��ʮ����ƫ��������Ȼδ���. �����ֱ��ƫ�����⣬����ֻ�Ǽ򵥵���ʮ�ֶ��ߴ������ߣ���û�вɼ����ĸ��յ�
                  ��Ҫ���ݹյ㻭�����߰������ܼ򵥵ز����ߡ�
          161207:б��ʮ��������Ч�������ǲ��ȶ���Ӱ�����ض࣬������ֱ�����У����账��ã�������
          161223:��·�޸��У�Ӱ�����ض࣬���ж����ȡ�
          170101�����Ż�����ǰ�ж�б��ʮ�� ��ֹ���̫�������� Բ��O ʮ��
          170102��������������ѽ�� ��б��ʮ�ֳַ����жϺ��� Բ��Ҳ�� ����б��ʮ����Ȼ���� ͨ���ɼ����ĸ��յ�����
                  ����������ʲôԭ��!!!!!Ҫ����
          170105����ʱȡ��ȡ�ĸ�����뷨����ΪŪ��4�죬ʧ�ܡ����罫100��ͷ����90������ ���������ã�����100��������������+
                  �˲����뷨 ���ܲ���ʮ�ֶ��������ܶ࣬�����ǻᶶ�㣬֮ǰ��90б��ʮ�ֽ���ȥ���Ż������ܲ��У��������˲��� -1.9m
          170105:�Ż����Һ��ߵ������ԣ�������·����������ʮ�֣�������Ȼ��ȱ�����ᶶ�������ȶ��� ���Ż� ͳ�ƶ�ʧ�������Ǹ�BUG�� -2.0m
          170111���ٴ��Ż�ʮ�֣�ʮ�ֳ���������٣�������Ȼ����죬����ʱ��Ҳ��ż�����ˣ����Ż�����14s�� ���13.98s 1800�ٶ� -2.24m 
          170214:1850 14.2s 2.21m/s ʱ���ܷ� ��֪��Ϊʲô����
          170215��Բ���������ߣ����ǻ�ʱ��ʧ�����߲�ϰ�ߡ�
          170217����������⣺���Ż�ʮ�� 2�㡢3�㡢4�㣻��Բ��ԭ�����⻷�׳��죻���ٶ��Ѵﵽ����ṹƿ��������or�Ľṹ��
          170303��δ������⣺1������Բ����ʮ��ǰ+���� 2������Բ��������
          170406: ���»���Ҫ��������Ϊ����ͼ������Ϊ������
          170412:��D2500-2.5m �������� Բ���Һ����⣬ʮ�ֻ��ܸĽ�������΢Ťһ�� Ŀǰ�����ϰ����µ�δ����
          170505:���½������ʮ��
          170510:δ��ɣ���¼Բ���ĸ������ϰ����µ�
          170730����Բ���붼��ʶ�� ���������� �ϰ�����ҪԲ��Щ ����̫����
          170802����150��Բ�����붼�У����ǲ���ƽ����100����ȥ���ո����������ʼ��٣���Ȼ���ڣ����������ڰ���Ǯ���������ϰ������޸�
          170803���ϰ�������
*************************************************************************************************/
//�˺�����0x00��ʾ��ɫ����0xff��ʾ��ɫ����
//17-1-1 �Ż��� 
//���Ż�����ǰ�ж�б��ʮ�� ��ֹ���̫�������� 
#include "common.h"
#include "include.h"

#define Black 0x00
#define White 0xff

//��������ͷ25cm-32cm ̫�ͻ���̫���أ�̫��ת������̫�� 

int i,j,Flag_Right=0,Flag_Left=0,jj,ii;        //�����б���i=0~60���б���j=0~80
extern uint8 img[CAMERA_H][CAMERA_W];      //��չ����ͼ��洢����img
extern int32 Direction_Control,Mada_Duty;  //
uint16 Left_Black_Lose,Right_Black_Lose;    //
int32 LB_Lose[60],RB_Lose[60];
uint16 Left_Excess[60],Right_Excess[60];
uint16 Mid_Line[60],Error[60];                        //�к��ߴ洢����
int32 Left_Black[60],Left_Black_Old;      //��ߺ��ߴ洢����
int32 Right_Black[60],Right_Black_Old;    //�ұߺ��ߴ洢����
int16  LB_Lose_Count=0,RB_Lose_Count=0,LB_Lose_Begin,LB_Lose_End,
      RB_Lose_Begin,RB_Lose_End,Left_and_Right_Lose_count,Left_and_Right_Lose_count_Last,
      LB_Lose_Count_Last,RB_Lose_Count_Last;
int32 Left_Board_Min,Left_Board_Max,Right_Board_Min,Right_Board_Max;
uint8 Cross_Road_Connect_Flag=0;//ʮ�����ӱ�־λ
uint8 Cross_Road_Flag=0;//ʮ�ֱ�־λ����·��־λ
uint8 Frist_Right_Black_lose,Frist_Left_Black_lose;//��һ�ж��߱�־λ
extern uint8 qipao ;
extern float Picture_detal,Dir_detal,Picture_detal_Old,Picture_detal_detal;
extern float g_fCarSpeedSetL,//1200,//600,800~35.7s  1000~28.64~165m�ȶ����� 
              g_fCarSpeedSetR,//1200;//600;//120~123s
              g_fCarSpeedSet,
              g_fCarSpeedSet_Stand,
              g_fCarSpeedSet_Max;
extern float P_Direction_Control,//4.5,//4.5~600,//3.4,//3.5//0.25 1000~6.3
               P_Direction_Control_now,
               D_Direction_Control,
               g_fSpeedControlP,//1.2,//2.02,//2.7,//1.95,
               g_fSpeedControlP_Init;//;//25;//0.35;//0.32

extern uint16 DIP_Number,Number_3,Number_2,Number_1,Number_0,Number_7;//���뿪��״̬��ȡ
int32 count_Slope,//��б�ʷֳɼ�������
       Charge_Slope_Range,//б�ʷ��ű仯�Ķ���
       fixs_Start_Line=0,//Ҫ�ı��б�ʿ�ʼ��
       Fixuse_Slope_number=0;//��Ч��б�ʶ�����������ƽ��
       
float   Charge_Slope_Symbol=0.00,//�жϵ�ǰ��б�ʺ���һ��б�ʵķ���
        Fixuse_Slope=0.00,//Y=KX+B �� K����
        Left_Black_K=0.00,
        Left_Black_B=0.00,//Y=KX+B �� B����
        Right_Black_K=0.00,
        Right_Black_B=0.00,
        O_Left_Line_K=0.00,
        O_Left_Line_B=0.00,
        O_Right_Line_K=0.00,
        O_Right_Line_B=0.00;
int16 O_Add=0;
int16 Right_Black_sixty_lose=0, //60
      Left_Black_sixty_lose=0,
      Right_Black_fourtyfive_lose=0,//45
      Left_Black_fourtyfive_lose=0,
      Right_Black_thrity_lose=0,//30
      Left_Black_thrity_lose=0,
      Right_Black_fiveteen_lose=0,//15
      Left_Black_fiveteen_lose=0,
      Right_Black_zero_lose=0,//0
      Left_Black_zero_lose=0;
      
int16   Left_Black_Slope0_Count=0,
         Right_Black_Slope0_Count=0,
         Left_Black_Ave=0,
         Right_Black_Ave=0,
         Left_Black_Slope0_Add=0,
         Right_Black_Slope0_Add=0,
         Qipa_Count;
      
int16 Left_Black_Add=0,
      Left_Black_Average=0,
      Right_Black_Add=0,
      Right_Black_Average=0,
      Left_Black_Variance=0,
      Right_Black_Variance=0,
      All_Variance=0;

int16 LDI=0,LDJ=0,LUI=0,LUJ=0,RDI=0,RDJ=0,RUI=0,RUJ=0,LU=0,RU=0,LD=0,RD=0,iMax,iMin;
int16 O_LDI=0,O_LDJ=0,O_RDI=0,O_RDJ=0,O_OI=0,O_OJ=0,O_OJ_Check=0,
      O_LD_Flower_Black=0,O_RD_Flower_Black=0,O_LD_Flower_Up_Black,O_RD_Flower_Up_Black,
      O_LD_Flower_Black_Mark=0,O_RD_Flower_Black_Mark=0;
int16 O_O_Left_I=0,O_O_Left_J=0,O_O_Right_I=0,O_O_Right_J=0;
int16 TLDI=0, TLDJ=0, TRDI=0, TRDJ=0, TLUI=0, TLUJ=0, TRUI=0, TRUJ=0;
int16 Block_Left_LU_i=0,Block_Left_LU_j=0,Block_Left_LD_i=0,Block_Left_LD_j=0,Block_Left=0,Block_Right,Block_Flag=0;//שͷ
int16 Block_Right_Count_Mark=0,Block_Right_Count=0,Block_Left_Count_Mark=0,Block_Left_Count=0,Block_Left_Real=0,Block_Right_Real=0;
int16 Block_Right_Up_i=0,Block_Right_Up_j=0,Block_Right_Down_i=0,Block_Right_Down_j=0,Mid_Right_Black=0;//שͷ
int16 Block_Left_Up_i=0,Block_Left_Up_j=0,Block_Left_Down_i=0,Block_Left_Down_j=0,Mid_Left_Black=0,Right_No_Continue;//שͷ
int16 Block_Jump_Flag=0,Brick_Jump_Point;
int16 Ramp_Flag=0,Ramp_Timer_Set=90,Ramp_Timer=50,Ramp_Next_Delay=1,Pingping_Flag,Zhidao_Flag,Zhidao_Real_Flag=0,Po_B=15;//�µ�
extern float g_fSpeedDeltaL,g_fSpeedDeltaR,g_fSpeedDeltaPreL,g_fSpeedDeltaPreR,Speed_Left,Speed_Right;
extern uint32 Direction_Control_Max,Direction_Control_Min;//2.25 �ٺ�
extern uint16 count,Jump_Change_Point_Count,Stop_Check_Begin_Count;
extern uint8 Live_Watch_Mark;
float Left_Black_Slope[5],Right_Black_Slope[5];
uint16 DL_Cross_Road_Flag=0,DR_Cross_Road_Flag=0,LD_Flag=0,RD_Flag=0;
uint16 Left_Many_Lose=0,Right_Many_Lose=0,Stright_Ten_Road_Flag=0,ML_Black_Spot_count=0,
       ML_White_Spot_count=0,O_Up_Blank_Point=0,O_White_spot_count=0,O_Up_Blank_Point2=0,O_Up_Blank_Point3=0;
uint16 O_Road_Flag=0,O_Get_O_Flag=0,O_Road_Num=0,O_Road_Num_Delay=0,O_Out_Flag=0,
       O_Get_O_Flag_Time=30,O_Out_Flag_Time=30,zhenTMdeyuanhuan_IN_FLAG=0;
uint16 O_Road_Gothrough=40,O_Left_Point_Flag=0,O_Right_Point_Flag=0,O_Black_Flag=0;//�������þ�����·ʱ��
uint16 Road_Kind,Road_Kind_Last,Pass_Time=100,Pass_Hole_On_Flag,Road_Kind_Mark;
float TKL=0,TBL=0,TKR=0,TBR=0;
uint16 Left_And_Right_Lose_Continue=0,Fuck_Up_Mark=0,Fuck_Up_Mark_Time=5;
uint16 Finish_Check_Count=0,Jump_Change_Point_Count=0,Zebra_Mark=0,Zebra_Mark_Time=5,
       Zebra_Mark_Rea=0,Zebra_Mark_Rea_Timer=0,Zebra_Mark_Rea_Timer_Set=140;
int16 CR_Start,CR_End,LD_Point,LU_Point,RD_Point,RU_Point;//ʮ�ֵ���ʼ�С������У��ĸ��յ�
int16 LD_Point_j,LU_Point_j,RD_Point_j,RU_Point_j;
int16 look_temp,Change_Point=0;
int16 Detal_Column=28,Detal_Column_Init=28;
int16 Shizi_in=0,Shizi_out=0;

  int Remedy_Left_Line[60]={-15,-14,-14,-13,-13,-12,-12,-11,-11,-10,//0-10  //��������
                            -10,-10,-10,-9,-9,-9,-9,-8,-8,-8,//11-20
                            -8,-7,-7,-7,-6,-6,-5,-5,-5,-5,//21-30
                            -4,-4,-4,-3,-3,-3,-2,-2,-2,-1,//31-40
                            -1,-1,0,0,0,0,0,0,1,1,   //41-50
                            2,2,2,3,3,3,4,4,4,4};//51-60 ���Ľ�
  
  int Remedy_Right_Line[60]={95,94,94,93,93,92,92,91,91,90,//0-10//��������
                             90,90,90,89,89,89,89,88,88,88,//11-20
                             88,87,87,87,86,86,85,85,85,85,//21-30
                             84,84,84,83,83,83,82,82,82,81,//31-40
                             81,81,80,80,80,80,80,80,79,79,//41-50
                             78,78,78,77,77,77,76,76,76,76};//51-60
  
  /*
  int Remedy_Left_Line[60]={-40,-40,-39,-38,-37,-36,-35,-34,-33,-32,//0-10  //��������
                            -31,-30,-29,-28,-27,-26,-25,-24,-23,-22,//11-20
                            -21,-20,-19,-18,-17,-16,-15,-14,-13,-12,//21-30
                            -12,-11,-10,-9,-8,-7,-6,-5,-4,-3,//31-40
                            -2,-1,0,0,0,0,0,0,1,1,   //41-50
                            2,2,2,3,3,3,4,4,4,4};//51-60 ���Ľ�
  
  int Remedy_Right_Line[60]={120,120,120,119,118,117,116,115,114,113,//0-10//��������
                             112,111,110,109,108,107,106,105,104,103,//11-20
                             102,101,100,99,98,97,96,95,94,93,//21-30
                             92,91,90,89,88,87,86,85,84,83,//31-40
                             82,81,80,80,80,80,80,80,79,79,//41-50
                             78,78,78,77,77,77,76,76,76,76};//51-60
  */
  
  int Remedy_Left_T_Line[60]={38,38,38,37,37,37,37,36,36,36,//0-10  //��������
                              35,35,35,34,34,34,33,33,32,31,//11-20
                              30,29,28,27,26,25,24,23,22,21,//21-30
                              20,20,19,19,18,18,17,17,16,16,//31-40
                              16,15,15,15,15,14,14,14,13,13,   //41-50
                              13,13,12,12,12,12,11,11,10,10};//51-60 ���Ľ�
  
  int Remedy_Right_T_Line[60]={42,42,42,43,43,43,43,44,44,44,//0-10//��������
                               45,45,45,46,46,46,47,47,48,49,//11-20
                               50,51,52,53,54,55,56,57,58,59,//21-30
                               60,60,61,61,62,62,63,63,64,64,//31-40
                               64,65,65,65,65,66,66,66,67,67,//41-50
                               67,67,68,68,68,68,69,69,70,70};//51-60
  
  int Change_Line_Wide[60]={0,0,0,0,0,14,14,14,15,15,//0-10  //original
                            16,16,16,17,17,17,18,18,18,19,//11-20
                            19,19,20,20,21,21,22,22,23,23,//21-30
                            24,24,24,25,25,26,26,27,27,28,//31-40
                            28,29,29,29,30,30,30,32,32,32,//41-50
                            32,32,33,33,33,0,0,0,0,0};//51-60
  
  /*
  //-2
  int Change_Line_Wide[60]={0,0,0,0,0,12,12,12,13,13,//0-10
                            14,14,14,15,15,15,16,16,16,17,//11-20
                            17,17,18,18,19,19,20,20,21,21,//21-30
                            22,22,22,23,23,24,24,25,25,26,//31-40
                            26,27,27,27,28,28,28,29,29,29,//41-50
                            30,30,31,31,31,0,0,0,0,0};//51-60
  */
    /*//-4
  int Change_Line_Wide[60]={0,0,0,0,0,10,10,10,11,11,//0-10  
                            12,12,12,13,13,13,14,14,14,15,//11-20
                            15,15,16,16,17,17,18,18,19,19,//21-30
                            20,20,20,21,21,22,22,23,23,24,//31-40
                            24,25,25,25,26,26,26,27,27,27,//41-50
                            28,28,29,29,29,0,0,0,0,0};//51-60
  */
  /*
  int Change_Line_Wide[60]={0,0,0,0,0,41,41,41,40,40,//0-10
                            39,39,39,38,38,38,37,37,37,36,//11-20
                            36,36,35,35,34,34,33,33,32,32,//21-30
                            31,31,31,30,30,31,31,32,32,33,//31-40
                            33,34,34,34,35,35,35,36,36,36,//41-50
                            37,37,38,38,38,0,0,0,0,0};//51-60
  */

/*
�������ƣ���������
����޸�ʱ�䣺201610
���ߣ�������ʵ�������ܳ���-��ӿ
�������µ��ʣ�13950986716��lczq@vip.qq.com
δ������⣺1��U������ղ���ȥ��
            2����ʱû�в��ߣ�Ӧ�ں����м��룻
            3�����ֻ��P���ڣ���������D��
**************************************************************************************************/

void Get_Middle_Line(void)  //��ȡ���ߡ�
{

  DL_Cross_Road_Flag=0;
  DR_Cross_Road_Flag=0;
  Left_Many_Lose=0;
  Right_Many_Lose=0;
  Stright_Ten_Road_Flag=0;
  //Jump_Change_Point_Count=0;
  ML_Black_Spot_count=0;
  ML_White_Spot_count=0;
  LD_Flag=0;
  RD_Flag=0;
  O_LDI=0;O_LDJ=0;O_RDI=0;O_RDJ=0;O_OI=0;O_OJ=0;O_OJ_Check=0;
  O_Left_Point_Flag=0;
  O_Right_Point_Flag=0;
  O_Up_Blank_Point2=0;O_Up_Blank_Point3=0;
  O_Black_Flag=0;
  O_O_Left_I=0;O_O_Left_J=0;O_O_Right_I=0;O_O_Right_J=0;
  TLDI=0;TLDJ=0;TRDI=0;TRDJ=0;TLUI=0;TLUJ=0;TRUI=0;TRUJ=0;
  TKL=0;TBL=0;TKR=0;TBR=0;
  LDI=0;LDJ=0;LUI=0;LUJ=0;RDI=0;RDJ=0;RUI=0;RUJ=0;
  LB_Lose_Count=0;RB_Lose_Count=0;
  Left_and_Right_Lose_count=0;
  O_Up_Blank_Point=0;
  LU=0;RU=0;LD=0;RD=0;
  O_White_spot_count=0;
  O_LD_Flower_Black=0;O_RD_Flower_Black=0;
  O_LD_Flower_Up_Black=0;O_RD_Flower_Up_Black=0;
  O_LD_Flower_Black_Mark=0;
  O_RD_Flower_Black_Mark=0;


  //ĳЩ�ض������Ƿ���
  {
  for(i=59;i>58;i--)//�жϵ�60�����Һ��Ƿ��ߣ�����б�ʲ���
  {
    for(j=40;j>1;j--)                        //�м�Ϊ��㣬����Ѱ�Һ���
    {
      if(img[i][j]==0xff && img[i][j-1]==0x00 && img[i][j-2]==0x00) //ͬһ���У������ǰ���ص�Ϊ������һ����Ϊ�ף���Ϊ�������
      {
        Left_Black_sixty_lose=0;
        break;
      }
      else
      {
        Left_Black_sixty_lose=1;
      }
     }
    for(j=41;j<79;j++)                        //�м�Ϊ��㣬����Ѱ�Һ���
    {
     if(img[i][j]==0x00 && img[i][j-1]==0xff && img[i][j-2]==0xff)//ͬһ���У������ǰ���ص�Ϊ������һ����Ϊ�ף���Ϊ�����
     {
       Right_Black_sixty_lose=0;
       break;
     }
     else
     {
       Right_Black_sixty_lose=1;                         //�����Ұ�С����
     }
    }
  }
  for(i=45;i>44;i--)//45
  {
    for(j=40;j>1;j--)                        //�м�Ϊ��㣬����Ѱ�Һ���
    {
      if(img[i][j]==0xff && img[i][j-1]==0x00 && img[i][j-2]==0x00) //ͬһ���У������ǰ���ص�Ϊ������һ����Ϊ�ף���Ϊ�������
      {
        Left_Black_fourtyfive_lose=0;
        break;
      }
      else
      {
        Left_Black_fourtyfive_lose=1;
      }
     }
    for(j=41;j<79;j++)                        //�м�Ϊ��㣬����Ѱ�Һ���
    {
     if(img[i][j]==0x00 && img[i][j-1]==0xff && img[i][j-2]==0xff)//ͬһ���У������ǰ���ص�Ϊ������һ����Ϊ�ף���Ϊ�����
     {
       Right_Black_fourtyfive_lose=0;
       break;
     }
     else
     {
       Right_Black_fourtyfive_lose=1;                         //�����Ұ�С����
     }
    }
  }
  for(i=30;i>29;i--)//30
  {
    for(j=40;j>1;j--)                        //�м�Ϊ��㣬����Ѱ�Һ���
    {
      if(img[i][j]==0xff && img[i][j-1]==0x00 && img[i][j-2]==0x00) //ͬһ���У������ǰ���ص�Ϊ������һ����Ϊ�ף���Ϊ�������
      {
        Left_Black_thrity_lose=0;
        break;
      }
      else
      {
        Left_Black_thrity_lose=1;
      }
     }
    for(j=41;j<79;j++)                        //�м�Ϊ��㣬����Ѱ�Һ���
    {
     if(img[i][j]==0x00 && img[i][j-1]==0xff && img[i][j-2]==0xff)//ͬһ���У������ǰ���ص�Ϊ������һ����Ϊ�ף���Ϊ�����
     {
       Right_Black_thrity_lose=0;
       break;
     }
     else
     {
       Right_Black_thrity_lose=1;                         //�����Ұ�С����
     }
    }
  }
  for(i=15;i>14;i--)//15
  {
    for(j=40;j>1;j--)                        //�м�Ϊ��㣬����Ѱ�Һ���
    {
      if(img[i][j]==0xff && img[i][j-1]==0x00 && img[i][j-2]==0x00) //ͬһ���У������ǰ���ص�Ϊ������һ����Ϊ�ף���Ϊ�������
      {
        Left_Black_fiveteen_lose=0;
        break;
      }
      else
      {
        Left_Black_fiveteen_lose=1;
      }
     }
    for(j=41;j<79;j++)                        //�м�Ϊ��㣬����Ѱ�Һ���
    {
     if(img[i][j]==0x00 && img[i][j-1]==0xff && img[i][j-2]==0xff)//ͬһ���У������ǰ���ص�Ϊ������һ����Ϊ�ף���Ϊ�����
     {
       Right_Black_fiveteen_lose=0;
       break;
     }
     else
     {
       Right_Black_fiveteen_lose=1;                         //�����Ұ�С����
     }
    }
  }
  for(i=1;i>0;i--)//1
  {
    for(j=40;j>1;j--)                        //�м�Ϊ��㣬����Ѱ�Һ���
    {
      if(img[i][j]==0xff && img[i][j-1]==0x00 && img[i][j-2]==0x00) //ͬһ���У������ǰ���ص�Ϊ������һ����Ϊ�ף���Ϊ�������
      {
        Left_Black_zero_lose=0;
        break;
      }
      else
      {
        Left_Black_zero_lose=1;
      }
     }
    for(j=41;j<79;j++)                        //�м�Ϊ��㣬����Ѱ�Һ���
    {
     if(img[i][j]==0x00 && img[i][j-1]==0xff && img[i][j-2]==0xff)//ͬһ���У������ǰ���ص�Ϊ������һ����Ϊ�ף���Ϊ�����
     {
       Right_Black_zero_lose=0;
       break;
     }
     else
     {
       Right_Black_zero_lose=1;                         //�����Ұ�С����
     }
    }
  }
  }
  
  //if(Left_Black_sixty_lose && Left_Black_fourtyfive_lose && Left_Black_thrity_lose )
  //�ж�ͼ����м��м����ڵ�
  
  for(i=0;i<59;i++)
  {
    if(img[i][40]==0x00)
    {
      ML_Black_Spot_count++;
    }
  }
  for(i=59;i>0;i--)
  {
    if(img[i][40]==0xff)
    {
      ML_White_Spot_count++;
    }
    else
      break;
  }
  /*
  for(i=55;i>10;i--) //�ж�б��ʮ��ǰ������ ������ ��δ����16-12-23
  {
    for(j=1;j<40;j++)
    {
      if(img[i][j]==0x00 && img[i][j+1]==0x00 && img[i][j+2]==0x00 && img[i][j+3]==0x00 &&
         img[i+1][j]==0xff && img[i+1][j+1]==0xff && img[i+1][j+2]==0xff && img[i+1][j+3]==0xff)
      {
        Left_Many_Lose=1;
      }
    }
    for(j=40;j<79;j++)
    {
      if(img[i][j]==0x00 && img[i][j+1]==0x00 && img[i][j+2]==0x00 && img[i][j+3]==0x00 &&
         img[i+1][j]==0xff && img[i+1][j+1]==0xff && img[i+1][j+2]==0xff && img[i+1][j+3]==0xff)
      {
        Right_Many_Lose=1;
      }
    }
    
    for(j=1;j<79;j++)
    {
      if(img[i][j]==0x00 && img[i][j+2]==0xff && img[i][j-2]==0x00 &&
         img[i-2][j]==0x00 && img[i+2][j]==0xff)
      {
        Left_Many_Lose=1;
      }
      if(img[i][j]==0x00 && img[i][j+2]==0x00 && img[i][j-2]==0xff &&
         img[i-2][j]==0x00 && img[i+2][j]==0xff)
      {
        Right_Many_Lose=1;
      }
    }
  }
  */
  
  O_Road_Scan();//Բ���ж�ɨ��
  
  Finish_Check();//�����߼��
  
  Block_Scan();//שͷʶ��17-05-21 20:46
  
  //gpio_set (PTA14,0);
  
  //�ٲ���б��С������ ���U��ڵ�
  //�ڿ�б�� ������ ��UҲ������� �������俨�� ��ȡ���� ���Ż���
  //�ۿ�б�� ������ СUҲ�� ����
  
 /*
  if(DIP_Number==2 || DIP_Number==3)
  {
  if(Road_Kind!=3 && !Fuck_Up_Mark && !O_Get_O_Flag 
     && ((Right_Black_sixty_lose && Right_Black_fourtyfive_lose && !Left_Black_sixty_lose) 
         || (Left_Black_fourtyfive_lose && Left_Black_sixty_lose && !Right_Black_sixty_lose)))//��
  //if(Road_Kind!=3 && !Fuck_Up_Mark && !O_Get_O_Flag && (Right_Black_sixty_lose || Left_Black_sixty_lose ||(Left_Black_fourtyfive_lose && Right_Black_fourtyfive_lose)))//��
  {
    Slope_Left();
    Slope_Right();
  }
  }
  */
  
  //Ten_Road_Check();//һ��������ж� ����ɶԭ��
  if(Left_Many_Lose && Right_Many_Lose)
  {
    Stright_Ten_Road_Flag=1;
  }
  
for(i=55;i>=45;i--)      //ͼ��������45-55��  927 OK                    //�ӵ�i�п�ʼ�����µ��ϣ��������ֳ����룬�ֱ����������ʹ��������ҡ�
{
  //if(i>=40)                                 //������20�п��Ŷȸߣ����м���������
  for(j=59;j>1;j--)                        //�м�Ϊ��㣬����Ѱ�Һ���
 {
  if(img[i][j]==0xff && img[i][j-1]==0x00 && img[i][j-2]==0x00 && img[i][j+1]==0xff ) //ͬһ���У������ǰ���ص�Ϊ������һ����Ϊ�ף���Ϊ�������
  {
    Left_Black[i]=j;                       //��¼��ǰ���������
    Left_Black_Old=Left_Black[i];          //������ Left_Black_Old �У��Է���һ���ɲ���������
    Left_Black_Lose=0;//���߶�ʧ��־λ
    LB_Lose[i]=0;
    break;
  }

  else
  {
    Left_Black_Lose++;                       //�Ҳ�����i�е���ߺڵ�
    LB_Lose[i]=1;
    continue;                               //�����Ұ�С����
  }
 }

  for(j=20;j<79;j++)                        //�м�Ϊ��㣬����Ѱ�Һ���
 {
  if(img[i][j]==0x00 && img[i][j-1]==0xff && img[i][j-2]==0xff && img[i][j+1]==0x00 )//ͬһ���У������ǰ���ص�Ϊ������һ����Ϊ�ף���Ϊ�����
  {
    Right_Black[i]=j;                     //��¼��ǰ���������
    Right_Black_Old=Right_Black[i];       //������ Right_Black_Old �У��Է���һ���ɲ���������
    Right_Black_Lose=0;//���߶�ʧ��־λ
    RB_Lose[i]=0;
    break;
  }
  else
  {
    Right_Black_Lose++;                    //�Ҳ�����i�е��ұߺڵ� ���޸� ������ 17-01-05
    RB_Lose[i]=1;
    continue;                             //�����Ұ�С����
  }
 }
 
 if(Right_Black[i]-Left_Black[i]<20)
 {
   Left_Black_Lose=1;
   Right_Black_Lose=1;
 }
 
 if(Left_Black_Lose==0 && Right_Black_Lose!=0 && !Left_And_Right_Lose_Continue)//��߲ɵ����ұ߶�ʧ
 {
    Right_Black[i]=Remedy_Right_Line[i];
 }
 
 else if(Left_Black_Lose!=0 && Right_Black_Lose==0 && !Left_And_Right_Lose_Continue)//�ұ߲ɵ�����߶�ʧ
 {
    Left_Black[i]=Remedy_Left_Line[i];
 }
   
  else if((Left_Black_Lose!=0) && (Right_Black_Lose!=0) && !O_Road_Flag)    //����������߶��ɲ����ڵ�
 {
    Left_and_Right_Lose_count++;
    Fuck_Up_Mark=1;
    Left_Black[i]=Remedy_Left_T_Line[i];//Left_Black_Old;//Remedy_Left_Line[i]; //17-01-01
    Right_Black[i]=Remedy_Right_T_Line[i];//Right_Black_Old;//Remedy_Right_Line[i];
    //Left_Black[i]=Left_Black[i+1];
    //Right_Black[i]=Right_Black[i+1];
 }
   if(i<54 && Road_Kind_Mark!=3)
   {
     if(Left_Black[i]-Left_Black[i+1]>4)
      {
        Left_Black[i]=Left_Black[i+1]+2;
      }
      else if(Left_Black[i+1]-Left_Black[i]>4)
      {
        Left_Black[i]=Left_Black[i+1]-2;
      }
      
      if(Right_Black[i]-Right_Black[i+1]>4)
      {
        Right_Black[i]=Right_Black[i+1]+2;
      }
      else if(Right_Black[i+1]-Right_Black[i]>4)
      {
        Right_Black[i]=Right_Black[i+1]-2;
      }
   }
      
  
 Mid_Line[i]=(Left_Black[i] + Right_Black[i])/2;
 
}

for(i=46;i>=5;i--)        //ͼ������35�У������ʺܸߣ����ش���
{
  if(
     Road_Kind_Mark!=3 && ML_Black_Spot_count<20 &&O_Road_Flag==0 &&O_Out_Flag==0 
      && (Left_and_Right_Lose_count_Last>15 || (LB_Lose[54]==1 && RB_Lose[54]==1) 
      || (Fuck_Up_Mark_Time>0&&Fuck_Up_Mark_Time<10) ||Fuck_Up_Mark)
        )//������ȫ�׶���ʮ��
  {
  for(j=40;j>5;j--)//13��Ӱ�죬ѡ���ٴ��� ��Ҫ�޷� 16-1118δ���
  {
    if(img[i][j]==0xff && img[i][j-1]==0x00 && img[i][j-2]==0x00 && img[i][j+1]==0xff && img[i][j+2]==0xff)       //��ǰ��Ϊ�ף���һ����Ϊ�� �ڡ��� Ϊ����غ���
    {
      Left_Black[i]=j;
      Left_Black_Old=Left_Black[i];
      Left_Black_Lose=0;
      LB_Lose[i]=0;
      break;
    }

    else
    {
      Left_Black_Lose++;
      LB_Lose[i]=1;
    }
  }
  
  for(j=40;j<75;j++)//����������Ļ���
  {
    if(img[i][j]==0x00 && img[i][j-1]==0xff && img[i][j-2]==0xff && img[i][j+1]==0x00 && img[i][j+2]==0x00)       //��ǰ��Ϊ�ڣ���һ����Ϊ�� �ס��� Ϊ�ұ��غ���
    {
      Right_Black[i]=j;
      Right_Black_Old=Right_Black[i];
      Right_Black_Lose=0;
      RB_Lose[i]=0;
      break;
    }
    else
    {
      Right_Black_Lose++;
      RB_Lose[i]=1;
    }
  }
    //gpio_set (PTE12,1);
  
 if(Left_Black_Lose==0 && Right_Black_Lose!=0 && !Left_And_Right_Lose_Continue)//��߲ɵ����ұ߶�ʧ
 {
   Right_Black[i]=40-Left_Black[i]+40;
   //Right_Black[i]=Right_Black[i+1];
 }    
 else if(Left_Black_Lose!=0 && Right_Black_Lose==0 && !Left_And_Right_Lose_Continue)//�ұ߲ɵ�����߶�ʧ
 {
   Left_Black[i]=40-(Right_Black[i]-40);
   //Left_Black[i]=Left_Black[i+1];
 }
  else if((Left_Black_Lose!=0) && (Right_Black_Lose!=0) && !O_Road_Flag)    //����������߶��ɲ����ڵ�
 {
   //gpio_set (PTE12,1);
    //Fuck_Up_Mark=1;//��������������Ϊɶ����ʱ���ܽ� ���ʱ�����������
    //Fuck_Up_Mark=1;
    Left_and_Right_Lose_count++;
    //Left_Black[i]=Left_Black[i+1];
    //Right_Black[i]=Right_Black[i+1];
    Left_Black[i]=Remedy_Left_T_Line[i];
    Right_Black[i]=Remedy_Right_T_Line[i];
 }
    Mid_Line[i]=(Left_Black[i] + Right_Black[i])/2;
  }

  else
  {
    //gpio_set (PTE12,0);
  Left_Board_Min=Left_Black[i+1]-20;
  Left_Board_Max=Left_Black[i+1]+20;
  if(Left_Board_Min<3)
    Left_Board_Min=3;
  if(Left_Board_Max>77)
    Left_Board_Max=77;
  for(j=Left_Board_Max;j>Left_Board_Min;j--)//13��Ӱ�죬ѡ���ٴ��� ��Ҫ�޷� 16-1118δ���
  {
    if(img[i][j]==0xff && img[i][j-1]==0x00 && img[i][j-2]==0x00 && img[i][j+1]==0xff && img[i][j+2]==0xff)       //��ǰ��Ϊ�ף���һ����Ϊ�� �ڡ��� Ϊ����غ���
    {
      Left_Black[i]=j;
      Left_Black_Old=Left_Black[i];
      Left_Black_Lose=0;
      LB_Lose[i]=0;
      break;
    }

    else
    {
      Left_Black_Lose++;
      LB_Lose[i]=1;
    }
  }
  Right_Board_Min=Right_Black[i+1]-20;
  Right_Board_Max=Right_Black[i+1]+20;
  if(Right_Board_Min<3)
    Right_Board_Min=3;
  if(Right_Board_Max>77)
    Right_Board_Max=77;  
  for(j=Right_Board_Min;j<Right_Board_Max;j++)//����������Ļ���
  {
    if(img[i][j]==0x00 && img[i][j-1]==0xff && img[i][j-2]==0xff && img[i][j+1]==0x00 && img[i][j+2]==0x00)       //��ǰ��Ϊ�ڣ���һ����Ϊ�� �ס��� Ϊ�ұ��غ���
    {
      Right_Black[i]=j;
      Right_Black_Old=Right_Black[i];
      Right_Black_Lose=0;
      RB_Lose[i]=0;
      break;
    }
    else
    {
      Right_Black_Lose++;
      RB_Lose[i]=1;
      
    }
  }
    //gpio_set (PTE12,0);
  if(Left_Black_Lose==0 && Right_Black_Lose==0 && !Left_And_Right_Lose_Continue)//���ɵ�
 {
   Mid_Line[i]=(Left_Black[i] + Right_Black[i])/2;
 }
 else if(Left_Black_Lose==0 && Right_Black_Lose!=0 && !Left_And_Right_Lose_Continue)//��߲ɵ����ұ߶�ʧ
 {
   Mid_Line[i]=Left_Black[i]+Change_Line_Wide[i];
   Right_Black[i]=Remedy_Right_Line[i];
 }
 else if(Left_Black_Lose!=0 && Right_Black_Lose==0 && !Left_And_Right_Lose_Continue)//�ұ߲ɵ�����߶�ʧ
 {
   Mid_Line[i]=Right_Black[i]-Change_Line_Wide[i];
   Left_Black[i]=Remedy_Left_Line[i];
 }
  else if((Left_Black_Lose!=0) && (Right_Black_Lose!=0) && !O_Road_Flag)    //����������߶��ɲ����ڵ�
 {
    //Fuck_Up_Mark=1;
    Left_and_Right_Lose_count++;
    Left_Black[i]=Remedy_Left_T_Line[i];
    Right_Black[i]=Remedy_Right_T_Line[i];
    Mid_Line[i]=(Left_Black[i] + Right_Black[i])/2;
 }
 }
 
 if(Left_Black[i]>Right_Black[i])
 {
   LB_Lose[i]=1;
   RB_Lose[i]=1;
 }
 
}
 /*
  if(Cross_Road_Connect_Flag==1 && ML_Black_Spot_count<20) //�����Ϸ��� �����Ļɨ�� l����ʮ�ֵĶ�������������ɨ������������
  {
    for(j=40;j>1;j--)                        //�м�Ϊ��㣬����Ѱ�Һ���
    {
      if(img[i][j]==0xff && img[i][j-1]==0x00 && img[i][j-2]==0x00) //ͬһ���У������ǰ���ص�Ϊ������һ����Ϊ�ף���Ϊ�����
      {
        Left_Black[i]=j;                       //��¼��ǰ���������
        Left_Black_Old=Left_Black[i];          //������ Left_Black_Old �У��Է���һ���ɲ���������
        Left_Black_Lose=0;//���߶�ʧ��־λ
        break;
      }
      else
      {
        Left_Black_Lose++;                       //�Ҳ�����i�е���ߺڵ�
        continue;                               //�����Ұ�С����
      }
    }

    for(j=41;j<79;j++)                        //�м�Ϊ��㣬����Ѱ�Һ���
    {
      if(img[i][j]==0x00 && img[i][j-1]==0xff && img[i][j-2]==0xff)//ͬһ���У������ǰ���ص�Ϊ������һ����Ϊ�ף���Ϊ�����
      {
        Right_Black[i]=j;                     //��¼��ǰ���������
        Right_Black_Old=Right_Black[i];       //������ Right_Black_Old �У��Է���һ���ɲ���������
        Right_Black_Lose=0;//���߶�ʧ��־λ
        break;
      }
      else
      {
        Right_Black_Lose++;                    //�Ҳ�����i�е��ұߺڵ�
        continue;                             //�����Ұ�С����
       }
    
    }
    //Left_Black_Lose=0;
    //Right_Black_Lose=0;
    Cross_Road_Connect_Flag=0;
  }
  */
  //Mid_Line[i]=(Left_Black[i] + Right_Black[i])/2;  //��¼���� 
    ShiZi_Handle();//ʮ�ִ��Դ���
    Block_Handle();//�ϰ�����  �µ��ϻ��жϳ��ϰ���������ٶȼ��
  
    //else
      //gpio_set (PTE12,0);
  //if(LU && RU && abs(LUI-RUI)<20)
    //gpio_set (PTE12,1);//������
  //else
    //gpio_set (PTE12,0);//������

  Left_and_Right_Lose_count_Last=Left_and_Right_Lose_count;

 if(Fuck_Up_Mark)
 {
   //gpio_set (PTE12,1);
   /*
   if(abs(LB_Lose_Begin-RB_Lose_Begin)<10 && abs(RB_Lose_End-LB_Lose_End)<10)
   {
     //gpio_set (PTE12,1);
     for(i=Max(RB_Lose_End,LB_Lose_End);i<Min(LB_Lose_Begin,RB_Lose_Begin);i++)
     {
       if(Mid_Line[i]-Mid_Line[i-1]>3)
      {
        Mid_Line[i]=Mid_Line[i-1]+1;
      }
      if(Mid_Line[i-1]-Mid_Line[i]>3)
      {
        Mid_Line[i]=Mid_Line[i-1]-1;
      }
     }
   }*/
   
   Fuck_Up_Mark_Time--;
 }
 if(Fuck_Up_Mark_Time==0)
 {
   //gpio_set (PTE12,0);
   Fuck_Up_Mark=0;
   Fuck_Up_Mark_Time=30;//Change
 }
  for(i=55;i>=15;i--)//iԽ��Խ��������
 {
   if(LB_Lose[i]==1)//��¼��ڱ��м�����
   {
     LB_Lose_Count++;
   }
   if(RB_Lose[i]==1)//��¼�Һڱ��м�����
   {
     RB_Lose_Count++;
   }
   if(LB_Lose[i]==1 && LB_Lose[i+1]==0 && LB_Lose[i+2]==0 && LB_Lose[i-1]==1 && LB_Lose[i-2]==1)
   {
     LB_Lose_Begin=i;//��ߺ��ߴ����п�ʼ���ߵ�
   }
   if(LB_Lose[i]==1 && LB_Lose[i+1]==1 && LB_Lose[i+2]==1 && LB_Lose[i-1]==0 && LB_Lose[i-2]==0)
   {
     LB_Lose_End=i;//�����������
   }
   if(RB_Lose[i]==1 && RB_Lose[i+1]==0 && RB_Lose[i+2]==0 && RB_Lose[i-1]==1 && RB_Lose[i-2]==1)
   {
     RB_Lose_Begin=i;//�ұߺ��ߴ����п�ʼ���ߵ�
   }
   if(RB_Lose[i]==1 && RB_Lose[i+1]==1 && RB_Lose[i+2]==1 && RB_Lose[i-1]==0 && RB_Lose[i-2]==0)
   {
     RB_Lose_End=i;//�����������
   }
 }
   LB_Lose_Count_Last=LB_Lose_Count;
   RB_Lose_Count_Last=RB_Lose_Count;
  //����switch��·����ÿ��ִֻ��һ��·������ÿ�γ���һ��ʱ��
  //��ֱ��ʮ�� ��б��ʮ��  ��Բ�� ���µ�
 
  //��ֱ��
  if  (ML_Black_Spot_count<3 && (ML_Black_Spot_count==0 || ML_Black_Spot_count>0) && 
     !Right_Black_sixty_lose && !Right_Black_fourtyfive_lose && !Right_Black_thrity_lose &&
     !Left_Black_sixty_lose && !Left_Black_fourtyfive_lose && !Left_Black_thrity_lose 
       &&Zhidao_Real_Flag) //ԭʼͼ�����ĺ��߸�����0-3֮��
  {
    Road_Kind=0;//������� ����S��ֱ��
  }
 /*
  if(Block_Flag)
  {
    Road_Kind=1;
  }
 */
  /*
  else if((Left_and_Right_Lose_count>15 || (LB_Lose[54]==1 && RB_Lose[54]==1)) && !Pass_Hole_On_Flag)
  {
    Road_Kind=1;
  }
  */
  /*
  else if(LUI && LUJ && !RUI && !RUJ)//17-01-04 ����˫�����ѽ���
  {
    Road_Kind=1;
  }
*/
  //��б����ʮ��

  /*
  else if(RUI && RUJ && !LUI && !LUJ)//17-01-04
  {
    Road_Kind=2;
  }
  */
  
  //��Բ��
  else if(//!Left_Black_sixty_lose && !Left_Black_fourtyfive_lose && !Left_Black_thrity_lose &&
          //!Right_Black_sixty_lose && !Right_Black_fourtyfive_lose && !Right_Black_thrity_lose &&
          ML_Black_Spot_count>5 && ML_Black_Spot_count<40 && (O_Get_O_Flag ||O_Out_Flag))
  {
    Road_Kind=3;
    
  }
  
  /*
  else if(RUI && RUJ && LUI && LUJ)
  {
    Road_Kind=4;
  }
  */
  
  if(Road_Kind_Last==0 && Road_Kind==3)//���������Ļ�����ĳ��ģʽһ��ʱ��
  {
    /*
    //ͳ��Բ����Ŀ�����￪ʼ ���Ҳ�����������
    if(!O_Road_Num_Delay)
    {
      O_Road_Num++;
      O_Road_Num_Delay=200;
    }
    */
    //Road_Kind=Road_Kind_Last;
    Road_Kind_Mark=Road_Kind;
    Pass_Hole_On_Flag=1;
  }
  if(Pass_Hole_On_Flag && Road_Kind_Mark==3)//
  {
    Pass_Time--;
    Road_Kind=Road_Kind_Mark;
    //gpio_set (PTA15,1);
  }
  if(Pass_Hole_On_Flag && Road_Kind_Last!=0)//��ֹ1->3 2->3
  {
    Road_Kind=Road_Kind_Mark;
  }
  
  if(O_Road_Num_Delay>0)
    O_Road_Num_Delay--;
  
  if(Pass_Time==1)
  {
    Pass_Hole_On_Flag=0;
    Pass_Time=100;  //200*12ms �ɿ�����޸� �ٶ�Խ�� ͨ��Բ��ʱ��Խ��
    //gpio_set (PTA15,0);
    Road_Kind_Mark=0;
  }

  switch(Road_Kind)
  {
  case 0:
    {
      /*
      if(Block_Flag==1)
        gpio_set (PTE12,1);
      else if(Block_Flag==0)
        gpio_set (PTE12,0);*/
      
    }break;
  case 1:;break;
  //case 2:Slope_Right_Line();break; //б��ʮ��˲�䲻��������
  case 3:O_Road_Handle();break;
  //case 4:Slope_Left_Line();Slope_Right_Line();break;
  /*
  case 4: 
    {
      for(i=55;i>=10;i--)
      {
        for(j=0;j<80;j++) 
        {
          if(img[i][j]==0xff && img[i][j-1]==0x00 && img[i][j-2]==0x00)
          {
            Left_Black[i]=j;
            break;
          }
          else
          {
            Left_Black[i]=Remedy_Left_T_Line[i];
          }
        }
        for(j=0;j<80;j++)
        {
          if(img[i][j]==0x00 && img[i][j-1]==0xff && img[i][j-2]==0xff)
          {
            Right_Black[i]=j;
            break;
          }
          else
          {
            Right_Black[i]=Remedy_Right_T_Line[i];
          }
        }
      }
    }break;
  */
  default: ; break;
  }
  
  /*
  if(Road_Kind==1 || Road_Kind==2)
  {
    gpio_set (PTC12,1);
  }
  else
  {
    gpio_set (PTC12,0);
  }
  */
  
  Road_Kind_Last=Road_Kind;
  
  Po_Scan();//�µ�ɨ��
  
for(i=55;i>=5;i--)      //927 OK               //�ӵ�60�е���40�л������ߣ������ڽ�ѹ���imgͼ��
  {
    if(i<55 && i>=5 && Road_Kind_Mark!=3 
       && !(Left_and_Right_Lose_count_Last>15 || (LB_Lose[54]==1 && RB_Lose[54]==1) )
         )//�Ż��������߸�ƽ��Щ
    {
      /*
      if(abs(Mid_Line[i]-Mid_Line[i+1])>3)
      {
        Mid_Line[i]=(Mid_Line[i+1]-Mid_Line[i+5])/2+Mid_Line[i+1];
      }
      */
      
      if(Mid_Line[i]-Mid_Line[i+1]>3)
      {
        Mid_Line[i]=Mid_Line[i+1]+1;
      }
      else if(Mid_Line[i+1]-Mid_Line[i]>3)
      {
        Mid_Line[i]=Mid_Line[i+1]-1;
      }
    }

    Error[i]=Mid_Line[i]-39.5;
    /*
    img[i][Mid_Line[i]+1]=0x00;
    img[i][Mid_Line[i]]=0x00;
    img[i][Mid_Line[i]-1]=0x00;
    */
  }
    
}

void Draw_Midline(void)
{
     for(i=55;i>=5;i--)
      {
        img[i][Mid_Line[i]+1]=0x00;
        img[i][Mid_Line[i]]=0x00;
        img[i][Mid_Line[i]-1]=0x00;
      }
}


/*
����Ҫ�ȰѴ��е����ĺ����ó�������Ӻ��ߡ�
*/
/*
�������ƣ�б�ʼ��㣬��·����·
start��2016-12-01
end  ��unknown
���ߣ�������ʵ�������ܳ���-��ӿ
�������µ��ʣ�13950986716��lczq@vip.qq.com
**************************************************************************************************/
/*
void Slope(void)
{
  
  Fixuse_Slope_number=1;
  Left_Black_Slope[0]=(float)(5)/(float)(Left_Black[55]-Left_Black[45]);
  Left_Black_Slope[1]=(float)(5)/(float)(Left_Black[45]-Left_Black[35]);
  Left_Black_Slope[2]=(float)(5)/(float)(Left_Black[35]-Left_Black[25]);
  Left_Black_Slope[3]=(float)(5)/(float)(Left_Black[25]-Left_Black[15]);
  Left_Black_Slope[4]=(float)(5)/(float)(Left_Black[15]-Left_Black[5]);
  
  
  for(count_Slope=0;count_Slope<4;count_Slope++)
  {
    Charge_Slope_Symbol=Left_Black_Slope[count_Slope]*Left_Black_Slope[count_Slope+1];
    if(Charge_Slope_Symbol<0)//��ǰ�е�б�ʸı�
    {
      Fixuse_Slope=Left_Black_Slope[count_Slope];
      Charge_Slope_Range=count_Slope;//Mark slope's change line number 
      break;
    }
    else //б�ʷ��Ų��䣬û��б��ͻ��
    {
      ;
      //Fixuse_Slope_number++;
      //Fixuse_Slope=Fixuse_Slope+Left_Black_Slope[count_Slope]; //Add all same symbol slope     
    }
  }
  
  if(Fixuse_Slope_number<5) //б�ʱ仯��Ч����ȷ����б��ʮ�� //���ܻ�����ֱ���䣬����û���������
  {
  fixs_Start_Line=45-Charge_Slope_Range*10; 
  
  Left_Black_K=Fixuse_Slope/Fixuse_Slope_number;//line K
  
  //B        =          Y         -      K     *    X
  Left_Black_B=(60-fixs_Start_Line)-Fixuse_Slope*Left_Black[fixs_Start_Line];//line B
  
  if(fixs_Start_Line>10)
  {
    for(i=fixs_Start_Line;i>10;i--)
    {
      //   X       = (  Y  -      B      )/K;
      Left_Black[i]=(int32)(((60-i)-Left_Black_B)/Fixuse_Slope);
    }
  }
  
  }
  
}
*/
/*
�������ƣ�б��ʮ�ֵ����ж�
����޸�ʱ�䣺2016-12-23
���ߣ�������ʵ�������ܳ���-��ӿ
�ο������ഺ
����ʱ�䣺
�������µ��ʣ�13950986716��lczq@vip.qq.com
**************************************************************************************************/
void Slope_Left(void)
{
  for(i=57;i>15;i--)
  {
    if(!LDI && !LDJ)
    {
      for(j=65;j>2;j--)
      {
        if(img[i][j]==0x00 && img[i][j-2]==0x00 && img[i][j+2]==0xff && img[i-2][j]==0xff && img[i+2][j]==0x00 &&
           img[i-2][j+2]==0xff && img[i+2][j-2]==0x00) //�������е���ĵ� ��������ǿ
        {
          LDI=i;
          LDJ=j;
          LD_Flag=1;
          break;
        }
      //else
      //{
      //  LD_Flag=0;
      //}
      }
    }
    else
    {
      break;
    }
  }
    
  if(LD_Flag==1)
  {
  for(i=LDI-5;i>3;i--)
  {
    if(!LUI && !LUJ )
    {
      for(j=3;j<65;j++)
      {
        if(img[i][j]==0x00 && img[i-2][j-2]==0x00 && img[i][j+2]==0xff && img[i+2][j+2]==0xff && img[i+2][j]==0xff && abs(i-LDI)<30)
        {
          LUI=i;
          LUJ=j;
          break;
        }
      //else
      //{
      //  LUI=1;
      //  LUJ=79;
      //}
      }
    }
    else
    {
      break;
    }
  }
  }
  /*
  else
  {
    for(i=60;i>10;i--)
    {
      if(!LUI && !LUJ )
      {
        for(j=0;j<40;j++)
        {
          if(img[i][j]==0x00 && img[i][j+2]==0xff && img[i+2][j]==0xff && img[i+2][j+2]==0xff && img[i-2][j-2]==0x00 )
          {
            LUI=i;
            LUJ=j;
            break;
           }
      //else
      //{
      //  LUI=1;
      //  LUJ=79;
      //}
        }
      }
      else
      {
        break;
      }
    }
  }
  */
  
}
/*
�������ƣ�б��ʮ�ֵ����ж�
����޸�ʱ�䣺2016-12-23
���ߣ�������ʵ�������ܳ���-��ӿ
�ο������ഺ
����ʱ�䣺
�������µ��ʣ�13950986716��lczq@vip.qq.com
**************************************************************************************************/
void Slope_Right(void)
{
  
  for(i=57;i>15;i--)
  {
    if(!RDI && !RDJ)
    {
      for(j=78;j>15;j--)
      {
        if(img[i][j]==0x00 && img[i+2][j]==0xff && img[i-2][j]==0xff && img[i][j-2]==0xff && img[i][j+2]==0x00)
        {
          RDI=i;
          RDJ=j;
          RD_Flag=1;
          break;
        }
      //else
      //{
      //  RD_Flag=0;
      //}
      }
    }
    else
    {
      break;
    }
  }
    
  if(RD_Flag==1)
  {
  for(i=RDI-5;i>3;i--)
  {
    if(!RUI && !RUJ)
    {
      for(j=78;j>15;j--)
      {
        if(img[i][j]==0x00 && img[i-2][j]==0x00 && img[i+2][j]==0xff && img[i][j-2]==0xff && img[i][j+2]==0x00)
        {
          RUI=i;
          RUJ=j;
          break;
        }
      }
    }
    else
    {
      break;
    }
  }
  }
  /*
  else
  {
    for(i=60;i>10;i--)
    {
      if(!RUI && !RUJ)
      {
        for(j=80;j>40;j--)
        {
          if(img[i][j]==0x00 && img[i][j-2]==0xff && img[i+2][j]==0xff && img[i+2][j-2]==0xff && img[i-2][j+2]==0x00)
          {
            RUI=i;
            RUJ=j;
            break;
          }
        }
      }
      else
      {
        break;
      }
    }
  }
  */
  
  
}

/*
�������ƣ�����ʮ�ֲ���
����޸�ʱ�䣺20170803
���ߣ�������ʵ�������ܳ���-��ӿ
�������µ��ʣ�13950986716��lczq@vip.qq.com
δ������⣺
**************************************************************************************************/
void Slope_Right_Line(void)
{
  if(RUI && RUJ)
  {
    Right_Black_K=(float)((float)(RDJ-RUJ)/(float)(RDI-RUI));
    Right_Black_B=(float)((RDJ+RUJ)/2-Right_Black_K*((RUI+RDI)/2));
  
    for(i=RDI;i>RUI;i--)
    {
      Right_Black[i]=(int)(Right_Black_K*i+Right_Black_B);
    }
  }
}

/*
�������ƣ�����ʮ�ֲ���
����޸�ʱ�䣺20170803
���ߣ�������ʵ�������ܳ���-��ӿ
�������µ��ʣ�13950986716��lczq@vip.qq.com
δ������⣺
**************************************************************************************************/
void Slope_Left_Line(void)
{
   if(LUI && LUJ)
  {
    Left_Black_K=(float)((LUJ-LDJ)/(LDI-LUI));
    Left_Black_B=(float)(((LUJ+LDJ)/2)-Left_Black_K*((LUI+LDI)/2));
  
    for(i=LUI;i<LDI;i++)
    //for(j=LDJ;j<LUJ;j++)
      {
        Left_Black[i]=(int)(Left_Black_K*i+Left_Black_B);
      }
  }
}

/*
�������ƣ���·����
����޸�ʱ�䣺20170803
���ߣ�������ʵ�������ܳ���-��ӿ
�������µ��ʣ�13950986716��lczq@vip.qq.com
δ������⣺
**************************************************************************************************/
void O_Road_Handle(void)
{
  //���Բ��ߣ���·
  //ѡ���ߣ�����ڣ�ѡ���ߣ����Һڣ�
  if(O_Road_Flag==0)
  {
     if(((O_LDI && O_LDJ ) && (O_RDI && O_RDJ)) && O_Get_O_Flag ) //�ɵ����� �ǻ�·���
    {
       O_Road_Flag=1;//�ñ�־λ
    }
  }
  if(O_Road_Flag==1 ||O_Out_Flag)
  {
    //gpio_set (PTA15,1);
    if( (((O_LDI && O_LDJ) || (O_RDI && O_RDJ)) && O_Get_O_Flag ) 
       //|| (Left_Black_sixty_lose && Right_Black_sixty_lose && Left_Black_fourtyfive_lose && 
       //Right_Black_fourtyfive_lose && O_Get_O_Flag  )
        ) //��Բ��
    {
      zhenTMdeyuanhuan_IN_FLAG=1;
      //gpio_set (PTA15,1);
      if(!O_Road_Num_Delay)
      {
        O_Road_Num++;//��¼Բ������
        O_Road_Num_Delay=1000;
      }
      if(Number_0==0)//��Բ���ҹ� 17-08-04 ok
      {
        if(O_LDI==0 || O_LDJ==0)
        {
          O_LDI=59;
          O_LDJ=40;//60
        }
        O_Left_Line_K=((float)(O_OJ+10-O_LDJ)/(float)(O_OI-O_LDI));
        O_Left_Line_B=(float)((O_OJ+10+O_LDJ)/2-O_Left_Line_K*(O_LDI+O_OI)/2);
        for(i=O_OI-3;i<O_LDI+3;i++)
        {
          Left_Black[i]=(int)(O_Left_Line_K*i+O_Left_Line_B+O_Add);//+8�ı�ó������ó���Բ���յø��� �ж���Բ�� �ҹգ�
          for(j=40;j<78;j++)
          {
            if(img[i][j]==0x00 && img[i][j+1]==0x00&& img[i][j+2]==0x00 && img[i][j-1]==0xff&& img[i][j-2]==0xff)
            {
              Right_Black[i]=j;
              break;
            }
            else
            {
              Right_Black[i]=80;
            }
          }
          Mid_Line[i]=(Left_Black[i] + Right_Black[i])/2;
        }
      }
      else if(Number_0==1)//��Բ����� 17-08-04 ok ��Բ��СԲ����OK
      {
        if(O_RDI==0 || O_RDJ==0)
        {
          O_RDI=59;
          O_RDJ=40;
        }
        O_Right_Line_K=((float)(O_OJ-10-O_RDJ)/(float)(O_OI-O_RDI));
        O_Right_Line_B=(float)((O_OJ-10+O_RDJ)/2-O_Right_Line_K*(O_RDI+O_OI)/2);
        for(i=15;i<O_RDI+3;i++)
        {
          //Left_Black[i]=Left_Black[i]-8;
          Right_Black[i]=(int)(O_Right_Line_K*i+O_Right_Line_B-O_Add);//-8�ı�ó������ó���Բ���յø��� �ж���Բ�� ��գ�
          for(j=2;j<40;j++)
          {
            if(img[i][j]==0x00 && img[i][j-1]==0x00&& img[i][j-2]==0x00 && img[i][j+1]==0xff&& img[i][j+2]==0xff)
            {
              Left_Black[i]=j;
              break;
            }
            else
            {
              Left_Black[i]=0;
            }
          }
          Mid_Line[i]=(Left_Black[i] + Right_Black[i])/2;
          //Mid_Line[i]=Right_Black[i]-Change_Line_Wide[i];
        }
      }
    }
    
    else if( (((O_LDI && O_LDJ) || (O_RDI && O_RDJ)) && O_Out_Flag) || 
      (Left_Black_sixty_lose && Right_Black_sixty_lose && Left_Black_fourtyfive_lose && 
       Right_Black_fourtyfive_lose && O_Out_Flag) )  //��Բ��
    {
      //gpio_set (PTA15,0);
      zhenTMdeyuanhuan_IN_FLAG=0;
      if(Number_0==0) //��Բ���ҹ� 17-08-04 ��gg
      {
        if(O_LDI==0 || O_LDJ==0)
        {
          O_LDI=59;
          O_LDJ=40;//60
        }
        if(O_RDI==0 || O_RDJ==0)
        {
          O_RDI=53;
          O_RDJ=70;//60
        }
        O_Left_Line_K=((float)(O_OJ-O_LDJ)/(float)(O_OI-O_RDI));
        O_Left_Line_B=(float)((O_OJ+O_LDJ)/2-O_Left_Line_K*(O_RDI+O_OI)/2);
        for(i=O_OI-6;i<O_RDI+6;i++)
        {
          Left_Black[i]=(int)(O_Left_Line_K*i+O_Left_Line_B+O_Add+18);//+8�ı�ó������ó���Բ���յø��� �ж���Բ�� �ҹգ�
          Mid_Line[i]=(Left_Black[i] + Right_Black[i])/2;
        }
      }
      else if(Number_0==1) //��Բ����� 17-08-04 ��СԲ����OK
      {
        if(O_RDI==0 || O_RDJ==0)
        {
          O_RDI=59;
          O_RDJ=40;
        }
        if(O_LDI==0 || O_LDJ==0)
        {
          O_LDI=53;
          O_LDJ=10;
        }
        O_Right_Line_K=((float)(O_OJ-O_RDJ)/(float)(O_OI-O_LDI));
        O_Right_Line_B=(float)((O_OJ+O_RDJ)/2-O_Right_Line_K*(O_LDI+O_OI)/2);
        for(i=O_OI-6;i<O_LDI+6;i++)
        {
          Right_Black[i]=(int)(O_Right_Line_K*i+O_Right_Line_B-O_Add-18);//-8�ı�ó������ó���Բ���յø��� �ж���Բ�� ��գ�
          Mid_Line[i]=(Left_Black[i] + Right_Black[i])/2;
        }
      }
    }
    
    else
      zhenTMdeyuanhuan_IN_FLAG=0;
    /*
    else
    {
      for(i=55;i>=10;i--)
      {
        Left_Black[i]=Left_Black[i];
        Right_Black[i]=Right_Black[i];
      }
    }*/
  }
  else
    zhenTMdeyuanhuan_IN_FLAG=0;//gpio_set (PTA15,0);

  if(O_Road_Flag==1)
  {
    O_Road_Gothrough--;
  }
  
  if(O_Road_Gothrough==1)//��ʱ�Ѿ����껷·
  {
    O_Road_Flag=0;//���־
    O_Road_Gothrough=60;//����ʱ��
    O_Right_Line_K=0;
    O_Right_Line_B=0;
    O_Left_Line_K=0;
    O_Left_Line_B=0;
  }
}

/*
�������ƣ�����������������
����޸�ʱ�䣺20170803
���ߣ�������ʵ�������ܳ���-��ӿ
�������µ��ʣ�13950986716��lczq@vip.qq.com
δ������⣺ 1��ͼ�������������ֵӦ��΢�� OK
**************************************************************************************************/

void Direction_Detal_Caculate(void) //�������� ��10-45�е��м����
{
  if(zhenTMdeyuanhuan_IN_FLAG||O_Out_Flag)
  {
    Detal_Column=Detal_Column_Init-3;
    
  }
  else
  {
    Detal_Column=Detal_Column_Init;
  }
  for(i=Detal_Column+5;i>Detal_Column-5;i--) //
  {
    if(Left_Black[i]<-30)//control the mix
      Left_Black[i]=-30;
    if(Right_Black[i]>110)//control the max
      Right_Black[i]=110;
    if(Mid_Line[i]>79)         //�˳��ӵ㣬�������߳������㣬ʵ���и������30000+
      Mid_Line[i]=79;
    if(Mid_Line[i]<1)
      Mid_Line[i]=1;
    
    /*
    if(Mid_Line[i]-Mid_Line[i+1]>7) //����˵�����һ��ƫ���ʮ�����ж�Ϊ��Ч��
      Mid_Line[i]=Mid_Line[i+1]+3;   //������һ�������¸�ֵ
    else if(Mid_Line[i]-Mid_Line[i+1]<6) //����˵�����һ��ƫ���ʮ�����ж�Ϊ��Ч��
      Mid_Line[i]=Mid_Line[i+1]-3; //������һ�������¸�ֵ
    */
    
    Picture_detal+=(Mid_Line[i]-39.5); //һ��ͼƬ��ƫ��Ϊһ����
  }
  Picture_detal=Picture_detal*0.1;
  

  
  if(Live_Watch_Mark)
  {
    if((Speed_Right>50||Zebra_Mark) && Qipa_Count<400)
    {
      Qipa_Count++;
    //Qipa_Count=Qipa_Count%200;
    }
    if(Speed_Right<750 && Qipa_Count<398)
    {
      //gpio_set (PTA15,1);
      if(Number_7==0)//�ϰ��ұ�
        Picture_detal=Picture_detal-3;
      else if(Number_7==1)//�ϰ����
        Picture_detal=Picture_detal+4;
    }
    else
      ;
      //gpio_set (PTA15,0);
  }
  
  /*
  if(Zebra_Mark_Rea_Timer>0 && Zebra_Mark_Rea_Timer<100)
  {
    if(Number_7==1)//�ϰ��ұ�
      Picture_detal=Picture_detal-5;
    else if(Number_7==0)//�ϰ����
      Picture_detal=Picture_detal+5;
  }
  */
  /*
  if(Zebra_Mark)
  {
    Picture_detal=Picture_detal-5;
  }
  */
}


void Caculate_Variance(void)//���㷽��
{
  Left_Black_Variance=0;
  Right_Black_Variance=0;
  Left_Black_Add=0;
  Right_Black_Add=0;
  
  for(i=55;i>=10;i--)
  {
    Left_Black_Add+=Left_Black[i];
    Right_Black_Add+=Right_Black[i];
  }
  Left_Black_Average=abs(Left_Black_Add/45);
  Right_Black_Average=abs(Right_Black_Add/45);
  
  for(i=55;i>=10;i--)
  {
    Left_Black_Variance=Left_Black_Variance+abs(Left_Black[i]-Left_Black_Average);
    Right_Black_Variance=Right_Black_Variance+abs(Right_Black[i]-Right_Black_Average);
  }
  
  
  All_Variance=Left_Black_Variance+Right_Black_Variance;
}

int Min(int a,int b)
{
  if((a-b)>0)
    return b;
  else 
    return a;
}

int Max(int a,int b)
{
  if((a-b)>0)
    return a;
  else
    return b;
}


/*
�������ƣ��յ�ɲ��������һ�ι��˰�����
����޸�ʱ�䣺20161210
���ߣ����ഺ���Ƹĸ�
�������µ��ʣ�13950986716��lczq@vip.qq.com
δ������⣺ 
**************************************************************************************************/

void Finish_Check(void)
{
  Jump_Change_Point_Count=0;
  //i=30;
   for(i=15;i<50;i+=2)
   {
     if(!Jump_Change_Point_Count)
     {
       for(j=5;j<75;j++)
      {
        if( img[i][j]==0xff && img[i][j+1]==0x00 )
        {
          Jump_Change_Point_Count++;//��¼�м�����ɫ��
        }
      }
      if(!(Jump_Change_Point_Count>6 && Jump_Change_Point_Count<12))
      {
        Jump_Change_Point_Count=0;
      }
     }
     else
       break;
   }
  
  if(Jump_Change_Point_Count>7 && Jump_Change_Point_Count<12)
  {
    //gpio_set (PTE12,1);
    Zebra_Mark=1;//�����߱�־
  }
  else
  {
    //gpio_set (PTE12,0);
  }
  
  if(Zebra_Mark)
  {
    Zebra_Mark_Time++;//���������߼�ʱ
  }
  if(Zebra_Mark_Time>0 && Zebra_Mark_Time<5)
  {
    
    //gpio_set (PTC12,1);
    for(i=50;i>10;i--)
    {
      /*
      Right_Black[i]=Remedy_Right_Line[i];
      Left_Black[i]=Remedy_Left_Line[i];
      Mid_Line[i]=(Left_Black[i] + Right_Black[i])/2;
      */
      for(j=Left_Black[i+1]-5;j<Right_Black[i+1];j++)
      {
        if(img[i][j]==0x00 && img[i][j+1]==0xff)
        {
          Left_Black[i]=j;
          break;
        }
      }
        
    }
    for(i=50;i>10;i--)
    {
      for(j=Right_Black[i+1]+5;j>Left_Black[i+1];j--)
      {
        if(img[i][j]==0x00 && img[i][j-1]==0xff)
        {
          Right_Black[i]=j;
          break;
        }
      }
    }
  }
  else
  {
    //gpio_set (PTC12,0);
    Zebra_Mark=0;//���־λ
    Zebra_Mark_Time=0;//���������� 30���޸�
  }
}

void Cross_Road(void)
{
  
  for(i=55;i>0;i--)
  {
    if(LB_Lose[i]==1 && RB_Lose[i]==1)
    {
      CR_Start=i;//�·�ʮ�ֵ���ʼ��
      break;
    }
  }
  for(i=CR_Start;i>0;i--)
  {
    if(LB_Lose[i]==0 && RB_Lose[i]==0 && LB_Lose[i+1]==1 && RB_Lose[i+1]==1)
    {
      CR_End=i;//�Ϸ�ʮ�ֵĽ����� MIS
      break;
    }
  }
  if(CR_Start-CR_End>20)//ȷ�������ʮ��
  {
    for(i=55;i>=CR_Start;i--)
    {
      if(LB_Lose[i]==0 && LB_Lose[i+1]==0 && LB_Lose[i-1]==0 
         && (Left_Black[i]-Left_Black[i+1])>=0 && (Left_Black[i]-Left_Black[i-1])>=1)
      {
        LD_Point=i;//����
        break;
      }
    }
    for(i=55;i>=CR_Start;i--)
    {
      if(RB_Lose[i]==0 && RB_Lose[i+1]==0 && RB_Lose[i-1]==0 
         && (Right_Black[i]-Right_Black[i+1])<=0 && (Right_Black[i]-Right_Black[i-1])<=1)
      {
        RD_Point=i;//����
        break;
      }
    }
    if(LD_Point>RD_Point && LD_Point<55)//�Կ����������Ϊ��ʧ��ʼ��
      CR_Start=LD_Point;
    else if(RD_Point>LD_Point && RD_Point<55)//�Կ����������Ϊ��ʧ��ʼ��
      CR_Start=RD_Point;
    else if(LD_Point>0 && RD_Point==0)//һ���Ҳ���������һ��Ϊ��ʼ��
      CR_Start=LD_Point;
    else if(RD_Point>0 && LD_Point==0)//һ���Ҳ���������һ��Ϊ��ʼ��
      CR_Start=RD_Point;
    else
      CR_Start=55;
    LD_Point_j=Left_Black[CR_Start];
    RD_Point_j=Right_Black[CR_Start];
    if(RD_Point_j==0)
      RD_Point_j=75;
  }
  for(i=CR_Start;i>0;i--)
  {
    for(j=LD_Point_j+30;j>LD_Point_j;j--)//j+30������
    {
      if(img[i][j]==0x00 && img[i][j+1]==0xff)
      {
        LU_Point=i;//���Ϲյ㣬���� MIS
        LU_Point_j=j;//���Ϲյ㣬����
        break;
      }
    }
    if(LU_Point_j)
      break;
  }
  for(i=CR_Start;i>0;i--)
  {
    for(j=RD_Point_j-30;j<RD_Point_j;j++)
    {
      if(img[i][j]==0x00 && img[i][j-1]==0xff)
      {
        RU_Point=i;//���Ϲյ㣬����
        RU_Point_j=j;//���Ϲյ㣬����
        break;
      }
    }
    if(RU_Point_j)
      break;
  }
  if(LD_Point_j>0 && LU_Point_j>0)//���º����Ϲյ��ҵ�
  {
    Mid_Line[LD_Point]=LD_Point_j+Change_Line_Wide[LD_Point];
    for(i=CR_Start-1;i>LU_Point-1;i--)
    {
      Mid_Line[i]=Mid_Line[CR_Start]+((LU_Point_j-LD_Point_j)/(LU_Point-CR_Start))*(CR_Start-i);
    }
  }
  if(RD_Point_j>0 && RU_Point_j>0)//���º����Ϲյ��ҵ�
  {
    Mid_Line[RD_Point]=RD_Point_j+Change_Line_Wide[RD_Point];
    for(i=CR_Start-1;i>RU_Point-1;i--)
    {
      Mid_Line[i]=Mid_Line[CR_Start]+((RU_Point_j-RD_Point_j)/(RU_Point-CR_Start))*(CR_Start-i);
    }
  }
  if(LD_Point_j>0 && RD_Point_j>0 && LU_Point_j==0 && RU_Point_j==0)//���������ҵ� ��������û�ҵ�
  {
    for(i=CR_Start-1;i>RD_Point-1;i--)
    {
      Mid_Line[i]=Mid_Line[CR_Start];
    }
  }
  if(LD_Point_j==0 && RD_Point_j==0 && LU_Point_j>0 && RU_Point_j>0)//��������û�ҵ� ���������ҵ�
  {
    i=Min(LU_Point,RU_Point);
    Mid_Line[i]=(Left_Black[i]+Right_Black[i])/2;
    for(i=i+1;i<CR_Start;i++)
    {
      Mid_Line[i]=Mid_Line[i+1];
    }
  }
}



/*
�������ƣ���·ɨ��
����޸�ʱ�䣺20170803
���ߣ�������ʵ�������ܳ���-��ӿ
�������µ��ʣ�13950986716��lczq@vip.qq.com
δ������⣺
**************************************************************************************************/
void O_Road_Scan(void)
{
  for(i=15;i<59;i++)
  {
    for(j=10;j<40;j++)
    {
      //���½Ǻ����� yes
      if(img[i][j]==0x00 && img[i][j-2]==0x00 && img[i][j+2]==0xff && img[i-2][j]==0xff )
      {
        O_LDI=i;
        O_LDJ=j;
        break;
      }
    }
    //if(O_LDI&&O_LDJ) break;
  }
  for(i=15;i<59;i++)
  {
    for(j=40;j<70;j++)
    {
      //���½Ǻ����� yes
      if(img[i][j]==0x00 && img[i][j-2]==0xff && img[i][j+2]==0x00 && img[i-2][j]==0xff )
      {
        O_RDI=i;
        O_RDJ=j;
        break;
      }
    }
    //if(O_RDI&&O_RDJ) break;
  }
  
  if(O_LDI&&O_RDI)
  {
    for(j=O_LDJ;j>0;j--)
    {
      if(img[O_LDI][j]==0x00)
        O_LD_Flower_Black++;
      else
        break;
    }
    if(O_LDJ-O_LD_Flower_Black<3)
      O_LD_Flower_Black_Mark=1;
    if(O_LD_Flower_Black>12)
    {
      jj=O_LDJ-O_LD_Flower_Black;
      for(ii=O_LDI;ii>5;ii--)
      {
        if(img[ii][jj]==0x00)
          O_LD_Flower_Up_Black++;
        else
          break;
      }
    }
    
    for(j=O_RDJ;j<79;j++)
    {
      if(img[O_RDI][j]==0x00)
        O_RD_Flower_Black++;
      else
        break;
    }
    if(O_RDJ+O_RD_Flower_Black>77)
      O_RD_Flower_Black_Mark=1;
    if(O_RD_Flower_Black>12)
    {
      jj=O_RDJ+O_RD_Flower_Black;
      for(ii=O_RDI;ii>5;ii--)
      {
        if(img[ii][jj]==0x00)
          O_RD_Flower_Up_Black++;
        else
          break;
      }
    }
  }
  
  
  for(i=10;i<59;i++)
  {
    for(j=20;j<59;j++)
    {
        //�м��Բ����������������
        if(img[i][j]==0x00 && img[i-1][j]==0x00 &&
           img[i+1][j]==0xff && (img[i][j-10]==0xff && img[i][j+10]==0xff) 
             &&abs(j-40)<10 &&!Zhidao_Real_Flag )//Ҫ�Ľ�
        {
          O_OJ_Check=j;
          for(jj=0;jj<30;jj++)
          {
            if(img[i+1][O_OJ_Check-jj]==0xff &&img[i+1][O_OJ_Check+jj]==0xff)
            {
              O_White_spot_count++;
            }
            else
              break;
          }
          if(O_White_spot_count>30)
          {
            O_OI=i;//�м��ɫԲ�����漫ֵ�� yes
            O_OJ=j;
            break;
          }
          else
          {
            O_OI=0;//�м��ɫԲ�����漫ֵ�� yes
            O_OJ=0;
          }
          
        }
    }
    if(O_OI&&O_OJ) 
      break;
  }
  
  if(O_OI && O_OJ)
  {
    for(i=O_OI;i>1;i--)
    {
    for(j=5;j<75;j++)
    {
      if(img[i][j]==0x00 && img[i-8][j-8]==0x00 && img[i-8][j+8]==0x00 && img[i+8][j-8]==0x00 && img[i+8][j+8]==0x00)
      {
        O_Black_Flag=1;//��һ��պڿ�
        break;
      }
    }
    }
  }
  
  if(O_OI>0 && O_OJ>0 && O_Black_Flag && abs(O_LDI-O_RDI)<20 && abs((O_RDI+O_LDI)/2-O_OI)<30
      && abs((O_RDJ+O_LDJ)/2-O_OJ)<10 && O_LDJ<O_RDJ //&&abs(LB_Lose_Count_Last-RB_Lose_Count_Last)<10
        && !Ramp_Flag &&!Zhidao_Flag &&!Zebra_Mark &&!Block_Left &&!Block_Right 
          &&ML_White_Spot_count<45 &&!LU &&!RU &&O_LD_Flower_Black_Mark &&O_RD_Flower_Black_Mark
            //&&(O_LD_Flower_Up_Black>0 ||O_RD_Flower_Up_Black>0 )
              )//�㶼�ɵ��� 17-3-5��
  {
    i=(60-ML_White_Spot_count)-2;
    for(j=0;j<39;j++)
    {
      if(img[i][40-j]==0x00 &&img[i][40]==0x00 &&img[i][40+j]==0x00)
      {
        O_Up_Blank_Point2++;
      }
      else
        break;
    }
      //else
        //break;
    i=(60-ML_White_Spot_count)-3;
    for(j=0;j<39;j++)
    {
      if(img[i][40-j]==0x00 &&img[i][40]==0x00 &&img[i][40+j]==0x00)
      {
        O_Up_Blank_Point3++;
      }
      else
        break;
    }
    
    if(O_Up_Blank_Point2>5 &&O_Up_Blank_Point3>10)
    {
      
      for(i=Max(O_LDI,O_RDI);i<55;i++)
      {
        if((Left_Black[i]>Left_Black[i-1])||(Right_Black[i]<Right_Black[i-1]))//������ֱ�ߵ�����
        {
          break;
        }
      }
      if(i==55)//ȷ����ֱ����
      {
        O_Get_O_Flag=1;
        gpio_set (PTA15,1);
      }
      else
      {
        O_Get_O_Flag=0;
        gpio_set (PTA15,0);
      }
    }
    else
    {
      gpio_set (PTA15,0);
      O_Get_O_Flag=0;
    }
    //gpio_set (PTA15,1);
  }
  else
  {
    gpio_set (PTA15,0);
    //O_OI=0;
    //O_OJ=0;
    zhenTMdeyuanhuan_IN_FLAG=0;
    O_Get_O_Flag=0;
  }
  
  if((Number_0
      &&( (RB_Lose_Count_Last<10 &&LB_Lose_Count_Last>12 )
                 ||(LB_Lose_Count_Last>15 &&RB_Lose_Count_Last<=2)
                   )
       &&O_LDJ<O_OJ &&O_OJ<O_RDJ &&O_OI>0 && O_OJ>0 &&O_Black_Flag
         &&O_Get_O_Flag==0 &&!Block_Left &&!Block_Right &&ML_White_Spot_count<45 &&!Shizi_in &&!Shizi_out )//���Բ��
     ||
       (Number_0==0&&( (LB_Lose_Count_Last<10 &&RB_Lose_Count_Last>12)
                      ||(RB_Lose_Count_Last>15 &&LB_Lose_Count_Last<=2)
                        )
         &&O_LDJ<O_OJ  &&O_OI>0 && O_OJ>0 &&O_Black_Flag
           &&O_Get_O_Flag==0 &&!Block_Left &&!Block_Right &&ML_White_Spot_count<45 &&!Shizi_in &&!Shizi_out))//�ҳ�Բ��
  {
    O_Out_Flag=1;
  }
  /*
  if(O_Get_O_Flag==1)
  {
    if(O_Get_O_Flag_Time>1)
    {
      //gpio_set (PTA15,1);
      O_Get_O_Flag_Time--;
    }
    else if(O_Get_O_Flag_Time==1)
    {
      O_Get_O_Flag=0;
      O_Get_O_Flag_Time=30;
      //gpio_set (PTA15,0);
    }
  }
  */
  if(O_Out_Flag==1)
  {
    if(O_Out_Flag_Time>1)
    {
      //gpio_set (PTA15,1);
      O_Out_Flag_Time--;
    }
    else if(O_Out_Flag_Time==1)
    {
      O_Out_Flag=0;
      O_Out_Flag_Time=30;
      //gpio_set (PTA15,0);
    }
  }
}

/*
�������ƣ��ϰ�ɨ��
����޸�ʱ�䣺20170803
���ߣ�������ʵ�������ܳ���-��ӿ
�������µ��ʣ�13950986716��lczq@vip.qq.com
δ������⣺ 
**************************************************************************************************/
void Block_Scan(void)
{
  if(/*!Zebra_Mark &&*/  LB_Lose_Count_Last<10 && RB_Lose_Count_Last<10 //�����ж�TMDE
         &&ML_Black_Spot_count<15 
          &&(LB_Lose[45]==0&&RB_Lose[45]==0) &&(LB_Lose[40]==0&&RB_Lose[40]==0) &&(LB_Lose[35]==0&&RB_Lose[35]==0)
            && !Right_Black_thrity_lose && !Left_Black_thrity_lose &&!Left_Black_fiveteen_lose &&!Right_Black_fiveteen_lose
              &&!Shizi_in &&!Shizi_out
          //&&(LB_Lose[30]==0&&RB_Lose[30]==0) &&(LB_Lose[25]==0&&RB_Lose[25]==0) &&(LB_Lose[20]==0&&RB_Lose[20]==0)
          )
    {
         Zhidao_Flag=1;
         for(i=55;i>20;i=i-3)
          {
          if(!Block_Jump_Flag)
          {
            for(j=Left_Black[i]-3;j<Right_Black[i]+3;j++)
           {
             if( img[i][j]==0xff && img[i][j+1]==0x00 )
             {
               Brick_Jump_Point++;//��¼�м�����->��ɫ��
             }
           }
           if(Brick_Jump_Point>1 && Brick_Jump_Point<4 && (RB_Lose_Count<5 || LB_Lose_Count<5))
           {
             Block_Jump_Flag=1;
             Brick_Jump_Point=0;
             break;
           }
           else
           {
             Block_Jump_Flag=0;
             Brick_Jump_Point=0;
           }
          }
          }
     
    for(i=55;i>20;i--)
    {
      if((Left_Black[i]>Left_Black[i-1])||(Right_Black[i]<Right_Black[i-1]))//������ֱ�ߵ�����
      {
        break;
      }
    }
    if(i==20)//ȷ����ֱ����
    {
      Zhidao_Real_Flag=1;
        //17-05-20
      //�ұ��ϰ�begin YES
      {
        for(i=55;i>10;i--)//ɨ���ұ�שͷ����ͻ���
        {
          if(Right_Black[i-1]-Right_Black[i]<-5 && abs(Right_Black[i]-Right_Black[i+1])<3
             && abs(Right_Black[i]-Right_Black[i+2])<3 &&Right_Black[i-2]-Right_Black[i]<-5)
          {
            Block_Right_Down_i=i;
            Block_Right_Down_j=Right_Black[i];
            break;
          }
          else
          {
            Block_Right_Down_i=0;
            Block_Right_Down_j=0;
          }
        }
        for(i=Block_Right_Down_i;i>10;i--)//ɨ���ұ�שͷ��ͻ���
        {
          if(Right_Black[i-1]-Right_Black[i]>5 && abs(Right_Black[i]-Right_Black[i+1])<3
             && abs(Right_Black[i]-Right_Black[i+2])<3 &&Right_Black[i-2]-Right_Black[i]>5)
          {
            Block_Right_Up_i=i;
            Block_Right_Up_j=Right_Black[i];
            break;
          }
        }
        //if(Block_Right_Down_i==0 && Block_Right_Down_j==0)
        if(Block_Right_Up_i && Block_Right_Down_i)
        {
          Mid_Right_Black=(Block_Right_Up_i+Block_Right_Down_i)/2;
        }
      if(Block_Right_Down_i||Block_Right_Up_i && Block_Jump_Flag) //&& abs(Block_Right_Up_j-Block_Right_Down_j)<10)
      {
        Block_Right=1;
        Block_Flag=1;
      }
      else
      {
        ;//Block_Right=0;
      }
    }//�ұ��ϰ�end
    
      //����ϰ�begin NO
      {
        for(i=55;i>10;i--)//ɨ��zuo��שͷ����ͻ���
        {
          if(Left_Black[i-1]-Left_Black[i]>5 && abs(Left_Black[i]-Left_Black[i+1])<3
             && abs(Left_Black[i]-Left_Black[i+2])<3 && Left_Black[i-2]-Left_Black[i]>5)
          {
            Block_Left_Down_i=i;
            Block_Left_Down_j=Left_Black[i];
            break;
          }
          else
          {
            Block_Left_Down_i=0;
            Block_Left_Down_j=0;
          }
        }
        for(i=Block_Left_Down_i;i>10;i--)//ɨ��zuo��שͷ��ͻ���
        {
          if(Left_Black[i-1]-Left_Black[i]<-5 && abs(Left_Black[i]-Left_Black[i+1])<3
             && abs(Left_Black[i]-Left_Black[i+2])<3 &&Left_Black[i-2]-Left_Black[i]<-5)
          {
            Block_Left_Up_i=i;
            Block_Left_Up_j=Left_Black[i];
            break;
          }
        }
        //if(Block_Right_Down_i==0 && Block_Right_Down_j==0)
        if(Block_Left_Up_i && Block_Left_Down_i)
        {
          Mid_Left_Black=(Block_Left_Up_i+Block_Left_Down_i)/2;
        }
      if(Block_Left_Down_i>Block_Left_Up_i && Block_Jump_Flag) //&& abs(Block_Right_Up_j-Block_Right_Down_j)<10)
      {
        Block_Left=1;
        Block_Flag=1;
      }
      }//����ϰ�end
    }
    
     else
    {
      Zhidao_Real_Flag=0;
    }
    
    }
    else
    {
      Zhidao_Real_Flag=0;
      Zhidao_Flag=0;
      Block_Flag=0;
      Block_Right=0;
      Block_Right_Down_i=0;
      Block_Right_Down_j=0;
      Block_Right_Up_i=0;
      Block_Right_Up_j=0;
      Block_Left=0;
      Block_Left_Down_i=0;
      Block_Left_Down_j=0;
      Block_Left_Up_i=0;
      Block_Left_Up_j=0;
      Right_No_Continue=0;
      Change_Point=0;
      Block_Jump_Flag=0;
    }
    
    /*
    if(Block_Left)
      Block_Left_Count_Mark=1;
    if(Block_Left_Count_Mark &&Block_Left_Count<10)
      Block_Left_Count++;
    if(Block_Left_Count==10 &&Block_Left)
    {
      Block_Left_Count=0;
      Block_Left_Real=1;
    }
    else
    {
      Block_Left_Count_Mark=0;
      Block_Left_Count=0;
      //Block_Left_Real=0;
    }
    
    if(Block_Right)
      Block_Right_Count_Mark=1;
    if(Block_Right_Count_Mark &&Block_Right_Count<10)
      Block_Right_Count++;
    if(Block_Right_Count==10 &&Block_Right)
    {
      Block_Right_Real=1;
      Block_Right_Count=0;
    }
    else
    {
      //Block_Right_Real=0;
      Block_Right_Count_Mark=0;
      Block_Right_Count=0;
    }
    */
}

/*
�������ƣ�ʮ�ִ���
����޸�ʱ�䣺20170803
���ߣ�������ʵ�������ܳ���-��ӿ
�������µ��ʣ�13950986716��lczq@vip.qq.com
δ������⣺ 
**************************************************************************************************/
void ShiZi_Handle(void)
{
  //����ʮ�� �������油����ȥ170526 sick fuck u ass
     if(Road_Kind_Mark!=3 && ML_Black_Spot_count<30 && O_Road_Flag==0 &&O_Out_Flag==0 
        &&(LB_Lose[54]==0&&RB_Lose[54]==0) &&RB_Lose_Count_Last>10 &&LB_Lose_Count_Last>10)
    {
      for(i=50;i>10;i--) //���½�
      {
        for(j=5;j<75;j++)
        {
          if(img[i][j]==0x00 && img[i-1][j]==0xff &&LB_Lose[i+2]==0&&LB_Lose[i-2]==1)
          {
            LDI=i;
            LDJ=j;
            LD=1;
            break;
          }
        }
        if(LD)
          break;
      }
      
      for(i=50;i>10;i--)//���½�
      {
        for(j=5;j<75;j++)
        {
          if(img[i][j]==0x00 && img[i+1][j]==0xff &&RB_Lose[i+2]==0&&RB_Lose[i-2]==1)
          {
            RDI=i;
            RDJ=j;
            RD=1;
            break;
          }
        }
        if(RD)
          break;
      }
      
      if(LD && RD && abs(LDI-RDI)<20 &&LDJ<RDJ)
      {
        for(i=LDI;i>15;i--)//���Ͻ�
        {
          for(j=5;j<75;j++)
          {
            if(img[i][j]==0x00 && img[i+1][j]==0xff &&LB_Lose[i+2]==1&&LB_Lose[i-2]==0)
            {
              LUI=i;
              LUJ=j;
              LU=1;
              break;
            }
          }
          if(LU)
            break;
        }
        for(i=RDI;i>15;i--)//���Ͻ�
        {
          for(j=5;j<75;j++)
          {
            if(img[i][j]==0x00 && img[i+1][j]==0xff &&RB_Lose[i+2]==1&&RB_Lose[i-2]==0)
            {
              RUI=i;
              RUJ=j;
              RU=1;
              break;
            }
          }
          if(RU)
            break;
        }
      }
      if(LD &&RD &&LU &&RU &&abs(LUI-RUI)<20 &&LUJ<RUJ)
      {
        Left_Black_K=(LDJ-LUJ)/(LDI-LUI);
        Left_Black_B=(LDJ+LUJ)/2-Left_Black_K*(LDI+LUI)/2;
        Right_Black_K=(RDJ-RUJ)/(RDI-RUI);
        Right_Black_B=(RDJ+RUJ)/2-Right_Black_K*(RDI+RUI)/2;
        //if(Left_Black_K<Right_Black_K)
        {
          iMax=Max(LDI,RDI)+3;
          iMax=(iMax>54?54:iMax);
          iMin=Min(LUI,RUI)-3;
          iMin=(iMin<15?15:iMin);
          for(i=iMax;i>iMin;i--)
          {
            
            Left_Black[i]=(int)(Left_Black_K*i+Left_Black_B);
            Right_Black[i]=(int)(Right_Black_K*i+Right_Black_B);
            Mid_Line[i]=(Left_Black[i]+Right_Black[i])/2;
          }
        }
        Shizi_in=1;
      }
      else
      {
        Shizi_in=0;
      }
    
    }
   //��ʮ�� �������油������ 170525 OK
    else if(
     Road_Kind_Mark!=3 && !O_Road_Flag && O_Road_Flag==0 &&O_Out_Flag==0 &&!O_Black_Flag
     && (RB_Lose_Count_Last>10 && LB_Lose_Count_Last>10)
       && (LB_Lose[55]==1&&RB_Lose[55]==1)//||((LB_Lose[55]==1||RB_Lose[55]==1)&&RB_Lose_Count_Last>20 && LB_Lose_Count_Last>20))
         && (LB_Lose[53]==1&&RB_Lose[53]==1)&&(LB_Lose[51]==1&&RB_Lose[51]==1)
     )
    {
      //gpio_set (PTE12,0);
      for(i=50;i>20;i--)//����
      {
        for(j=5;j<75;j++)
        {
          if(img[i][j]==0x00 && img[i+1][j]==0xff && img[i][j+1]==0xff && img[i+1][j+1]==0xff
             && img[i-1][j-1]==0x00)
          {
            LUI=i;
            LUJ=j;
            LU=1;
            break;
          }
        }
        if(LU)
          break;
      }
      
      for(i=50;i>20;i--)//����
      {
        for(j=5;j<75;j++)
        {
          if(img[i][j]==0x00 && img[i+1][j]==0xff && img[i][j-1]==0xff && img[i+1][j-1]==0xff
             && img[i-1][j+1]==0x00)
          {
            RUI=i;
            RUJ=j;
            RU=1;
            break;
          }
        }
        if(RU)
          break;
      }
      
      if(LU && RU && abs(LUI-RUI)<20)
      {
        //i=max(LU,RU)-3;
        for(i=Min(LUI,RUI)-2;i<55;i++)
        {
          Mid_Line[i]=Mid_Line[i-1];
          Left_Black[i]=Left_Black[i-1];
          Right_Black[i]=Right_Black[i-1];
        }
        Shizi_out=1;
      }
      else
      {
        Shizi_out=0;
      }
      /*
      else if(LU&&!RU)
      {
        for(i=LU-1;i<55;i++)
        {
          Mid_Line[i]=Mid_Line[i-1];
          Left_Black[i]=Left_Black[i-1];
          Right_Black[i]=Right_Black[i-1];
        }
      }
      else if(!LU&&RU)
      {
        for(i=RU-1;i<55;i++)
        {
          Mid_Line[i]=Mid_Line[i-1];
          Left_Black[i]=Left_Black[i-1];
          Right_Black[i]=Right_Black[i-1];
        }
      }
      */
    }
    else
    {
      Shizi_in=0;
      Shizi_out=0;
    }
}

/*
�������ƣ��ϰ�����
����޸�ʱ�䣺20170803
���ߣ�������ʵ�������ܳ���-��ӿ
�������µ��ʣ�13950986716��lczq@vip.qq.com
δ������⣺ 
**************************************************************************************************/
void Block_Handle(void)
{
  //���ϰ����� OK
  if(Block_Right)
  {
    //gpio_set (PTA15,1);
    Block_Right_Up_i=Block_Right_Up_i-20;//�Ӵ�Χ ������䣬�Ƴٰ���
    Block_Right_Down_i=Block_Right_Down_i+20;//�Ӵ�Χ ������䣬�Ƴٰ���
    Block_Right_Up_i=(Block_Right_Up_i<5?5:Block_Right_Up_i);//�޷� ��ֹ����Խ��
    Block_Right_Down_i=(Block_Right_Down_i>55?55:Block_Right_Down_i);//�޷� ��ֹ����Խ��
    /*
    if(Speed_Right<750)
    {
      for(i=Block_Right_Down_i;i>Block_Right_Up_i;i--)
      {
        Mid_Line[i]=(Left_Black[i]+Right_Black[Mid_Right_Black])/2-20;//-8
      }
      //g_fSpeedControlP=2*g_fSpeedControlP_Init;
    }
    else*/
      
    {
      for(i=Block_Right_Down_i;i>Block_Right_Up_i;i--)
      {
        Mid_Line[i]=(Left_Black[i]+Right_Black[Mid_Right_Black])/2-10;//-8
      }
    }
  }
  else
  {
    Block_Right_Up_i=0;
    Block_Right_Down_i=0;
    Mid_Right_Black=0;
  }
  
  //���ϰ�����
  if(Block_Left)
  {
    Block_Left_Up_i=Block_Left_Up_i-25;//�Ӵ�Χ ������䣬�Ƴٰ���
    Block_Left_Down_i=Block_Left_Down_i+25;//�Ӵ�Χ ������䣬�Ƴٰ���
    Block_Left_Up_i=(Block_Left_Up_i<5?5:Block_Left_Up_i);//�޷� ��ֹ����Խ��
    Block_Left_Down_i=(Block_Left_Down_i>55?55:Block_Left_Down_i);//�޷� ��ֹ����Խ��
    /*
    if(Speed_Right<750)
    {
      for(i=Block_Left_Down_i;i>Block_Left_Up_i;i--)
      {
        Mid_Line[i]=(Right_Black[i]+Left_Black[Mid_Left_Black])/2+20;//+10
        //Mid_Line[i]=Right_Black[i]-0.5*Change_Line_Wide[i];
      }
      g_fSpeedControlP=2*g_fSpeedControlP_Init;
    }
    else*/
    {
      for(i=Block_Left_Down_i;i>Block_Left_Up_i;i--)
      {
        Mid_Line[i]=(Right_Black[i]+Left_Black[Mid_Left_Black])/2+11;//+10
        //Mid_Line[i]=Right_Black[i]-0.5*Change_Line_Wide[i];
      }
      g_fSpeedControlP=1*g_fSpeedControlP_Init;
    }
  }
  else
  {
    Block_Left_Up_i=0;
    Block_Left_Down_i=0;
    Mid_Left_Black=0;
  }
  /*
  if(Block_Right||Block_Left)
  {
    gpio_set (PTA15,1);
  }
  else
    gpio_set (PTA15,0);
    */
}

/*
�������ƣ��µ�ɨ��
����޸�ʱ�䣺20170803
���ߣ�������ʵ�������ܳ���-��ӿ
�������µ��ʣ�13950986716��lczq@vip.qq.com
δ������⣺ 
**************************************************************************************************/
void Po_Scan(void)
{
  if(Zhidao_Flag==1 && Ramp_Next_Delay==1 && Number_2)//�µ�����
  {
    for(i=55;i>20;i--)
    {
      if((Left_Black[i]>Left_Black[i-1])||(Right_Black[i]<Right_Black[i-1]))//������ֱ�ߵ�����
      {
        break;
      }
    }
    if(i==20)//ȷ����ֱ����
    {
      for(i=30;i<33;i++)
      {
        if(abs(Right_Black[i]-Left_Black[i])>i*1.0+Po_B)
        {
          //gpio_set (PTA14,0);
          Ramp_Flag=1;
          //qipao=1;
          break;
        }
        else
          ;//gpio_set (PTA14,1);
      }
      
      //gpio_set (PTA14,0);
    }
    else
      ;//gpio_set (PTA14,1);
    
  }
  else
    ;//gpio_set (PTA14,1);

}