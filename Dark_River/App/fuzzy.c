#include "common.h"
#include "include.h"
//右弯道误差为正，左弯道误差为负
//跑道内误差范围E[-22,27]；
//误差变化率EC[]；
#define E_Max 25
#define E_Min -25
#define Ec_Max 10
#define Ec_Min -10
#define NL 1
#define NB 2
#define NM 3
#define NS 4
#define O  5
#define PS 6
#define PM 7
#define PB 8
#define PL 9
//待整定
/*

          -22 -15 -8  0  8   15  22
          1   2   3   4  5   6   7
          NL  NM  NS  O  PS  PM  PL
10  7 PL
6   6 PM
3   5 PS
0   4 O
-3  3 NS
-6  2 NM
-10 1 NL

*/
/*
float Fuzzy_Kp[9][9]={
                    //-4  -3  -2  -1   0     1   2   3   4
                      3.2,2.8,2.2,1.4, 0.6, 1.0,1.0,0.4,0.4,//-4
                      2.2,2.8,2.2,1.4, 0.8, 0.8,1.2,0.8,0.2,//-3
                      1.2,1.7,1.9,1.2, 0.4, 0.8,1.0,0.8,1.2,//-2
                      0.2,0.7,1.9,1.2, 0.2, 0.8,0.6,0.8,1.2,//-1
                      0.2,0.7,0.9,1.1, 0.2, 1.1,0.9,0.7,0.2,//0
                      1.2,0.8,1.2,0.6, 0.2, 1.1,1.9,0.7,0.2,//1
                      1.2,0.9,1.2,0.8, 0.4, 1.2,1.9,1.7,1.2,//2
                      0.2,1.0,1.4,0.8, 0.8, 1.4,2.2,2.8,2.2,//3
                      0.4,0.4,1.4,1.0, 0.6, 1.4,2.2,2.8,3.2,//4
                      };//2.4稳定不飘 2.5gg 经历过省赛

*/

float Fuzzy_Kp[9][9]={
                    //-4  -3  -2  -1   0     1   2   3   4
                      3.2,2.8,2.2,1.4, 0.6, 1.0,1.0,0.4,0.4,//-4
                      3.2,2.8,2.2,1.4, 0.6, 0.8,1.2,0.8,0.2,//-3
                      3.0,2.7,1.9,1.2, 0.4, 0.8,1.0,0.8,1.2,//-2
                      3.0,2.7,1.9,1.2, 0.2, 0.8,0.6,0.8,1.2,//-1
                      2.8,2.4,1.7,1.0, 0.0, 1.0,1.7,2.4,2.8,//0
                      1.2,0.8,1.2,0.6, 0.2, 1.2,1.9,2.7,3.0,//1
                      1.2,0.9,1.2,0.8, 0.4, 1.2,1.9,2.7,3.0,//2
                      0.2,1.0,1.4,0.8, 0.6, 1.4,2.2,2.8,3.2,//3
                      0.4,0.4,1.4,1.0, 0.6, 1.4,2.2,2.8,3.2,//4
                      };//2.4稳定不飘 2.5待加强 经历过省赛

//float Fuzzy_Speed_Kp[10]={0.2,0.3,0.5,0.7,0.95,1.2,1.5,2.0,2.5,2.1};
float Fuzzy_Speed_Kp[20]={-5,-4.5,-4,-3.5,-3,-2.5,-2,-1.5,-1.0,-0.5,
                           0,0.5,1,1.5,2,2.5,3,3.5,4.0,4.5};//3.0稳定不飘
//float Fuzzy_Kd[9][9]={};
extern float Dir_detal,Picture_detal_detal,P_Direction_Control,P_Direction_Control_now,
             D_Direction_Control,D_Direction_Control_now,g_fSpeedDeltaL,Speed_Left;
float E,EC,E_Speed;
int8 E_Number,EC_Number,E_Rank,EC_Rank,E_Speed_Rank,Dir_Speed_Fuzzy;
extern int32 g_dLeftMotorPulseSigma,	  //编码器左轮脉冲计数
              g_dRightMotorPulseSigma;  //编码器右轮脉冲计数
int16 FS_Spd=1100,Dir_Spd_Fuzzy_Max=8,Dir_Spd_Fuzzy_Min=-8; //10 -8
float Dir_Spd_Fuzzy_constant=1.3;

/*
函数名称：模糊应用
最后修改时间：20170803
作者：厦理创新实验室智能车队-暗涌
参考：林青春
程序时间：
有疑问致电邮：13950986716，lczq@vip.qq.com
**************************************************************************************************/
void Fuzzy_Apply(float Dir_detal,float Picture_detal_detal)
{
  E=Dir_detal;
  EC=Picture_detal_detal;
  //方法①
  /*
  E_Number=E/4;
  EC_Number=EC/4;
  if(E_Number>4)E_Number=4;
  if(EC_Number>4)EC_Number=4;
  if(E_Number<-4)E_Number=-4;
  if(EC_Number<-4)EC_Number=-4;
  E_Rank=E_Number+4;
  EC_Rank=EC_Number+4;
  */
  
  //方法②
  E_Rank=(int8)(4*E/E_Max)+4;
  EC_Rank=(int8)(4*EC/Ec_Max)+4;
  if(E_Rank>8)E_Rank=8;
  if(E_Rank<0)E_Rank=0;
  if(EC_Rank>8)EC_Rank=8;
  if(EC_Rank<0)EC_Rank=0;
  //P_Direction_Control_now=P_Direction_Control+Fuzzy_Kp[E_Rank][EC_Rank];
  //D_Direction_Control_now=D_Direction_Control+Fuzzy_Kd[E_Rank][EC_Rank];
  
  //方向P与速度大小
  E_Speed=Speed_Left-FS_Spd;
#define E_Speed_Min 0
#define E_Speed_Max 1800
  Dir_Speed_Fuzzy=(int)E_Speed/50;
  Dir_Speed_Fuzzy=(Dir_Speed_Fuzzy>Dir_Spd_Fuzzy_Max?Dir_Spd_Fuzzy_Max:Dir_Speed_Fuzzy);
  Dir_Speed_Fuzzy=(Dir_Speed_Fuzzy<Dir_Spd_Fuzzy_Min?Dir_Spd_Fuzzy_Min:Dir_Speed_Fuzzy);
  //E_Speed_Rank=(int8)(E_Speed)/10+10;//分9份
  //if(E_Speed_Rank<0)E_Speed_Rank=0;
  //if(E_Speed_Rank>19)E_Speed_Rank=19;
  P_Direction_Control_now=P_Direction_Control+Dir_Speed_Fuzzy*Dir_Spd_Fuzzy_constant;//+Fuzzy_Speed_Kp[E_Speed_Rank];
  
}