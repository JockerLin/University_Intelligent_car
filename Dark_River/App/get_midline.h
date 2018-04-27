#ifndef     _get_midline_H_ //先测试get_midline.h是否被宏定义过
#define     _get_midline_H_  //如果get_midline.h没有被宏定义过，定义get_midline.h，并编译以下程序段
/*
函数名称：get_midline.h
最后修改时间：20170803
作者：厦理创新实验室-暗涌队
有疑问致电邮：13950986716，lczq@vip.qq.com
未解决问题：肚子饿
**************************************************************************************************/
#include "math.h"
#include "include.h"
#include "common.h"

void Direction_Detal_Caculate();      //方向环误差计算
void O_Road_Handle(void);                 //环路
void Slope_Left(void);               //左斜入十字补线
void Slope_Right(void);              //右斜入十字补线
void Slope_Left_Line(void);          //左斜入十字补线
void Slope_Right_Line(void);         //右斜入十字补线
void Check_S_Road(void);             //S弯道检测 待整定
void Caculate_Variance(void);        //方差计算
void Ten_Road_Check(void);           //十字路口检测
void Lose_Line_Check(void);          //特定行丢线检测
void O_Road_Ckeck(void);             //环路检测
int Min(int a,int b);                //求最小
int Max(int a,int b);                //求最大
void T_Line(void);                   //十字补线
void Finish_Check(void);             //斑马线检测
void Cross_Road(void);               //十字路口
void Draw_Midline(void);             //画线
void O_Road_Scan(void);              //环路检测
void Block_Scan(void);               //砖头检测
void ShiZi_Handle(void);             //十字处理
void Block_Handle(void);             //砖头处理
void Po_Scan(void);                  //坡道检测

#endif  //终止if