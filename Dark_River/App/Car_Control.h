#ifndef _Car_control_h_
#define _Car_control_h_

#include "common.h"
#include "include.h"
/*
函数名称：Car_Control.h
最后修改时间：20170803
作者：厦理创新实验室智能车队-暗涌
有疑问致电邮：13950986716，lczq@vip.qq.com
未解决问题：肚子饿
**************************************************************************************************/
void Angle_Control();
void AngleCalculate(float Acc_Z,float Gyro_Y);        //清华滤波方案
void Balance_Filter(float Acc_Z,float Gyro_Z);        //互补滤波
void Get_Speed();                                      //获取编码器速度
void Angle_In_Loop_Control();                          //角度内环
void Angle_Out_Loop_Control();                         //角度外环
void Angle_All_Control();                              //角度环整合
void Speed_Loop_Control();                             //速度控制
void Speed_Loop_Output();                              //速度输出
void Dir_Loop_Control();                               //方向环控制
void Dir_Loop_Output();                                //方向环输出
void MotorOutput();                                    //最终占空比计算
void SetMotorVoltage(float fLeftVal, float fRightVal);//占空比输出
void Get_Acc_And_Gyro(void);                           //采集加速度和陀螺仪的位置信息
void Send_Data_Up();                                   //数据发送至上位机
void Direction_Detal_Caculate();                       //方向误差计算
void Stop_Check(void);                                 //紧急停车
void All_Start(void);                                  //全部初始化
float Begin_Stand_Check(void);                         //起跑站立检测
void Part_Road(void);                                  //道路分段处理
void DIP_status(void);                                 //拨码开关状态
void plan_Chose(void);                                 //档位选择
extern void Get_Middle_Line(void);                    //获取中线
extern void Direction_Control_Output(void);           //方向控制输出
extern void Part_Road(void);                          //道路分段处理
extern void Data_Send(UARTn_e uratn,unsigned short int *pst);//数据发送到上位机

#endif