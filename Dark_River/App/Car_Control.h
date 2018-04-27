#ifndef _Car_control_h_
#define _Car_control_h_

#include "common.h"
#include "include.h"
/*
�������ƣ�Car_Control.h
����޸�ʱ�䣺20170803
���ߣ�������ʵ�������ܳ���-��ӿ
�������µ��ʣ�13950986716��lczq@vip.qq.com
δ������⣺���Ӷ�
**************************************************************************************************/
void Angle_Control();
void AngleCalculate(float Acc_Z,float Gyro_Y);        //�廪�˲�����
void Balance_Filter(float Acc_Z,float Gyro_Z);        //�����˲�
void Get_Speed();                                      //��ȡ�������ٶ�
void Angle_In_Loop_Control();                          //�Ƕ��ڻ�
void Angle_Out_Loop_Control();                         //�Ƕ��⻷
void Angle_All_Control();                              //�ǶȻ�����
void Speed_Loop_Control();                             //�ٶȿ���
void Speed_Loop_Output();                              //�ٶ����
void Dir_Loop_Control();                               //���򻷿���
void Dir_Loop_Output();                                //�������
void MotorOutput();                                    //����ռ�ձȼ���
void SetMotorVoltage(float fLeftVal, float fRightVal);//ռ�ձ����
void Get_Acc_And_Gyro(void);                           //�ɼ����ٶȺ������ǵ�λ����Ϣ
void Send_Data_Up();                                   //���ݷ�������λ��
void Direction_Detal_Caculate();                       //����������
void Stop_Check(void);                                 //����ͣ��
void All_Start(void);                                  //ȫ����ʼ��
float Begin_Stand_Check(void);                         //����վ�����
void Part_Road(void);                                  //��·�ֶδ���
void DIP_status(void);                                 //���뿪��״̬
void plan_Chose(void);                                 //��λѡ��
extern void Get_Middle_Line(void);                    //��ȡ����
extern void Direction_Control_Output(void);           //����������
extern void Part_Road(void);                          //��·�ֶδ���
extern void Data_Send(UARTn_e uratn,unsigned short int *pst);//���ݷ��͵���λ��

#endif