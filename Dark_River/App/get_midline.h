#ifndef     _get_midline_H_ //�Ȳ���get_midline.h�Ƿ񱻺궨���
#define     _get_midline_H_  //���get_midline.hû�б��궨���������get_midline.h�����������³����
/*
�������ƣ�get_midline.h
����޸�ʱ�䣺20170803
���ߣ�������ʵ����-��ӿ��
�������µ��ʣ�13950986716��lczq@vip.qq.com
δ������⣺���Ӷ�
**************************************************************************************************/
#include "math.h"
#include "include.h"
#include "common.h"

void Direction_Detal_Caculate();      //����������
void O_Road_Handle(void);                 //��·
void Slope_Left(void);               //��б��ʮ�ֲ���
void Slope_Right(void);              //��б��ʮ�ֲ���
void Slope_Left_Line(void);          //��б��ʮ�ֲ���
void Slope_Right_Line(void);         //��б��ʮ�ֲ���
void Check_S_Road(void);             //S������ ������
void Caculate_Variance(void);        //�������
void Ten_Road_Check(void);           //ʮ��·�ڼ��
void Lose_Line_Check(void);          //�ض��ж��߼��
void O_Road_Ckeck(void);             //��·���
int Min(int a,int b);                //����С
int Max(int a,int b);                //�����
void T_Line(void);                   //ʮ�ֲ���
void Finish_Check(void);             //�����߼��
void Cross_Road(void);               //ʮ��·��
void Draw_Midline(void);             //����
void O_Road_Scan(void);              //��·���
void Block_Scan(void);               //שͷ���
void ShiZi_Handle(void);             //ʮ�ִ���
void Block_Handle(void);             //שͷ����
void Po_Scan(void);                  //�µ����

#endif  //��ֹif