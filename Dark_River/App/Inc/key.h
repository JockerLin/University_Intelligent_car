#ifndef KEY_H_
#define KEY_H_
#include  "include.h"
#include "VCAN_TSL1401.h"

#define key_on   1
#define key_off  0

#define ok_key   PTE8
#define back_key   PTE7

#define up_key   PTE10
#define down_key  PTE11

#define plus_key   PTE9
#define minus_key   PTE6

//#define down_key   PTA29

#define keyUP     7
#define keyPLUS   9
#define keyMINUS  10
#define keyOK     6
#define keyBACK   12
#define keyDOWN   8
#define keychaxun  13
#define keyqueren  14
#define keyshifang 15


#define 		KEY_PRESS_TIME1			111111
#define   		KEY_PRESS_TIME2			555
void windows(void);
void windows1(void);
void windows2(void);
void windows3(void);
void windows4(void);
void windows5(void);
void windows6(void);
void windows7(void);
void main_scan();

/*角度菜单模块*/
void OLED_Angle_SHOW(void);
void OLED_Angle_IP_SHOW(void);
void OLED_Angle_II_SHOW(void);
void OLED_Angle_ID_SHOW(void);
void OLED_Angle_OP_SHOW(void);
void OLED_Angle_OI_SHOW(void);
void OLED_Angle_OD_SHOW(void);
void OLED_Angle_IO_PID(void);
void OLED_Angle_IP(void);//箭头指向P
void OLED_Angle_II(void);//箭头指向I
void OLED_Angle_ID(void);//箭头指向D
void OLED_Angle_OP(void);
void OLED_Angle_OI(void);
void OLED_Angle_OD(void);
/*速度模块*/
void OLED_Speed_SHOW(void);//速度调节
void OLED_Speed_PID(void);
void OLED_Speed_P_SHOW(void);
void OLED_Speed_I_SHOW(void);
void OLED_Speed_D_SHOW(void);
void OLED_Speed_Set_SHOW(void);
void OLED_Speed_PID(void);
void OLED_Speed_P(void);//箭头指向P
void OLED_Speed_I(void);//箭头指向I
void OLED_Speed_D(void);//箭头指向D
void OLED_Speed_Set(void);

/*方向模块*/
void OLED_Direction_SHOW(void);//方向调节
void OLED_Direction_P_SHOW(void);//P调节界面
void OLED_Direction_D_SHOW(void);//D调节界面
void OLED_Direction_Po_SHOW(void);//D调节界面
void OLED_Direction_W_SHOW(void);//模糊速度标准值
void OLED_Direction_C_SHOW(void);//模糊常数
void OLED_Direction_Live_Watch_Show(void);//动态参数显示开关界面
void OLED_O_Show(void);//圆环偏移量
void OLED_Direction_PID(void);//PID参数界面
void OLED_Direction_P(void);//箭头指向P
void OLED_Direction_D(void);//箭头指向D
void OLED_Direction_Po(void);//坡道时间
void OLED_Direction_W(void);//箭头指向FS
void OLED_Direction_C(void);//箭头指向C
void OLED_Direction_Live_Watch(void);//箭头指向Live_Watch
void OLED_Direction_O(void);//箭头指向圆环偏差的数值

/*图像模块*/
void OLED_IMA_SHOW(void);//显示图像
void Key_Init(void);
void oled_draw_ima(uint8 buffa[3][128]);
void show_oled_image(void);
void show_oled_main3(void);
uint8 readkey(void);
#endif /* KEY_H_ */