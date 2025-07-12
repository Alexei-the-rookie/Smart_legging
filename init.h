#include "zf_common_headfile.h"
#ifndef CODE_INIT_H_
#define CODE_INIT_H_

#define SERVO_1                 (ATOM2_CH2_P33_6)       //定义主板上舵机1对应引脚
#define SERVO_2                 (ATOM2_CH3_P33_7)       //定义主板上舵机2对应引脚
#define SERVO_3                 (ATOM1_CH4_P02_4)       //定义主板上舵机3对应引脚
#define SERVO_4                 (ATOM0_CH5_P02_5)       //定义主板上舵机4对应引脚
#define SERVO_FREQ              (300)                   //定义主板上舵机频率
#define SERVO1_MID              (4433)                    //舵机1中值越小越收  左前电机板
#define SERVO2_MID              (4633)                    //舵机2中值越小越蹬  右前电机板&陀螺仪
#define SERVO3_MID              (4400)                    //舵机3中值越小越收  右后
#define SERVO4_MID              (4500)                    //舵机4中值越小越蹬  左后WiFi


void all_init(void);
void servo_init(void);


#endif /* CODE_INIT_H_ */
