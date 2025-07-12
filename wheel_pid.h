/*
 * wheel_pid.h
 *
 *  Created on: 2025Äê3ÔÂ1ÈÕ
 *      Author: local_user
 */

#ifndef CODE_WHEEL_PID_H_
#define CODE_WHEEL_PID_H_
#include "zf_common_headfile.h"
#include "imu.h"
typedef struct
{
        float target;
        float real;
        float error;
        float form_error;
        float kp;
        float ki;
        float kd;
        float correct;
        float max_i;
        float max_correct;
}pid;

extern pid gyro_pid, angel_pid, speed_pid, yaw_pid, roll_pid;

void gyro_pid_init(float gp,float gi,float gd,float gmi,float gmc);
void angel_pid_init(float ap,float ai,float ad,float ami,float amc);
void speed_pid_init(float sp,float si,float sd,float smi,float smc);
void yaw_pid_init(float yp,float yi,float yd,float ymi,float ymc);
void roll_pid_init(float rp,float ri,float rd,float rmi,float rmc);

int16 gyro_pid_cal(pid gyro_pid);
float angel_pid_cal(pid angel_pid);
float speed_pid_cal(pid speed_pid);
float yaw_pid_cal(pid yaw_pid);
float roll_pid_cal(pid roll_pid);

#endif /* CODE_WHEEL_PID_H_ */
