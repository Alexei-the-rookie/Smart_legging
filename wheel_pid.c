/*
 * wheel_pid.c
 *
 *  Created on: 2025年3月1日
 *      Author: local_user
 */


#include "wheel_pid.h"
//给轮子做PID控制，从内到外分别是角速度环，角度环，速度环
//用三个函数来做，最后在一个函数里调用
//转向环单做
pid gyro_pid, angel_pid, speed_pid, yaw_pid, roll_pid;

void gyro_pid_init(float gp,float gi,float gd,float gmi,float gmc)
{
    gyro_pid.form_error=0;
    gyro_pid.kp=gp;
    gyro_pid.ki=gi;
    gyro_pid.kd=gd;
    gyro_pid.max_i=gmi;
    gyro_pid.max_correct=gmc;
}

void angel_pid_init(float ap,float ai,float ad,float ami,float amc)
{
    angel_pid.form_error=0;
    angel_pid.kp=ap;
    angel_pid.ki=ai;
    angel_pid.kd=ad;
    angel_pid.max_i=ami;
    angel_pid.max_correct=amc;
}
void speed_pid_init(float sp,float si,float sd,float smi,float smc)
{
    speed_pid.form_error=0;
    speed_pid.kp=sp;
    speed_pid.ki=si;
    speed_pid.kd=sd;
    speed_pid.max_i=smi;
    speed_pid.max_correct=smc;
}

void yaw_pid_init(float yp,float yi,float yd,float ymi,float ymc)
{
    yaw_pid.form_error=0;
    yaw_pid.kp=yp;
    yaw_pid.ki=yi;
    yaw_pid.kd=yd;
    yaw_pid.max_i=ymi;
    yaw_pid.max_correct=ymc;
}

void roll_pid_init(float rp,float ri,float rd,float rmi,float rmc)
{
    roll_pid.form_error=0;
    roll_pid.kp=rp;
    roll_pid.ki=ri;
    roll_pid.kd=rd;
    roll_pid.max_i=rmi;
    roll_pid.max_correct=rmc;
}

int16 gyro_pid_cal(pid gyro_pid)
{
    float p=0,i=0,d=0;
    gyro_pid.error=gyro_pid.target-gyro_pid.real;
    p=gyro_pid.kp*gyro_pid.error;
    i+=gyro_pid.ki*gyro_pid.error;

    if(i>=gyro_pid.max_i)
        i=gyro_pid.max_i;
    else if(i<=(-gyro_pid.max_i))
        i=-gyro_pid.max_i;//积分值限制防止过大

    d=gyro_pid.kd*(gyro_pid.error-gyro_pid.form_error)/0.001f;
    gyro_pid.correct=p+i+d;
    gyro_pid.form_error=gyro_pid.error;

    if(gyro_pid.correct>=gyro_pid.max_correct)
        gyro_pid.correct=gyro_pid.max_correct;
    else if(gyro_pid.correct<=(-gyro_pid.max_correct))
        gyro_pid.correct=-gyro_pid.max_correct;//输出值要做限制，不可过大（代价：一个无刷电机）

    return gyro_pid.correct;//使用之前先把form_error置零！
}

float angel_pid_cal(pid angel_pid)
{
    float p,i=0,d;

    angel_pid.error=angel_pid.target-angel_pid.real;
    p=angel_pid.kp*angel_pid.error;
    i+=angel_pid.ki*angel_pid.error;

    if(i>=angel_pid.max_i)
        i=angel_pid.max_i;
    else if(i<=(-angel_pid.max_i))
        i=-angel_pid.max_i;//积分值限制防止过大

    d=angel_pid.kd*(angel_pid.error-angel_pid.form_error)/0.005f;
    angel_pid.correct=p+i+d;
    angel_pid.form_error=angel_pid.error;

    if(angel_pid.correct>=angel_pid.max_correct)
        angel_pid.correct=angel_pid.max_correct;
    else if(angel_pid.correct<=(-angel_pid.max_correct))
        angel_pid.correct=-angel_pid.max_correct;//输出值要做限制，不可过大（代价：一个无刷电机）

    return angel_pid.correct;//使用之前先把form_error置零！
}

float speed_pid_cal(pid speed_pid)
{
    float p,i=0,d;
    speed_pid.error=speed_pid.target-speed_pid.real;
    p=speed_pid.kp*speed_pid.error;
    i+=speed_pid.ki*speed_pid.error;

    if(i>=speed_pid.max_i)
        i=speed_pid.max_i;
    else if(i<=(-speed_pid.max_i))
        i=-speed_pid.max_i;//积分值限制防止过大

    d=speed_pid.kd*(speed_pid.error-speed_pid.form_error)/0.02f;
    speed_pid.correct=p+i+d;
    speed_pid.form_error=speed_pid.error;

    if(speed_pid.correct>=speed_pid.max_correct)
        speed_pid.correct=speed_pid.max_correct;
    else if(speed_pid.correct<=(-speed_pid.max_correct))
        speed_pid.correct=-speed_pid.max_correct;//输出值要做限制，不可过大（代价：一个无刷电机）

    return speed_pid.correct;//使用之前先把form_error置零！
}

float yaw_pid_cal(pid yaw_pid)
{
    float p,i=0.0f,d;
    yaw_pid.error=yaw_pid.target-yaw_pid.real;
    p=yaw_pid.kp*yaw_pid.error;
    i+=yaw_pid.ki*yaw_pid.error;

    if(i>=yaw_pid.max_i)
        i=yaw_pid.max_i;
    else if(i<=(-yaw_pid.max_i))
        i=-yaw_pid.max_i;//积分值限制防止过大

    d=yaw_pid.kd*(yaw_pid.error-yaw_pid.form_error)/0.001f;
    yaw_pid.correct=p+i+d;
    yaw_pid.form_error=yaw_pid.error;

    if(yaw_pid.correct>=yaw_pid.max_correct)
        yaw_pid.correct=yaw_pid.max_correct;
    else if(yaw_pid.correct<=(-yaw_pid.max_correct))
        yaw_pid.correct=-yaw_pid.max_correct;//输出值要做限制，不可过大（代价：一个无刷电机）

    return yaw_pid.correct;//使用之前先把form_error置零！
}

float roll_pid_cal(pid roll_pid)
{
    float p,i=0.0f,d;
    roll_pid.error=roll_pid.target-roll_pid.real;
    p=roll_pid.kp*roll_pid.error;
    i+=roll_pid.ki*roll_pid.error;

    if(i>=roll_pid.max_i)
        i=roll_pid.max_i;
    else if(i<=(-roll_pid.max_i))
        i=-roll_pid.max_i;//积分值限制防止过大

    d=roll_pid.kd*(roll_pid.error-roll_pid.form_error)/0.02f;
    roll_pid.correct=p+i+d;
    roll_pid.form_error=roll_pid.error;

    if(roll_pid.correct>=roll_pid.max_correct)
        roll_pid.correct=roll_pid.max_correct;
    else if(roll_pid.correct<=(-roll_pid.max_correct))
        roll_pid.correct=-roll_pid.max_correct;//输出值要做限制，不可过大（代价：一个无刷电机）

    return roll_pid.correct;//使用之前先把form_error置零！
}

