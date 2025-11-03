/*
 * imu.c
 *
 *  Created on: 2025年3月3日
 *      Author: local_user
 */

#include "imu.h"
#include "zf_device_imu963ra.h"

IMU_data imu_init;

void IMU_offset(int offset)//获取IMU启动误差
{
    int i;
    imu_init.offset_gx=0;
    imu_init.offset_gy=0;
    imu_init.offset_gz=0;
    for(i=0;i<offset;i++)
    {
        system_delay_ms(1);
        imu963ra_get_acc();
        imu963ra_get_gyro();//在每个循环中获取角速度
        if(imu963ra_gyro_x==imu963ra_gyro_y)
        {
            i--;
        }
        else
        {
            imu_init.offset_gx+=imu963ra_gyro_x;
            imu_init.offset_gy+=imu963ra_gyro_y;
            imu_init.offset_gz+=imu963ra_gyro_z;
        }
    }
    imu_init.offset_gx/=offset;
    imu_init.offset_gy/=offset;
    imu_init.offset_gz/=offset;
}

float FOCF(float acc_m,float gyro_m,float* last_angle)//一阶互补滤波
{
    float temp_angle;
    temp_angle=Ka*acc_m+(1-Ka)*(*last_angle+gyro_m*dt);
    *last_angle=temp_angle;
    return temp_angle;
}
