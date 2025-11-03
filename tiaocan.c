/*
 * tiaocan.c
 *
 *  Created on: 2025年3月5日
 *      Author: local_user
 */

#include "tiaocan.h"

void tiaocan()
{
    seekfree_assistant_data_analysis();

    // 遍历
    for(uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
    {
        // 更新标志位
        if(seekfree_assistant_parameter_update_flag[i])
        {
            seekfree_assistant_parameter_update_flag[i] = 0;

            // 通过DEBBUG串口发送信息


            if(i==0)
            {
                gyro_pid.kp=seekfree_assistant_parameter[0];
            }
            else if(i==1)
            {
                gyro_pid.kd=seekfree_assistant_parameter[1];
            }
    /*          else    if(i==2)
                {
                    steer_ctrl.outer_kp=seekfree_assistant_parameter[2];
                }
                else   if(i==3)
                {
                    speed_ctrl_right.kp=seekfree_assistant_parameter[3];
                }
                else   if(i==4)
                {
                    speed_ctrl_right.ki=seekfree_assistant_parameter[4];
                }
                else   if(i==5)
                {
                    steer_ctrl.inner_kp=seekfree_assistant_parameter[5];
                }
                else    if(i==6)
                {
                    base_speed=seekfree_assistant_parameter[6];
                }
                else   if(i==7)
                {
                stop_flag=seekfree_assistant_parameter[7];
                }*/
        }
    }
}
