/*
 * imu.h
 *
 *  Created on: 2025Äê3ÔÂ3ÈÕ
 *      Author: local_user
 */

#ifndef CODE_IMU_H_
#define CODE_IMU_H_

#include "zf_common_headfile.h"

#define Ka        (0.80)
#define dt        (0.001)
#define Pi        (3.1416)
typedef struct
{
        float pitch;
        float yaw;
        float roll;

        float pitch_a;
        float roll_a;
        float pitch_g;
        float roll_g;

        float last_pitch;
        float last_roll;

        int16 offset_gx;
        int16 offset_gy;
        int16 offset_gz;
}IMU_data;

extern IMU_data imu_init;

void IMU_offset(int offset);
float FOCF(float acc_m,float gyro_m,float* last_angle);

#endif /* CODE_IMU_H_ */
