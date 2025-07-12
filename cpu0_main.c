/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "isr_config.h"
#include "isr.h"
#include "zf_common_headfile.h"
#include "wheel_pid.h"
#include "small_driver_uart_control.h"
#include "math.h"
#include "tiaocan.h"
#include "vofa.h"
#include "imu.h"
#include "init.h"
#include "vmc.h"
#pragma section all "cpu0_dsram"

// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
#define WHEEL_PIT               (CCU60_CH0)//无刷电机定时器中断
#define TOP_PIT                 (CCU60_CH1)//陀螺仪定时器中断
#define ANGEL_PIT               (CCU61_CH0)//角度环
#define SPEED_PIT               (CCU61_CH1)//速度环
#define MAX_SPEED               (30)                                                // 最大 测试 占空比

#define WIFI_SSID_TEST          "Test"
#define WIFI_PASSWORD_TEST      "12345678"
#define TCP_TARGET_IP           "192.168.43.122"             // 连接目标的 IP
#define TCP_TARGET_PORT         "8086"                      // 连接目标的端口
#define WIFI_LOCAL_PORT         "6666"                      // 本机的端口 0：随机  可设置范围2048-65535  默认 6666

#define LED1                    (P20_9)//核心板上的小灯

extern uint8 bridge_flag,jump_flag;//单边桥标志位、跳跃标志位
//int16 pwm_ph1, pwm_ph4, pwm_ph2, pwm_ph3;//腿部偏移量
float high = 6.0f, leg_angle = 0.0f;//计算五连杆使用
float s_out_filter=0;//速度环低通滤波使用
extern float s=0,angel_out=0,r=0;//速度环&角度环&滚转环输出量
int16 g=0,trans=0;//三环输出

float gp=40.0f,gi=0.0f,gd=0.0f,gmi=3000,gmc=5000;//角速度环系数

float ap=3.45f,ai=0.0f,ad=0.00f,ami=3000,amc=5000;//角度环系数

float sp=3.00f,si=0.0f,sd=0.00f,smi=3000,smc=5000;//速度环系数

float yp=40.0f,yi=0.0f,yd=0.00f,ymi=3000,ymc=5000;//偏航角速度系数

float rp=1.0f,ri=0.0f,rd=0.0f,rmi=3000,rmc=5000;//滚转角度环系数

float ideal_speed=0.0f,ideal_angel=0.0f,ideal_yaw_gyro=0.0f;//PID数组里的目标值
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    servo_init();                   //init文件查看舵机初始化以及舵机中点 调机械零点用
    // 此处编写用户代码 例如外设初始化代码等
    /*
    pwm_init(PWM_CH1, 50, 500);// 初始化 PWM 通道 频率 17KHz 初始占空比 0%
    pwm_init(PWM_CH2, 50, 500);// 初始化 PWM 通道 频率 17KHz 初始占空比 0%
    pwm_init(PWM_CH3, 50, 500);// 初始化 PWM 通道 频率 17KHz 初始占空比 0%
    pwm_init(PWM_CH4, 50, 500);// 舵机初始化

    pwm_set_duty(PWM_CH1,700);//左前，调大向下转 250-1250
    pwm_set_duty(PWM_CH2,800);//右后，调大向上转
    pwm_set_duty(PWM_CH3,700);//右前，调大向下转
    pwm_set_duty(PWM_CH4,800);//左后，调大向上转//750是平的
    */
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);//小灯初始化
    yaw_pid_init(yp,yi,yd,ymi,ymc);//偏航角速度环初始化
    gyro_pid_init(gp,gi,gd,gmi,gmc);//角速度环参数初始化
    angel_pid_init(ap,ai,ad,ami,amc);//角度环参数初始化
    speed_pid_init(sp,si,sd,smi,smc);//速度环参数初始化
    roll_pid_init(rp,ri,rd,rmi,rmc);//滚转环参数初始化
    while(1)
        {
            if(imu963ra_init())//陀螺仪初始化
            {
               printf("\r\nIMU963RA init error.");                                 // IMU963RA 初始化失败
            }
            else
            {
               break;
            }
            gpio_toggle_level(LED1);                                                // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
        }
    //IMU_offset(1500);//50ms陀螺仪计算启动静止偏移
    pit_ms_init(TOP_PIT,1);//陀螺仪定时器中断
    pit_ms_init(SPEED_PIT,20);//速度环20ms定时器
    pit_ms_init(ANGEL_PIT,5);//角度环5ms定时器
    pit_ms_init(WHEEL_PIT,1);//无刷电机定时器中断

    small_driver_uart_init();//无刷双驱初始化
    small_driver_get_speed();
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕

/*
    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);                                                   // 初始化失败 等待 100ms
    }

    printf("\r\n module version:%s",wifi_spi_version);                          // 模块固件版本
    printf("\r\n module mac    :%s",wifi_spi_mac_addr);                         // 模块 MAC 信息
    printf("\r\n module ip     :%s",wifi_spi_ip_addr_port);                     // 模块 IP 地址


    // zf_device_wifi_spi.h 文件内的宏定义可以更改模块连接(建立) WIFI 之后，是否自动连接 TCP 服务器、创建 UDP 连接
    if(1 != WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开启自动连接 就需要手动连接目标 IP
    {
        while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
            "TCP",                                                              // 指定使用TCP方式通讯
            TCP_TARGET_IP,                                                      // 指定远端的IP地址，填写上位机的IP地址
            TCP_TARGET_PORT,                                                    // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
            WIFI_LOCAL_PORT))                                                   // 指定本机的端口号
        {
            // 如果一直建立失败 考虑一下是不是没有接硬件复位
            printf("\r\n Connect TCP Servers error, try again.");
            system_delay_ms(100);                                               // 建立连接失败 等待 100ms
        }
    }


    // 逐飞助手初始化 数据传输使用高速WIFI SPI
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    while (TRUE)
    {
        // 写入需要发送的数据，有几个通道就写多少个数据
        // 这里仅写入4个通道数据
        seekfree_assistant_oscilloscope_data.data[0] += 0.1;
        seekfree_assistant_oscilloscope_data.data[1] += 0.5;
        seekfree_assistant_oscilloscope_data.data[2] += 1;
        seekfree_assistant_oscilloscope_data.data[3] += 2;
//        detector_oscilloscope_data.data[4] = 10;
//        detector_oscilloscope_data.data[5] = 100;
//        detector_oscilloscope_data.data[6] = 1000;
//        detector_oscilloscope_data.data[7] = 10000;

        // 设置本次需要发送几个通道的数据
        seekfree_assistant_oscilloscope_data.channel_num = 4;

        // 这里进发送了4个通道的数据，最大支持8通道
        seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);

        system_delay_ms(20);
        // 有可能会在逐飞助手软件上看到波形更新不够连续，这是因为使用WIFI有不确定的延迟导致的

        // 解析上位机发送过来的参数，解析后数据会存放在seekfree_assistant_oscilloscope_data数组中，可以通过在线调试的方式查看数据
        // 例程为了方便因此写在了主循环，实际使用中推荐放到周期中断等位置，需要确保函数能够及时的被调用，调用周期不超过20ms
        seekfree_assistant_data_analysis();
        tiaocan();
        // 此处编写需要循环执行的代码
    }


*/
    while (TRUE)//主函数死循环
    {
        // 此处编写需要循环执行的代码
        gpio_toggle_level(LED1);//小灯闪烁
        vofa_sendData(imu_init.last_pitch,imu_init.roll,speed_pid.real);//使用vofa上位机调参
        //printf("%f,%f,%f",imu_init.last_pitch,imu_init.roll,speed_pid.real);
        //vofa_sendData(angel_out,imu_init.pitch,(motor_value.receive_left_speed_data+motor_value.receive_right_speed_data)/2);//imu_init.pitch
/*      printf("\r\nWheel Speed: L=%5d,R=%5d\r\n",motor_value.receive_left_speed_data,motor_value.receive_right_speed_data);// 翻转 LED 引脚输出电平 控制 LED 亮灭
        printf("\r\npitch=%3.2f\r\n",imu_init.pitch);
        printf("\r\ntarget:%3.2f\r\n",angel_pid.real);*/
        /*ideal_speed=50.0f;
        system_delay_ms(1500);
        ideal_speed=0.0f;
        system_delay_ms(2000);
        ideal_speed=-50.0f;
        system_delay_ms(1500);
        ideal_speed=0.0f;
        system_delay_ms(2000);*/
       // imu963ra_gyro_x/14.3f
        system_delay_ms(100);
        // 此处编写需要循环执行的代码
    }
}

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)//定时器cc60_pit_ch0控制无刷电机转动
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);

    yaw_pid.real=(float)(imu963ra_gyro_z/14.3f);
    yaw_pid.target=ideal_yaw_gyro;
    trans=yaw_pid_cal(yaw_pid);//转向角速度环

    gyro_pid.real=(float)(imu963ra_gyro_x/14.3f);//俯仰角速度环
    gyro_pid.target=0+angel_out;
    g=gyro_pid_cal(gyro_pid);
    small_driver_set_duty(g-trans, g+trans);//执行：驱动电机
}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH0);

    angel_pid.target=ideal_angel;
    angel_pid.real=imu_init.pitch;//角度环
    angel_out=angel_pid_cal(angel_pid);//角度输出
}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH1);

    speed_pid.target=ideal_speed;//速度环
    speed_pid.real=((motor_value.receive_left_speed_data-motor_value.receive_right_speed_data)/2);
    s=speed_pid_cal(speed_pid);
    s_out_filter=(s_out_filter*19+s)/20.0f;

    roll_pid.target=0.0f;
    roll_pid.real=imu_init.roll;
    r=roll_pid_cal(roll_pid);//滚转环

    pwm_set_duty(SERVO_1,SERVO1_MID-s_out_filter+r);
    pwm_set_duty(SERVO_2,SERVO2_MID+s_out_filter+r);
    pwm_set_duty(SERVO_3,SERVO3_MID+s_out_filter-r);
    pwm_set_duty(SERVO_4,SERVO4_MID-s_out_filter-r);//舵机既关系俯仰(s_out_filter)又关系滚转(r)
}

IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH1);



    imu963ra_get_acc();                             // 获取 IMU963RA 的加速度测量数值
    imu963ra_get_gyro();                            // 获取 IMU963RA 的角速度测量数值
    imu963ra_get_mag();                             // 获取 IMU963RA 的地磁计测量数值


/*
    imu963ra_gyro_x-=imu_init.offset_gx;
    imu963ra_gyro_y-=imu_init.offset_gy;
    imu963ra_gyro_z-=imu_init.offset_gz;//消除启动静止噪音
*/
    if((imu963ra_gyro_x>-3)&&(imu963ra_gyro_x<3))
        imu963ra_gyro_x=0;
    if((imu963ra_gyro_y>-5)&&(imu963ra_gyro_y<5))
        imu963ra_gyro_y=0;
    if((imu963ra_gyro_z>-5)&&(imu963ra_gyro_z<5))
        imu963ra_gyro_z=0;

    imu_init.pitch_a=atan2(imu963ra_acc_y,imu963ra_acc_z)/(PI/180);
    imu_init.pitch_g=-(imu963ra_gyro_x)/14.3;
    imu_init.pitch=FOCF(imu_init.pitch_a,imu_init.pitch_g,&imu_init.last_pitch);//互补滤波计算pitch姿态角

    imu_init.roll_a=atan2(imu963ra_acc_x,imu963ra_acc_z)/(PI/180);
    imu_init.roll_g=-(imu963ra_gyro_y)/14.3;
    imu_init.roll=FOCF(imu_init.roll_a,imu_init.roll_g,&imu_init.last_roll);//互补滤波计算roll姿态角

}

#pragma section all restore
// **************************** 代码区域 ****************************
