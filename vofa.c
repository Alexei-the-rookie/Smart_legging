/*
 * vofa.c
 *
 *  Created on: 2025年3月13日
 *      Author: local_user
 */

#include "vofa.h"

void Float_to_Byte(float f,unsigned char byte[])
{
    FloatLongType fl;
    fl.fdata=f;
    byte[0]=(unsigned char)fl.ldata;
    byte[1]=(unsigned char)(fl.ldata>>8);
    byte[2]=(unsigned char)(fl.ldata>>16);
    byte[3]=(unsigned char)(fl.ldata>>24);
}



void JustFloat_Test(void)
{
    float a=1,b=2;
    int i;
    uint8 byte[4]={0};
    uint8 tail[4]={0x00,0x00,0x80,0x7f};

    Float_to_Byte(a,byte);
    for(i=0;i<4;i++)
        uart_write_byte(DEBUG_UART_INDEX,byte[i]);
//    printf("%f\r\n",a);
//    printf("%f\r\n",b);
    Float_to_Byte(b,byte);
        for(i=0;i<4;i++)
            uart_write_byte(DEBUG_UART_INDEX,byte[i]);
    for(i=0;i<4;i++)
    {
//        printf("%c",tail[i]);//发送帧尾
        uart_write_byte(DEBUG_UART_INDEX,tail[i]);
    }
//    printf("\r\n");
}

void vofa_sendData(float a,float b,float c)
{
    int i;
    uint8 byte[4]={0};
    uint8 tail[4]={0x00,0x00,0x80,0x7f};
//    printf("%f\r\n",a);
    Float_to_Byte(a,byte);
        for(i=0;i<4;i++)
            uart_write_byte(DEBUG_UART_INDEX,byte[i]);
//    printf("%f\r\n",b);
    Float_to_Byte(b,byte);
        for(i=0;i<4;i++)
            uart_write_byte(DEBUG_UART_INDEX,byte[i]);
//    printf("%f\r\n",c);
    Float_to_Byte(c,byte);
        for(i=0;i<4;i++)
            uart_write_byte(DEBUG_UART_INDEX,byte[i]);
    for(i=0;i<4;i++)
    {
//        printf("%c",tail[i]);//发送帧尾
        uart_write_byte(DEBUG_UART_INDEX,tail[i]);
    }
}
