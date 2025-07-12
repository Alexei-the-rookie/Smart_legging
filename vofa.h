/*
 * vofa.h
 *
 *  Created on: 2025Äê3ÔÂ13ÈÕ
 *      Author: local_user
 */

#ifndef CODE_VOFA_H_
#define CODE_VOFA_H_

#include "zf_common_headfile.h"

typedef union
{
        float fdata;
        unsigned long ldata;
}FloatLongType;

void Float_to_Byte(float f,unsigned char byte[]);
void JustFloat_Test(void);
void vofa_sendData(float a,float b,float c);

#endif /* CODE_VOFA_H_ */
