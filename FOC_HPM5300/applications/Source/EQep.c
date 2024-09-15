/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-09-14     17932       the first version
 */
#include "EQep.h"

QEP qep=QEP_DEFAULTS;

void EQep_Init(QEP *p)
{

    //GPIO MUX
    EALLOW;                       // Enable EALLOW
    GpioCtrlRegs.GPBGMUX2.bit.GPIO62 = 1;   // Configure  EQEP3A
    GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 1;   // Configure  EQEP3B
    GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 1;   // Configure  EQEP3S
    GpioCtrlRegs.GPCGMUX1.bit.GPIO65 = 1;   // Configure  EQEP3I
    GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // 设置输入
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   //
    GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 1;   //
    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 1;   //
    GpioCtrlRegs.GPBCTRL.bit.QUALPRD3=0x08;
    GpioCtrlRegs.GPCCTRL.bit.QUALPRD0=0x08;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 2;  //*6
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 2;
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 2;
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 2;

    EDIS;                         // Disable EALLOW

    //QDU
    EQep3Regs.QDECCTL.all   = 0;    // QEP quadrature count mode

    //QEP

    EQep3Regs.QEPCTL.bit.FREE_SOFT = 2;
    EQep3Regs.QEPCTL.bit.QPEN= 1;        // QEP enable

    EQep3Regs.QEPCTL.bit.IEI = 2;
    EQep3Regs.QEPCTL.bit.IEL = 1;

    //PCCU
    EQep3Regs.QPOSCTL.all = 0;
    EQep3Regs.QPOSMAX = p->Max_Count;

    p->MechScaler = 2*3.1415926f/(float)p->Max_Count;

}


void EQep_Calc(QEP *p)
{

// Check an index occurrence
    if(EQep3Regs.QFLG.bit.IEL == 1)
    {
       p->Index = 1;
       p->Index_OK =1;
       EQep3Regs.QCLR.bit.IEL = 1;
    }
    else
        p->Index = 0;

// Motor rotation direction 正转+1，反转-1
    if(EQep3Regs.QEPSTS.bit.QDF)
        p->DirectionQep = 1;
    else
        p->DirectionQep = -1;

//Raw Angle
    p->RawTheta = EQep3Regs.QPOSCNT + p->CalibratedAngle;
    if(p->RawTheta < 0)
        p->RawTheta += p->Max_Count;

//Motor Mechanical Angle
    if(p->RawTheta>p->Max_Count)
        p->RawTheta -= p->Max_Count;
    p->MechTheta = p->MechScaler * (float)p->RawTheta;

//Motor Electrical angle
    p->RawTheta *= p->PolePairs;
    if(p->RawTheta>p->Max_Count)
        p->RawTheta-=p->Max_Count;
    p->ElecTheta = p->MechScaler * (float)p->RawTheta;


//// ->(0-2pi)
//#define pi 3.1415926f
//    while(p->MechTheta > pi) p->MechTheta -=2*pi;
//    while(p->ElecTheta > pi) p->ElecTheta -=2*pi;
    // ->(0-2pi)
    #define pi 3.1415926f
        while(p->MechTheta > 2*pi) p->MechTheta -=2*pi;
        while(p->MechTheta < 0   ) p->MechTheta +=2*pi;
        while(p->ElecTheta > 2*pi) p->ElecTheta -=2*pi;
        while(p->ElecTheta < 0   ) p->MechTheta +=2*pi;

//1ms测速  测速上限1000r/s=60000r/min //测速下限
    p->counter++;

    if((p->counter>=20)&&(p->Index_OK))
    {
        p->counter=0;
        p->pos_new = EQep3Regs.QPOSCNT;



        // 新的转速判断方法
                if((p->pos_new < (p->Max_Count>>2)) && (p->pos_old > ((p->Max_Count*3)>>2)))
                {
                    p->deter_p =  p->pos_new - p->pos_old + p->Max_Count;
                }
                else if((p->pos_old < (p->Max_Count>>2)) && (p->pos_new > ((p->Max_Count*3)>>2)))
                {
                    p->deter_p =  p->pos_new - p->pos_old - p->Max_Count;
                }
                else
                {
                    p->deter_p =  p->pos_new - p->pos_old;
                }


                if(p->deter_p > 0)
                    p->DirectionQep = 1;
                else
                    p->DirectionQep = -1;








        if(p->DirectionQep==1)
        {
            if(p->pos_new >= p->pos_old)
                p->deter_p = p->pos_new - p->pos_old;
            else
                p->deter_p = p->Max_Count + p->pos_new - p->pos_old;
        }

        else
        {
            if(p->pos_old >= p->pos_new)
                p->deter_p =  p->pos_old - p->pos_new ;
            else
                p->deter_p = p->Max_Count + p->pos_old - p->pos_new ;
        }

        p->pos_old = p->pos_new;


        p->Speed_w=(float)p->deter_p * p->MechScaler * 1000.0f;
        p->Speed_n=(float)p->deter_p * p->MechScaler * 1000.0f*30/pi;// 1.8310546875f;

    }



}
