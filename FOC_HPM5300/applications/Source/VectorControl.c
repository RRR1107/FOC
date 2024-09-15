/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-09-14     17932       the first version
 */

#include "VectorControl.h"
#include "Dsp_Inc.h"

#pragma CODE_SECTION(pi_calc,  "ramfuncs");
#pragma CODE_SECTION(pi_clear, "ramfuncs");
#pragma CODE_SECTION(pr_calc,  "ramfuncs");
#pragma CODE_SECTION(pr_clear, "ramfuncs");

#pragma CODE_SECTION(svgendq_calc, "ramfuncs");



//*******************************************************************************
//   锁相
//*******************************************************************************
/*
void SOGI (SOGI_PLL* v)
{
    v->error=v->vin-v->v;
    v->osg=(v->error*1.4142135623f-v->q_v)*314.15926f;
    v->v=v->v+v->osg*0.00005f;
    v->q_v=v->q_v+314.15926f*0.00005f*v->v;

    v->AMP=1/sqrt(v->v*v->v+v->q_v*v->q_v);

    v->cos=v->v*v->AMP;
    v->sin=v->q_v*v->AMP;
}
*/

//*******************************************************************************
//   PI计算
//*******************************************************************************
void pi_calc(PI *v)
{
    v->Err = v->Ref - v->Fdb;
    v->Up = (v->Kp) * (v->Err);

    //抗积分饱和（超出阈值，积分不再累加）（考虑改为累计负偏差）
//    if(v->Out == v->OutPre)
        v->Ui = v->Ui + (v->Ki)*(v->Err);

    v->OutPre = v->Up + v->Ui;

    if (v->OutPre > v->OutMax)
        {v->Out =  v->OutMax;}
    else if (v->OutPre < v->OutMin)
        {v->Out =  v->OutMin;}
    else
        {v->Out = v->OutPre;}
}

void pi_calc_nf(PI *v)
{

    v->Up = (v->Kp) * (v->Err);
    //抗积分饱和（超出阈值，积分不再累加）（考虑改为累计负偏差）
//    if(v->Out == v->OutPre)
        v->Ui = v->Ui + (v->Ki)*(v->Err);

    v->OutPre = v->Up + v->Ui;

    if (v->OutPre > v->OutMax)
        {v->Out =  v->OutMax;}
    else if (v->OutPre < v->OutMin)
        {v->Out =  v->OutMin;}
    else
        {v->Out = v->OutPre;}
}

void pi_clear(PI *v)
{
    v->Ref=0.0f;
    v->Fdb=0.0f;
    v->Err=0.0f;
    v->Up=0.0f;
    v->Ui=0.0f;
    v->OutPre=0.0f;
    v->Out=0.0f;

}




#define PI_M  3.14159265358979323846f
#define sqrt2_def 1.4142135623731f

#if defined(F_10kHz_1)
    #define Dsp_Ts 0.00010f  //10Khz
#endif

#if defined(F_10kHz_2) || defined(F_20kHz_1)
   #define Dsp_Ts 0.00005f  //20Khz
#endif

#if defined(F_20kHz_2)
  #define Dsp_Ts 0.000025f //40Khz
#endif

void pr_init(float Kp,float Ki,float Max,float Freq,PR *obj)
{
    obj->Freq=Freq;
    obj->cos_wT=cosf(2.0f*PI_M*obj->Freq*Dsp_Ts);
    obj->Ref = obj->Fdb = 0.0f;
    obj->Err = obj->Err_l = 0.0f;
    obj->Up =  0.0f;
    obj->Ur  = obj->Ur_l  = obj->Ur_ll = 0.0f;
    obj->Out = obj->OutPre  = 0.0f;

    obj->OutMin = -Max;
    obj->OutMax = Max;

    obj->Kp = Kp;
//    obj->Ki = sqrt2_def*2.0f*PI_M*Freq*Ki/20000.0f;
    obj->Ki = 0.01523f;
}


void pr_calc(PR *obj)
{
    obj->Err=obj->Ref-obj->Fdb;

    obj->Up=obj->Kp*obj->Err;
    obj->Ur=2.0f*obj->cos_wT*obj->Ur_l - obj->Ur_ll + obj->Ki*(obj->Err - obj->cos_wT*obj->Err_l);

    obj->Err_l=obj->Err;
    obj->Ur_ll=obj->Ur_l;
    obj->Ur_l=obj->Ur;

    obj->OutPre = obj->Up + obj->Ur;

    if (obj->OutPre > obj->OutMax)
        {obj->Out =  obj->OutMax;}
    else if (obj->OutPre < obj->OutMin)
        {obj->Out =  obj->OutMin;}
    else
        {obj->Out = obj->OutPre;}
}


void pr_clear(PR *v)
{
    v->Err_l=v->Err=0.0f;
    v->Ur_ll=v->Ur_l=v->Ur=0.0f;

    v->Up=0.0f;
    v->Ur=0.0f;
    v->OutPre=0.0f;
    v->Out=0.0f;

}

/*
*********************************************************************************************************
* Function:
* Description:
* Call:
* Called By:
* Input:
* Output:
* Return:
* Others:
*********************************************************************************************************
*/

#define gen_3 1.7320508f

float Vref1,Vref2,Vref3,
       X,Y,Z,Vdcinvt;
int16 A,B,C, sector;
float t1,t2,t1sat,t2sat;
float taon,tbon,tcon;
void svgendq_calc(SVGENDQ *v)
{

    //计算Vref1,Vref2,Vref3,X,Y,Z
     Vref1=v->Beta;
     Vref2= gen_3*v->Alfa-v->Beta;
     Vref3=-gen_3*v->Alfa-v->Beta;

     Vdcinvt=v->Tpwm/(2.0*v->Vdc);
     X=gen_3*Vdcinvt*v->Beta;
     Y=gen_3*Vdcinvt*v->Beta*0.5f+1.5f*Vdcinvt*v->Alfa;
     Z=gen_3*Vdcinvt*v->Beta*0.5f-1.5f*Vdcinvt*v->Alfa;

    //判断扇区
    if(Vref1>0)
        A=1;
    else
        A=0;
    if(Vref2>0)
        B=1;
    else
        B=0;
    if(Vref3>0)
        C=1;
    else
        C=0;

    sector = A + 2*B + 4*C;


    switch (sector)
    {
        case 1:
                t1 = Z;
                t2 = Y;
                break;
        case 2:
                t1 = Y;
                t2 = -X;
                break;
        case 3:
                t1 = -Z;
                t2 = X;
                break;
        case 4:
                t1 = -X;
                t2 = Z;
                break;
        case 5:
                t1 = X;
                t2 = -Y;
                break;
        default:
                t1 = -Y;
                t2 = -Z;
    }

    //判断是否进入饱合，如果进入饱合则按相位不变的原则进行处理
    if((t1+t2)>(v->Tpwm*0.5f))
    {
        t1sat=t1*v->Tpwm/(2.0f*(t1+t2));
        t2sat=t2*v->Tpwm/(2.0f*(t1+t2));
        t1=t1sat;
        t2=t2sat;
    }

    //给taon,tbon,tcon赋值
     taon=((v->Tpwm*0.5f)-t1-t2)*0.5f;
     tbon=taon+t1;
     tcon=tbon+t2;

    //给CMPR1,CMPR2,CMPR3赋值
    switch (sector)
    {
        case 1:
                v->Ta = tbon;
                v->Tb = taon;
                v->Tc = tcon;
                break;
        case 2:
                v->Ta = taon;
                v->Tb = tcon;
                v->Tc = tbon;
                break;
        case 3:
                v->Ta = taon;
                v->Tb = tbon;
                v->Tc = tcon;
                break;
        case 4:
                v->Ta = tcon;
                v->Tb = tbon;
                v->Tc = taon;
                break;
        case 5:
                v->Ta = tcon;
                v->Tb = taon;
                v->Tc = tbon;
                break;
        default:
                v->Ta = tbon;
                v->Tb = tcon;
                v->Tc = taon;
    }

    return;
}
