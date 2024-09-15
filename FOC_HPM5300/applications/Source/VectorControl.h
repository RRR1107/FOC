/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-09-14     17932       the first version
 */
#ifndef APPLICATIONS_SOURCE_VECTORCONTROL_H_
#define APPLICATIONS_SOURCE_VECTORCONTROL_H_

#include "math.h"
#include "Dsp_Cfg.h"
//#include "include.h"

//*******************************************************************************
//   电网信息
//*******************************************************************************
typedef struct{
                  float a,b,c;
                  float ab,bc,ca;
                  float alpha,beta;
                  float sin,cos;
                  float d,q,zero;
                }Three_Phase_Info;

#define Three_Phase_Info_Default {0.0f,0.0f,0.0f, 0.0f,0.0f,0.0f, \
                                   0.0f,0.0f, 0.0f,0.0f, 0.0f,0.0f,0.0f}

//*******************************************************************************
//   坐标变换
//*******************************************************************************
#define CLARKE(v)                                   \
    v.alpha=((0.66666666667f)*(v.a - ((0.5f)*(v.b+v.c))));    \
    v.beta =((0.57735026913f)*(v.b - v.c));\
    v.zero =((0.33333333333f)*(v.a + v.b + v.c));

#define i_CLARKE(v)                              \
    v.a = v.alpha + 0.5f*v.zero;                         \
    v.b = -0.5f*v.alpha + 0.8660254f*v.beta + 0.5f*v.zero; \
    v.c = -0.5f*v.alpha - 0.8660254f*v.beta + 0.5f*v.zero;

#define PARK(v)                 \
    v.d= v.alpha*v.cos + v.beta*v.sin;  \
    v.q=-v.alpha*v.sin + v.beta*v.cos;


#define i_PARK(v)            \
    v.alpha = v.d*v.cos - v.q*v.sin;\
    v.beta  = v.d*v.sin + v.q*v.cos;
#define L2P(v)            \
    v.a = (v.ab-v.ca)*0.33333333f;\
    v.b = (v.bc-v.ab)*0.33333333f;\
    v.c = (v.ca-v.bc)*0.33333333f;

#define P2L(v)    \
    v.ab = v.a-v.b;\
    v.bc = v.b-v.c;\
    v.ca = v.c-v.a;\

//*******************************************************************************
//   Dsogi
//*******************************************************************************
//DSOGI
typedef struct {
                  float  vin;   // Input
                  float  error,osg;
                  float  u,q_u,k;
                  float  w,ef,t;
                 } DSOGI;

#define DSOGI_Default {0.01,0.01,0.01,0.01,0.01,0,0.45,0.01,0.01}
//void SOGI (SOGI_PLL* v);

#if defined(F_10kHz_1)
    #define Dsp_Ts 0.00010f  //10Khz
#endif

#if defined(F_10kHz_2) || defined(F_20kHz_1)
   #define Dsp_Ts 0.00005f  //20Khz
#endif

#if defined(F_20kHz_2)
  #define Dsp_Ts 0.000025f //40Khz
#endif

#define DSOGI(v)  \
     v.error=v.vin-v.u;\
     v.osg=(v.error*1.4142135623f-v.q_u);\
     v.u=v.u+v.w*v.osg*Dsp_Ts;\
     v.t=v.t+Dsp_Ts*v.u;\
     v.q_u=v.t*v.w;\
     v.ef=v.error*v.q_u;\
    // v.k=v.ef*-65/(v.q_u*v.q_u+v.u*v.u);\
     v.w=v.w+v.w*v.k*Dsp_Ts;\




//方法二：二阶低通滤波器
typedef struct{  float  Vab,Vbc;    // Input: line voltage

                    float  Vab_f90,Vbc_f90;
                    float  Vab_f180,Vbc_f180;

                    float  Vab_f90k_1,Vab_f90k_2;
                    float  Vbc_f90k_1,Vbc_f90k_2;

                    float  Vab_f180k_1,Vab_f180k_2;
                    float  Vbc_f180k_1,Vbc_f180k_2;

                    float  Valfa_fp,Vbeta_fp,Vamp_inv;    // Output:

                    float  cos,sin;

              } Fliter_PLL;
#define Fliter_PLL_Default {0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0, 0,0}
//方法三：alpha,beta下二阶低通滤波器
 typedef struct{  float  Valfa,Vbeta;    // Input: voltage
                  float  Valfa_90, Valfa_90_1, Valfa_90_2;
                  float  Valfa_180,Valfa_180_1,Valfa_180_2;
                  float  Vbeta_90, Vbeta_90_1, Vbeta_90_2;
                  float  Vbeta_180,Vbeta_180_1,Vbeta_180_2;
                  float  Valfa_p,Vbeta_p,Vamp_inv;
                  float  cos,sin;// Output:

  } Fliter_PLL1;
#define PLL1_Default {0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0}

#define PLL_Cal_1(v)  \
   v.Valfa_90 = Fliter_Cal(v.Valfa_90_1,v.Valfa_90_2,v.Valfa);\
   v.Valfa_90_2 = v.Valfa_90_1;\
   v.Valfa_90_1 = v.Valfa_90;\
\
           \
   v.Vbeta_90 = Fliter_Cal(v.Vbeta_90_1,v.Vbeta_90_2,v.Vbeta);\
   v.Vbeta_90_2 = v.Vbeta_90_1;\
   v.Vbeta_90_1 = v.Vbeta_90;\
   v.Valfa_180 = Fliter_Cal(v.Valfa_180_1,v.Valfa_180_2,v.Valfa_90);\
   v.Valfa_180_2 = v.Valfa_180_1;\
   v.Valfa_180_1 = v.Valfa_180;\
\
   v.Vbeta_180 = Fliter_Cal(v.Vbeta_180_1,v.Vbeta_180_2,v.Vbeta_90);\
   v.Vbeta_180_2 = v.Vbeta_180_1;\
   v.Vbeta_180_1 = v.Vbeta_180;\
\
   v.Vamp_inv = 1/sqrtf(v.Valfa*v.Valfa+v.Vbeta*v.Vbeta);\
\
   v.sin = v.Vbeta*v.Vamp_inv;  \
   v.cos = v.Valfa*v.Vamp_inv;


#if defined(F_10kHz_1)
    #define Fliter_Cal(y_1,y_2,x) \
         (1.9681009e+00f*y_1)-(9.6907243e-01f*y_2)+(9.7145857e-04f*x) //10KHz
#endif

#if defined(F_10kHz_2) || defined(F_20kHz_1)
    #define Fliter_Cal(y_1,y_2,x) \
        (1.9841700e+00f*y_1)-(9.8441476e-01f*y_2)+(2.4480226e-04f*x) //20KHz
#endif

#if defined(F_20kHz_2)
    #define Fliter_Cal(y_1,y_2,x) \
        (1.9921153e+00f*y_1)-(9.9217678e-01f*y_2)+(6.1442792e-05f*x) //40KHz
#endif

#define PLL_Cal_2(v)  \
     v.Vab_f90 = Fliter_Cal(v.Vab_f90k_1,v.Vab_f90k_2,v.Vab);\
     v.Vab_f90k_2 = v.Vab_f90k_1;\
     v.Vab_f90k_1 = v.Vab_f90;\
\
     v.Vbc_f90 = Fliter_Cal(v.Vbc_f90k_1,v.Vbc_f90k_2,v.Vbc);\
     v.Vbc_f90k_2 = v.Vbc_f90k_1;\
     v.Vbc_f90k_1 = v.Vbc_f90;\
\
     v.Vab_f180 = Fliter_Cal(v.Vab_f180k_1,v.Vab_f180k_2,v.Vab_f90k_2);\
     v.Vab_f180k_2 = v.Vab_f180k_1;\
     v.Vab_f180k_1 = v.Vab_f180;\
\
     v.Vbc_f180 = Fliter_Cal(v.Vbc_f180k_1,v.Vbc_f180k_2,v.Vbc_f90k_2);\
     v.Vbc_f180k_2 = v.Vbc_f180k_1;\
     v.Vbc_f180k_1 = v.Vbc_f180;\
\
     v.Valfa_fp = -0.28867513466f*v.Vbc_f90 - 0.33333333333f*v.Vab_f180 - 0.16666666667f*v.Vbc_f180;\
     v.Vbeta_fp =  0.33333333333f*v.Vab_f90 + 0.16666666667f*v.Vbc_f90  - 0.28867513466f*v.Vbc_f180;\
\
     v.Vamp_inv = 1/sqrtf(v.Valfa_fp*v.Valfa_fp+v.Vbeta_fp*v.Vbeta_fp);\
\
     v.sin = v.Vbeta_fp*v.Vamp_inv;  \
     v.cos = v.Valfa_fp*v.Vamp_inv;

//*******************************************************************************
//   PI
//*******************************************************************************
typedef struct {  float  Ref,Fdb,Err;
                    float  Kp,Ki;
                    float  Up,Ui;
                    float  Out,OutPre;
                    float  OutMax,OutMin;
                    } PI;

#define PI_Default {0,0,0, 0,0,0,0, 0,0,0,0}
void pi_calc(PI *v);
void pi_calc_nf(PI *v);
void pi_clear(PI *v);

//*******************************************************************************
//   PR
//*******************************************************************************
typedef struct {  float  Ref,Fdb;
                    float  Kp,Ki;
                    float  cos_wT,Freq;
                    float  Err,Err_l;
                    float  Up;
                    float  Ur,Ur_l,Ur_ll;
                    float  Out,OutPre;
                    float  OutMin,OutMax;
                    } PR;
#define PR_Default {0.0f,0.0f, 0.0f,0.0f, 0.0f,0.0f, 0.0f,0.0f, 0.0f, 0.0f,0.0f,0.0f, 0.0f,0.0f, 0.0f,0.0f}

void pr_init(float Kp,float Ki,float Max,float Freq,PR *obj);
void pr_calc(PR *obj);
void pr_clear(PR *v);


//*******************************************************************************
//   SVPWM
//*******************************************************************************
typedef struct {  float  Alfa;    // Input: reference alpha-axis phase voltage
                    float  Beta;    // Input: reference beta-axis phase voltage
                    float  Vdc;    // Input: the DC bus voltage
                    float  Tpwm;    // Input: PWM period
                    float  Ta;    // Output: reference phase-a1 switching function
                    float  Tb;    // Output: reference phase-b1 switching function
                    float  Tc;    // Output: reference phase-c1 switching function
                    void (*calc)();    // Pointer to calculation function
               } SVGENDQ;

typedef SVGENDQ *SVGENDQ_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the SVGENDQ object.
-----------------------------------------------------------------------------*/
#define SVGENDQ_DEFAULTS {    0, 0, 1, 0, 0, 0, 0, \
                              (void (*)(Uint32))svgendq_calc}

/*------------------------------------------------------------------------------
Prototypes for the functions in SVGEN_DQ.C
------------------------------------------------------------------------------*/


void svgendq_calc(SVGENDQ *v);


#endif /* APPLICATIONS_SOURCE_VECTORCONTROL_H_ */
