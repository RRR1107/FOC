/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-09-14     17932       the first version
 */
#ifndef APPLICATIONS_SOURCE_ZJ_H_
#define APPLICATIONS_SOURCE_ZJ_H_

#include <math.h>
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
typedef struct {
  real32_T Volt1[3];                   /* '<Root>/Volt1 (电网线电压)' */
  real32_T Curr1[3];                   /* '<Root>/Curr1 (定子线电流)' */
  real32_T Volt2[3];                   /* '<Root>/Volt2 (双馈线电压)' */
  real32_T Curr2[3];                   /* '<Root>/Curr2 (转子线电流)' */
  real32_T Vdc1[2];                    /* '<Root>/Vdc1 (直流母线电压)' */
  real32_T Vdc2[2];                    /* '<Root>/Vdc2 (直流母线电压)' */
  real32_T SpeedRps;                   /* '<Root>/SpeedRps (电机转速)' */
  real32_T RotorTheta;                 /* '<Root>/RotorTheta (转子电角度)' */
  uint16_T State;                      /* '<Root>/State (按键控制系统状态)' */
} ExtU_MBD_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T ScopeCh[14];                /* '<Root>/ScopeCh' */
  real32_T DebugCh[17];                /* '<Root>/DebugCh' */
  real32_T DA_Ch[4];                   /* '<Root>/DA_Ch (数模转换输出)' */
  real32_T Vout1Vdc[2];                /* '<Root>/Vout1(α,β)//Vdc' */
  boolean_T Pwm1;                      /* '<Root>/Pwm1' */
  boolean_T MainRelay1;                /* '<Root>/MainRelay1' */
  boolean_T SoftStartRelay1;           /* '<Root>/SoftStartRelay1' */
  real32_T Vout2Vdc[2];                /* '<Root>/Vout2(α,β)//Vdc' */
  boolean_T Pwm2;                      /* '<Root>/Pwm2' */
  boolean_T MainRelay2;                /* '<Root>/MainRelay2' */
  boolean_T SoftStartRelay2;           /* '<Root>/SoftStartRelay2' */
  int16_T PolePairs;                   /* '<Root>/PolePairs' */
  int16_T InitAngle;                   /* '<Root>/InitAngle' */
  boolean_T Fault;                     /* '<Root>/Fault' */
} ExtY_MBD_T;


#endif /* APPLICATIONS_SOURCE_ZJ_H_ */
