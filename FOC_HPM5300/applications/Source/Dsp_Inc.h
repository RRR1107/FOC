/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-09-14     17932       the first version
 */
#ifndef APPLICATIONS_SOURCE_DSP_INC_H_
#define APPLICATIONS_SOURCE_DSP_INC_H_

#include "IO1.h"

#include "GlobalTypes.h"
#include "math.h"

#include "Ethernet\ethernet.h"

#include "VectorControl.h"

//*******************************************************************************
//   #define  SVPWM_TEST      ///<逆变测试
//   #define  Dui_Ding        ///<对顶测试
//   #define  Protect_Test
//   #define  I_Loop          ///<单电流环测试
//   #define  PV_Double_Loop


//之前： #define SVPWM_TEST     ///<单电流环测试
//改之后m：
// #define  I_Loop          ///<单电流环测试
//*******************************************************************************

#define DSPTimeDly(a) DELAY_US(a*1000);

extern Uint16 RlyStatus,InvState,InvFault;
extern float Isr_Time;

void Dsp_Init(void);
//void VectorControl_Init(void);
//*******************************************************************************
//   Pwm
//*******************************************************************************
void InitEPwm(void);

void EnablePWM(void);
void DisablePWM(void);


//*******************************************************************************
//   Adc
//*******************************************************************************
void InitAdc(void);

//实验室系统
#define Grid_Volt_AB_Result  AdcbResultRegs.ADCRESULT0
#define Grid_Volt_BC_Result  AdcaResultRegs.ADCRESULT0
#define Grid_Volt_CA_Result  AdcdResultRegs.ADCRESULT0

#define Grid_Curr_A_Result   AdcdResultRegs.ADCRESULT1
#define Grid_Curr_B_Result   AdcbResultRegs.ADCRESULT1
#define Grid_Curr_C_Result   AdcaResultRegs.ADCRESULT1

#define BUS_Volt_P_Result    AdcdResultRegs.ADCRESULT2
#define BUS_Volt_N_Result    AdcbResultRegs.ADCRESULT2


//*******************************************************************************
//   IO
//*******************************************************************************


//  //并网继电器
//#define RSTRLY_PFC_ON      GpioDataRegs.GPBSET.bit.GPIO53 = 1;    RlyStatus |= 0x1000    //闭合并网继电器
//#define RSTRLY_PFC_OFF     GpioDataRegs.GPBCLEAR.bit.GPIO53 = 1;  RlyStatus &= 0x0FFF    //断开并网继电器
//
//   //软启继电器
//#define SOFTRLY_PFC_ON     GpioDataRegs.GPBSET.bit.GPIO55 = 1;    RlyStatus |= 0x0100    //闭合软启继电器
//#define SOFTRLY_PFC_OFF    GpioDataRegs.GPBCLEAR.bit.GPIO55 = 1;  RlyStatus &= 0xF0FF    //断开软启继电器
//
//  //电压采样继电器
//#define VOLTRLY_PFC_ON     GpioDataRegs.GPBSET.bit.GPIO54 = 1;    RlyStatus |= 0x0010    //闭合采样继电器
//#define VOLTRLY_PFC_OFF    GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1;  RlyStatus &= 0xFF0F    //断开采样继电器





//*******************************************************************************
//   全局变量
//*******************************************************************************
extern Uint16 VoltOffsetA, VoltOffsetB, VoltOffsetC;            ///<网侧三相电压偏移误差
extern Uint16 CurrOffsetA, CurrOffsetB, CurrOffsetC;            ///<三相电流偏移误差
extern Uint16 VoltOffsetAB, VoltOffsetBC, VoltOffsetCA;            ///<网侧三相电压偏移误差





extern float Id_Soft,Iq_Soft,Ubus_Soft;
extern PI Id_PICtl,Iq_PICtl,Ubus_PICtl;



void Key_On_Press(void);
void Key_Off_Press(void);
void Init_Gpio_Key(void);

#endif /* APPLICATIONS_SOURCE_DSP_INC_H_ */
