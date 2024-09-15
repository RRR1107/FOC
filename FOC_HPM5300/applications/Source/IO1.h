/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-09-14     17932       the first version
 */
#ifndef APPLICATIONS_SOURCE_IO1_H_
#define APPLICATIONS_SOURCE_IO1_H_

//*******************************************************************************
//      继电器
//*******************************************************************************

#define SOFTRLY_PFC_OFF     GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;// pre-charge relay hjk
#define SOFTRLY_PFC_ON      GpioDataRegs.GPASET.bit.GPIO3 = 1;

#define RSTRLY_PFC_OFF      {GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;}
#define RSTRLY_PFC_ON       {GpioDataRegs.GPASET.bit.GPIO2 = 1;GpioDataRegs.GPASET.bit.GPIO4 = 1;GpioDataRegs.GPASET.bit.GPIO10 = 1;GpioDataRegs.GPASET.bit.GPIO11 = 1;}

#define VOLTRLY_PFC_OFF
#define VOLTRLY_PFC_ON

#define RSTRLY_DFIG_OFF
#define RSTRLY_DFIG_ON

#define VOLTRLY_DFIG_OFF
#define VOLTRLY_DFIG_ON

//*******************************************************************************
//      Enable/Disable gate drivers added by hjk
//*******************************************************************************
#define DRIVER_DISABLE     GpioDataRegs.GPACLEAR.bit.GPIO1= 1;
#define DRIVER_ENABLE       GpioDataRegs.GPASET.bit.GPIO1 = 1;
//*******************************************************************************
//      ADC
//*******************************************************************************
extern Uint16 AdcCal[16];

//#define Grid_Curr_C_Result    ((int16)(AdcRegs.ADCRESULT0-AdcCal[0])>>4)
//#define Grid_Curr_B_Result    ((int16)(AdcRegs.ADCRESULT1-AdcCal[1])>>4)
//#define Grid_Curr_A_Result    ((int16)(AdcRegs.ADCRESULT2-AdcCal[2])>>4)
//
//#define Grid_Volt_BC_Result   ((int16)(AdcRegs.ADCRESULT3-AdcCal[3])>>4)
//#define Grid_Volt_AB_Result   ((int16)(AdcRegs.ADCRESULT4-AdcCal[4])>>4)
//#define Grid_Volt_CA_Result   ((int16)(AdcRegs.ADCRESULT5-AdcCal[5])>>4)
//
//#define BUS_Volt_N_Result     ((int16)(AdcRegs.ADCRESULT6>>4))
//#define BUS_Volt_P_Result     ((int16)(AdcRegs.ADCRESULT7>>4))
//
//#define DFIG_Curr_C_Result    ((int16)(AdcRegs.ADCRESULT8-AdcCal[8])>>4)
//#define DFIG_Curr_B_Result    ((int16)(AdcRegs.ADCRESULT9-AdcCal[9])>>4)
//#define DFIG_Curr_A_Result    ((int16)(AdcRegs.ADCRESULT10-AdcCal[10])>>4)
//
//#define DFIG_Volt_AB_Result   ((int16)(AdcRegs.ADCRESULT11-AdcCal[11])>>4)
//#define DFIG_Volt_BC_Result   ((int16)(AdcRegs.ADCRESULT12-AdcCal[12])>>4)
//#define DFIG_Volt_CA_Result   ((int16)(AdcRegs.ADCRESULT13-AdcCal[13])>>4)
//
//#define ADCRESULT14           ((int16)(AdcRegs.ADCRESULT14>>4))
//#define ADCRESULT15           ((int16)(AdcRegs.ADCRESULT15>>4))



//*******************************************************************************
//      LED
//*******************************************************************************

#define LED3_On()        GPIO_WritePin(93, 0)
#define LED3_Off()       GPIO_WritePin(93, 1)
#define LED3_TOGGLE()    GpioDataRegs.GPCTOGGLE.bit.GPIO93=1

#define LED4_On()        GPIO_WritePin(94, 0)
#define LED4_Off()       GPIO_WritePin(94, 1)
#define LED4_TOGGLE()    GpioDataRegs.GPCTOGGLE.bit.GPIO94=1

#define W5300ResetOn()      GPIO_WritePin(68, 0)
#define W5300ResetOff()     GPIO_WritePin(68, 1)

//*******************************************************************************
//      按键
//*******************************************************************************
#define System_ON    GpioDataRegs.GPBDAT.bit.GPIO99/*开机*/
#define System_OFF   GpioDataRegs.GPBDAT.bit.GPIO27/*关机*/

//#define OE_ON    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;/*开启硬件保护*/
//#define OE_OFF    GpioDataRegs.GPASET.bit.GPIO25 = 1;/*禁止硬件保护*/
//#define CPLD_FLT    GpioDataRegs.GPADAT.bit.GPIO24/*硬件电流保护关机*/

//*******************************************************************************
//      网络
//*******************************************************************************
//#define W5300ResetOn()  GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
//#define W5300ResetOff();    GpioDataRegs.GPBSET.bit.GPIO52 = 1;


#endif /* APPLICATIONS_SOURCE_IO1_H_ */
