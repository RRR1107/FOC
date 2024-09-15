/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-09-14     17932       the first version
 */
#ifndef APPLICATIONS_SOURCE_DSP_CFG_H_
#define APPLICATIONS_SOURCE_DSP_CFG_H_

//Mode：
//   #define  SVPWM_TEST      ///<逆变测试
//   #define  Dui_Ding        ///<对顶测试
//   #define  Protect_Test
//   #define  I_Loop          ///<单电流环测试
//   #define  PV_Double_Loop
//   #define SVG_Double_Loop     ///<单电流环测试

//Frequency：
//#define  F_10kHz_1    //10kHz开关频率，10Hz控制频率
//#define  F_10kHz_2    //10kHz开关频率，20kHz控制频率
#define  F_20kHz_1    //20kHz开关频率，20kHz控制频率
//#define  F_20kHz_2    //20kHz开关频率，40Hz控制频率


#endif /* APPLICATIONS_SOURCE_DSP_CFG_H_ */
