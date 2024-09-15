/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-09-14     17932       the first version
 */
#ifndef APPLICATIONS_SOURCE_EQEP_H_
#define APPLICATIONS_SOURCE_EQEP_H_

#include "Dsp_Inc.h"
#define QEP_DEFAULTS { (Uint32)1250*4, 5,0,   \
                        0,0,0,0,0,0,0,         \
                        0,0,0,0,0,0,           \
                      (void (*)())EQep_Init,  \
                      (void (*)())EQep_Calc}
/*-----------------------------------------------------------------------------
Define the structure of the QEP (Quadrature Encoder) Driver Object
-----------------------------------------------------------------------------*/
typedef struct {
                    // Parameter:
                    Uint32 Max_Count;       //   line encoder *4
                    Uint16 PolePairs;       //  Number of pole pairs
                    int32 CalibratedAngle;  //  Raw angular offset between encoder index

                    // Variable:
                    float MechScaler;      //   2*pi/Max_Count
                    int32 RawTheta;         //  Raw Angle

                    // Output:
                    float ElecTheta;       //  Motor Electrical Angle
                    float MechTheta;       //  Motor Mechanical Angle
                    int16 DirectionQep;     //  Motor rotation direction 正转+1，反转-1
                    int16 Index;            //  An index occurrence Index=1
                    int16 Index_OK;    //  Count When index occur

                    //测速
                    float Speed_w;      //rad/s
                    float Speed_n;      //r/min
                    int32  deter_p;
                    int32  pos_old,pos_new;
                    Uint16 counter;

                    // func
                    void (*init)();         // Pointer to the init function
                    void (*calc)();         // Pointer to the calc function
                    }  QEP;

 extern   QEP qep;

void EQep_Init(QEP *p);
void EQep_Calc(QEP *p);

#endif /* APPLICATIONS_SOURCE_EQEP_H_ */
