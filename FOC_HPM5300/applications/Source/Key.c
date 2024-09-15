/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-09-14     17932       the first version
 */
#include "include.h"

#include "Dsp_Inc.h"

void Task_Key_Scan(void);//需10-50ms 调用一次
#define Key1_Gpio    99//99
#define Key2_Gpio   27//27
//#define Key3_Gpio    26
//#define Key4_Gpio    25
//#define Key5_Gpio    24
//#define Key6_Gpio    133

#define Key1     GPIO_ReadPin(Key1_Gpio)
#define Key2     GPIO_ReadPin(Key2_Gpio)
//#define Key3     GPIO_ReadPin(Key3_Gpio)
//#define Key4     GPIO_ReadPin(Key4_Gpio)
//#define Key5     GPIO_ReadPin(Key5_Gpio)
//#define Key6     GPIO_ReadPin(Key6_Gpio)

#define No_Push -1
#define Key(n)      n



void Init_Gpio_Key(void)
{
    GPIO_SetupPinMux(Key1_Gpio, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(Key1_Gpio, GPIO_INPUT, GPIO_PULLUP|GPIO_QUAL6);//Right

    GPIO_SetupPinMux(Key2_Gpio, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(Key2_Gpio, GPIO_INPUT, GPIO_PULLUP|GPIO_QUAL6);//OK

//    GPIO_SetupPinMux(Key3_Gpio, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(Key3_Gpio, GPIO_INPUT, GPIO_PULLUP|GPIO_QUAL6);//Down

//    GPIO_SetupPinMux(Key4_Gpio, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(Key4_Gpio, GPIO_INPUT, GPIO_PULLUP|GPIO_QUAL6);//Up

//    GPIO_SetupPinMux(Key5_Gpio, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(Key5_Gpio, GPIO_INPUT, GPIO_PULLUP|GPIO_QUAL6);//Left

//    GPIO_SetupPinMux(Key6_Gpio, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(Key6_Gpio, GPIO_INPUT, GPIO_PULLUP|GPIO_QUAL6);//Cancel


}


int Dep_Key_Scan(void)
{
char num=No_Push;

if(!Key1|!Key2)
{
    DSPTimeDly(5);//按键消抖
    if(!Key1)     num=Key(1);
    if(!Key2)     num=Key(2);
//   if(!Key3)     num=Key(3);
//   if(!Key4)     num=Key(4);
//   if(!Key5)     num=Key(5);
//   if(!Key6)     num=Key(6);

}

return num;
}



int Key_value_last=-1;
int Key_value_now =-1;


//按一次开机键
void Key_On_Press(void)
{
//    if(!InvFault)
//    {
        DSPTimeDly(500);
                InvState++;

//     }
//    else
//        {
//            InvState = 0;
//        }
  }

//按一次关机键
void Key_Off_Press(void)
{
    InvState = 0;
    DSPTimeDly(200);

     DisablePWM();
        InvWorkMode=Ivn_Closed;

}


void Task_Key_Scan(void)
{
        Key_value_now= Dep_Key_Scan();
        if(Key_value_now!=Key_value_last)//检测到按键按下了
        {
            if(Key_value_now>0)//按键按下事件
                {
                    if(Key_value_now==Key(1))//按键1(开机)
                    {

                    }
                    if(Key_value_now==Key(2))//按键2(关机)
                    {

                    }
                }
            if(Key_value_now<0)//按键弹起事件
                {
                     if(Key_value_last==Key(1))//按键1(开机)
                     {
                         Key_On_Press();
                     }
                     if(Key_value_last==Key(2))//按键2(关机)
                     {
                         Key_Off_Press();
                     }

                }
            Key_value_last=Key_value_now;
        }

}
