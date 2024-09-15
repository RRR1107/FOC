/*
 * Copyright (c) 2021 hpmicro
 *
 * Change Logs:
 * Date         Author          Notes
 * 2021-08-13   Fan YANG        first version
 *
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "rtt_board.h"
#include "include.h"

void thread_entry(void *arg);



int main(void)
{

    app_init_led_pins();

    static uint32_t led_thread_arg = 0;
    rt_thread_t led_thread = rt_thread_create("led_th", thread_entry, &led_thread_arg, 1024, 1, 10);
    rt_thread_startup(led_thread);

    return 0;
}


void thread_entry(void *arg)
{
    while(1){
        app_led_write(0, APP_LED_ON);
        rt_thread_mdelay(500);
        app_led_write(0, APP_LED_OFF);
        rt_thread_mdelay(500);
        app_led_write(1, APP_LED_ON);
        rt_thread_mdelay(500);
        app_led_write(1, APP_LED_OFF);
        rt_thread_mdelay(500);
        app_led_write(2, APP_LED_ON);
        rt_thread_mdelay(500);
        app_led_write(2, APP_LED_OFF);
        rt_thread_mdelay(500);
    }
}

Uint16 InvFault=0,InvState=0;
volatile enum INV_WORK_MODE InvWorkMode=Ivn_Closed;        //工作模式

void main()
{
                                                                                                                                                                                                                                                                                                InitSysCtrl();
#ifdef _FLASH
    //IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);RLY
#endif

    Dsp_Init();

//初始化任务控制定时器
    InitCpuTimers();

    ConfigCpuTimer(&CpuTimer0, 200, 20000);//20ms
    CpuTimer0Regs.TCR.all = 0x4001;

//    ConfigCpuTimer(&CpuTimer1, 200, 50000);//50ms
//    CpuTimer1Regs.TCR.all = 0x4001;

    ConfigCpuTimer(&CpuTimer2, 200, 500000);//500ms


    CpuTimer2Regs.TCR.all = 0x4001;

    EINT;
    ERTM;

//扫描任务
    for(;;)
    {
        //10ms 任务
        if(CpuTimer0Regs.TCR.bit.TIF == 1){
            CpuTimer0Regs.TCR.bit.TIF = 1;

            extern void Task_Key_Scan(void);
            Task_Key_Scan();

            void Task_Ethernet(void);
            Task_Ethernet();

              }

        //50ms 任务
        if(CpuTimer1Regs.TCR.bit.TIF == 1){
           CpuTimer1Regs.TCR.bit.TIF = 1;

            void Task_Ethernet(void);
            Task_Ethernet();
        }

        //500ms 任务
        if(CpuTimer2Regs.TCR.bit.TIF == 1){
            CpuTimer2Regs.TCR.bit.TIF = 1;

            void Task_Led(void);
            Task_Led();
        }
    }



}
Average rms1;

void Task_Led(void )
{
    LED3_TOGGLE();  //CPU1
    if(InvFault)  LED3_On();  //CPU1

    LED4_TOGGLE();  //CPU1
    if(InvWorkMode)  LED4_On();

}

