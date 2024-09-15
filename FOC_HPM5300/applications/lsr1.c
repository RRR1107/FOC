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
#include "Dsp_Cfg.h"
#include "include.h"
#include "math.h"
#include "zj.h"
#include "EQep.h"
real32_T SpeedRps;
real32_T RotorTheta;
uint16_T State;
PI theta_sogi1=PI_Default;
PI Id_PICtl=PI_Default;
PI Iq_PICtl=PI_Default;
PI Qepd_PICtl=PI_Default;
PI Qepq_PICtl=PI_Default;
#pragma CODE_SECTION(MainIsr, "ramfuncs");
#pragma CODE_SECTION(GridCurr_Limit, "ramfuncs");
void GridCurr_Limit(float Curr_Limit_Val); //参数：过流阈值
//*******************************************************************************
//      变量定义
//*******************************************************************************
float theta_r0,CNT_r0;
//矢量控制用
Fliter_PLL PLL2 = Fliter_PLL_Default;
SVGENDQ svpwm1 = SVGENDQ_DEFAULTS;
float Isr_Time = 0.0f;
//float theta_sogi = 3.14;
//INT32 Sys_time=0;
volatile Three_Phase_Info GridVolt=Three_Phase_Info_Default;
volatile Three_Phase_Info GridCurr=Three_Phase_Info_Default;
volatile Three_Phase_Info GridCurrf=Three_Phase_Info_Default;
volatile Three_Phase_Info Inv_Volt=Three_Phase_Info_Default;
volatile DSOGI Ea=DSOGI_Default;
volatile DSOGI Eb=DSOGI_Default;
volatile float Sin_theta_1,Cos_theta_1;
volatile float Sin_theta_m,Cos_theta_m;
volatile float Sin_theta_r,Cos_theta_r;

extern Uint16 InvState,InvFault;
//float Isr_Time = 0.0f;

#define GRID_FREQ 50.0f
#define ISR_FREQUENCY 20000.0f
#define Pi 3.1415926f

volatile float GridCurrA,GridCurrB,GridCurrC,
       GridVoltAB,GridVoltBC,GridVoltCA,
       GridVoltA,GridVoltB,GridVoltC,
       CapVoltA,CapVoltB,CapVoltC,w_,flux_r_alpha,flux_r_beta,SpeedA,theta_sogi,SpeedCtl,
       BusVoltP,BusVoltN,BusVoltPN,BusCurr,BusCurr2,BusCurr3;
int t11;
float BusVoltF=0;
float BusVoltFIR=0;

void VectorControl_Init(void)
{
            Qepd_PICtl.OutMax=50.0f;
            Qepd_PICtl.OutMin=-50.0f;
            Qepq_PICtl.OutMax=30.0f;
            Qepq_PICtl.OutMin=-30.0f;

            Id_PICtl.OutMax=22.0f;//3.0f;
            Id_PICtl.OutMin=-22.0f;

            Iq_PICtl.OutMax=22.0f;
            Iq_PICtl.OutMin=-22.0f;


            Id_PICtl.Kp=0.00267f;//0.267f;//0.33896f;//0.1913f;//0.28184f;//0.19133f;//0.028184f;//33896f;//0.168948f;//0.26684112f;//0.9333333f;////0.1896f;
            Id_PICtl.Ki=0.00688f;//0.00988f;//0.0083f;//0.00325f;//46.7f;//0.000325f;//0.00494f;//0.0078f;//0.0126667;////0.08f;

            Iq_PICtl.Kp=0.00493f;//0.534f;//0.7208848f;//0.1959f;//0.28184f;//0.26684112f;//0.9333333f;//0.26684112f;
            Iq_PICtl.Ki=0.00688f;//0.00988f;//0.0083f;//0.00325f;//0.0078f;//0.0126667;//0.0078f;
            theta_sogi1.Kp=287.992f;
            theta_sogi1.Ki=394784.16f;
//            Qepd_PICtl.Kp=24.733f;//25.6889f;
//            Qepd_PICtl.Ki=0.000107f;//0.0000107f;//1.71259f;
            Qepq_PICtl.Kp=0.24733f;//0.08f;//0.152f;//0.42776f;//25.6889f;//0.24733f;//0.0024733f;
            Qepq_PICtl.Ki=0.0000107f;//0.00000855;//0.00077;//0.00556388f;//0.0006f;////0.000000507f;//0.000000107f;//0.0000107f;//1.71259f;


}

float Sin_theta,Cos_theta;
float Sin_theta1,Cos_theta1;
float start_time,stop_time;
int angle_4,angle;
//*******************************************************************************
//      中断
//*******************************************************************************
interrupt void MainIsr(void)
{//初始化电机参数
    qep.PolePairs = 5;
    qep.CalibratedAngle = 300;//1750//250//479;
    EQep_Calc(&qep);
//    T_start=EPwm1Regs.TBCTR;

//*******************************************************************************
//      采样
//*******************************************************************************

    while(AdcaRegs.ADCCTL1.bit.ADCBSY == 1);
    while(AdcbRegs.ADCCTL1.bit.ADCBSY == 1);
    while(AdccRegs.ADCCTL1.bit.ADCBSY == 1);
    while(AdcdRegs.ADCCTL1.bit.ADCBSY == 1);
    asm(" RPT #8 || NOP");


    #define GridCurr_Coeff 0.170897f//0.170897f//0.146484375f//0.12562900000f//0.5*0.0585937500000f  //3.0V*1000/4096/5.83
    #define GridCurrB_Coeff -0.170897f//-0.170897f//-0.146484375f//-0.12562900000f//0.5*0.0585937500000f  //3.0V*1000/4096/12.5
    #define GridVolt_Coeff -0.073608400f//0.0150146500000f//  3.00V*(10K +1.0M*2)/(20K*4096)

    #define BusVolt_Coeff  0.147949220f  // 3.00V*(20K +1.0M*2)/(10K*4096)
    #define BusCurr_Coeff  0.0854485f//0.170897f

    GridCurrA =((float)AdccResultRegs.ADCRESULT2-CurrOffsetA)*GridCurr_Coeff;
    GridCurrB =((float)AdcaResultRegs.ADCRESULT4-CurrOffsetB)*GridCurrB_Coeff;
    GridCurrC =((float)AdccResultRegs.ADCRESULT4-CurrOffsetC)*GridCurr_Coeff;
    BusCurr=(float)(AdcaResultRegs.ADCRESULT3)*BusCurr_Coeff;

    GridVoltCA =((float)AdcaResultRegs.ADCRESULT0-VoltOffsetC)*GridVolt_Coeff;
    GridVoltBC =((float)AdcaResultRegs.ADCRESULT1-VoltOffsetB)*GridVolt_Coeff;
    GridVoltAB =((float)AdcaResultRegs.ADCRESULT2-VoltOffsetA)*GridVolt_Coeff;

    //GridVoltAB = GridVoltA-GridVoltB;
    //GridVoltBC = GridVoltB-GridVoltC;
    //GridVoltCA = GridVoltC-GridVoltA;

    BusVoltN=(float)(AdcbResultRegs.ADCRESULT0)*BusVolt_Coeff;
    BusVoltP=(float)(AdcbResultRegs.ADCRESULT1)*BusVolt_Coeff;
  //  BusCurr1=(float)(AdcbResultRegs.ADCRESULT2)*BusVolt_Coeff;
   // BusCurr2 =(float)(AdcabResultRegs.ADCRESULT3)*BusVolt_Coeff;
   // BusCurr=(float)(AdcaResultRegs.ADCRESULT3)*BusCurr_Coeff;



    //*******************************************************************************
    //     过流保护
    //*******************************************************************************
    GridCurr_Limit(130.f);  //20A
    BusVoltPN=BusVoltP+BusVoltN;




    RotorTheta =  qep.ElecTheta;
    SpeedRps = qep.Speed_n*(float)qep.DirectionQep;
    State = InvState;

    Cos_theta_m = __cos((float)qep.ElecTheta);//
    Sin_theta_m =__sin((float)qep.ElecTheta);//

   //*******************************************************************************
   //      I_Grid:abc->alpha,beta->dq0
   //*******************************************************************************
            GridCurr.a=GridCurrA;
            GridCurr.b=GridCurrB;
            GridCurr.c=GridCurrC;
            GridCurr.cos=__cos(theta_sogi);//Cos_theta_m;//__cos(theta_sogi);//
            GridCurr.sin=__sin(theta_sogi);//Sin_theta_m;//__sin(theta_sogi);//Sin_theta_m;//

            CLARKE(GridCurr);
            PARK(GridCurr);
    //*******************************************************************************
    //      I_DIFG:abc->alpha,beta->dq0
    //*******************************************************************************
            GridVolt.ab=GridVoltAB;
            GridVolt.bc=GridVoltBC;
            GridVolt.ca=GridVoltCA;
            GridVolt.cos=__cos(theta_sogi);//Cos_theta_m;//__cos(theta_sogi);//__cos(theta_sogi);//Cos_theta_m;//__cos(theta_sogi);//Cos_theta_m;//__cos(theta_sogi);//;//__cos(theta_sogi);//
            GridVolt.sin=__sin(theta_sogi);//Sin_theta_m;//__sin(theta_sogi);//__sin(theta_sogi);//Sin_theta_m;//__sin(theta_sogi);//Sin_theta_m;//__sin(theta_sogi);////;//__sin(theta_sogi);//

            L2P( GridVolt);
            CLARKE( GridVolt);
            PARK( GridVolt);
   //*******************************************************************************
            //  DSOGI无位置
   //*******************************************************************************
//            Ea.vin=sqrt(GridVolt.alpha*GridVolt.alpha+GridVolt.beta*GridVolt.beta)*(1-0.006995);//0.01255;
//
//            Ea.w=0.04+Ea.w+Ea.ef*(-46)/(Ea.u*Ea.u+Ea.q_u*Ea.q_u)*Ea.w*Ea.w*Dsp_Ts;
//            Eb.w=Ea.w;



            Ea.vin=GridVolt.alpha-GridCurr.alpha*0.025;//0.019;
            Eb.vin=GridVolt.beta-GridCurr.beta*0.025;//0.019;

//            Ea.w=0.04+Ea.w+Ea.ef*(-0.146)/(Ea.u*Ea.u+Ea.q_u*Ea.q_u)*Ea.w*Ea.w*Dsp_Ts;
            Ea.w=0.004+Ea.w+Ea.ef*(-46)/(Ea.u*Ea.u+Ea.q_u*Ea.q_u)*Ea.w*Dsp_Ts;
//           w_=Ea.w*1.414/(0.5*(Ea.u*Ea.u+Ea.q_u*Ea.q_u)+0.5*(Eb.u*Eb.u+Eb.q_u*Eb.q_u));
//           Ea.w=0.04+Ea.w+(Ea.ef+Eb.ef)*(-46)*w_*Dsp_Ts;
            Eb.w=Ea.w;
//            w_=Ea.ef*(-46)*Ea.w*Ea.w/(Ea.u*Ea.u+Ea.q_u*Ea.q_u);
//            Ea.w=0.0001+Ea.w+w_*Dsp_Ts;
//          Eb.w=0.04+Eb.w+Eb.ef*(-46)/(Eb.u*Eb.u+Eb.q_u*Eb.q_u)*Dsp_Ts;
 //           Eb.w=Ea.w;
            DSOGI(Ea);
            DSOGI(Eb);
              flux_r_alpha=Ea.q_u-GridCurr.alpha*0.000070;
              flux_r_beta=Eb.q_u-GridCurr.beta*0.000125;

            theta_sogi1.Ref=__cos(theta_sogi)*flux_r_beta/(flux_r_alpha*flux_r_alpha+flux_r_beta*flux_r_beta) ; //
            theta_sogi1.Fdb=__sin(theta_sogi)*flux_r_alpha/(flux_r_alpha*flux_r_alpha+flux_r_beta*flux_r_beta);
            pi_calc(&theta_sogi1);
            theta_sogi=atan2(flux_r_beta,flux_r_alpha);//theta_sogi+theta_sogi1.Out*Dsp_Ts;
            SpeedA=Eb.w*1.909853f;//9.549267484/qep.PolePairs;//*qep.PolePairsf;//w*30/pi/p
#define pi 3.1415926f
       while(theta_sogi > 2*pi) theta_sogi -=2*pi;
       while(theta_sogi < 0   ) theta_sogi +=2*pi;
           //*******************************************************************************
 //*******************************************************************************
 //      开环，恒压频比 2Hz     0.02 //ZX
 //*******************************************************************************
if(InvState==1)
 {

        angle++;if(angle>=5000) angle=0;
        Sin_theta_r= __sin((float)((float)angle*2.0f*Pi*2e-4f));
        Cos_theta_r = __cos((float)((float)angle*2.0f*Pi*2e-4f));
        Inv_Volt.d=1.2f;
        Inv_Volt.q=0.0f;
     //*******************************************************************************
     //      V_pos:dq0->alpha beta//ZX
     //*******************************************************************************
             Inv_Volt.sin=Sin_theta_r;
             Inv_Volt.cos=Cos_theta_r;
             i_PARK(Inv_Volt);
             InvWorkMode=Inv_Start_up1;
 EnablePWM();
}

 /***********************开机二次电流环'*****************************/
if(InvState==2)
 {

     Id_PICtl.Ref=0.0f;
     Id_PICtl.Fdb = GridCurr.d;
     pi_calc(&Id_PICtl);

     Iq_PICtl.Ref=2.5f;
     Iq_PICtl.Fdb = GridCurr.q;
     pi_calc(&Iq_PICtl);


     Inv_Volt.sin=__sin(theta_sogi);//Sin_theta_m;//__sin(theta_sogi);//
     Inv_Volt.cos=__cos(theta_sogi);//Cos_theta_m;//__cos(theta_sogi);//Cos_theta_m;

     Inv_Volt.d=Id_PICtl.Out;//
     Inv_Volt.q=Iq_PICtl.Out;//
     i_PARK(Inv_Volt);
 //EnablePWM();
 InvWorkMode=Inv_Start_up2;
 }


 /***********************开机三次转速环*****************************/
if(InvState==3 )
 {

//*******************************************************************************

//    t11++;
//     if(t11==200&&SpeedCtl<1000){
//     SpeedCtl++;
//     t11=0;}

     Qepq_PICtl.Ref=800*(2*pi/60);//SpeedCtl*(2*pi/60);////3000*(2*pi/60);//
     Qepq_PICtl.Fdb = SpeedA*(2*pi/60);
     pi_calc(&Qepq_PICtl);

     Iq_PICtl.Ref=Qepq_PICtl.Out;
     Iq_PICtl.Fdb = GridCurr.q;
     pi_calc(&Iq_PICtl);

     Id_PICtl.Ref=0;//2.5-SpeedA*0.05*qep.PolePairs/60;//3-SpeedA*0.06*qep.PolePairs/60;////17-sqrt(0.00002025+0.000000005625*GridCurr.q*GridCurr.q)/0.00015;//0.0045/0.00015   0.0f;//
     Id_PICtl.Fdb = GridCurr.d;
     pi_calc(&Id_PICtl);

     Iq_PICtl.Ref=Qepq_PICtl.Out;
     Iq_PICtl.Fdb = GridCurr.q;
     pi_calc(&Iq_PICtl);

     Inv_Volt.sin=__sin(theta_sogi);//Sin_theta_m;//
     Inv_Volt.cos=__cos(theta_sogi);//Cos_theta_m;//

     Inv_Volt.d=Id_PICtl.Out;//
     Inv_Volt.q=Iq_PICtl.Out;//
     i_PARK(Inv_Volt);
 EnablePWM();
 InvWorkMode=Inv_Start_up3;
 }

if(qep.Index)
{
    //坐标变换锁相
//    theta_r0 = atan2(DFIGVolt.beta,DFIGVolt.alpha)*0.33333333f;//除极对数（8）
//    CNT_r0 = theta_r0*5215.1892f;


    theta_r0 = (atan2(-GridVolt.d,GridVolt.q)) *198.943f;//1250*4/8pi



}

 //*******************************************************************************
   //      SVPWM cal
   //*******************************************************************************
         svpwm1.Alfa = Inv_Volt.alpha;
         svpwm1.Beta = Inv_Volt.beta;
         svpwm1.Tpwm = (float)(EPwm7Regs.TBPRD*2);    //
         svpwm1.Vdc = BusVoltPN;


    svgendq_calc(&svpwm1);

    EPwm5Regs.CMPA.bit.CMPA = svpwm1.Ta; //A相；
    EPwm11Regs.CMPA.bit.CMPA = svpwm1.Tb; //B相；
    EPwm10Regs.CMPA.bit.CMPA = svpwm1.Tc; //C相；



//*******************************************************************************
//   计算数据
//*******************************************************************************

     extern Average rms1;

     rms1.add[0] ++;
     rms1.add[1] += GridVolt.d*GridVolt.d + GridVolt.q*GridVolt.q;
     rms1.add[2] += GridCurr.d*GridCurr.d + GridCurr.q*GridCurr.q;
     rms1.add[3] += BusVoltPN*BusVoltPN;
     rms1.add[4] += qep.Speed_n*qep.Speed_n;

//*******************************************************************************
//   数据上传及记录
//*******************************************************************************
extern void NetUploadData_v2(void);
NetUploadData_v2();


//     stop_time = 0.0f;
//     STOP_TIMER(stop_time);
//     Isr_Time = (start_time - stop_time);//中断时间  单位：*5ns

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//数据上传 实验室上位机用
#pragma CODE_SECTION(UploadData_v2, "ramfuncs");
Uint16 UploadData_v2 (volatile int16 *add)
{
#define Curr_Coeff 100.0f   //最大显示电流65A
#define Volt_Coeff 10.0f    //最大显示电压850V
#define PLL_Coeff 10000.0f
#define Speed_Coeff 10.0f

        //Data 1 - 5
    *add = (int16)(GridVoltAB*Curr_Coeff);
    *add = (int16)(GridVoltBC*Curr_Coeff);
    *add = (int16)(GridVoltCA*Curr_Coeff);
    *add = (int16)(theta_sogi*Volt_Coeff);
    *add = (int16)(Eb.q_u*Curr_Coeff);
        //Data 6 - 10
    *add = (int16)(Ea.w);
    *add = (int16)(Ea.error*Curr_Coeff);
    *add = (int16)(Ea.osg*Curr_Coeff);
    *add = (int16)(GridCurr.d);//(qep.ElecTheta*Curr_Coeff);
    *add = (int16)(GridCurr.q);//(qep.MechTheta*Curr_Coeff);

        //Data 11 - 14
    *add = (int16)(GridCurrA*Volt_Coeff);
    *add = (int16)(GridCurrB*Volt_Coeff);
    *add = (int16)(GridCurrC*Volt_Coeff);
    *add = (int16)(SpeedA);
    //  *add = (int16)(ScopeCh[12]);
    //  *add = (int16)(ScopeCh[13]);
    //每秒钟上传一次
   extern int16 Sys_Info[10];


   Sys_Info[1]=InvState;//
   Sys_Info[2]=InvWorkMode;//GpioCtrlRegs.GPAMUX2.bit.GPIO25;//InvFault;//(int16)(Isr_Time*10);//0.1us
   Sys_Info[3]=(SpeedRps);
   Sys_Info[4]=(SpeedA);
   Sys_Info[5]=(BusVoltPN*Volt_Coeff);
   Sys_Info[6]=SpeedA;

        return 14;
    }
UINT16 UploadData (UINT16 pos)
{
    //要转化为INT16
//#define Curr_Coeff 500.0f   //最大显示电流65A HJK
#define Volt_Coeff 10.0f    //最大显示电压850V
#define  PLL_Coeff 10000.0f

    int i=0;

    DisplayBuf[pos++] = (int16)(GridCurr.d*Curr_Coeff);
    DisplayBuf[pos++] = (int16)(GridVolt.d*Volt_Coeff);
    DisplayBuf[pos++] = (int16)(GridCurr.q*Curr_Coeff);
    DisplayBuf[pos++] =  (int16)(GridVolt.q*Volt_Coeff);
    DisplayBuf[pos++] = (int16)(Inv_Volt.d*Volt_Coeff);
    DisplayBuf[pos++] = (int16)(Inv_Volt.q*Volt_Coeff);

    //Data 6 - 10
    DisplayBuf[pos++] = (INT16)(i++);  //I_A
    DisplayBuf[pos++] = (INT16)(i++);  //I_A
    DisplayBuf[pos++] = (INT16)(i++);  //I_A
    DisplayBuf[pos++] = (INT16)(i++);  //I_A
    DisplayBuf[pos++] = (INT16)(i++);  //I_A
    //Data 11 - 15
    DisplayBuf[pos++] = (INT16)(i++);  //I_A
    DisplayBuf[pos++] = (INT16)(i++);  //I_A
    DisplayBuf[pos++] = (INT16)(i++);  //I_A
    DisplayBuf[pos++] = (INT16)(i++);  //I_A
    DisplayBuf[pos++] = (INT16)(i++);  //I_A
    //Data 16 - 20
    DisplayBuf[pos++] = (INT16)(Cos_theta*PLL_Coeff);
    DisplayBuf[pos++] = (INT16)(Sin_theta*PLL_Coeff);
    DisplayBuf[pos++] = (INT16)(InvFault);
    DisplayBuf[pos++] = (INT16)(Isr_Time);//转为秒  *0.0000
    DisplayBuf[pos++] = (INT16)(i++);  //I_A


//R(CLA)
    DisplayBuf[pos++] = (INT16)(i++);  //I_A
    DisplayBuf[pos++] = (INT16)(i++);  //I_A
    DisplayBuf[pos++] = (INT16)(i++);  //I_A
    DisplayBuf[pos++] = (INT16)(i++);  //I_A
    DisplayBuf[pos++] = (INT16)(i++);  //I_A
    DisplayBuf[pos++] = (INT16)(i++);  //I_A

    return pos;
}


#define FAULT_OVER_CUR_A    0x0001  ///<A相过流故障
#define FAULT_OVER_CUR_B    0x0002  ///<B相过流故障
#define FAULT_OVER_CUR_C    0x0004  ///<C相过流故障
#define FAULT_TZ_B_FAULT    0x0008  ///<未用

#define FAULT_TZ_INV_FAULT  0x0010  ///<未用
#define FAULT_TZ_OC_A       0x0020  ///<A相限流
#define FAULT_TZ_OC_B       0x0040  ///<B相限流
#define FAULT_TZ_OC_C       0x0080  ///<C相限流

#define FAULT_OVER_VOL_P    0x0100  ///<P过压故障
#define FAULT_OVER_VOL_N    0x0200  ///<N过压故障
#define FAULT_LOW_VOL       0x0400  ///<欠压故障
#define FAULT_ERR_VOL       0x0800  ///<电压偏差过大


#define FAULT_IGBT_A        0x1000  ///<A相IGBT_Fault
#define FAULT_IGBT_B        0x2000  ///<A相IGBT_Fault
#define FAULT_IGBT_C        0x4000  ///<A相IGBT_Fault
#define FAULT_CPLD_ERR      0x8000  ///<CPLD保护
//*******************************************************************************
//   过流保护
//*******************************************************************************

float I_A,I_B,I_C;
void GridCurr_Limit(float Curr_Limit_Val)//参数：过流阈值
{
     I_A = fabsf(GridCurrA);
     I_B = fabsf(GridCurrB);
     I_C = fabsf(GridCurrC);

    if((I_A>Curr_Limit_Val)||(I_B>Curr_Limit_Val)||(I_C>Curr_Limit_Val))
    {
        DisablePWM();
        InvState=0;


        if((I_A >= Curr_Limit_Val))//|| (Temp_CurrentA_Pre>=GRID_CUR_PRE_UPPER_LMT))
            InvFault |= FAULT_OVER_CUR_A; //过流保护标志

        if((I_B >= Curr_Limit_Val))// || (Temp_CurrentB_Pre>=GRID_CUR_PRE_UPPER_LMT))
            InvFault |= FAULT_OVER_CUR_B; //过流保护标志

        if((I_C >= Curr_Limit_Val))// || (Temp_CurrentC_Pre>=GRID_CUR_PRE_UPPER_LMT))
            InvFault |= FAULT_OVER_CUR_C; //过流保护标志

        InvWorkMode=Inv_Fault;
    }

}
