/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 ��ƽ    ̨�������������ܿƼ�TC264DA���İ�
 ����    д��ZYF/chiusir
 ��E-mail  ��chiusir@163.com
 ������汾��V1.1 ��Ȩ���У���λʹ��������ϵ��Ȩ
 �������¡�2020��10��28��
 �������Ϣ�ο����е�ַ��
 ����    վ��http:// www.lqist.cn
 ���Ա����̡�http:// longqiu.taobao.com
 ------------------------------------------------
 ��dev.env.��AURIX Development Studio1.2.2�����ϰ汾
 ��Target �� TC264DA/TC264D
 ��Crystal�� 20.000Mhz
 ��SYS PLL�� 200MHz
 ________________________________________________________________
 ����iLLD_1_0_1_11_0�ײ����,

 ʹ�����̵�ʱ�򣬽������û�пո��Ӣ��·����
 ����CIFΪTC264DA�����⣬�����Ĵ������TC264D
 ����Ĭ�ϳ�ʼ����EMEM��512K������û�ʹ��TC264D��ע�͵�EMEM_InitConfig()��ʼ��������
 ������\Libraries\iLLD\TC26B\Tricore\Cpu\CStart\IfxCpu_CStart0.c��164�����ҡ�
 ________________________________________________________________

 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ
 *  ��    ע��TC264 ������CCU6ģ��  ÿ��ģ��������������ʱ��  ������ʱ���ж�
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "LQ_CCU6.h"

#include "../APP/LQ_GPIO_LED.h"
#include "LQ_ADC.h"
#include "LQ_GPT12_ENC.h"
#include "LQ_ImageProcess.h"
#include "LQ_MotorServo.h"
#include "LQ_PID.h"
#include <CompilerTasking.h>

volatile sint16 ECPULSE1 = 0;  // �ٶ�ȫ�ֱ���
volatile sint16 ECPULSE2 = 0;  // �ٶ�ȫ�ֱ���
volatile sint32 RAllPulse = 0; // �ٶ�ȫ�ֱ���

volatile sint16 Target_Speed1 = 25; // �ٶ�ȫ�ֱ���
volatile sint16 Target_Speed2 = 25; // �ٶ�ȫ�ֱ���
unsigned char Power_On = 0;         //����־λ    0�����  1��磬
unsigned char Power_Off = 0;        //����־λ    0�����  1���
unsigned char motor_flag = 0;       //�����ͣ��־λ
extern unsigned short val0, val1, val2,
    val3; //��Ȧ��Ϣ1 ��Ȧ��Ϣ2   �����Ϣ    �����Ϣ
unsigned int sum = 0; //�������ۼ�ֵ
sint16 pw_err = 0;    //λ�����
unsigned char Pw_flag = 0;

IFX_INTERRUPT(CCU60_CH0_IRQHandler, CCU60_VECTABNUM, CCU60_CH0_PRIORITY);
IFX_INTERRUPT(CCU60_CH1_IRQHandler, CCU60_VECTABNUM, CCU60_CH1_PRIORITY);
IFX_INTERRUPT(CCU61_CH0_IRQHandler, CCU61_VECTABNUM, CCU61_CH0_PRIORITY);
IFX_INTERRUPT(CCU61_CH1_IRQHandler, CCU61_VECTABNUM, CCU61_CH1_PRIORITY);

/** CCU6�ж�CPU��� */
const uint8 Ccu6IrqVectabNum[2] = {CCU60_VECTABNUM, CCU61_VECTABNUM};

/** CCU6�ж����ȼ� */
const uint8 Ccu6IrqPriority[4] = {CCU60_CH0_PRIORITY, CCU60_CH1_PRIORITY,
                                  CCU61_CH0_PRIORITY, CCU61_CH1_PRIORITY};

/** CCU6�жϷ�������ַ */
const void *Ccu6IrqFuncPointer[4] = {
    &CCU60_CH0_IRQHandler, &CCU60_CH1_IRQHandler, &CCU61_CH0_IRQHandler,
    &CCU61_CH1_IRQHandler};

extern pid_param_t LSpeed_PID;
extern pid_param_t RSpeed_PID;
extern sint16 MotorDuty1;
extern sint16 MotorDuty2;
/***********************************************************************************************/
/********************************CCU6�ⲿ�ж�
 * ������******************************************/
/***********************************************************************************************/

/*************************************************************************
 *  �������ƣ�void CCU60_CH0_IRQHandler(void)
 *  ����˵����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��30��
 *  ��    ע��CCU60_CH0ʹ�õ��жϷ�����
 *************************************************************************/
void CCU60_CH0_IRQHandler(void) {
    /* ����CPU�ж�  �����жϲ���Ƕ�� */
    //   IfxCpu_enableInterrupts();
    // ����жϱ�־
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU60,
                                     IfxCcu6_InterruptSource_t12PeriodMatch);

    /* �û����� */
    //Ԫ����Ϣ����
    CameraCar();
}

/*************************************************************************
 *  �������ƣ�void CCU60_CH1_IRQHandler(void)
 *  ����˵����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��30��
 *  ��    ע��CCU60_CH1ʹ�õ��жϷ�����
 *************************************************************************/
void CCU60_CH1_IRQHandler(void) {
    /* ����CPU�ж�  �����жϲ���Ƕ�� */
    IfxCpu_enableInterrupts();

    // ����жϱ�־
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU60,
                                     IfxCcu6_InterruptSource_t13PeriodMatch);

    /* �û����� */
    /* ��ȡ������ֵ */

    ECPULSE1 = -ENC_GetCounter(
        ENC4_InPut_P02_8); // ���� ĸ���ϱ�����1��С��ǰ��Ϊ��ֵ
    ECPULSE2 = ENC_GetCounter(
        ENC6_InPut_P20_3); // �ҵ�� ĸ���ϱ�����2��С��ǰ��Ϊ��ֵ

    /********************��ⷢ����Ȧλ�ò�У׼********************************/
    //����
    if ((val0 > 2000) && (val1 > 1800) && (Power_On == 0) && (Power_Off == 0)) {
        Power_On = 1;
        motor_flag = 0;
    }
    //������ִ��
    if ((Power_On == 1) && (Pw_flag == 0)) {
        pw_err = (val0 - val1) / 20;
        if ((val3 > 900) || (val0 > 2250)) {
            Pw_flag = 1;
        }
    }
    if (Pw_flag == 1) {
        pw_err = (val0 - val1) / 20;
        if (val2 > 1500) //������
        {
            Power_Off = 1;
            Power_On = 0;
        }
    }
    if (Power_Off == 1) {
        motor_flag = 0;
        sum += ECPULSE1;
        if (sum > 15000) {
            Power_Off = 0;
            sum = 0;
            Pw_flag = 0;
        }
    }

    /***************************����ջ��Ͳ���********************************/
    if ((Power_On == 0) && (motor_flag == 0)) {
        //���ٴ������ٱ����Լ��޸�
        if (ServoDuty > 0) {
            MotorDuty1 = (int)PidIncCtrl( &LSpeed_PID,
                (float)(Target_Speed1 - ECPULSE1 - ServoDuty / 10)); //-ServoDuty
            MotorDuty2 = (int)PidIncCtrl(
                &RSpeed_PID, (float)(Target_Speed2 - ECPULSE2 + ServoDuty / 10)); // ���߲�+servoduty
        } else {
            MotorDuty1 = (int)PidIncCtrl(
                &LSpeed_PID, (float)(Target_Speed1 - ECPULSE1 - ServoDuty / 10)); // ���߲�-servoduty
            MotorDuty2 = (int)PidIncCtrl(
                &RSpeed_PID,
                (float)(Target_Speed2 - ECPULSE2 + ServoDuty / 10)); //+ServoDuty
        }
    } else {
        MotorDuty1 = (int)PidIncCtrl(
            &LSpeed_PID, (float)(0 - ECPULSE1 + pw_err)); //-ServoDuty
        MotorDuty2 =
            (int)PidIncCtrl(&RSpeed_PID, (float)(0 - ECPULSE2 + pw_err));
    }
    const int max_pwm = 3000;
    //����޷�
    if (MotorDuty1 > max_pwm)
        MotorDuty1 = max_pwm;
    else if (MotorDuty1 < -max_pwm)
        MotorDuty1 = -max_pwm;
    if (LSpeed_PID.out > max_pwm)
        LSpeed_PID.out = max_pwm;
    else if (LSpeed_PID.out < -max_pwm)
        LSpeed_PID.out = -max_pwm;

    if (MotorDuty2 > max_pwm)
        MotorDuty2 = max_pwm;
    else if (MotorDuty2 < -max_pwm)
        MotorDuty2 = -max_pwm;
    if (RSpeed_PID.out > max_pwm)
        RSpeed_PID.out = max_pwm;
    else if (RSpeed_PID.out < -max_pwm)
        RSpeed_PID.out = -max_pwm;

    MotorCtrl(MotorDuty1, MotorDuty2);
}

/*************************************************************************
 *  �������ƣ�void CCU61_CH0_IRQHandler(void)
 *  ����˵����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��30��
 *  ��    ע��CCU61_CH0ʹ�õ��жϷ�����
 *************************************************************************/
void CCU61_CH0_IRQHandler(void) {
    /* ����CPU�ж�  �����жϲ���Ƕ�� */
    IfxCpu_enableInterrupts();

    // ����жϱ�־
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61,
                                     IfxCcu6_InterruptSource_t12PeriodMatch);

    /* ADC�ɼ� */
    val1 = ADC_Read(ADC0);
    val0 = ADC_Read(ADC1);
    val2 = ADC_Read(ADC2);
    val3 = ADC_Read(ADC3);
}
/*************************************************************************
 *  �������ƣ�void CCU61_CH1_IRQHandler(void)
 *  ����˵����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��30��
 *  ��    ע��CCU61_CH1ʹ�õ��жϷ�����
 *************************************************************************/
void CCU61_CH1_IRQHandler(void) {
    /* ����CPU�ж�  �����жϲ���Ƕ�� */
    IfxCpu_enableInterrupts();

    // ����жϱ�־
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61,
                                     IfxCcu6_InterruptSource_t13PeriodMatch);

    /* �û����� */
    LED_Ctrl(LED0, RVS); // ��ƽ��ת,LED��˸
}

/*************************************************************************
 *  �������ƣ�CCU6_InitConfig CCU6
 *  ����˵������ʱ�������жϳ�ʼ��
 *  ����˵����ccu6    �� ccu6ģ��            CCU60 �� CCU61
 *  ����˵����channel �� ccu6ģ��ͨ��  CCU6_Channel0 �� CCU6_Channel1
 *  ����˵����us      �� ccu6ģ��  �ж�����ʱ��  ��λus
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��30��
 *  ��    ע��    CCU6_InitConfig(CCU60, CCU6_Channel0, 100);  //
 *100us����һ���ж�
 *************************************************************************/
void CCU6_InitConfig(CCU6_t ccu6, CCU6_Channel_t channel, uint32 us) {
    IfxCcu6_Timer_Config timerConfig;

    Ifx_CCU6 *module = IfxCcu6_getAddress((IfxCcu6_Index)ccu6);

    uint8 Index = ccu6 * 2 + channel;

    uint32 period = 0;

    uint64 clk = 0;

    /* �ر��ж� */
    boolean interrupt_state = disableInterrupts();

    IfxCcu6_Timer_initModuleConfig(&timerConfig, module);

    clk = IfxScuCcu_getSpbFrequency();

    /* ����ʱ��Ƶ��  */
    uint8 i = 0;
    while (i++ < 16) {
        period = (uint32)(clk * us / 1000000);
        if (period < 0xffff) {
            break;
        } else {
            clk = clk / 2;
        }
    }
    switch (channel) {
    case CCU6_Channel0:
        timerConfig.timer = IfxCcu6_TimerId_t12;
        timerConfig.interrupt1.source = IfxCcu6_InterruptSource_t12PeriodMatch;
        timerConfig.interrupt1.serviceRequest = IfxCcu6_ServiceRequest_1;
        timerConfig.base.t12Frequency = (float)clk;
        timerConfig.base.t12Period = period; // ���ö�ʱ�ж�
        timerConfig.clock.t12countingInputMode =
            IfxCcu6_CountingInputMode_internal;
        timerConfig.timer12.counterValue = 0;
        timerConfig.interrupt1.typeOfService = Ccu6IrqVectabNum[ccu6];
        timerConfig.interrupt1.priority = Ccu6IrqPriority[Index];
        break;

    case CCU6_Channel1:
        timerConfig.timer = IfxCcu6_TimerId_t13;
        timerConfig.interrupt2.source = IfxCcu6_InterruptSource_t13PeriodMatch;
        timerConfig.interrupt2.serviceRequest = IfxCcu6_ServiceRequest_2;
        timerConfig.base.t13Frequency = (float)clk;
        timerConfig.base.t13Period = period;
        timerConfig.clock.t13countingInputMode =
            IfxCcu6_CountingInputMode_internal;
        timerConfig.timer13.counterValue = 0;
        timerConfig.interrupt2.typeOfService = Ccu6IrqVectabNum[ccu6];
        timerConfig.interrupt2.priority = Ccu6IrqPriority[Index];
        break;
    }

    timerConfig.trigger.t13InSyncWithT12 = FALSE;

    IfxCcu6_Timer Ccu6Timer;

    IfxCcu6_Timer_initModule(&Ccu6Timer, &timerConfig);

    IfxCpu_Irq_installInterruptHandler(
        (void *)Ccu6IrqFuncPointer[Index],
        Ccu6IrqPriority[Index]); // �����жϺ������жϺ�

    restoreInterrupts(interrupt_state);

    IfxCcu6_Timer_start(&Ccu6Timer);
}

/*************************************************************************
 *  �������ƣ�CCU6_DisableInterrupt(CCU6_t ccu6, CCU6_Channel_t channel)
 *  ����˵����ֹͣCCU6ͨ���ж�
 *  ����˵����ccu6    �� ccu6ģ��            CCU60 �� CCU61
 *  ����˵����channel �� ccu6ģ��ͨ��  CCU6_Channel0 �� CCU6_Channel1
 *  �������أ���
 *  �޸�ʱ�䣺2020��5��6��
 *  ��    ע��
 *************************************************************************/
void CCU6_DisableInterrupt(CCU6_t ccu6, CCU6_Channel_t channel) {
    Ifx_CCU6 *module = IfxCcu6_getAddress((IfxCcu6_Index)ccu6);
    IfxCcu6_clearInterruptStatusFlag(
        module, (IfxCcu6_InterruptSource)(7 + channel * 2));
    IfxCcu6_disableInterrupt(module,
                             (IfxCcu6_InterruptSource)(7 + channel * 2));
}

/*************************************************************************
 *  �������ƣ�CCU6_DisableInterrupt(CCU6_t ccu6, CCU6_Channel_t channel)
 *  ����˵����ʹ��CCU6ͨ���ж�
 *  ����˵����ccu6    �� ccu6ģ��            CCU60 �� CCU61
 *  ����˵����channel �� ccu6ģ��ͨ��  CCU6_Channel0 �� CCU6_Channel1
 *  �������أ���
 *  �޸�ʱ�䣺2020��5��6��
 *  ��    ע��
 *************************************************************************/
void CCU6_EnableInterrupt(CCU6_t ccu6, CCU6_Channel_t channel) {
    Ifx_CCU6 *module = IfxCcu6_getAddress((IfxCcu6_Index)ccu6);
    IfxCcu6_clearInterruptStatusFlag(
        module, (IfxCcu6_InterruptSource)(7 + channel * 2));
    IfxCcu6_enableInterrupt(module, (IfxCcu6_InterruptSource)(7 + channel * 2));
}
