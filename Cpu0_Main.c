/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�TC264DA���İ�
����    д��chiusir
��E-mail��chiusir@163.com
������汾��V1.1 ��Ȩ���У���λʹ��������ϵ��Ȩ
�������¡�2021��6��16��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��dev.env.��AURIX Development Studio1.4�����ϰ汾
��Target �� TC264DA/TC264D
��Crystal�� 20.000Mhz
��SYS PLL�� 200MHz
________________________________________________________________
����iLLD_1_0_1_11_0�ײ����,

ʹ�����̵�ʱ�򣬽������û�пո��Ӣ��·����
����CIFΪTC264DA�����⣬�����Ĵ������TC264D
����Ĭ�ϳ�ʼ����EMEM��512K������û�ʹ��TC264D��ע�͵�EMEM_InitConfig()��ʼ��������
������\Libraries\iLLD\TC26B\Tricore\Cpu\CStart\IfxCpu_CStart0.c��164�����ҡ�
=================================================================
����������Ƶ��ַ��https://space.bilibili.com/95313236
=================================================================
����ͷ�ӿ�                  ��������ģ��
�� ���ݶ˿ڣ�P02.0-P02.7�ڣ���8λ��������ͷ�����ݶ˿ڣ�
�� SCCB�˿ڣ�P11_2  P11_3
�� ʱ�����أ��ⲿ�жϵ�0�飺P00_4��
�� ���źţ��ⲿ�жϵ�3�飺P15_1��
-----------------------------------------------------------------
�Ƽ�GPT12ģ�飬������ʵ��5·�����������������������ݴ�������������źŲɼ�������ѡ����·���ɣ�
P33_7, P33_6   ����TCĸ�������1 ����
P02_8, P33_5   ����TCĸ�������2 ����
P10_3, P10_1   ����TCĸ�������3
P20_3, P20_0   ����TCĸ�������4
-----------------------------------------------------------------
Ĭ�ϵ���ӿ�
ʹ��GTMģ�飬ATOM�ĸ�ͨ���ɲ���4*8��32·PWM�����Ҹ���Ƶ�ʺ�ռ�ձȿɵ����Ƽ�ʹ��ATOM0��0-7ͨ����
��һ��˫·ȫ������
P23_1         ����TCĸ��MOTOR1_P
P32_4         ����TCĸ��MOTOR1_N
P21_2         ����TCĸ��MOTOR2_P
P22_3         ����TCĸ��MOTOR2_N
�ڶ���˫·ȫ������
P21_4         ����TCĸ��MOTOR3_P
P21_3         ����TCĸ��MOTOR3_N
P20_8         ����TCĸ��MOTOR4_P
P21_5         ����TCĸ��MOTOR4_N
-----------------------------------------------------------------
Ĭ�϶���ӿ�
ʹ��GTMģ�飬ATOM�ĸ�ͨ���ɲ���4*8��32·PWM�����Ҹ���Ƶ�ʺ�ռ�ձȿɵ����Ƽ�ʹ��ATOM2��
P33_10        ����TCĸ����1
P33_13        ����TCĸ����2
-----------------------------------------------------------------
 Ĭ����Ļ��ʾ�ӿڣ��û�����ʹ��TFT����OLEDģ��
TFTSPI_CS     P20_14     ����TCĸ�� CS�ܽ� Ĭ�����ͣ����Բ���
TFTSPI_SCK    P20_11     ����TCĸ�� SPI SCK�ܽ�
TFTSPI_SDI    P20_10     ����TCĸ�� SPI MOSI�ܽ�
TFTSPI_DC     P20_12     ����TCĸ�� D/C�ܽ�
TFTSPI_RST    P20_13     ����TCĸ�� RESET�ܽ�
-----------------------------------------------------------------
OLED_CK       P20_14     ����TCĸ��OLED CK�ܽ�
OLED_DI       P20_11     ����TCĸ��OLED DI�ܽ�
OLED_RST      P20_10     ����TCĸ��OLED RST�ܽ�
OLED_DC       P20_12     ����TCĸ��OLED DC�ܽ�
OLED_CS       P20_13     ����TCĸ��OLED CS�ܽ� Ĭ�����ͣ����Բ���
----------------------------------------------------------------
Ĭ�ϰ����ӿ�
KEY0p         P22_0      ����TCĸ���ϰ���0
KEY1p         P22_1      ����TCĸ���ϰ���1
KEY2p         P22_2      ����TCĸ���ϰ���2
DSW0p         P33_9      ����TCĸ���ϲ��뿪��0
DSW1p         P33_11     ����TCĸ���ϲ��뿪��1
-----------------------------------------------------------------
Ĭ��LED�ӿ�
LED0p         P10_6      ����TCĸ����İ���LED0 ����
LED1p         P10_5      ����TCĸ����İ���LED1 ����
LED2p         P20_6      ����TCĸ����LED0
LED3p         P20_7      ����TCĸ����LED1
-----------------------------------------------------------------
Ĭ�Ϸ������ӿ�
BEEPp         P33_8      ����TCĸ���Ϸ������ӿ�
-----------------------------------------------------------------
��ʱ��
������CCU6ģ��  ÿ��ģ��������������ʱ��  ������ʱ���ж�
�Ƽ�ʹ��CCU6ģ�飬STM����ϵͳʱ�ӻ�����ʱ��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <IfxCpu.h>
#include <IfxScuCcu.h>
#include <IfxScuWdt.h>
#include <IfxStm.h>
#include <IfxStm_reg.h>
#include <stdio.h>
#include "LQ_TFT18.h"
#include "src/APP/LQ_ADC_test.h"
#include "src/APP/LQ_Atom_Motor.h"
#include "src/APP/LQ_CAMERA.h"
#include "src/APP/LQ_EEPROM_TEST.h"
#include "src/APP/LQ_FFT_TEST.h"
#include "src/APP/LQ_GPIO_ExInt.h"
#include "src/APP/LQ_GPIO_KEY.h"
#include "src/APP/LQ_GPIO_LED.h"
#include "src/APP/LQ_GPT_mini512.h"
#include "src/APP/LQ_I2C_9AX.h"
#include "src/APP/LQ_I2C_VL53.h"
#include "src/APP/LQ_ICM20602.h"
#include "src/APP/LQ_OLED096.h"
#include "src/APP/LQ_STM_Timer.h"
#include "src/APP/LQ_TFT18.h"
#include "src/APP/LQ_Tim_InputCature.h"
#include "src/APP/LQ_Tom_Servo.h"
#include "src/APP/LQ_UART_Bluetooth.h"
#include "src/Driver/include.h"
#include "src/Driver/LQ_STM.h"
#include "src/Driver/LQ_UART.h"
#include "src/User/LQ_MotorServo.h"
#include "LQ_ImageProcess.h"
#include "LQ_PID.h"
#include "LQ_CCU6.h"
#include "LQ_TFT2.h"
#include "LQ_IIC_Gyro.h"
#include "LQ_ADC.h"
App_Cpu0 g_AppCpu0;						// brief CPU 0 global data
IfxCpu_mutexLock mutexCpu0InitIsOk = 1; // CPU0 ��ʼ����ɱ�־λ
volatile char mutexCpu0TFTIsOk = 0;		// CPU1 0ռ��/1�ͷ� TFT

void LQ_drv_val(unsigned short *valure0, unsigned short *valure1);
// extern pid_param_t BalDirgyro_PID;
pid_param_t LSpeed_PID;
pid_param_t RSpeed_PID;
unsigned short val0;
unsigned short val1;
unsigned short val2;
unsigned short val3;

extern unsigned char Powern_On;
extern unsigned char Power_Off;
extern unsigned char motor_flag;
extern unsigned char Pw_flag;

/*************************************************************************
 *  �������ƣ�int core0_main (void)
 *  ����˵����CPU0������
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��10��
 *  ��    ע��
 *************************************************************************/
int core0_main(void)
{
	// �ر�CPU���ж�
	IfxCpu_disableInterrupts();

	// �رտ��Ź�����������ÿ��Ź�ι����Ҫ�ر�
	IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
	IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

	// ��ȡ����Ƶ��
	g_AppCpu0.info.pllFreq = IfxScuCcu_getPllFrequency();
	g_AppCpu0.info.cpuFreq = IfxScuCcu_getCpuFrequency(IfxCpu_getCoreIndex());
	g_AppCpu0.info.sysFreq = IfxScuCcu_getSpbFrequency();
	g_AppCpu0.info.stmFreq = IfxStm_getFrequency(&MODULE_STM0);

	TFTSPI_Init(0);																  // TFT1.8��ʼ��0:����  1������
	TFTSPI_CLS(u16BLACK);														  // ����
	// TFTSPI_Show_Logo(0, 37);													  // ��ʾ����LOGO
	TFTSPI_P16x16Str(0, 0, (unsigned char *)"����", u16RED, u16BLUE); // �ַ�����ʾ
	TFTSPI_CLS(u16BLACK);														  // ����
	// ������ʼ��
	GPIO_KEY_Init();
	// LED������P10.6��P10.5��ʼ��
	GPIO_LED_Init();
	// ����P14.0�ܽ����,P14.1���룬������115200
	UART_InitConfig(UART0_RX_P14_1, UART0_TX_P14_0, 115200);
	// ����CPU���ж�

	IfxCpu_enableInterrupts();
	// ֪ͨCPU1��CPU0��ʼ�����
	IfxCpu_releaseMutex(&mutexCpu0InitIsOk);
	// �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
	mutexCpu0TFTIsOk = 0; // CPU1�� 0ռ��/1�ͷ� TFT
						  //λ��

	ServoInit(); // �����ʼ��
	MotorInit(); // �����ʼ��
	EncInit();	 // ��������ʼ��

	// PID��������
	PidInit(&LSpeed_PID);
	PidInit(&RSpeed_PID);
	LSpeed_PID.kp = 250;
	LSpeed_PID.ki = 1.2;
	LSpeed_PID.kd = 0.5;
	RSpeed_PID.kp = 250;
	RSpeed_PID.ki = 1.2;
	RSpeed_PID.kd = 0.5;

	/* ����ͷ��ʼ�� */
	CAMERA_Init(50);

	ADC_InitConfig(ADC0, 80000); // ADC�ɼ���ʼ��
	ADC_InitConfig(ADC1, 80000);
	ADC_InitConfig(ADC2, 80000);
	ADC_InitConfig(ADC3, 80000);

	//����CCU6��ʱ��
	CCU6_InitConfig(CCU60, CCU6_Channel1, 5000); // 5ms����һ���ж�

	char txt[32];
	while (1) //��ѭ��
	{
		//�����˲�
		LQ_drv_val(&val0, &val1);
		// ��Ļ��Ϣ��ʾ
		sprintf(txt, "FSIG: %04d", val0); //ǰ�źż���
		TFTSPI_P6X8Str(0, 5, txt, u16WHITE, u16BLUE);
		sprintf(txt, "RSIG: %04d", val1); //���źż���
		TFTSPI_P6X8Str(0, 6, txt, u16WHITE, u16BLUE);

		sprintf(txt, "ADC2: %04d", val2); //��ص���
		TFTSPI_P6X8Str(0, 2, txt, u16WHITE, u16BLACK);
		sprintf(txt, "ADC3: %04d", val3); //����ٶ�
		TFTSPI_P6X8Str(0, 3, txt, u16WHITE, u16BLACK);

		// TFTSPI_P8X16Str(bigger)

		sprintf(txt, "LENC: %04d", ECPULSE1); //���ֱ�����
		TFTSPI_P6X8Str(0, 13, txt, u16RED, u16BLUE);
		sprintf(txt, "RENC: %04d", ECPULSE2); //���ֱ�����
		TFTSPI_P6X8Str(0, 14, txt, u16RED, u16BLUE);

		sprintf(txt, "LPWM: %04d", MotorDuty1); //����PWMֵ
		TFTSPI_P6X8Str(12, 13, txt, u16RED, u16BLUE);
		sprintf(txt, "RPWM: %04d", MotorDuty2); //����PWMֵ
		TFTSPI_P6X8Str(12, 14, txt, u16RED, u16BLUE);


		TFT_Show_Camera_Info();
	}
}

/*************************************************************************
 *  �������ƣ�void LQ_drv_val (void)
 *  ����˵����CPU0������
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2021��12��10��
 *  ��    ע��
 *************************************************************************/
void LQ_drv_val(unsigned short *valure0, unsigned short *valure1)
{
	static unsigned short num0;
	static unsigned short num1;

	num0 = num0 * 6 / 10 + *valure0 * 4 / 10;
	num1 = num1 * 6 / 10 + *valure1 * 4 / 10;

	*valure0 = num0;
	*valure1 = num1;
}
