/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技TC264DA核心板
【编    写】chiusir
【E-mail】chiusir@163.com
【软件版本】V1.1 版权所有，单位使用请先联系授权
【最后更新】2021年6月16日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【dev.env.】AURIX Development Studio1.4及以上版本
【Target 】 TC264DA/TC264D
【Crystal】 20.000Mhz
【SYS PLL】 200MHz
________________________________________________________________
基于iLLD_1_0_1_11_0底层程序,

使用例程的时候，建议采用没有空格的英文路径，
除了CIF为TC264DA独有外，其它的代码兼容TC264D
本库默认初始化了EMEM：512K，如果用户使用TC264D，注释掉EMEM_InitConfig()初始化函数。
工程下\Libraries\iLLD\TC26B\Tricore\Cpu\CStart\IfxCpu_CStart0.c第164行左右。
=================================================================
程序配套视频地址：https://space.bilibili.com/95313236
=================================================================
摄像头接口                  龙邱神眼模块
● 数据端口：P02.0-P02.7口，共8位，接摄像头的数据端口；
● SCCB端口：P11_2  P11_3
● 时钟像素：外部中断第0组：P00_4；
● 场信号：外部中断第3组：P15_1；
-----------------------------------------------------------------
推荐GPT12模块，共可以实现5路正交解码增量编码器（兼容带方向编码器）信号采集，任意选择四路即可；
P33_7, P33_6   龙邱TC母板编码器1 右轮
P02_8, P33_5   龙邱TC母板编码器2 左轮
P10_3, P10_1   龙邱TC母板编码器3
P20_3, P20_0   龙邱TC母板编码器4
-----------------------------------------------------------------
默认电机接口
使用GTM模块，ATOM四个通道可产生4*8共32路PWM，而且各自频率和占空比可调，推荐使用ATOM0的0-7通道；
第一组双路全桥驱动
P23_1         龙邱TC母板MOTOR1_P
P32_4         龙邱TC母板MOTOR1_N
P21_2         龙邱TC母板MOTOR2_P
P22_3         龙邱TC母板MOTOR2_N
第二组双路全桥驱动
P21_4         龙邱TC母板MOTOR3_P
P21_3         龙邱TC母板MOTOR3_N
P20_8         龙邱TC母板MOTOR4_P
P21_5         龙邱TC母板MOTOR4_N
-----------------------------------------------------------------
默认舵机接口
使用GTM模块，ATOM四个通道可产生4*8共32路PWM，而且各自频率和占空比可调，推荐使用ATOM2；
P33_10        龙邱TC母板舵机1
P33_13        龙邱TC母板舵机2
-----------------------------------------------------------------
 默认屏幕显示接口，用户可以使用TFT或者OLED模块
TFTSPI_CS     P20_14     龙邱TC母板 CS管脚 默认拉低，可以不用
TFTSPI_SCK    P20_11     龙邱TC母板 SPI SCK管脚
TFTSPI_SDI    P20_10     龙邱TC母板 SPI MOSI管脚
TFTSPI_DC     P20_12     龙邱TC母板 D/C管脚
TFTSPI_RST    P20_13     龙邱TC母板 RESET管脚
-----------------------------------------------------------------
OLED_CK       P20_14     龙邱TC母板OLED CK管脚
OLED_DI       P20_11     龙邱TC母板OLED DI管脚
OLED_RST      P20_10     龙邱TC母板OLED RST管脚
OLED_DC       P20_12     龙邱TC母板OLED DC管脚
OLED_CS       P20_13     龙邱TC母板OLED CS管脚 默认拉低，可以不用
----------------------------------------------------------------
默认按键接口
KEY0p         P22_0      龙邱TC母板上按键0
KEY1p         P22_1      龙邱TC母板上按键1
KEY2p         P22_2      龙邱TC母板上按键2
DSW0p         P33_9      龙邱TC母板上拨码开关0
DSW1p         P33_11     龙邱TC母板上拨码开关1
-----------------------------------------------------------------
默认LED接口
LED0p         P10_6      龙邱TC母板核心板上LED0 翠绿
LED1p         P10_5      龙邱TC母板核心板上LED1 蓝灯
LED2p         P20_6      龙邱TC母板上LED0
LED3p         P20_7      龙邱TC母板上LED1
-----------------------------------------------------------------
默认蜂鸣器接口
BEEPp         P33_8      龙邱TC母板上蜂鸣器接口
-----------------------------------------------------------------
定时器
有两个CCU6模块  每个模块有两个独立定时器  触发定时器中断
推荐使用CCU6模块，STM用作系统时钟或者延时；
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
IfxCpu_mutexLock mutexCpu0InitIsOk = 1; // CPU0 初始化完成标志位
volatile char mutexCpu0TFTIsOk = 0;		// CPU1 0占用/1释放 TFT

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
 *  函数名称：int core0_main (void)
 *  功能说明：CPU0主函数
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年3月10日
 *  备    注：
 *************************************************************************/
int core0_main(void)
{
	// 关闭CPU总中断
	IfxCpu_disableInterrupts();

	// 关闭看门狗，如果不设置看门狗喂狗需要关闭
	IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
	IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

	// 读取总线频率
	g_AppCpu0.info.pllFreq = IfxScuCcu_getPllFrequency();
	g_AppCpu0.info.cpuFreq = IfxScuCcu_getCpuFrequency(IfxCpu_getCoreIndex());
	g_AppCpu0.info.sysFreq = IfxScuCcu_getSpbFrequency();
	g_AppCpu0.info.stmFreq = IfxStm_getFrequency(&MODULE_STM0);

	TFTSPI_Init(0);																  // TFT1.8初始化0:横屏  1：竖屏
	TFTSPI_CLS(u16BLACK);														  // 清屏
	// TFTSPI_Show_Logo(0, 37);													  // 显示龙邱LOGO
	TFTSPI_P16x16Str(0, 0, (unsigned char *)"测试", u16RED, u16BLUE); // 字符串显示
	TFTSPI_CLS(u16BLACK);														  // 清屏
	// 按键初始化
	GPIO_KEY_Init();
	// LED灯所用P10.6和P10.5初始化
	GPIO_LED_Init();
	// 串口P14.0管脚输出,P14.1输入，波特率115200
	UART_InitConfig(UART0_RX_P14_1, UART0_TX_P14_0, 115200);
	// 开启CPU总中断

	IfxCpu_enableInterrupts();
	// 通知CPU1，CPU0初始化完成
	IfxCpu_releaseMutex(&mutexCpu0InitIsOk);
	// 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示
	mutexCpu0TFTIsOk = 0; // CPU1： 0占用/1释放 TFT
						  //位置

	ServoInit(); // 舵机初始化
	MotorInit(); // 电机初始化
	EncInit();	 // 编码器初始化

	// PID参数设置
	PidInit(&LSpeed_PID);
	PidInit(&RSpeed_PID);
	LSpeed_PID.kp = 250;
	LSpeed_PID.ki = 1.2;
	LSpeed_PID.kd = 0.5;
	RSpeed_PID.kp = 250;
	RSpeed_PID.ki = 1.2;
	RSpeed_PID.kd = 0.5;

	/* 摄像头初始化 */
	CAMERA_Init(50);

	ADC_InitConfig(ADC0, 80000); // ADC采集初始化
	ADC_InitConfig(ADC1, 80000);
	ADC_InitConfig(ADC2, 80000);
	ADC_InitConfig(ADC3, 80000);

	//开启CCU6定时器
	CCU6_InitConfig(CCU60, CCU6_Channel1, 5000); // 5ms进入一次中断

	char txt[32];
	while (1) //主循环
	{
		//数据滤波
		LQ_drv_val(&val0, &val1);
		// 屏幕信息显示
		sprintf(txt, "FSIG: %04d", val0); //前信号检测板
		TFTSPI_P6X8Str(0, 5, txt, u16WHITE, u16BLUE);
		sprintf(txt, "RSIG: %04d", val1); //后信号检测板
		TFTSPI_P6X8Str(0, 6, txt, u16WHITE, u16BLUE);

		sprintf(txt, "ADC2: %04d", val2); //电池电量
		TFTSPI_P6X8Str(0, 2, txt, u16WHITE, u16BLACK);
		sprintf(txt, "ADC3: %04d", val3); //充电速度
		TFTSPI_P6X8Str(0, 3, txt, u16WHITE, u16BLACK);

		// TFTSPI_P8X16Str(bigger)

		sprintf(txt, "LENC: %04d", ECPULSE1); //左轮编码器
		TFTSPI_P6X8Str(0, 13, txt, u16RED, u16BLUE);
		sprintf(txt, "RENC: %04d", ECPULSE2); //右轮编码器
		TFTSPI_P6X8Str(0, 14, txt, u16RED, u16BLUE);

		sprintf(txt, "LPWM: %04d", MotorDuty1); //左轮PWM值
		TFTSPI_P6X8Str(12, 13, txt, u16RED, u16BLUE);
		sprintf(txt, "RPWM: %04d", MotorDuty2); //右轮PWM值
		TFTSPI_P6X8Str(12, 14, txt, u16RED, u16BLUE);


		TFT_Show_Camera_Info();
	}
}

/*************************************************************************
 *  函数名称：void LQ_drv_val (void)
 *  功能说明：CPU0主函数
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2021年12月10日
 *  备    注：
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
