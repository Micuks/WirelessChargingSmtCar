/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技TC264DA核心板
【编    写】zyf/chiusir
【E-mail  】chiusir@163.com
【软件版本】V1.0
【最后更新】2020年4月10日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【dev.env.】Hightec4.9.3/Tasking6.3及以上版本
【Target 】 TC264DA
【Crystal】 20.000Mhz
【SYS PLL】 200MHz
基于iLLD_1_0_1_11_0底层程序
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "LQ_ImageProcess.h"

#include "../APP/LQ_CAMERA.h"
#include "../APP/LQ_GPIO_KEY.h"
#include "../APP/LQ_GPIO_LED.h"
#include "../APP/LQ_TFT18.h"
#include "../Driver/LQ_ADC.h"
#include "../Driver/LQ_CCU6.h"
#include "../Driver/LQ_STM.h"
#include "LQ_GPT12_ENC.h"
#include "LQ_Inductor.h"
#include "LQ_MotorServo.h"
#include "LQ_PID.h"
#include "LQ_UART.h"
#include <Platform_Types.h>
#include <stdio.h>

/**  @brief    转向误差  */
sint16 g_sSteeringError = 0;
/**  @brief    丢线标志位  */
uint8_t g_ucIsNoSide = 0;

/**  @brief    主跑行  */
#define ROAD_MAIN_ROW 40

/**  @brief    使用起始行  */
#define ROAD_START_ROW 115

/**  @brief    使用结束行  */
#define ROAD_END_ROW 10

/**  @brief    环岛标志位  */
uint8_t g_ucFlagRoundabout = 0;

/**  @brief    十字标志位  */
uint8_t g_ucFlagCross = 0;

/**  @brief    斑马线标志位  */
uint8_t g_ucFlagZebra = 0;

/**  @brief    Y型岔口标志位  */
uint8_t g_ucFlagFork = 0;
uint8_t g_ucForkNum = 0;

/**  @brief    T型岔口标志位  */
uint8_t g_ucFlagT = 0;

pid_param_t BalDirgyro_PID; // 方向PID

// 误差放大倍数 1.1
uint8_t Servo_P = 11;

char txt[30];

/*!
 * @brief    画左右边线
 *
 * @param
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/28 星期日
 */
void TFTSPI_BinRoadSide(uint8_t imageOut[LCDH][2]) {
    uint8_t i = 0;

    for (i = 0; i < ROAD_START_ROW; i++) {
        TFTSPI_Draw_Dot(imageOut[i][0], i, u16RED);

        TFTSPI_Draw_Dot(imageOut[i][1], i, u16GREEN);
    }
}

/*!
 * @brief    画上下边线
 *
 * @param
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/28 星期日
 */
void TFTSPI_BinRoad_UpdownSide(uint8_t imageOut[2][LCDW]) {
    uint8_t i = 0;

    for (i = 0; i < LCDW; i++) {
        TFTSPI_Draw_Dot(i, imageOut[0][i], u16YELLOW);

        TFTSPI_Draw_Dot(i, imageOut[1][i], u16ORANGE);
    }
}

/*!
 * @brief    判断上边线是否单调
 * @param    X1 :起始X点
 * @param    X2 :终止X点              X1 < X2
 * @param    imageIn ： 边线数组
 *
 * @return   0：不单调or错误， 1：单调递增， 2：单调递减
 *
 * @note
 *
 * @see
 *
 * @date     2021/11/30 星期二
 */
uint8_t RoadUpSide_Mono(uint8_t X1, uint8_t X2, uint8_t imageIn[2][LCDW]) {
    uint8_t i = 0, num = 0;

    for (i = X1; i < X2 - 1; i++) {
        if (imageIn[0][i] >= imageIn[0][i + 1])
            num++;
        else
            num = 0;
        if (num >= (X2 - X1) * 4 / 5)
            return 1;
    }

    for (i = X1; i < X2 - 1; i++) {
        if (imageIn[0][i] <= imageIn[0][i + 1])
            num++;
        else
            num = 0;
        if (num >= (X2 - X1) * 4 / 5)
            return 2;
    }
    return 0;
}

/*!
 * @brief    判断是否是直道
 *
 * @param    image ： 二值图像信息
 *
 * @return   0：不是直道， 1：直道
 *
 * @note     思路：两边边线都单调
 *
 * @see
 *
 * @date     2020/6/23 星期二
 */
uint8_t RoadIsStraight(uint8_t imageSide[LCDH][2]) {
    uint8_t i = 0;
    uint8_t leftState = 0, rightState = 0;

    /* 左边线是否单调 */
    for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
        if (imageSide[i][0] + 5 < imageSide[i + 1][0]) {
            leftState = 1;
            break;
        }
    }

    /* 右边线是否单调 */
    for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
        if (imageSide[i][1] - 5 > imageSide[i + 1][1]) {
            rightState = 1;
            break;
        }
    }

    if (leftState == 1 && rightState == 1) {
        return 1;
    }

    return 0;
}

/*!
 * @brief    判断是否是斑马线
 *
 * @param    image ： 二值图像信息
 *
 * @return   0：不是， 1：是
 *
 * @note     思路：
 *
 * @see
 *
 * @date     2020/6/23 星期二
 */
uint8_t RoadIsZebra(uint8_t image[LCDH][LCDW], uint8_t *flag) {
    int i = 0, j = 0;
    int count = 0;

    for (i = ROAD_MAIN_ROW - 30; i > ROAD_MAIN_ROW + 30; i++) {
        for (j = 1; j < LCDW; j++) {
            if (image[i][j] == 1 && image[i][j - 1] == 0) {
                count++;
            }
        }
        if (count > 5) {
            *flag = 1;
            return 1;
        }
    }

    return 0;
}

/*!
 * @brief    判断是否是T字
 *
 * @param    imageSide ： 图像边线信息
 * @param    flag      ： T字状态信息
 *
 * @return   0：不是， 1：是
 *
 * @note     思路：上线0-80单调增， 80-159单调减
 * ，整体是一个大弧，右边线全丢，左边线115-50单调增
 *
 * @see
 *
 * @date     2020/6/23 星期二
 */
uint8_t RoadIsT(uint8_t imageUp[2][LCDW], uint8_t imageSide[LCDH][2],
                uint8_t *flag) {
    uint8_t i = 0;
    uint8_t errU2 = 0, errL1 = 0;
    uint8_t errR1 = 0, errU1 = 0;
    uint8_t leftState = 0, rightState = 0;
    uint8_t count = 0, num = 0, py;
    uint8_t index = 0;

    /* 检测右侧边线丢线？ --  */
    for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
        if (imageSide[i][1] == 159)
            num++;
        if (num >= 85) {    // num >= 130?
            rightState = 1; //右为丢线
            break;
        }
    }

    for (i = ROAD_START_ROW - 1; i > 20; i--) {
        if (imageSide[i][0] <= imageSide[i + 1][0]) {
            if (index < 4)
                index = 0;
            count++;
        } else {
            if (count < 15)
                count = 0;
            index++;
        }
        if (count >= 15 && index >= 4) {
            leftState = 1; // 左弧标志
            break;
        }
    }
    errL1 = RoundaboutGetArc(imageSide, 1, 5, &py); //左线有弧
    errR1 = RoundaboutGetArc(imageSide, 2, 5, &py); //右线有弧
    errU1 = RoadUpSide_Mono(10, 70, imageUp);       //上单调增
    errU2 = RoadUpSide_Mono(80, 150, imageUp);      //上单调减
    if (rightState == 1 && errU2 == 2 &&
        errL1 == 1) { // 右线丢线, 上线单调递减, 左边有弧
        *flag = 1;
        return 1;
    }
    return 0;
}

/*!
 * @brief    T字处理
 *
 * @param    imageInput ： 图像边缘信息
 *           imageUp    ： 上下边线
 *           imageSide  ：左右边线
 *           *flag      ：标志位
 *
 * @return   0：结束
 *
 * @note
 *
 * @see
 *
 * @date     2021/9/23 星期四
 */
uint8_t TProcess(uint8_t imageInput[LCDH][LCDW], uint8_t imageUp[2][LCDW],
                 uint8_t imageSide[LCDH][2], uint8_t *flag) {
    uint8_t py, i, num = 0;
    uint8_t errU1 = 0, errU2 = 0, errL1 = 0;
    switch (*flag) {
    case 1:
        //重新确定上边线
        Roundabout_Get_UpDowmSide(imageInput, imageUp, 1);
        errL1 = RoundaboutGetArc(imageSide, 1, 5, &py); //左线有弧
        errU1 = RoadUpSide_Mono(10, 140, imageUp);      //上单调增

        if (errU1 == 2) // && errL1 == 0
            *flag = 2;

        //补线左转，增加转弯半径
        ImageAddingLine(imageSide, 1, 90, 30, 0, ROAD_START_ROW);
        break;

    case 2:
        errU2 = RoundaboutGetArc(imageSide, 2, 5, &py); //检查右边线是否有弧
        for (i = 0; i < 159; i++) {
            if (imageUp[1][i] <= 118)
                num++;
            if (num >= 140)
                errU2 = 1;
        }
        if (errU2) {
            Target_Speed1 = 15;
            Target_Speed2 = 15;
            Servo_P = 12;
            *flag = 0;
            break;
        }
        ImageAddingLine(imageSide, 2, 60, 30, 159,
                        ROAD_START_ROW); //参数自行修改
        break;
    }
    return 0;
}

/*!
 * @brief    判断是否是十字
 *
 * @param    imageSide ： 图像边线信息
 * @param    flag      ： 十字状态信息
 *
 * @return   0：不是， 1：是
 *
 * @note     思路：两条边线距离车头近的行丢线 -- 然后一部分行不丢线  --
 * 接着又丢线 则证明有十字
 *
 * @see
 *
 * @date     2020/6/23 星期二
 */
uint8_t RoadIsCross(uint8_t imageSide[LCDH][2], uint8_t *flag) {
    int i = 0;
    uint8_t errR = 0, errF = 0;
    uint8_t rightState = 0, leftState = 0;
    int start[5] = {0, 0, 0, 0, 0}, end[5] = {0, 0, 0, 0, 0};
    uint8_t count = 0;
    uint8_t index = 0;

    /* 检测右侧边线距离车头近的行丢线 -- 然后一部分行不丢线  --  接着又丢线 */
    for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
        if (imageSide[i][1] == 159)
            count++;
        else {
            if (count > 10 && index < 5) {
                start[index] = i + count;
                end[index] = i;
                index++;
            }
            count = 0;
        }
    }
    if (index > 1) {
        if (end[0] - start[1] > 10)
            rightState = 1;
    }
    index = 0;

    /* 检测左侧边线距离车头近的行丢线 -- 然后一部分行不丢线  --  接着又丢线 */
    for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
        if (imageSide[i][0] == 0)
            count++;
        else {
            if (count > 10 && index < 5) {
                start[index] = i + count;
                end[index] = i;
                index++;
            }
            count = 0;
        }
    }
    if (index > 1) {
        if (end[0] - start[1] > 10)
            leftState = 1;
    }

    if (errR && errF) {
        count = 0;
        index = 0;
        //上线是否右突起
        for (i = 159 - 1; i > 0; i--) {
            if (UpdowmSide[0][i] != 1 && UpdowmSide[0][i + 1] != 1) {
                if (UpdowmSide[0][i] >= UpdowmSide[0][i + 1])
                    index++;
                else
                    count++;
                /* 有弧线 */
                if (index > 20 && count > 20) {
                    *flag = 1;
                    return 1;
                }
            } else {
                index = 0;
                count = 0;
            }
        }
    }

    return 0;
}

/*!
 * @brief    检测环岛
 *
 * @param    Upimage：上下边线
 *           imageInput：边缘数组
 *           image：左右边线
 *           *flag：标志位
 * @return   成功与否
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/24 星期三
 */
uint8_t RoadIsRoundabout(uint8_t Upimage[2][LCDW],
                         uint8_t imageInput[LCDH][LCDW], uint8_t image[LCDH][2],
                         uint8_t *flag) {
    uint8_t i = 0;
    uint8_t errL = 0, errR = 0;
    uint8_t leftState = 0, rightState = 0;
    uint8_t count = 0;
    uint8_t num = 0, py;

    if (RoadUpSide_Mono(5, 120, Upimage))
        return 0;
    /* 从车头往前 左边线是否单调 */
    for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
        if (image[i][0] == 0)
            continue;
        if (image[i][0] >= image[i + 1][0]) // i是Y坐标值  0 是图像左线X坐标
        {
            num++;
            if (num == 50) {
                num = 0;
                leftState = 1; // 左单调标志
                break;
            }
        } else {
            num = 0;
        }
        if (i == ROAD_END_ROW + 1) // Y加到11  清0
            num = 0;
    }
    errL = RoundaboutGetArc(image, 1, 5, &py);
    errR = RoundaboutGetArc(image, 2, 5, &py);

    /* 右边线是否单调 */
    for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
        if (image[i][1] + 3 < image[i + 1][1]) {
            num++;
            if (num == 50) {
                num = 0;
                rightState = 1;
                break;
            }
        }
        if (i == ROAD_END_ROW + 1)
            num = 0;
    }

    /* 左边单调， 检测右侧是否是环岛 */
    if (leftState == 1 && rightState == 0 && errL == 0) {
        count = 0;

        if (RoundaboutGetArc(image, 2, 5,
                             &count)) //左圆弧检测 (5个连续增 且 5个连续减)
        {
            *flag = 1;
            return 1;
        } else {
            return 0;
        }
    }

    /* 右边单调， 检测左侧是否是环岛 */
    if (rightState == 1 && leftState == 0) {
        count = 0;
        if (RoundaboutGetArc(image, 1, 5,
                             &count)) //左圆弧检测 (5个连续增 且 5个连续减)
        {
            *flag = 2;
            return 2;
        }
    }
    return 0;
}

/*!
 * @brief    获取环岛边线
 *
 * @param    imageInput ： 二值图像信息
 * @param    imageOut   ： 边线数组
 * @param    status     ： 1：左环岛(边线)  2：右环岛(边线)
 *
 * @return   void
 *
 * @note     思路：环岛一边边线严格单调，根据一边边线，获取另一边线
 *
 * @see
 *
 * @date     2020/6/23 星期二
 */
void RoundaboutGetSide(uint8_t imageInput[LCDH][LCDW],
                       uint8_t imageSide[LCDH][2], uint8_t status) {
    uint8_t i = 0, j = 0;

    switch (status) {

        /* 左环岛 */
    case 1: {
        /* 重新确定左边界 */
        for (i = ROAD_START_ROW; i > ROAD_END_ROW; i--) {
            for (j = LCDW / 2; j > 0; j--) {
                if (!imageInput[i][j]) {
                    imageSide[i][0] = j;
                    break;
                }
            }
        }
        break;
    }

    case 2: {
        /* 重新确定右边界 */
        for (i = ROAD_START_ROW; i > ROAD_END_ROW; i--) {
            for (j = LCDW / 2; j < LCDW; j++) {
                if (!imageInput[i][j]) {
                    imageSide[i][1] = j;
                    break;
                }
            }
        }
        break;
    }
    }
}

/*!
 * @brief    获取上下边线
 *
 * @param    imageInput ： 二值图像信息
 * @param    imageSide  ： 边线数组
 * @param    status     ： 1：上边线  2：下边线
 *
 * @return   void
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/23 星期二
 */
void Roundabout_Get_UpDowmSide(uint8_t imageInput[LCDH][LCDW],
                               uint8_t imageSide[2][LCDW], uint8_t status) {
    uint8_t i = 0, j = 0;

    switch (status) {
    case 1: {
        /* 重新确定上边界 */
        for (i = 159; i > 0; i--) {
            for (j = LCDH / 2; j > 0; j--) {
                if (!imageInput[j][i]) {
                    imageSide[0][i] = j;
                    break;
                }
            }
        }
        break;
    }

    case 2: {
        /* 重新确定下边界 */
        for (i = 159; i > 0; i--) {
            for (j = LCDH / 2; j < LCDH; j++) {
                if (!imageInput[j][i]) {
                    imageSide[1][i] = j;
                    break;
                }
            }
        }
        break;
    }
    }
}
/*!
 * @brief    判断边线是否存在弧形
 *
 * @param    imageInput ： 二值图像信息
 * @param    imageOut   ： 边线数组
 * @param    status     ： 1：左边线  2：右边线
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/23 星期二
 */
uint8_t RoundaboutGetArc(uint8_t imageSide[LCDH][2], uint8_t status,
                         uint8_t num, uint8_t *index) {
    int i = 0;
    uint8_t inc = 0, dec = 0, n = 0;
    switch (status) {
    case 1:
        for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
            if (imageSide[i][0] != 0 && imageSide[i + 1][0] != 0) {
                if (imageSide[i][0] == imageSide[i + 1][0]) {
                    n++;
                    continue;
                }
                if (imageSide[i][0] > imageSide[i + 1][0]) {
                    inc++;
                    inc += n;
                    n = 0;
                } else {
                    dec++;
                    dec += n;
                    n = 0;
                }

                /* 有弧线 */
                if (inc > num && dec > num) {
                    *index = i + num;
                    return 1;
                }
            } else {
                inc = 0;
                dec = 0;
                n = 0;
            }
        }

        break;

    case 2:
        for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
            if (imageSide[i][1] != 159 && imageSide[i + 1][1] != 159) {
                if (imageSide[i][1] == imageSide[i + 1][1]) {
                    n++;
                    continue;
                }
                if (imageSide[i][1] > imageSide[i + 1][1]) {
                    inc++;
                    inc += n;
                    n = 0;
                } else {
                    dec++;
                    dec += n;
                    n = 0;
                }

                /* 有弧线 */
                if (inc > num && dec > num) {
                    *index = i + num;
                    return 1;
                }
            } else {
                inc = 0;
                dec = 0;
                n = 0;
            }
        }

        break;
    }

    return 0;
}

/*!
 * @brief    判断边线是否存在弧形
 *
 * @param    SideInput ： 上边线数组
 * @param    num       ： 检测幅度
 * @param    index     ： 最低点
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2021/12/01 星期三
 */
uint8_t UpSideErr(uint8_t SideInput[2][LCDW], uint8_t status, uint8_t num,
                  uint8_t *index) {
    uint8_t dec = 0, inc = 0, i;
    //上线是否右突起
    switch (status) {
    case 1:
        for (i = 159 - 1; i > 0; i--) {
            if (UpdowmSide[0][i] > 1 && UpdowmSide[0][i + 1] > 1) {
                if (UpdowmSide[0][i] >= UpdowmSide[0][i + 1])
                    inc++;
                else
                    dec++;
                /* 有弧线 */
                if (inc > num && dec > num) {
                    *index = i + num;
                    return 1;
                }
            } else {
                inc = 0;
                dec = 0;
            }
        }
        break;
    //下边线
    case 2:
        for (i = 159 - 1; i > 0; i--) {
            if (UpdowmSide[1][i] != 1 && UpdowmSide[1][i + 1] != 1) {
                if (UpdowmSide[1][i] >= UpdowmSide[1][i + 1])
                    inc++;
                else
                    dec++;
                /* 有弧线 */
                if (inc > num && dec > num) {
                    *index = i + num;
                    return 1;
                }
            } else {
                inc = 0;
                dec = 0;
            }
        }
        break;
    }

    return 0;
}

/*!
 * @brief    补线处理
 *
 * @param    imageSide  : 边线
 * @param    status     : 1：左边线补线   2：右边线补线
 * @param    startX     : 起始点 列数
 * @param    startY     : 起始点 行数
 * @param    endX       : 结束点 列数
 * @param    endY       : 结束点 行数
 *
 * @return
 *
 * @note     endY 一定要大于 startY
 *
 * @see
 *
 * @date     2020/6/24 星期三
 */
void ImageAddingLine(uint8_t imageSide[LCDH][2], uint8_t status, uint8_t startX,
                     uint8_t startY, uint8_t endX, uint8_t endY) {
    int i = 0;

    /* 直线 x = ky + b*/
    float k = 0.0f, b = 0.0f;
    switch (status) {
    case 1: //左补线
    {
        k = (float)((float)endX - (float)startX) /
            (float)((float)endY - (float)startY);
        b = (float)startX - (float)startY * k;

        for (i = startY; i < endY; i++) {
            imageSide[i][0] = (uint8_t)(k * i + b);
        }
        break;
    }

    case 2: //右补线
    {
        k = (float)((float)endX - (float)startX) /
            (float)((float)endY - (float)startY);
        b = (float)startX - (float)startY * k;

        for (i = startY; i < endY; i++) {
            imageSide[i][1] = (uint8_t)(k * i + b);
        }
        break;
    }
    }
}

/*!
 * @brief    寻找跳变点
 *
 * @param    imageSide   ： 边线数组
 * @param    status      ：1：左边界   2：右边界
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/24 星期三
 */
uint8_t ImageGetHop(uint8_t imageSide[LCDH][2], uint8_t state, uint8_t *x,
                    uint8_t *y) {
    int i = 0;
    uint8_t px = 0, py = 0;
    uint8_t count = 0;
    switch (state) {
    case 1:
        /* 寻找跳变点 */
        for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
            if (imageSide[i][0] == 0 && i > (ROAD_END_ROW + 5)) {
                count++;

                if (count > 5) {
                    if (imageSide[i - 1][0] > (imageSide[i][0] + 20)) {
                        py = i - 1;
                        px = imageSide[py][0];
                        break;
                    }
                    if (imageSide[i - 2][0] > (imageSide[i - 1][0] + 20)) {
                        py = i - 2;
                        px = imageSide[py][0];
                        break;
                    }
                    if (imageSide[i - 3][0] > (imageSide[i - 2][0] + 20)) {
                        py = i - 3;
                        px = imageSide[py][0];
                        break;
                    }
                    if (imageSide[i - 4][0] > (imageSide[i - 3][0] + 20)) {
                        py = i - 4;
                        px = imageSide[py][0];
                        break;
                    }
                }
            } else {
                count = 0;
            }
        }

        if (py != 0) {
            *x = px;
            *y = py;
            return 1;
        }

        break;

    case 2:
        /* 寻找跳变点 */
        for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
            if (imageSide[i][1] == 159 && i > (ROAD_END_ROW + 5)) {
                count++;

                if (count > 5) {
                    if (imageSide[i - 1][1] < (imageSide[i][1] - 20)) {
                        py = i - 1;
                        px = imageSide[py][1];
                        break;
                    }
                    if (imageSide[i - 2][1] < (imageSide[i - 1][1] - 20)) {
                        py = i - 2;
                        px = imageSide[py][1];
                        break;
                    }
                    if (imageSide[i - 3][1] < (imageSide[i - 2][1] - 20)) {
                        py = i - 3;
                        px = imageSide[py][1];
                        break;
                    }
                    if (imageSide[i - 4][1] < (imageSide[i - 3][1] - 20)) {
                        py = i - 4;
                        px = imageSide[py][1];
                        break;
                    }
                }
            } else {
                count = 0;
            }
        }

        if (py != 0) {
            *x = px;
            *y = py;
            return 1;
        }

        break;
    }

    return 0;
}

/*!
 * @brief    环岛补线处理
 *
 * @param    imageInput ： 二值图像信息
 * @param    imageSide  ： 边线数组
 * @param    status     ：环岛标志位
 * 奇数为右环岛，偶数为左环岛（0为结束，左环岛没有修改）
 *
 * @return
 *
 * @note     这里只写了左环岛，右环岛大家可以参考左环岛自己完善
 *
 * @see
 *
 * @date     2020/6/24 星期三
 */
void RoundaboutProcess(uint8_t imageInput[LCDH][LCDW],
                       uint8_t imageSide[LCDH][2], uint8_t UpdowmSide[2][LCDW],
                       uint8_t *state) {
    uint8_t i = 0, err5 = 0;
    uint8_t pointX = 0, pointY = 0, inc = 0, dec = 0;
    uint8_t flag = 0, Down_flag = 0;
    static uint8_t finderr = 0, Up_flag = 0, err1 = 0;
    switch (*state) {
        /* 发现右环岛 环岛出口处补线 */
    case 1:

        /* 重新确定右边界 */
        RoundaboutGetSide(imageInput, imageSide, 2);

        /* 检查弧线 */
        err1 = RoundaboutGetArc(imageSide, 2, 5, &pointY);

        /* 有弧线 进行补线 连接弧线最右点 和 图像左下角 */
        if (err1) {
            pointX = imageSide[pointY][1];
            //            UART_PutStr(UART0, "err\r\n");
            //
            //            /* 准备入环岛 */
            //            if((pointY + 10) > ROAD_MAIN_ROW)
            //            {
            //                * state = 3;
            //            }
            //补线
            ImageAddingLine(imageSide, 2, pointX, pointY, 159, ROAD_START_ROW);
            finderr = 1;
        } else {
            if (finderr)
                *state = 3; //准备进入环岛
        }

        break;

        /* 发现左环岛 环岛出口处补线 */
    case 2:

        /* 重新确定左边界 */
        RoundaboutGetSide(imageInput, imageSide, 1);

        /* 检查弧线 */
        err1 = RoundaboutGetArc(imageSide, 1, 5, &pointY);

        /* 有弧线 进行补线 连接弧线最右点 和 图像左下角 */
        if (err1) {
            pointX = imageSide[pointY][0];
            ImageAddingLine(imageSide, 1, 160 - pointX, 160 - pointY, 0,
                            ROAD_START_ROW);
            finderr = 1;
        } else {
            if (finderr)
                *state = 4;
        }
        break;

        /* 准备进入环岛， 左补线 */
    case 3:
        /* 重新确定上边界 */
        Roundabout_Get_UpDowmSide(imageInput, UpdowmSide, 1);
        pointY = 0;
        pointX = 0;

        /* 上边界最低点 */
        for (i = 40; i < 100; i++) {
            if (UpdowmSide[0][i] > pointY) {
                pointX = i;
                pointY = UpdowmSide[0][i];
            }
        }
        if (pointY >= 50) //最低点大于50（根据自己实际情况修改）
        {
            if (RoadUpSide_Mono(5, 100, UpdowmSide) == 1) //上线单调增进入下一步
                *state = 5;
            ImageAddingLine(imageSide, 1, 100 + 30, 40 - 10, 0,
                            ROAD_START_ROW); //补线（自行修改）
        } else
            ImageAddingLine(imageSide, 1, 60, 40 - 15, 0,
                            ROAD_START_ROW); //补线（补线角度自行修改）
        break;

    case 4:
        /* 重新确定上边界 */
        Roundabout_Get_UpDowmSide(imageInput, UpdowmSide, 1);
        pointY = 0;
        pointX = 0;

        /* 上边界最低点 */
        for (i = 40; i < 100; i++) {
            if (UpdowmSide[0][i] > pointY) {
                pointX = i;
                pointY = UpdowmSide[0][i];
            }
        }
        if (pointY >= 50) {
            if (RoadUpSide_Mono(5, 100, UpdowmSide) == 1)
                *state = 6;
            ImageAddingLine(imageSide, 2, 10, 40 - 10, 159, ROAD_START_ROW);
        } else
            ImageAddingLine(imageSide, 2, 100, 40 - 15, 159, ROAD_START_ROW);
        break;
        /* 出环岛， 直道处补线 */
    case 5:
        flag = 0;
        /* 检查弧线 */
        for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
            if (imageSide[i][0] != 0 && imageSide[i + 1][0] != 0) {
                if (imageSide[i][0] >= imageSide[i + 1][0])
                    inc++;
                else
                    dec++;
                /* 有弧线 */
                if (inc > 10 && dec > 10)
                    err5 = 1; //（参数10：弧的幅度，可自行修改）
            } else {
                inc = 0;
                dec = 0;
            }
        }

        //下线为119
        for (i = 159; i > 0; i--) {
            if (UpdowmSide[1][i] == 119)
                inc++;
            else
                dec++;
            if (dec <= 15) {
                Down_flag = 1;
                break;
            }
        }

        //检查上线单调性
        flag = RoadUpSide_Mono(20, 155, UpdowmSide);

        if (flag && err5 && Down_flag) {
            *state = 7;
        }
        break;

        /* 出环岛， 直道处补线 */
    case 6:
        flag = 0;
        /* 检查弧线 */
        for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
            if (imageSide[i][1] != 159 && imageSide[i + 1][1] != 159) {
                if (imageSide[i][1] > imageSide[i + 1][1])
                    inc++;
                else
                    dec++;
                /* 有弧线 */
                if (inc > 8 && dec > 8)
                    err5 = 1;
            } else {
                inc = 0;
                dec = 0;
            }
        }

        //下线为119
        for (i = 159; i > 0; i--) {
            if (UpdowmSide[1][i] == 119)
                inc++;
            else
                dec++;
            if (dec <= 15) {
                Down_flag = 1;
                break;
            }
        }

        //检查上线单调性
        flag = RoadUpSide_Mono(20, 155, UpdowmSide);

        if (flag && err5 && Down_flag) {
            *state = 8;
            //                  ImageAddingLine(imageSide, 1, 145, 30,0,
            //                  ROAD_START_ROW);
        }
        break;
        //出环
    case 7:

        ImageAddingLine(imageSide, 1, 80, 10, 0, ROAD_START_ROW); //参数自行修改
        // ImageAddingLine(imageSide, 1, 100, 30, 0, ROAD_START_ROW);

        //判断上线是否有突起
        for (i = 159 - 1; i > 0; i--) {
            if (UpdowmSide[0][i] != 0 && UpdowmSide[0][i + 1] != 0) {
                if (UpdowmSide[0][i] >= UpdowmSide[0][i + 1])
                    inc++;
                else
                    dec++;
                if (inc > 20 && dec > 20) {
                    finderr = 0;
                    Up_flag = 0;
                    err1 = 0; //清空静态变量以便下次使用
                              //                        Target_Speed1 = 25;
                              //                        //速度回复 Target_Speed2
                              //                        = 25; Servo_P = 18;
                              //                        //转向回复
                    *state = 0; //环岛结束
                    break;
                }
            } else {
                inc = 0;
                dec = 0;
            }
        }
        break;

    case 8:
        Target_Speed1 = 15;
        Target_Speed1 = 15;
        Servo_P = 20;
        ImageAddingLine(imageSide, 1, 30, 30, 159, ROAD_START_ROW);
        //          Up_flag = RoadUpSide_Mono(20, 155,UpdowmSide);
        //          if(flag == 1)
        //          {
        for (i = 159 - 1; i > 0; i--) {
            if (UpdowmSide[0][i] != 0 && UpdowmSide[0][i + 1] != 0) {
                if (UpdowmSide[0][i] >= UpdowmSide[0][i + 1])
                    inc++;
                else
                    dec++;
                if (inc > 20 && dec > 20) {
                    finderr = 0;
                    Up_flag = 0;
                    err1 = 0;           //清空静态变量以便下次使用
                    Target_Speed1 = 15; //速度回复
                    Target_Speed2 = 15;
                    Servo_P = 15; //转向回复
                    *state = 0;
                    break;
                }
            } else {
                inc = 0;
                dec = 0;
            }
        }
        break;
    }
}

/*!
 * @brief    获取十字边线
 *
 * @param    imageInput ： 二值图像信息
 * @param    imageOut   ： 边线数组
 *
 * @return
 *
 * @note     思路：从中间向两边搜线
 *
 * @see
 *
 * @date     2020/6/23 星期二
 */
void CrossGetSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2]) {
    uint8_t i = 0, j = 0;

    for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
        for (j = 78; j > 1; j--) {
            if (imageInput[i][j]) {
                imageSide[i][0] = j;
                break;
            }
        }

        for (j = 78; j < 159; j++) {
            if (imageInput[i][j]) {
                imageSide[i][1] = j;
                break;
            }
        }
    }
}

/*!
 * @brief    十字补线处理
 *
 * @param    imageInput ： 二值图像信息
 * @param    imageSide  ： 边线数组
 * @param    status     ：十字标志位   1：发现十字    2：进入十字   3：出十字
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/24 星期三
 */
void CrossProcess(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2],
                  uint8_t *state) {

    uint8_t pointX = 0, pointY = 0;
    uint8_t leftIndex = 0;
    static uint8_t count = 0;
    switch (*state) {
    case 1: {
        /* 重新获取边线 */
        CrossGetSide(imageInput, imageSide);

        /* 寻找跳变点 */
        if (ImageGetHop(imageSide, 1, &pointX, &pointY)) {
            /* 补线 */
            ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);
        }

        leftIndex = pointY;
        pointX = 0;
        pointY = 0;

        /* 寻找跳变点 */
        if (ImageGetHop(imageSide, 2, &pointX, &pointY)) {
            /* 补线 */
            ImageAddingLine(imageSide, 2, pointX, pointY, (LCDW - 1),
                            ROAD_START_ROW);
        }

        if (leftIndex != 0 && pointY != 0 && leftIndex >= ROAD_MAIN_ROW &&
            pointY >= ROAD_MAIN_ROW) {
            *state = 2;
            count = 0;
        }

        if (count++ > 20) {
            *state = 2;
            count = 0;
        }

        break;
    }

    case 2: {

        /* 检查弧线 */
        if (RoundaboutGetArc(imageSide, 1, 5, &leftIndex)) {
            /* 重新确定左边界 */
            RoundaboutGetSide(imageInput, imageSide, 1);

            if (ImageGetHop(imageSide, 1, &pointX, &pointY)) {
                /* 补线 */
                ImageAddingLine(imageSide, 1, pointX, pointY,
                                imageSide[leftIndex][0], leftIndex);

                *state = 3;

                count = 0;
            } else {
                imageSide[ROAD_MAIN_ROW][0] = LCDW / 2;
                imageSide[ROAD_MAIN_ROW][1] = LCDW - 1;
            }
        }

        break;
    }

    case 3: {

        /* 重新确定左边界 */
        RoundaboutGetSide(imageInput, imageSide, 1);

        if (ImageGetHop(imageSide, 1, &pointX, &pointY)) {
            /* 检查弧线 */
            if (RoundaboutGetArc(imageSide, 1, 5, &leftIndex)) {
                /* 补线 */
                ImageAddingLine(imageSide, 1, pointX, pointY,
                                imageSide[leftIndex][0], leftIndex);
            } else {
                /* 补线 */
                ImageAddingLine(imageSide, 1, pointX, pointY, 0,
                                ROAD_START_ROW);
            }

            if (pointY >= ROAD_MAIN_ROW) {
                *state = 0;
                count = 0;
            }
        } else {
            imageSide[ROAD_MAIN_ROW][0] = 120;
            imageSide[ROAD_MAIN_ROW][1] = LCDW - 1;
        }

        if (count++ > 10) {
            *state = 0;
            count = 0;
        }

        break;
    }
    }
}

/*!
 * @brief    判断是否是Y型岔口
 *
 * @param    imageSide ： 图像边线信息
 * @param    flag      ： Y型状态信息
 *
 * @return   0：不是， 1：是
 *
 * @note     思路：三线成弧
 *
 * @see
 *
 * @date     2021/12/8 星期三
 */
uint8_t RoadIsFork(uint8_t imageInput[2][LCDW], uint8_t imageSide[LCDH][2],
                   uint8_t *flag, uint8_t *pY) {

    uint8_t i = 0, errR = 0, errF = 0;
    uint8_t inc = 0, dec = 0, num = 0;
    uint8_t pointY;

    /* 检查弧线 */
    errR = RoundaboutGetArc(imageSide, 2, 5, &pointY);
    errF = RoundaboutGetArc(imageSide, 1, 5, &pointY);

    if (errR) {
        if (UpSideErr(imageInput, 1, 20, &pointY)) {
            for (i = 110; i > 40; i--) {
                if (imageSide[i][0] == 0)
                    num++;
                if (num == 65) {
                    *flag = 1;
                    return 1;
                }
            }
        }
    }
    num = 0;
    if (errR && errF) {
        //判断上线是否有弧
        //（推荐判断弧的用这种办法，可自己封装成一个函数，之前的办法有一定的局限性，可自己替换）
        // UpSideErr(imageInput, 20, &(*pY));
        for (i = 159 - 1; i > 0; i--) {
            if (UpdowmSide[0][i] != 0 && UpdowmSide[0][i + 1] != 0) {
                if (UpdowmSide[0][i] == UpdowmSide[0][i + 1]) {
                    num++;
                    continue;
                }
                if (UpdowmSide[0][i] > UpdowmSide[0][i + 1]) {
                    inc++;
                    inc += num;
                    num = 0;
                }
                if (UpdowmSide[0][i] < UpdowmSide[0][i + 1]) {
                    dec++;
                    dec += num;
                    num = 0;
                }
                /* 有弧线 */
                if (inc > 15 && dec > 15) {
                    *flag = 1;
                    return 1;
                }
            } else {
                inc = 0;
                dec = 0;
                num = 0;
            }
        }
    }
    return 0;
}

/*!
 * @brief    Y字补线处理
 *
 * @param    imageInput ： 二值图像信息
 * @param    imageSide  ： 边线数组
 * @param    status     ：十字标志位   1：发现十字    2：进入十字   3：出十字
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/24 星期三
 */

sint32 RAllFork = 0;
void ForkProcess(uint8_t UpSideInput[2][LCDW], uint8_t imageSide[LCDH][2],
                 uint8_t *state) {
    uint8_t pointY;

    uint8_t i = 0, errR = 0, errF = 0;
    uint8_t inc = 0, dec = 0, num = 0;

    static uint8_t D_flag = 0, dou_flag;

    //重新获取上边线
    UpdownSideGet(Bin_Image, UpdowmSide);

    switch (*state) {
    case 1: //判断拐点 进入拐点
        // UpSideErr(UpSideInput, 1, 15, &pointY);
        for (i = 159 - 1; i > 0; i--) {
            if (UpdowmSide[0][i] != 0 && UpdowmSide[0][i + 1] != 0) {
                if (UpdowmSide[0][i] == UpdowmSide[0][i + 1]) {
                    num++;
                    continue;
                }
                if (UpdowmSide[0][i] > UpdowmSide[0][i + 1]) {
                    inc++;
                    inc += num;
                    num = 0;
                }
                if (UpdowmSide[0][i] < UpdowmSide[0][i + 1]) {
                    dec++;
                    dec += num;
                    num = 0;
                }
                /* 有弧线 */
                if (inc > 15 && dec > 15) {
                    pointY = i + 15;
                    // *flag = 1;
                    // return 1;
                }
            } else {
                inc = 0;
                dec = 0;
                num = 0;
            }
        }

        if ((UpSideInput[0][pointY] > 30) ||
            (D_flag)) { // 值30可能需要根据速度调整
            ImageAddingLine(imageSide, 1, 110, 35, 0,
                            ROAD_START_ROW); // 屏幕左下角连拐点（可自行修改）
            D_flag = 1;
        }
        if (D_flag == 1 && RoadUpSide_Mono(30, 150, UpSideInput) == 2) {
            *state = 2;
        }
        break;
    case 2: //出 补线

        if ((dou_flag == 1) && (!RoundaboutGetArc(imageSide, 2, 5, &pointY)))
            *state = 3;
        if (RoundaboutGetArc(imageSide, 2, 5, &pointY))
            dou_flag = 1;
        break;
    case 3:                                                        //出 补线
        ImageAddingLine(imageSide, 1, 100, 30, 0, ROAD_START_ROW); //可自行修改
        if (RoadUpSide_Mono(5, 90, UpSideInput)) //判断出口结束三岔口
        {

            Target_Speed1 = 20;
            Target_Speed2 = 20;
            Servo_P = 12;
            if (g_ucForkNum == 2) {
                Target_Speed1 = 22;
                Target_Speed2 = 22;
                Servo_P = 12;
            }
            D_flag = 0;
            dou_flag = 0;
            *state = 0;
        }
        break;
    }
}

/*!
 * @brief    停车线处理
 *
 * @param    imageSide  ： 边线数组
 * @param    state      ： 停车状态  1：车库在左侧   2：车库在右侧
 * @param    speed      ： 速度
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/24 星期三
 */
void ZebraProcess(uint8_t imageSide[LCDH][2], uint8_t state, int16_t *speed) {
    static uint16_t count = 0;

    count++;

    if (state == 1) {
        imageSide[ROAD_MAIN_ROW][0] = 0;
        imageSide[ROAD_MAIN_ROW][1] = LCDW / 2;
    } else {
        imageSide[ROAD_MAIN_ROW][0] = LCDW / 2;
        imageSide[ROAD_MAIN_ROW][1] = LCDW - 1;
    }

    if (count > 100) {
        *speed = 0;
        while (1)
            ;
    }
}

/*!
 * @brief    根据主跑行，求取舵机偏差
 *
 * @param
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/24 星期三
 */
int16_t RoadGetSteeringError(uint8_t imageSide[LCDH][2], uint8_t lineIndex) {

    return imageSide[lineIndex][0] + imageSide[lineIndex][1] - 158;
}

/*!
 * @brief    判断是否丢线
 *
 * @param    imageInput ： 二值图像信息
 * @param    imageOut   ： 边线数组
 * @param    lineIndex  ： 行
 *
 * @return   0：没有丢线   1:左边丢线  2：右边丢线  3： 左右都丢线   4：错误
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/24 星期三
 */
uint8_t RoadIsNoSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2],
                     uint8_t lineIndex) {
    uint8_t state = 0;
    uint8_t i = 0;
    static uint8_t last = 78;

    imageOut[lineIndex][0] = 0;
    imageOut[lineIndex][1] = 159;
    /* 用距离小车比较近的行 判断是否丢线 */
    for (i = last; i > 1; i--) {
        if (imageInput[lineIndex][i]) {
            imageOut[lineIndex][0] = i;
            break;
        }
    }

    if (i == 1) {
        /* 左边界丢线 */
        state = 1;
    }

    for (i = last; i < 159; i++) {
        if (imageInput[lineIndex][i]) {
            imageOut[lineIndex][1] = i;
            break;
        }
    }

    if (i == 159) {
        /* 左右边界丢线 */
        if (state == 1) {
            state = 3;
        }

        /* 右边界丢线 */
        else {
            state = 2;
        }
    }
    if (imageOut[lineIndex][1] <= imageOut[lineIndex][0]) {
        state = 4;
    }
    return state;
}

/*!
 * @brief    丢线处理
 *
 * @param    imageInput ： 二值图像信息
 * @param    imageOut   ： 边线数组
 * @param    mode       ： 那边丢线？   1：左边丢线  2：右边丢线
 * @param    lineIndex  ： 丢线行数
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/24 星期三
 */
void RoadNoSideProcess(uint8_t imageInput[LCDH][LCDW],
                       uint8_t imageOut[LCDH][2], uint8_t mode,
                       uint8_t lineIndex) {
    uint8_t i = 0, j = 0, count = 0;

    switch (mode) {
    case 1:
        for (i = imageOut[lineIndex][1]; i > 1; i--) {
            count++;
            for (j = lineIndex; j > ROAD_END_ROW && lineIndex > count; j--) {
                if (imageInput[j][i]) {
                    imageOut[lineIndex - count][0] = 0;
                    imageOut[lineIndex - count][1] = i;
                    break;
                }
            }
        }
        break;

    case 2:
        for (i = imageOut[lineIndex][0]; i < 159; i++) {
            count++;
            for (j = lineIndex; j > ROAD_END_ROW && lineIndex > count; j--) {
                if (imageInput[j][i]) {
                    imageOut[lineIndex - count][0] = i;
                    imageOut[lineIndex - count][1] = 159;
                    break;
                }
            }
        }
        break;
    }
}

/*!
 * @brief    获取边线
 *
 * @param    imageInput ： 二值图像信息
 * @param    imageOut   ： 边线数组
 *
 * @return   是否丢线
 *
 * @note     思路：从距离车头较近的行开始从中间向两边搜线
 *
 * @see
 *
 * @date     2020/6/23 星期二
 */
uint8_t ImageGetSide(uint8_t imageInput[LCDH][LCDW],
                     uint8_t imageOut[LCDH][2]) {
    uint8_t i = 0, j = 0;

    RoadIsNoSide(imageInput, imageOut, ROAD_START_ROW);

    /* 离车头近的40行 寻找边线 */
    for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
        imageOut[i][0] = 0;
        imageOut[i][1] = 159;

        /* 根据边界连续特性 寻找边界 */
        for (j = imageOut[i + 1][0] + 10; j > 0; j--) {
            if (!imageInput[i][j]) {
                imageOut[i][0] = j;
                break;
            }
        }
        for (j = imageOut[i + 1][1] - 10; j < 160; j++) {
            if (!imageInput[i][j]) {
                imageOut[i][1] = j;
                break;
            }
        }
        /* 如果左边界 即将超出中线 则检查是否右丢线 */
        if (imageOut[i][0] > (LCDW / 2 - 10) && imageOut[i][1] > (LCDW - 5)) {
            /* 右丢线处理 */
            RoadNoSideProcess(imageInput, imageOut, 2, i);

            if (i > 70) {
                imageOut[i][0] += 50;
            }
            return 1;
        }

        /* 如果右边界 即将超出中线 则检查是否左丢线 */
        if (imageOut[i][1] < (LCDW / 2 + 10) && imageOut[i][0] < (5)) {
            /* 左丢线处理 */
            RoadNoSideProcess(imageInput, imageOut, 1, i);

            if (i > 70) {
                imageOut[i][1] -= 50;
            }
            return 2;
        }
    }
    return 0;
}

/*!
 * @brief    获取边线
 *
 * @param    imageInput ： 二值图像信息
 * @param    imageOut   ： 边线数组
 *
 * @return   是否丢线
 *
 * @note     思路：从距离车头较近的行开始从中间向两边搜线
 *
 * @see
 *
 * @date     2021/11/30 星期二
 */
uint8_t UpdownSideGet(uint8_t imageInput[LCDH][LCDW],
                      uint8_t imageOut[2][LCDW]) {
    uint8_t i = 0, j = 0;
    uint8_t last = 60;

    imageOut[0][159] = 0;
    imageOut[1][159] = 119;
    /* 用中线比较近的行 判断是否丢线 */
    for (i = last; i >= 0; i--) {
        if (!imageInput[i][80]) {
            imageOut[0][80] = i;
            break;
        }
    }

    for (i = last; i < 120; i++) {
        if (!imageInput[i][80]) {
            imageOut[1][80] = i;
            break;
        }
    }

    /* 中线往左 寻找边线 */
    for (i = 80 - 1; i > 0; i--) {
        imageOut[0][i] = 0;
        imageOut[1][i] = 119;

        /* 根据边界连续特性 寻找边界 */
        for (j = imageOut[0][i + 1] + 10; j > 0; j--) {
            if (!imageInput[j][i]) {
                imageOut[0][i] = j;
                break;
            }
        }
        for (j = imageOut[1][i + 1] - 10; j < 120; j++) {
            if (!imageInput[j][i]) {
                imageOut[1][i] = j;
                break;
            }
        }
    }
    /*中线往右 寻找边线*/
    for (i = 80 + 1; i < 159; i++) {
        imageOut[0][i] = 0;
        imageOut[1][i] = 119;

        /* 根据边界连续特性 寻找边界 */
        for (j = imageOut[0][i - 1] + 10; j > 0; j--) {
            if (!imageInput[j][i]) {
                imageOut[0][i] = j;
                break;
            }
        }
        for (j = imageOut[1][i - 1] - 10; j < 120; j++) {
            if (!imageInput[j][i]) {
                imageOut[1][i] = j;
                break;
            }
        }
    }
    return 0;
}

/*!
 * @brief    除单一的噪点
 *
 * @param
 *
 * @return
 *
 * @note     思路： 检查边沿邻域内的9个点，如果大于设置值，则保留该点
 *
 * @see
 *
 * @date     2020/6/24 星期三
 */
void ImagePortFilter(uint8_t imageInput[LCDH][LCDW],
                     uint8_t imageOut[LCDH][LCDW]) {
    uint8_t temp = 0;

    for (int i = 1; i < LCDH - 1; i++) {
        for (int j = 1; j < LCDW - 1; j++) {
            temp = imageInput[i - 1][j - 1] + imageInput[i - 1][j] +
                   imageInput[i - 1][j + 1] + imageInput[i][j - 1] +
                   imageInput[i][j] + imageInput[i][j + 1] +
                   imageInput[i + 1][j - 1] + imageInput[i + 1][j] +
                   imageInput[i + 1][j + 1];

            /* 邻域内5个点是边沿 则保留该点 可以调节这里优化滤波效果 */
            if (temp > 4) {
                imageOut[i][j] = 1;
            } else {
                imageOut[i][j] = 0;
            }
        }
    }
}

uint8_t ImageSide[LCDH][2];  //上下边线数组
uint8_t UpdowmSide[2][LCDW]; //左右边线数组

/*!
 * @brief    信息屏幕显示
 *
 * @param      void
 *
 * @return     void
 *
 * @note
 *
 * @see
 *
 * @date     2022/1/05 星期三
 */
void TFT_Show_Camera_Info(void) {
    //按下K2锁死屏幕画面，不可恢复（用以观察现阶段边线特征）
    // if(KEY_Read(KEY2)==0) while(1);
    /* 调试时可以打开这里 */
    if (KEY_Read(KEY0) == 0) {
        // TFTSPI_BinRoad(0, 0, LCDH, LCDW, (uint8_t*)Image_Use); //图像显示
        TFTSPI_BinRoad(0, 0, LCDH, LCDW,
                       (unsigned char *)Bin_Image); //显示边缘提取图像
        TFTSPI_Draw_Line(80, 0, 80, 119, u16RED);   //显示中线
    } else {
        TFTSPI_CLS(u16BLACK); // 清屏
    }
    TFTSPI_BinRoadSide(ImageSide);         //左右边线
    TFTSPI_BinRoad_UpdownSide(UpdowmSide); //上下边线
    TFTSPI_Draw_Line(0, ROAD_MAIN_ROW, 159, ROAD_MAIN_ROW, u16RED); //领跑行显示
    char txt[32];
    sprintf(txt, "%05d", g_sSteeringError); //误差值
    TFTSPI_P6X8Str(0, 15, txt, u16RED, u16BLUE);

    sprintf(txt, "R[%02d]", g_ucFlagRoundabout); //环岛标志
    TFTSPI_P6X8Str(6, 15, txt, u16RED, u16BLUE);

    sprintf(txt, "T[%02d]", g_ucFlagT); // T口标志
    TFTSPI_P6X8Str(12, 15, txt, u16RED, u16BLUE);

    sprintf(txt, "Y[%01d]", g_ucFlagFork); // Y标志
    TFTSPI_P6X8Str(18, 15, txt, u16RED, u16BLUE);
}
/*************************************************************************
 *  函数名称：void CameraCar(void)
 *  功能说明：摄像头车双电机差速控制
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2022年1月10日
 *  备        注：驱动2个电机
 *************************************************************************/
static uint8_t g_ucFlagRoundabout_flag = 0;
static uint8_t g_ucFlagOutGarage = 0;
//环岛元素处理，其中自行修改得参数与小车的速度，误差放大比例有关，需要同学们自己根据实际情况来修改
void CameraCar(void) {

    LED_Ctrl(LED1, RVS); // LED闪烁 指示程序运行状态
    // if(g_ucFlagOutGarage == 0) {
    //     OutInGarage(OUT_GARAGE, 1); // 1 == 右出入库
    //     g_ucFlagOutGarage = 1; // 出库完成标志位
    // }

    uint8_t pointY;

    if (g_ucFlagRoundabout == 0 && g_ucFlagFork == 0 && g_ucFlagT == 0) {
        // 检测环岛
        RoadIsRoundabout(UpdowmSide, Bin_Image, ImageSide, &g_ucFlagRoundabout);
    }
    if (g_ucFlagRoundabout) {
        g_ucFlagRoundabout_flag = 1;
        //   环岛处理
        Target_Speed1 = 45; //速度调整
        Target_Speed2 = 45;
        Servo_P = 12; //误差放大
        RoundaboutProcess(Bin_Image, ImageSide, UpdowmSide,
                          &g_ucFlagRoundabout);
    }

    /*********************************************************************************************************************************/
    //      //十字部分未用到
    //     if(g_ucFlagRoundabout == 0 && g_ucFlagCross == 0 && g_ucFlagFork ==
    //     0)
    //     {
    //         /* 检测十字 */
    //         RoadIsCross(ImageSide, &g_ucFlagCross);
    //     }
    //     if(g_ucFlagCross)
    //     {
    //         /* 十字处理 */
    //         CrossProcess(Image_Use, ImageSide, &g_ucFlagCross);
    //     }

    /********************************T形路口**********************************************/

    if (g_ucFlagRoundabout == 0 && g_ucFlagFork == 0 && g_ucFlagT == 0) {
        //检查T字
        RoadIsT(UpdowmSide, ImageSide, &g_ucFlagT);
    }
    if (g_ucFlagT) {
        Target_Speed1 = 10; // 急弯减速
        Target_Speed2 = 10;
        //        Servo_P = 12;
        // T字处理
        TProcess(Bin_Image, UpdowmSide, ImageSide, &g_ucFlagT);
    }

    /************************************************************************
      2021/7/19测试代码  Y形路口
      ************************************************************************/
    if (g_ucFlagRoundabout == 0 && g_ucFlagFork == 0 && g_ucFlagT == 0) {
        RoadIsFork(UpdowmSide, ImageSide, &g_ucFlagFork, &pointY);
    }
    if (g_ucFlagFork == 1) {
        g_ucForkNum += 1;
    }

    if (g_ucFlagFork) //遇到岔口
    {
        Target_Speed1 = 18;
        Target_Speed2 = 18;
        Servo_P = 10;
        // Y字处理
        ForkProcess(UpdowmSide, ImageSide, &g_ucFlagFork);
    }
    /*************************车库识别代码***********************************/
    /*************************这部分未修改***********************************/
    if (g_ucFlagRoundabout == 0 && g_ucFlagCross == 0 && g_ucFlagZebra == 0 &&
        g_ucFlagFork == 0) {
        /* 检测车库 */
        RoadIsCross(ImageSide, &g_ucFlagZebra);
    }
    if (g_ucFlagZebra) {
        /* 车库处理 */
        ZebraProcess(Image_Use, 1, 1200);
    }

    /* 根据主跑行，求取舵机偏差 */
    g_sSteeringError = RoadGetSteeringError(ImageSide, ROAD_MAIN_ROW);
    //偏差放大
    ServoDuty = g_sSteeringError * Servo_P / 10;
    //偏差限幅
    if (ServoDuty > 170)
        ServoDuty = 170;
    if (ServoDuty < -170)
        ServoDuty = -170;
    //舵机打角
    ServoCtrl(Servo_Center_Mid - ServoDuty);
}
