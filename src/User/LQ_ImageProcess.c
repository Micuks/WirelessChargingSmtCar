/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�TC264DA���İ�
����    д��zyf/chiusir
��E-mail  ��chiusir@163.com
������汾��V1.0
�������¡�2020��4��10��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��dev.env.��Hightec4.9.3/Tasking6.3�����ϰ汾
��Target �� TC264DA
��Crystal�� 20.000Mhz
��SYS PLL�� 200MHz
����iLLD_1_0_1_11_0�ײ����
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

/**  @brief    ת�����  */
sint16 g_sSteeringError = 0;
/**  @brief    ���߱�־λ  */
uint8_t g_ucIsNoSide = 0;

/**  @brief    ������  */
#define ROAD_MAIN_ROW 40

/**  @brief    ʹ����ʼ��  */
#define ROAD_START_ROW 115

/**  @brief    ʹ�ý�����  */
#define ROAD_END_ROW 10

/**  @brief    ������־λ  */
uint8_t g_ucFlagRoundabout = 0;

/**  @brief    ʮ�ֱ�־λ  */
uint8_t g_ucFlagCross = 0;

/**  @brief    �����߱�־λ  */
uint8_t g_ucFlagZebra = 0;

/**  @brief    Y�Ͳ�ڱ�־λ  */
uint8_t g_ucFlagFork = 0;
uint8_t g_ucForkNum = 0;

/**  @brief    T�Ͳ�ڱ�־λ  */
uint8_t g_ucFlagT = 0;

pid_param_t BalDirgyro_PID; // ����PID

// ���Ŵ��� 1.1
uint8_t Servo_P = 11;

char txt[30];

/*!
 * @brief    �����ұ���
 *
 * @param
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/28 ������
 */
void TFTSPI_BinRoadSide(uint8_t imageOut[LCDH][2]) {
    uint8_t i = 0;

    for (i = 0; i < ROAD_START_ROW; i++) {
        TFTSPI_Draw_Dot(imageOut[i][0], i, u16RED);

        TFTSPI_Draw_Dot(imageOut[i][1], i, u16GREEN);
    }
}

/*!
 * @brief    �����±���
 *
 * @param
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/28 ������
 */
void TFTSPI_BinRoad_UpdownSide(uint8_t imageOut[2][LCDW]) {
    uint8_t i = 0;

    for (i = 0; i < LCDW; i++) {
        TFTSPI_Draw_Dot(i, imageOut[0][i], u16YELLOW);

        TFTSPI_Draw_Dot(i, imageOut[1][i], u16ORANGE);
    }
}

/*!
 * @brief    �ж��ϱ����Ƿ񵥵�
 * @param    X1 :��ʼX��
 * @param    X2 :��ֹX��              X1 < X2
 * @param    imageIn �� ��������
 *
 * @return   0��������or���� 1������������ 2�������ݼ�
 *
 * @note
 *
 * @see
 *
 * @date     2021/11/30 ���ڶ�
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
 * @brief    �ж��Ƿ���ֱ��
 *
 * @param    image �� ��ֵͼ����Ϣ
 *
 * @return   0������ֱ���� 1��ֱ��
 *
 * @note     ˼·�����߱��߶�����
 *
 * @see
 *
 * @date     2020/6/23 ���ڶ�
 */
uint8_t RoadIsStraight(uint8_t imageSide[LCDH][2]) {
    uint8_t i = 0;
    uint8_t leftState = 0, rightState = 0;

    /* ������Ƿ񵥵� */
    for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
        if (imageSide[i][0] + 5 < imageSide[i + 1][0]) {
            leftState = 1;
            break;
        }
    }

    /* �ұ����Ƿ񵥵� */
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
 * @brief    �ж��Ƿ��ǰ�����
 *
 * @param    image �� ��ֵͼ����Ϣ
 *
 * @return   0�����ǣ� 1����
 *
 * @note     ˼·��
 *
 * @see
 *
 * @date     2020/6/23 ���ڶ�
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
 * @brief    �ж��Ƿ���T��
 *
 * @param    imageSide �� ͼ�������Ϣ
 * @param    flag      �� T��״̬��Ϣ
 *
 * @return   0�����ǣ� 1����
 *
 * @note     ˼·������0-80�������� 80-159������
 * ��������һ���󻡣��ұ���ȫ���������115-50������
 *
 * @see
 *
 * @date     2020/6/23 ���ڶ�
 */
uint8_t RoadIsT(uint8_t imageUp[2][LCDW], uint8_t imageSide[LCDH][2],
                uint8_t *flag) {
    uint8_t i = 0;
    uint8_t errU2 = 0, errL1 = 0;
    uint8_t errR1 = 0, errU1 = 0;
    uint8_t leftState = 0, rightState = 0;
    uint8_t count = 0, num = 0, py;
    uint8_t index = 0;

    /* ����Ҳ���߶��ߣ� --  */
    for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
        if (imageSide[i][1] == 159)
            num++;
        if (num >= 85) {    // num >= 130?
            rightState = 1; //��Ϊ����
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
            leftState = 1; // �󻡱�־
            break;
        }
    }
    errL1 = RoundaboutGetArc(imageSide, 1, 5, &py); //�����л�
    errR1 = RoundaboutGetArc(imageSide, 2, 5, &py); //�����л�
    errU1 = RoadUpSide_Mono(10, 70, imageUp);       //�ϵ�����
    errU2 = RoadUpSide_Mono(80, 150, imageUp);      //�ϵ�����
    if (rightState == 1 && errU2 == 2 &&
        errL1 == 1) { // ���߶���, ���ߵ����ݼ�, ����л�
        *flag = 1;
        return 1;
    }
    return 0;
}

/*!
 * @brief    T�ִ���
 *
 * @param    imageInput �� ͼ���Ե��Ϣ
 *           imageUp    �� ���±���
 *           imageSide  �����ұ���
 *           *flag      ����־λ
 *
 * @return   0������
 *
 * @note
 *
 * @see
 *
 * @date     2021/9/23 ������
 */
uint8_t TProcess(uint8_t imageInput[LCDH][LCDW], uint8_t imageUp[2][LCDW],
                 uint8_t imageSide[LCDH][2], uint8_t *flag) {
    uint8_t py, i, num = 0;
    uint8_t errU1 = 0, errU2 = 0, errL1 = 0;
    switch (*flag) {
    case 1:
        //����ȷ���ϱ���
        Roundabout_Get_UpDowmSide(imageInput, imageUp, 1);
        errL1 = RoundaboutGetArc(imageSide, 1, 5, &py); //�����л�
        errU1 = RoadUpSide_Mono(10, 140, imageUp);      //�ϵ�����

        if (errU1 == 2) // && errL1 == 0
            *flag = 2;

        //������ת������ת��뾶
        ImageAddingLine(imageSide, 1, 90, 30, 0, ROAD_START_ROW);
        break;

    case 2:
        errU2 = RoundaboutGetArc(imageSide, 2, 5, &py); //����ұ����Ƿ��л�
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
                        ROAD_START_ROW); //���������޸�
        break;
    }
    return 0;
}

/*!
 * @brief    �ж��Ƿ���ʮ��
 *
 * @param    imageSide �� ͼ�������Ϣ
 * @param    flag      �� ʮ��״̬��Ϣ
 *
 * @return   0�����ǣ� 1����
 *
 * @note     ˼·���������߾��복ͷ�����ж��� -- Ȼ��һ�����в�����  --
 * �����ֶ��� ��֤����ʮ��
 *
 * @see
 *
 * @date     2020/6/23 ���ڶ�
 */
uint8_t RoadIsCross(uint8_t imageSide[LCDH][2], uint8_t *flag) {
    int i = 0;
    uint8_t errR = 0, errF = 0;
    uint8_t rightState = 0, leftState = 0;
    int start[5] = {0, 0, 0, 0, 0}, end[5] = {0, 0, 0, 0, 0};
    uint8_t count = 0;
    uint8_t index = 0;

    /* ����Ҳ���߾��복ͷ�����ж��� -- Ȼ��һ�����в�����  --  �����ֶ��� */
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

    /* ��������߾��복ͷ�����ж��� -- Ȼ��һ�����в�����  --  �����ֶ��� */
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
        //�����Ƿ���ͻ��
        for (i = 159 - 1; i > 0; i--) {
            if (UpdowmSide[0][i] != 1 && UpdowmSide[0][i + 1] != 1) {
                if (UpdowmSide[0][i] >= UpdowmSide[0][i + 1])
                    index++;
                else
                    count++;
                /* �л��� */
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
 * @brief    ��⻷��
 *
 * @param    Upimage�����±���
 *           imageInput����Ե����
 *           image�����ұ���
 *           *flag����־λ
 * @return   �ɹ����
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/24 ������
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
    /* �ӳ�ͷ��ǰ ������Ƿ񵥵� */
    for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
        if (image[i][0] == 0)
            continue;
        if (image[i][0] >= image[i + 1][0]) // i��Y����ֵ  0 ��ͼ������X����
        {
            num++;
            if (num == 50) {
                num = 0;
                leftState = 1; // �󵥵���־
                break;
            }
        } else {
            num = 0;
        }
        if (i == ROAD_END_ROW + 1) // Y�ӵ�11  ��0
            num = 0;
    }
    errL = RoundaboutGetArc(image, 1, 5, &py);
    errR = RoundaboutGetArc(image, 2, 5, &py);

    /* �ұ����Ƿ񵥵� */
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

    /* ��ߵ����� ����Ҳ��Ƿ��ǻ��� */
    if (leftState == 1 && rightState == 0 && errL == 0) {
        count = 0;

        if (RoundaboutGetArc(image, 2, 5,
                             &count)) //��Բ����� (5�������� �� 5��������)
        {
            *flag = 1;
            return 1;
        } else {
            return 0;
        }
    }

    /* �ұߵ����� �������Ƿ��ǻ��� */
    if (rightState == 1 && leftState == 0) {
        count = 0;
        if (RoundaboutGetArc(image, 1, 5,
                             &count)) //��Բ����� (5�������� �� 5��������)
        {
            *flag = 2;
            return 2;
        }
    }
    return 0;
}

/*!
 * @brief    ��ȡ��������
 *
 * @param    imageInput �� ��ֵͼ����Ϣ
 * @param    imageOut   �� ��������
 * @param    status     �� 1���󻷵�(����)  2���һ���(����)
 *
 * @return   void
 *
 * @note     ˼·������һ�߱����ϸ񵥵�������һ�߱��ߣ���ȡ��һ����
 *
 * @see
 *
 * @date     2020/6/23 ���ڶ�
 */
void RoundaboutGetSide(uint8_t imageInput[LCDH][LCDW],
                       uint8_t imageSide[LCDH][2], uint8_t status) {
    uint8_t i = 0, j = 0;

    switch (status) {

        /* �󻷵� */
    case 1: {
        /* ����ȷ����߽� */
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
        /* ����ȷ���ұ߽� */
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
 * @brief    ��ȡ���±���
 *
 * @param    imageInput �� ��ֵͼ����Ϣ
 * @param    imageSide  �� ��������
 * @param    status     �� 1���ϱ���  2���±���
 *
 * @return   void
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/23 ���ڶ�
 */
void Roundabout_Get_UpDowmSide(uint8_t imageInput[LCDH][LCDW],
                               uint8_t imageSide[2][LCDW], uint8_t status) {
    uint8_t i = 0, j = 0;

    switch (status) {
    case 1: {
        /* ����ȷ���ϱ߽� */
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
        /* ����ȷ���±߽� */
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
 * @brief    �жϱ����Ƿ���ڻ���
 *
 * @param    imageInput �� ��ֵͼ����Ϣ
 * @param    imageOut   �� ��������
 * @param    status     �� 1�������  2���ұ���
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/23 ���ڶ�
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

                /* �л��� */
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

                /* �л��� */
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
 * @brief    �жϱ����Ƿ���ڻ���
 *
 * @param    SideInput �� �ϱ�������
 * @param    num       �� ������
 * @param    index     �� ��͵�
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2021/12/01 ������
 */
uint8_t UpSideErr(uint8_t SideInput[2][LCDW], uint8_t status, uint8_t num,
                  uint8_t *index) {
    uint8_t dec = 0, inc = 0, i;
    //�����Ƿ���ͻ��
    switch (status) {
    case 1:
        for (i = 159 - 1; i > 0; i--) {
            if (UpdowmSide[0][i] > 1 && UpdowmSide[0][i + 1] > 1) {
                if (UpdowmSide[0][i] >= UpdowmSide[0][i + 1])
                    inc++;
                else
                    dec++;
                /* �л��� */
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
    //�±���
    case 2:
        for (i = 159 - 1; i > 0; i--) {
            if (UpdowmSide[1][i] != 1 && UpdowmSide[1][i + 1] != 1) {
                if (UpdowmSide[1][i] >= UpdowmSide[1][i + 1])
                    inc++;
                else
                    dec++;
                /* �л��� */
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
 * @brief    ���ߴ���
 *
 * @param    imageSide  : ����
 * @param    status     : 1������߲���   2���ұ��߲���
 * @param    startX     : ��ʼ�� ����
 * @param    startY     : ��ʼ�� ����
 * @param    endX       : ������ ����
 * @param    endY       : ������ ����
 *
 * @return
 *
 * @note     endY һ��Ҫ���� startY
 *
 * @see
 *
 * @date     2020/6/24 ������
 */
void ImageAddingLine(uint8_t imageSide[LCDH][2], uint8_t status, uint8_t startX,
                     uint8_t startY, uint8_t endX, uint8_t endY) {
    int i = 0;

    /* ֱ�� x = ky + b*/
    float k = 0.0f, b = 0.0f;
    switch (status) {
    case 1: //����
    {
        k = (float)((float)endX - (float)startX) /
            (float)((float)endY - (float)startY);
        b = (float)startX - (float)startY * k;

        for (i = startY; i < endY; i++) {
            imageSide[i][0] = (uint8_t)(k * i + b);
        }
        break;
    }

    case 2: //�Ҳ���
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
 * @brief    Ѱ�������
 *
 * @param    imageSide   �� ��������
 * @param    status      ��1����߽�   2���ұ߽�
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/24 ������
 */
uint8_t ImageGetHop(uint8_t imageSide[LCDH][2], uint8_t state, uint8_t *x,
                    uint8_t *y) {
    int i = 0;
    uint8_t px = 0, py = 0;
    uint8_t count = 0;
    switch (state) {
    case 1:
        /* Ѱ������� */
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
        /* Ѱ������� */
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
 * @brief    �������ߴ���
 *
 * @param    imageInput �� ��ֵͼ����Ϣ
 * @param    imageSide  �� ��������
 * @param    status     ��������־λ
 * ����Ϊ�һ�����ż��Ϊ�󻷵���0Ϊ�������󻷵�û���޸ģ�
 *
 * @return
 *
 * @note     ����ֻд���󻷵����һ�����ҿ��Բο��󻷵��Լ�����
 *
 * @see
 *
 * @date     2020/6/24 ������
 */
void RoundaboutProcess(uint8_t imageInput[LCDH][LCDW],
                       uint8_t imageSide[LCDH][2], uint8_t UpdowmSide[2][LCDW],
                       uint8_t *state) {
    uint8_t i = 0, err5 = 0;
    uint8_t pointX = 0, pointY = 0, inc = 0, dec = 0;
    uint8_t flag = 0, Down_flag = 0;
    static uint8_t finderr = 0, Up_flag = 0, err1 = 0;
    switch (*state) {
        /* �����һ��� �������ڴ����� */
    case 1:

        /* ����ȷ���ұ߽� */
        RoundaboutGetSide(imageInput, imageSide, 2);

        /* ��黡�� */
        err1 = RoundaboutGetArc(imageSide, 2, 5, &pointY);

        /* �л��� ���в��� ���ӻ������ҵ� �� ͼ�����½� */
        if (err1) {
            pointX = imageSide[pointY][1];
            //            UART_PutStr(UART0, "err\r\n");
            //
            //            /* ׼���뻷�� */
            //            if((pointY + 10) > ROAD_MAIN_ROW)
            //            {
            //                * state = 3;
            //            }
            //����
            ImageAddingLine(imageSide, 2, pointX, pointY, 159, ROAD_START_ROW);
            finderr = 1;
        } else {
            if (finderr)
                *state = 3; //׼�����뻷��
        }

        break;

        /* �����󻷵� �������ڴ����� */
    case 2:

        /* ����ȷ����߽� */
        RoundaboutGetSide(imageInput, imageSide, 1);

        /* ��黡�� */
        err1 = RoundaboutGetArc(imageSide, 1, 5, &pointY);

        /* �л��� ���в��� ���ӻ������ҵ� �� ͼ�����½� */
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

        /* ׼�����뻷���� ���� */
    case 3:
        /* ����ȷ���ϱ߽� */
        Roundabout_Get_UpDowmSide(imageInput, UpdowmSide, 1);
        pointY = 0;
        pointX = 0;

        /* �ϱ߽���͵� */
        for (i = 40; i < 100; i++) {
            if (UpdowmSide[0][i] > pointY) {
                pointX = i;
                pointY = UpdowmSide[0][i];
            }
        }
        if (pointY >= 50) //��͵����50�������Լ�ʵ������޸ģ�
        {
            if (RoadUpSide_Mono(5, 100, UpdowmSide) == 1) //���ߵ�����������һ��
                *state = 5;
            ImageAddingLine(imageSide, 1, 100 + 30, 40 - 10, 0,
                            ROAD_START_ROW); //���ߣ������޸ģ�
        } else
            ImageAddingLine(imageSide, 1, 60, 40 - 15, 0,
                            ROAD_START_ROW); //���ߣ����߽Ƕ������޸ģ�
        break;

    case 4:
        /* ����ȷ���ϱ߽� */
        Roundabout_Get_UpDowmSide(imageInput, UpdowmSide, 1);
        pointY = 0;
        pointX = 0;

        /* �ϱ߽���͵� */
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
        /* �������� ֱ�������� */
    case 5:
        flag = 0;
        /* ��黡�� */
        for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
            if (imageSide[i][0] != 0 && imageSide[i + 1][0] != 0) {
                if (imageSide[i][0] >= imageSide[i + 1][0])
                    inc++;
                else
                    dec++;
                /* �л��� */
                if (inc > 10 && dec > 10)
                    err5 = 1; //������10�����ķ��ȣ��������޸ģ�
            } else {
                inc = 0;
                dec = 0;
            }
        }

        //����Ϊ119
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

        //������ߵ�����
        flag = RoadUpSide_Mono(20, 155, UpdowmSide);

        if (flag && err5 && Down_flag) {
            *state = 7;
        }
        break;

        /* �������� ֱ�������� */
    case 6:
        flag = 0;
        /* ��黡�� */
        for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
            if (imageSide[i][1] != 159 && imageSide[i + 1][1] != 159) {
                if (imageSide[i][1] > imageSide[i + 1][1])
                    inc++;
                else
                    dec++;
                /* �л��� */
                if (inc > 8 && dec > 8)
                    err5 = 1;
            } else {
                inc = 0;
                dec = 0;
            }
        }

        //����Ϊ119
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

        //������ߵ�����
        flag = RoadUpSide_Mono(20, 155, UpdowmSide);

        if (flag && err5 && Down_flag) {
            *state = 8;
            //                  ImageAddingLine(imageSide, 1, 145, 30,0,
            //                  ROAD_START_ROW);
        }
        break;
        //����
    case 7:

        ImageAddingLine(imageSide, 1, 80, 10, 0, ROAD_START_ROW); //���������޸�
        // ImageAddingLine(imageSide, 1, 100, 30, 0, ROAD_START_ROW);

        //�ж������Ƿ���ͻ��
        for (i = 159 - 1; i > 0; i--) {
            if (UpdowmSide[0][i] != 0 && UpdowmSide[0][i + 1] != 0) {
                if (UpdowmSide[0][i] >= UpdowmSide[0][i + 1])
                    inc++;
                else
                    dec++;
                if (inc > 20 && dec > 20) {
                    finderr = 0;
                    Up_flag = 0;
                    err1 = 0; //��վ�̬�����Ա��´�ʹ��
                              //                        Target_Speed1 = 25;
                              //                        //�ٶȻظ� Target_Speed2
                              //                        = 25; Servo_P = 18;
                              //                        //ת��ظ�
                    *state = 0; //��������
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
                    err1 = 0;           //��վ�̬�����Ա��´�ʹ��
                    Target_Speed1 = 15; //�ٶȻظ�
                    Target_Speed2 = 15;
                    Servo_P = 15; //ת��ظ�
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
 * @brief    ��ȡʮ�ֱ���
 *
 * @param    imageInput �� ��ֵͼ����Ϣ
 * @param    imageOut   �� ��������
 *
 * @return
 *
 * @note     ˼·�����м�����������
 *
 * @see
 *
 * @date     2020/6/23 ���ڶ�
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
 * @brief    ʮ�ֲ��ߴ���
 *
 * @param    imageInput �� ��ֵͼ����Ϣ
 * @param    imageSide  �� ��������
 * @param    status     ��ʮ�ֱ�־λ   1������ʮ��    2������ʮ��   3����ʮ��
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/24 ������
 */
void CrossProcess(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2],
                  uint8_t *state) {

    uint8_t pointX = 0, pointY = 0;
    uint8_t leftIndex = 0;
    static uint8_t count = 0;
    switch (*state) {
    case 1: {
        /* ���»�ȡ���� */
        CrossGetSide(imageInput, imageSide);

        /* Ѱ������� */
        if (ImageGetHop(imageSide, 1, &pointX, &pointY)) {
            /* ���� */
            ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);
        }

        leftIndex = pointY;
        pointX = 0;
        pointY = 0;

        /* Ѱ������� */
        if (ImageGetHop(imageSide, 2, &pointX, &pointY)) {
            /* ���� */
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

        /* ��黡�� */
        if (RoundaboutGetArc(imageSide, 1, 5, &leftIndex)) {
            /* ����ȷ����߽� */
            RoundaboutGetSide(imageInput, imageSide, 1);

            if (ImageGetHop(imageSide, 1, &pointX, &pointY)) {
                /* ���� */
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

        /* ����ȷ����߽� */
        RoundaboutGetSide(imageInput, imageSide, 1);

        if (ImageGetHop(imageSide, 1, &pointX, &pointY)) {
            /* ��黡�� */
            if (RoundaboutGetArc(imageSide, 1, 5, &leftIndex)) {
                /* ���� */
                ImageAddingLine(imageSide, 1, pointX, pointY,
                                imageSide[leftIndex][0], leftIndex);
            } else {
                /* ���� */
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
 * @brief    �ж��Ƿ���Y�Ͳ��
 *
 * @param    imageSide �� ͼ�������Ϣ
 * @param    flag      �� Y��״̬��Ϣ
 *
 * @return   0�����ǣ� 1����
 *
 * @note     ˼·�����߳ɻ�
 *
 * @see
 *
 * @date     2021/12/8 ������
 */
uint8_t RoadIsFork(uint8_t imageInput[2][LCDW], uint8_t imageSide[LCDH][2],
                   uint8_t *flag, uint8_t *pY) {

    uint8_t i = 0, errR = 0, errF = 0;
    uint8_t inc = 0, dec = 0, num = 0;
    uint8_t pointY;

    /* ��黡�� */
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
        //�ж������Ƿ��л�
        //���Ƽ��жϻ��������ְ취�����Լ���װ��һ��������֮ǰ�İ취��һ���ľ����ԣ����Լ��滻��
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
                /* �л��� */
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
 * @brief    Y�ֲ��ߴ���
 *
 * @param    imageInput �� ��ֵͼ����Ϣ
 * @param    imageSide  �� ��������
 * @param    status     ��ʮ�ֱ�־λ   1������ʮ��    2������ʮ��   3����ʮ��
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/24 ������
 */

sint32 RAllFork = 0;
void ForkProcess(uint8_t UpSideInput[2][LCDW], uint8_t imageSide[LCDH][2],
                 uint8_t *state) {
    uint8_t pointY;

    uint8_t i = 0, errR = 0, errF = 0;
    uint8_t inc = 0, dec = 0, num = 0;

    static uint8_t D_flag = 0, dou_flag;

    //���»�ȡ�ϱ���
    UpdownSideGet(Bin_Image, UpdowmSide);

    switch (*state) {
    case 1: //�жϹյ� ����յ�
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
                /* �л��� */
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
            (D_flag)) { // ֵ30������Ҫ�����ٶȵ���
            ImageAddingLine(imageSide, 1, 110, 35, 0,
                            ROAD_START_ROW); // ��Ļ���½����յ㣨�������޸ģ�
            D_flag = 1;
        }
        if (D_flag == 1 && RoadUpSide_Mono(30, 150, UpSideInput) == 2) {
            *state = 2;
        }
        break;
    case 2: //�� ����

        if ((dou_flag == 1) && (!RoundaboutGetArc(imageSide, 2, 5, &pointY)))
            *state = 3;
        if (RoundaboutGetArc(imageSide, 2, 5, &pointY))
            dou_flag = 1;
        break;
    case 3:                                                        //�� ����
        ImageAddingLine(imageSide, 1, 100, 30, 0, ROAD_START_ROW); //�������޸�
        if (RoadUpSide_Mono(5, 90, UpSideInput)) //�жϳ��ڽ��������
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
 * @brief    ͣ���ߴ���
 *
 * @param    imageSide  �� ��������
 * @param    state      �� ͣ��״̬  1�����������   2���������Ҳ�
 * @param    speed      �� �ٶ�
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/24 ������
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
 * @brief    ���������У���ȡ���ƫ��
 *
 * @param
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/24 ������
 */
int16_t RoadGetSteeringError(uint8_t imageSide[LCDH][2], uint8_t lineIndex) {

    return imageSide[lineIndex][0] + imageSide[lineIndex][1] - 158;
}

/*!
 * @brief    �ж��Ƿ���
 *
 * @param    imageInput �� ��ֵͼ����Ϣ
 * @param    imageOut   �� ��������
 * @param    lineIndex  �� ��
 *
 * @return   0��û�ж���   1:��߶���  2���ұ߶���  3�� ���Ҷ�����   4������
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/24 ������
 */
uint8_t RoadIsNoSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2],
                     uint8_t lineIndex) {
    uint8_t state = 0;
    uint8_t i = 0;
    static uint8_t last = 78;

    imageOut[lineIndex][0] = 0;
    imageOut[lineIndex][1] = 159;
    /* �þ���С���ȽϽ����� �ж��Ƿ��� */
    for (i = last; i > 1; i--) {
        if (imageInput[lineIndex][i]) {
            imageOut[lineIndex][0] = i;
            break;
        }
    }

    if (i == 1) {
        /* ��߽綪�� */
        state = 1;
    }

    for (i = last; i < 159; i++) {
        if (imageInput[lineIndex][i]) {
            imageOut[lineIndex][1] = i;
            break;
        }
    }

    if (i == 159) {
        /* ���ұ߽綪�� */
        if (state == 1) {
            state = 3;
        }

        /* �ұ߽綪�� */
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
 * @brief    ���ߴ���
 *
 * @param    imageInput �� ��ֵͼ����Ϣ
 * @param    imageOut   �� ��������
 * @param    mode       �� �Ǳ߶��ߣ�   1����߶���  2���ұ߶���
 * @param    lineIndex  �� ��������
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/24 ������
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
 * @brief    ��ȡ����
 *
 * @param    imageInput �� ��ֵͼ����Ϣ
 * @param    imageOut   �� ��������
 *
 * @return   �Ƿ���
 *
 * @note     ˼·���Ӿ��복ͷ�Ͻ����п�ʼ���м�����������
 *
 * @see
 *
 * @date     2020/6/23 ���ڶ�
 */
uint8_t ImageGetSide(uint8_t imageInput[LCDH][LCDW],
                     uint8_t imageOut[LCDH][2]) {
    uint8_t i = 0, j = 0;

    RoadIsNoSide(imageInput, imageOut, ROAD_START_ROW);

    /* �복ͷ����40�� Ѱ�ұ��� */
    for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--) {
        imageOut[i][0] = 0;
        imageOut[i][1] = 159;

        /* ���ݱ߽��������� Ѱ�ұ߽� */
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
        /* �����߽� ������������ �����Ƿ��Ҷ��� */
        if (imageOut[i][0] > (LCDW / 2 - 10) && imageOut[i][1] > (LCDW - 5)) {
            /* �Ҷ��ߴ��� */
            RoadNoSideProcess(imageInput, imageOut, 2, i);

            if (i > 70) {
                imageOut[i][0] += 50;
            }
            return 1;
        }

        /* ����ұ߽� ������������ �����Ƿ����� */
        if (imageOut[i][1] < (LCDW / 2 + 10) && imageOut[i][0] < (5)) {
            /* ���ߴ��� */
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
 * @brief    ��ȡ����
 *
 * @param    imageInput �� ��ֵͼ����Ϣ
 * @param    imageOut   �� ��������
 *
 * @return   �Ƿ���
 *
 * @note     ˼·���Ӿ��복ͷ�Ͻ����п�ʼ���м�����������
 *
 * @see
 *
 * @date     2021/11/30 ���ڶ�
 */
uint8_t UpdownSideGet(uint8_t imageInput[LCDH][LCDW],
                      uint8_t imageOut[2][LCDW]) {
    uint8_t i = 0, j = 0;
    uint8_t last = 60;

    imageOut[0][159] = 0;
    imageOut[1][159] = 119;
    /* �����߱ȽϽ����� �ж��Ƿ��� */
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

    /* �������� Ѱ�ұ��� */
    for (i = 80 - 1; i > 0; i--) {
        imageOut[0][i] = 0;
        imageOut[1][i] = 119;

        /* ���ݱ߽��������� Ѱ�ұ߽� */
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
    /*�������� Ѱ�ұ���*/
    for (i = 80 + 1; i < 159; i++) {
        imageOut[0][i] = 0;
        imageOut[1][i] = 119;

        /* ���ݱ߽��������� Ѱ�ұ߽� */
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
 * @brief    ����һ�����
 *
 * @param
 *
 * @return
 *
 * @note     ˼·�� �����������ڵ�9���㣬�����������ֵ�������õ�
 *
 * @see
 *
 * @date     2020/6/24 ������
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

            /* ������5�����Ǳ��� �����õ� ���Ե��������Ż��˲�Ч�� */
            if (temp > 4) {
                imageOut[i][j] = 1;
            } else {
                imageOut[i][j] = 0;
            }
        }
    }
}

uint8_t ImageSide[LCDH][2];  //���±�������
uint8_t UpdowmSide[2][LCDW]; //���ұ�������

/*!
 * @brief    ��Ϣ��Ļ��ʾ
 *
 * @param      void
 *
 * @return     void
 *
 * @note
 *
 * @see
 *
 * @date     2022/1/05 ������
 */
void TFT_Show_Camera_Info(void) {
    //����K2������Ļ���棬���ɻָ������Թ۲��ֽ׶α���������
    // if(KEY_Read(KEY2)==0) while(1);
    /* ����ʱ���Դ����� */
    if (KEY_Read(KEY0) == 0) {
        // TFTSPI_BinRoad(0, 0, LCDH, LCDW, (uint8_t*)Image_Use); //ͼ����ʾ
        TFTSPI_BinRoad(0, 0, LCDH, LCDW,
                       (unsigned char *)Bin_Image); //��ʾ��Ե��ȡͼ��
        TFTSPI_Draw_Line(80, 0, 80, 119, u16RED);   //��ʾ����
    } else {
        TFTSPI_CLS(u16BLACK); // ����
    }
    TFTSPI_BinRoadSide(ImageSide);         //���ұ���
    TFTSPI_BinRoad_UpdownSide(UpdowmSide); //���±���
    TFTSPI_Draw_Line(0, ROAD_MAIN_ROW, 159, ROAD_MAIN_ROW, u16RED); //��������ʾ
    char txt[32];
    sprintf(txt, "%05d", g_sSteeringError); //���ֵ
    TFTSPI_P6X8Str(0, 15, txt, u16RED, u16BLUE);

    sprintf(txt, "R[%02d]", g_ucFlagRoundabout); //������־
    TFTSPI_P6X8Str(6, 15, txt, u16RED, u16BLUE);

    sprintf(txt, "T[%02d]", g_ucFlagT); // T�ڱ�־
    TFTSPI_P6X8Str(12, 15, txt, u16RED, u16BLUE);

    sprintf(txt, "Y[%01d]", g_ucFlagFork); // Y��־
    TFTSPI_P6X8Str(18, 15, txt, u16RED, u16BLUE);
}
/*************************************************************************
 *  �������ƣ�void CameraCar(void)
 *  ����˵��������ͷ��˫������ٿ���
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2022��1��10��
 *  ��        ע������2�����
 *************************************************************************/
static uint8_t g_ucFlagRoundabout_flag = 0;
static uint8_t g_ucFlagOutGarage = 0;
//����Ԫ�ش������������޸ĵò�����С�����ٶȣ����Ŵ�����йأ���Ҫͬѧ���Լ�����ʵ��������޸�
void CameraCar(void) {

    LED_Ctrl(LED1, RVS); // LED��˸ ָʾ��������״̬
    // if(g_ucFlagOutGarage == 0) {
    //     OutInGarage(OUT_GARAGE, 1); // 1 == �ҳ����
    //     g_ucFlagOutGarage = 1; // ������ɱ�־λ
    // }

    uint8_t pointY;

    if (g_ucFlagRoundabout == 0 && g_ucFlagFork == 0 && g_ucFlagT == 0) {
        // ��⻷��
        RoadIsRoundabout(UpdowmSide, Bin_Image, ImageSide, &g_ucFlagRoundabout);
    }
    if (g_ucFlagRoundabout) {
        g_ucFlagRoundabout_flag = 1;
        //   ��������
        Target_Speed1 = 45; //�ٶȵ���
        Target_Speed2 = 45;
        Servo_P = 12; //���Ŵ�
        RoundaboutProcess(Bin_Image, ImageSide, UpdowmSide,
                          &g_ucFlagRoundabout);
    }

    /*********************************************************************************************************************************/
    //      //ʮ�ֲ���δ�õ�
    //     if(g_ucFlagRoundabout == 0 && g_ucFlagCross == 0 && g_ucFlagFork ==
    //     0)
    //     {
    //         /* ���ʮ�� */
    //         RoadIsCross(ImageSide, &g_ucFlagCross);
    //     }
    //     if(g_ucFlagCross)
    //     {
    //         /* ʮ�ִ��� */
    //         CrossProcess(Image_Use, ImageSide, &g_ucFlagCross);
    //     }

    /********************************T��·��**********************************************/

    if (g_ucFlagRoundabout == 0 && g_ucFlagFork == 0 && g_ucFlagT == 0) {
        //���T��
        RoadIsT(UpdowmSide, ImageSide, &g_ucFlagT);
    }
    if (g_ucFlagT) {
        Target_Speed1 = 10; // �������
        Target_Speed2 = 10;
        //        Servo_P = 12;
        // T�ִ���
        TProcess(Bin_Image, UpdowmSide, ImageSide, &g_ucFlagT);
    }

    /************************************************************************
      2021/7/19���Դ���  Y��·��
      ************************************************************************/
    if (g_ucFlagRoundabout == 0 && g_ucFlagFork == 0 && g_ucFlagT == 0) {
        RoadIsFork(UpdowmSide, ImageSide, &g_ucFlagFork, &pointY);
    }
    if (g_ucFlagFork == 1) {
        g_ucForkNum += 1;
    }

    if (g_ucFlagFork) //�������
    {
        Target_Speed1 = 18;
        Target_Speed2 = 18;
        Servo_P = 10;
        // Y�ִ���
        ForkProcess(UpdowmSide, ImageSide, &g_ucFlagFork);
    }
    /*************************����ʶ�����***********************************/
    /*************************�ⲿ��δ�޸�***********************************/
    if (g_ucFlagRoundabout == 0 && g_ucFlagCross == 0 && g_ucFlagZebra == 0 &&
        g_ucFlagFork == 0) {
        /* ��⳵�� */
        RoadIsCross(ImageSide, &g_ucFlagZebra);
    }
    if (g_ucFlagZebra) {
        /* ���⴦�� */
        ZebraProcess(Image_Use, 1, 1200);
    }

    /* ���������У���ȡ���ƫ�� */
    g_sSteeringError = RoadGetSteeringError(ImageSide, ROAD_MAIN_ROW);
    //ƫ��Ŵ�
    ServoDuty = g_sSteeringError * Servo_P / 10;
    //ƫ���޷�
    if (ServoDuty > 170)
        ServoDuty = 170;
    if (ServoDuty < -170)
        ServoDuty = -170;
    //������
    ServoCtrl(Servo_Center_Mid - ServoDuty);
}
