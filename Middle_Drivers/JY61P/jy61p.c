#include "jy61p.h"

/* 换算系数，按 HWT905.md 参数表 + 维特协议文档报文格式（两者一致）：
 * 加速度 0.0005 g/LSB @ ±6g（HWT905.md 第 28 行）
 *         = 1g 约 2048 个计数，等价于协议文档的 /32768*16g。
 *         「±6g」是可用量程上限不是换算满量程；若平放实测 |A| 读到 ≈2.67
 *         而不是 ≈1.0，才说明该按 6.0f/32768.0f 算。
 * 角速度 0.061 (°/s)/LSB @ ±2000°/s -> 2000/32768
 * 角度   0.0055° 分辨率              -> 180/32768 */
#define ACC_LSB_G      (1.0f    / 2048.0f)      /* 等效 16.0f/32768.0f，亦等效 0.0005g/LSB */
#define GYRO_LSB_DPS   (2000.0f / 32768.0f)
#define ANG_LSB_DEG    (180.0f  / 32768.0f)

static uint8_t RxBuffer[11];        /* 接收数据数组 */
static volatile uint8_t RxState = 0;/* 接收状态标志位 */
static uint8_t RxIndex = 0;         /* 接收数组索引 */

/* 中断里写、主循环里读，故必须 volatile */
volatile float Roll, Pitch, Yaw;    /* 角度 ° */
volatile float Ax, Ay, Az;          /* 加速度 g */
volatile float Gx, Gy, Gz;          /* 角速度 °/s */

/* 小端两字节 -> 有符号数 */
static int16_t rd16(uint8_t idx)
{
    return (int16_t)((uint16_t)RxBuffer[idx] | ((uint16_t)RxBuffer[idx + 1] << 8));
}

/**
 * @brief       数据包处理函数
 * @param       串口接收的数据 RxData
 * @retval      无
 */
void jy61p_ReceiveData(uint8_t RxData)
{
    uint8_t i, sum = 0;

    if (RxState == 0)               /* 等待包头 */
    {
        if (RxData == 0x55)
        {
            RxBuffer[RxIndex] = RxData;
            RxState = 1;
            RxIndex = 1;            /* 进入下一状态 */
        }
    }
    else if (RxState == 1)
    {
        /* 0x51 加速度 / 0x52 角速度 / 0x53 角度；0x54 磁场和其余包不接 */
        if (RxData == 0x51 || RxData == 0x52 || RxData == 0x53)
        {
            RxBuffer[RxIndex] = RxData;
            RxState = 2;
            RxIndex = 2;            /* 进入下一状态 */
        }
    }
    else if (RxState == 2)          /* 接收数据 */
    {
        RxBuffer[RxIndex++] = RxData;
        if (RxIndex == 11)          /* 接收完成 */
        {
            for (i = 0; i < 10; i++)
            {
                sum = sum + RxBuffer[i];    /* 计算校验和 */
            }
            if (sum == RxBuffer[10])        /* 校验成功 */
            {
                switch (RxBuffer[1])
                {
                    case 0x51:  /* Ax Ay Az，第 8/9 字节是温度，本次不用 */
                        Ax = rd16(2) * ACC_LSB_G;
                        Ay = rd16(4) * ACC_LSB_G;
                        Az = rd16(6) * ACC_LSB_G;
                        break;

                    case 0x52:  /* Gx Gy Gz，第 8/9 字节是电压，本次不用 */
                        Gx = rd16(2) * GYRO_LSB_DPS;
                        Gy = rd16(4) * GYRO_LSB_DPS;
                        Gz = rd16(6) * GYRO_LSB_DPS;
                        break;

                    case 0x53:  /* Roll Pitch Yaw（保留原有功能） */
                        Roll  = rd16(2) * ANG_LSB_DEG;
                        Pitch = rd16(4) * ANG_LSB_DEG;
                        Yaw   = rd16(6) * ANG_LSB_DEG;
                        break;

                    default:
                        break;
                }
            }
            RxState = 0;
            RxIndex = 0;            /* 读取完成，回到最初状态，等待包头 */
        }
    }
}
