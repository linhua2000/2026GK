/*
 * SCServo.c
 * 飞特舵机硬件接口层程序
 * 日期: 2022.3.29
 * 作者:
 */

#include "stm32f4xx.h"
#include "uart.h"

uint32_t IOTimeOut = 5; // 通信超时
uint8_t wBuf[128];
uint8_t wLen = 0;

// 带有自定义超时时间的UART接收数据接口
// 参数：nDat-接收缓冲区指针，nLen-期望接收的字节数，TimeOut-自定义超时轮询次数限制
// 返回：实际接收到的字节数
int readSCSTimeOut(unsigned char *nDat, int nLen, uint32_t TimeOut)
{
    int Size = 0;
    int ComData;
    uint32_t t_user = 0;
    while (1)
    {
        ComData = Uart_Read(); // 从底层串口读取1字节数据
        if (ComData != -1)
        {
            if (nDat)
            {
                nDat[Size] = ComData; // 将读取到的数据存入缓冲区
            }
            Size++;
        }
        if (Size >= nLen) // 达到期望长度时退出
        {
            break;
        }
        t_user++;
        if (t_user > TimeOut) // 达到超时阈值时退出，防止程序死循环卡死
        {
            break;
        }
    }
    return Size;
}

// UART 接收数据接口 (使用全局默认超时 IOTimeOut)
// 参数：nDat-接收缓冲区指针，nLen-期望接收的字节数
// 返回：实际接收到的字节数
int readSCS(unsigned char *nDat, int nLen)
{
    int Size = 0;
    int ComData;
    uint32_t t_user = 0;
    while (1)
    {
        ComData = Uart_Read();
        if (ComData != -1)
        {
            if (nDat)
            {
                nDat[Size] = ComData;
            }
            Size++;
            t_user = 0; // 成功读到一个字节，重置超时计数器
        }
        if (Size >= nLen)
        {
            break;
        }
        t_user++;
        if (t_user > IOTimeOut) // 如果一直读不到，达到IOTimeOut次后放弃接收
        {
            break;
        }
    }
    return Size;
}

// UART 发送数据接口 (此处做缓冲处理，并不直接发往串口)
// 参数：nDat-要发送的数据指针，nLen-数据长度
// 机制：将数据压入全局发送缓冲区 wBuf，等待 wFlushSCS 函数统一物理发送
int writeSCS(unsigned char *nDat, int nLen)
{
    while (nLen--)
    {
        if (wLen < sizeof(wBuf)) // 防止缓冲区溢出
        {
            wBuf[wLen] = *nDat;
            wLen++;
            nDat++;
        }
    }
    return wLen;
}

// UART 单字节发送接口 (同样作为缓冲处理)
int writeByteSCS(unsigned char bDat)
{
    if (wLen < sizeof(wBuf))
    {
        wBuf[wLen] = bDat;
        wLen++;
    }
    return wLen;
}

// 总线收发状态切换延时
// 解释：因为飞特舵机采用的是半双工的单线串口（TX和RX并在一起），
// 控制器从“发送”切到“接收”状态需要有一段短暂的硬件转换空闲时间。
void SCSDelay(void)
{
    uint8_t i = 180;
    while (i--)
    {
    } // 0.056*i(us) 这个值根据单片机的主频不同可能需要微调
}

// 接收缓冲区刷新（丢弃当前串口硬件里所有未读的数据）
// 用于发送新指令前，把由于半双工总线上可能带来的各种杂波或者上一轮残留清除掉
void rFlushSCS()
{
    SCSDelay(); // 等待总线安静
    Uart_Flush(); // 调用底层驱动真正清空串口硬件接收缓冲
}

// 发送缓冲区刷新（将 wBuf 缓存的数据真正物理发送出去）
// 打包完毕的数据，在此处一次性通过 DMA 或串口发出去
void wFlushSCS()
{
    if (wLen)
    {
        Uart_Send(wBuf, wLen); // 调用底层发送核心函数
        wLen = 0;              // 指针复位，下次使用重新装填
    }
}
