/*
 * SCS.c
 * SCS串行舵机协议程序
 * 日期: 2022.3.29
 * 作者:
 */

#include <stdlib.h>
#include "INST.h"
#include "SCS.h"
#include "SCServo.h"

static uint8_t Level = 1; // 舵机返回等级
static uint8_t End = 0;   // 处理器大小端结构
// static uint8_t Error = 0;//舵机状态
uint8_t syncReadRxPacketIndex;
uint8_t syncReadRxPacketLen;
uint8_t *syncReadRxPacket;
uint8_t *syncReadRxBuff;
uint16_t syncReadRxBuffLen;
uint16_t syncReadRxBuffMax;
uint32_t syncTimeOut;

// 1个16位数拆分为2个8位数
// DataL为低位，DataH为高位
void Host2SCS(uint8_t *DataL, uint8_t *DataH, int Data)
{
    if (End)
    {
        *DataL = (Data >> 8);
        *DataH = (Data & 0xff);
    }
    else
    {
        *DataH = (Data >> 8);
        *DataL = (Data & 0xff);
    }
}

// 2个8位数组合为1个16位数
// DataL为低位，DataH为高位
int SCS2Host(uint8_t DataL, uint8_t DataH)
{
    int Data;
    if (End)
    {
        Data = DataL;
        Data <<= 8;
        Data |= DataH;
    }
    else
    {
        Data = DataH;
        Data <<= 8;
        Data |= DataL;
    }
    return Data;
}
/**
  * @brief  写缓冲区
  * @param ID 舵机ID
  * @param MemAddr 内存地址
  * @param nDat 数据指针
  * @param nLen 数据长度
  * @param Fun 功能码

  */
void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun)
{
    uint8_t i;
    uint8_t msgLen = 2;
    uint8_t bBuf[6];
    uint8_t CheckSum = 0;
    bBuf[0] = 0xff;
    bBuf[1] = 0xff;
    bBuf[2] = ID;
    bBuf[4] = Fun;
    if (nDat)
    {
        msgLen += nLen + 1;
        bBuf[3] = msgLen;
        bBuf[5] = MemAddr;
        writeSCS(bBuf, 6);
    }
    else
    {
        bBuf[3] = msgLen;
        writeSCS(bBuf, 5);
    }
    CheckSum = ID + msgLen + Fun + MemAddr;
    if (nDat)
    {
        for (i = 0; i < nLen; i++)
        {
            CheckSum += nDat[i];
        }
        writeSCS(nDat, nLen);
    }
    CheckSum = ~CheckSum;
    writeSCS(&CheckSum, 1);
}

// 普通写指令
// 舵机ID，MemAddr内存表地址，写入数据，写入长度
int genWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
    rFlushSCS();
    writeBuf(ID, MemAddr, nDat, nLen, INST_WRITE);
    wFlushSCS();
    return Ack(ID);
}

// 异步写指令
// 舵机ID，MemAddr内存表地址，写入数据，写入长度
int regWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
    rFlushSCS();
    writeBuf(ID, MemAddr, nDat, nLen, INST_REG_WRITE);
    wFlushSCS();
    return Ack(ID);
}

// 异步写执行行
int regAction(uint8_t ID)
{
    rFlushSCS();
    writeBuf(ID, 0, NULL, 0, INST_REG_ACTION);
    wFlushSCS();
    return Ack(ID);
}

// 同步写指令 (用于同时控制多个舵机，减少通信延迟)
// 参数：ID[]舵机ID数组，IDN数组长度，MemAddr内存表首地址，nDat写入数据，nLen单舵机写入长度
void syncWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
    uint8_t mesLen = ((nLen + 1) * IDN + 4); // 计算总消息长度
    uint8_t Sum = 0;
    uint8_t bBuf[7];
    uint8_t i, j;

    // 构造协议包头
    bBuf[0] = 0xff; // 帧头1
    bBuf[1] = 0xff; // 帧头2
    bBuf[2] = 0xfe; // 广播ID(0xFE)
    bBuf[3] = mesLen; // 数据长度
    bBuf[4] = INST_SYNC_WRITE; // 同步写指令包类型(0x83)
    bBuf[5] = MemAddr; // 控制表目标地址
    bBuf[6] = nLen;    // 单个舵机参数体长度

    rFlushSCS();
    writeSCS(bBuf, 7);

    // 计算校验和初始值
    Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;

    // 依次发送每个舵机的ID和参数，并累加校验和
    for (i = 0; i < IDN; i++)
    {
        writeSCS(&ID[i], 1);
        writeSCS(nDat + i * nLen, nLen);
        Sum += ID[i];
        for (j = 0; j < nLen; j++)
        {
            Sum += nDat[i * nLen + j];
        }
    }
    Sum = ~Sum; // 校验和取反
    writeSCS(&Sum, 1);
    wFlushSCS();
}

int writeByte(uint8_t ID, uint8_t MemAddr, uint8_t bDat)
{
    rFlushSCS();
    writeBuf(ID, MemAddr, &bDat, 1, INST_WRITE);
    wFlushSCS();
    return Ack(ID);
}

int writeWord(uint8_t ID, uint8_t MemAddr, uint16_t wDat)
{
    uint8_t buf[2];
    Host2SCS(buf + 0, buf + 1, wDat);
    rFlushSCS();
    writeBuf(ID, MemAddr, buf, 2, INST_WRITE);
    wFlushSCS();
    return Ack(ID);
}

// 读指令
// 舵机ID，MemAddr内存表地址，返回数据nData，数据长度nLen
int Read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen)
{
    int Size;
    uint8_t bBuf[4];
    uint8_t calSum;
    uint8_t i;
    rFlushSCS();
    writeBuf(ID, MemAddr, &nLen, 1, INST_READ);
    wFlushSCS();
    if (!checkHead())
    {
        return 0;
    }
    // Error = 0;
    if (readSCS(bBuf, 3) != 3)
    {
        return 0;
    }
    Size = readSCS(nData, nLen);
    if (Size != nLen)
    {
        return 0;
    }
    if (readSCS(bBuf + 3, 1) != 1)
    {
        return 0;
    }
    calSum = bBuf[0] + bBuf[1] + bBuf[2];
    for (i = 0; i < Size; i++)
    {
        calSum += nData[i];
    }
    calSum = ~calSum;
    if (calSum != bBuf[3])
    {
        return 0;
    }
    // Error = bBuf[2];
    return Size;
}

// 读1字节，超时返回-1
int readByte(uint8_t ID, uint8_t MemAddr)
{
    uint8_t bDat;
    int Size = Read(ID, MemAddr, &bDat, 1);
    if (Size != 1)
    {
        return -1;
    }
    else
    {
        return bDat;
    }
}

// 读2字节，超时返回-1
int readWord(uint8_t ID, uint8_t MemAddr)
{
    uint8_t nDat[2];
    int Size;
    uint16_t wDat;
    Size = Read(ID, MemAddr, nDat, 2);
    if (Size != 2)
        return -1;
    wDat = SCS2Host(nDat[0], nDat[1]);
    return wDat;
}

// Ping指令，返回舵机ID，超时返回-1
int Ping(uint8_t ID)
{
    uint8_t bBuf[4];
    uint8_t calSum;
    rFlushSCS();
    writeBuf(ID, 0, NULL, 0, INST_PING);
    wFlushSCS();
    // Error = 0;
    if (!checkHead())
    {
        return -1;
    }

    if (readSCS(bBuf, 4) != 4)
    {
        return -1;
    }
    if (bBuf[0] != ID && ID != 0xfe)
    {
        return -1;
    }
    if (bBuf[1] != 2)
    {
        return -1;
    }
    calSum = ~(bBuf[0] + bBuf[1] + bBuf[2]);
    if (calSum != bBuf[3])
    {
        return -1;
    }
    // Error = bBuf[2];
    return bBuf[0];
}
/**
  * @brief  检查协议头
  * @retval 1: 成功 0: 失败
  */
int checkHead(void)
{
    uint8_t bDat;
    uint8_t bBuf[2] = {0, 0};
    uint8_t Cnt = 0;
    while (1)
    {
        if (!readSCS(&bDat, 1))
        {
            return 0;
        }
        bBuf[1] = bBuf[0];
        bBuf[0] = bDat;
        if (bBuf[0] == 0xff && bBuf[1] == 0xff)
        {
            break;
        }
        Cnt++;
        if (Cnt > 10)
        {
            return 0;
        }
    }
    return 1;
}

// 指令应答
int Ack(uint8_t ID)
{
    uint8_t bBuf[4];
    uint8_t calSum;
    // Error = 0;
    if (ID != 0xfe && Level)
    {
        if (!checkHead())
        {
            return 0;
        }
        if (readSCS(bBuf, 4) != 4)
        {
            return 0;
        }
        if (bBuf[0] != ID)
        {
            return 0;
        }
        if (bBuf[1] != 2)
        {
            return 0;
        }
        calSum = ~(bBuf[0] + bBuf[1] + bBuf[2]);
        if (calSum != bBuf[3])
        {
            return 0;
        }
        // Error = bBuf[2];
    }
    return 1;
}
/**
  * @brief  同步读取数据包
  * @param ID 舵机ID数组
  * @param IDN 舵机数量
  * @param MemAddr 内存地址
  * @param nLen 数据长度
  * @retval 读取的数据长度

  */
int syncReadPacketTx(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t nLen)
{
    uint8_t checkSum;
    uint8_t i;
    rFlushSCS();
    syncReadRxPacketLen = nLen;
    checkSum = (4 + 0xfe) + IDN + MemAddr + nLen + INST_SYNC_READ;
    writeByteSCS(0xff);
    writeByteSCS(0xff);
    writeByteSCS(0xfe);
    writeByteSCS(IDN + 4);
    writeByteSCS(INST_SYNC_READ);
    writeByteSCS(MemAddr);
    writeByteSCS(nLen);
    for (i = 0; i < IDN; i++)
    {
        writeByteSCS(ID[i]);
        checkSum += ID[i];
    }
    checkSum = ~checkSum;
    writeByteSCS(checkSum);
    wFlushSCS();

    syncReadRxBuffLen = readSCSTimeOut(syncReadRxBuff, syncReadRxBuffMax, syncTimeOut);
    return syncReadRxBuffLen;
}
/**
  * @brief 同步读取数据包解析
  * @param ID 舵机ID  
  * @param nDat 数据指针
  * @retval 读取的数据长度，超时或错误返回0
  */
void syncReadBegin(uint8_t IDN, uint8_t rxLen, uint32_t TimeOut)
{
    syncReadRxBuffMax = IDN * (rxLen + 6);
    syncReadRxBuff = malloc(syncReadRxBuffMax);
    syncTimeOut = TimeOut;
}
/**
  * @brief  结束同步读取数据包
  * @retval 无
  * 说明：使用完同步读取后应调用此函数释放资源
  */
void syncReadEnd(void)
{
    if (syncReadRxBuff)
    {
        free(syncReadRxBuff);
        syncReadRxBuff = NULL;
    }
}
/**
  * @brief  解析同步读取数据包
  * @param ID 舵机ID
  * @param nDat 数据指针
  * @retval 读取的数据长度，超时或错误返回0
  * 说明：调用syncReadPacketTx发送同步读取指令后，使用此函数解析返回的数据包
  每次调用此函数会解析一个数据包，直到所有数据包解析完或发生错误
  解析成功返回数据长度，失败返回0
  解析过程中会将数据存储在nDat指针指向的缓冲区中，供后续读取使用
  使用完所有数据包后应调用syncReadEnd释放资源  
  */
int syncReadPacketRx(uint8_t ID, uint8_t *nDat)
{
    uint16_t syncReadRxBuffIndex = 0;
    syncReadRxPacket = nDat;
    syncReadRxPacketIndex = 0;
    while ((syncReadRxBuffIndex + 6 + syncReadRxPacketLen) <= syncReadRxBuffLen)
    {
        uint8_t bBuf[] = {0, 0, 0};
        uint8_t calSum = 0;
        while (syncReadRxBuffIndex < syncReadRxBuffLen)
        {
            bBuf[0] = bBuf[1];
            bBuf[1] = bBuf[2];
            bBuf[2] = syncReadRxBuff[syncReadRxBuffIndex++];
            if (bBuf[0] == 0xff && bBuf[1] == 0xff && bBuf[2] != 0xff)
            {
                break;
            }
        }
        if (bBuf[2] != ID)
        {
            continue;
        }
        if (syncReadRxBuff[syncReadRxBuffIndex++] != (syncReadRxPacketLen + 2))
        {
            continue;
        }
        // Error = syncReadRxBuff[syncReadRxBuffIndex++];
        calSum = ID + (syncReadRxPacketLen + 2) + syncReadRxBuff[syncReadRxBuffIndex++];
        for (uint8_t i = 0; i < syncReadRxPacketLen; i++)
        {
            syncReadRxPacket[i] = syncReadRxBuff[syncReadRxBuffIndex++];
            calSum += syncReadRxPacket[i];
        }
        calSum = ~calSum;
        if (calSum != syncReadRxBuff[syncReadRxBuffIndex++])
        {
            return 0;
        }
        return syncReadRxPacketLen;
    }
    return 0;
}

int syncReadRxPacketToByte(void)
{
    if (syncReadRxPacketIndex >= syncReadRxPacketLen)
    {
        return -1;
    }
    return syncReadRxPacket[syncReadRxPacketIndex++];
}
/**
  * @brief 从同步读取数据包中读取一个字  * @param negBit 负数标志位，指定Word中哪个位表示负数，如果该位为1则Word为负数，返回值会被转换为负数
  * @retval 读取的字值，超时或错误返回-1
    * 说明：调用syncReadPacketRx成功解析一个数据包后，使用此函数从数据包中逐字读取数据
         每次调用此函数会返回数据包中的下一个字（2字节），直到所有字读取完或发生错误
         读取成功返回字值，失败返回-1  
  */
int syncReadRxPacketToWrod(uint8_t negBit)
{
    if ((syncReadRxPacketIndex + 1) >= syncReadRxPacketLen)
    {
        return -1;
    }
    int Word = SCS2Host(syncReadRxPacket[syncReadRxPacketIndex], syncReadRxPacket[syncReadRxPacketIndex + 1]);
    syncReadRxPacketIndex += 2;
    if (negBit)
    {
        if (Word & (1 << negBit))
        {
            Word = -(Word & ~(1 << negBit));
        }
    }
    return Word;
}
