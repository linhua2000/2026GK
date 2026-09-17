/*
 * SMS_STS.c
 * 飞特SMS/STS系列串行舵机应用层程序
 * 日期: 2022.1.6
 * 作者:
 */

#include <string.h>
#include "INST.h"
#include "SCS.h"
#include "SMS_STS.h"

static uint8_t Mem[SMS_STS_PRESENT_CURRENT_H - SMS_STS_PRESENT_POSITION_L + 1];
static int Err = 0;

int getErr(void)
{
    return Err;
}

// 扩展写位置指令（控制单个舵机）
// 参数：ID-舵机号, Position-目标位置, Speed-运行速度, ACC-加速度
int WritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
{
    uint8_t bBuf[7];
    if (Position < 0)
    {
        Position = -Position;
        Position |= (1 << 15); // 最高位做方向位/符号位
    }

    bBuf[0] = ACC;
    Host2SCS(bBuf + 1, bBuf + 2, Position); // 位置拆分为2字节
    Host2SCS(bBuf + 3, bBuf + 4, 0);        // 延时时间(通常填0)
    Host2SCS(bBuf + 5, bBuf + 6, Speed);    // 速度拆分为2字节

    return genWrite(ID, SMS_STS_ACC, bBuf, 7);
}

/**
  * @brief 执行异步写指令 (RegWrite) 中的命令
  * @param ID: 舵机ID, Position: 目标位置, Speed: 运行速度, ACC: 加速度  
  * @retval 成功返回1，失败返回-1
  * @note   此函数与WritePosEx类似，但使用RegWrite指令发送数据，这意味着发送的命令不会立即执行，而是被存储在舵机的寄存器中，直到调用RegWriteAction函数后才会同时执行。这对于需要同时控制多个舵机的应用场景非常有用，可以实现更协调的动作。需要注意的是，在调用RegWriteAction之前，确保所有需要执行的RegWrite指令都已经正确发送并且没有遗漏。 
  */
int RegWritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
{
    uint8_t bBuf[7];
    if (Position < 0)
    {
        Position = -Position;
        Position |= (1 << 15);
    }

    bBuf[0] = ACC;
    Host2SCS(bBuf + 1, bBuf + 2, Position);
    Host2SCS(bBuf + 3, bBuf + 4, 0);
    Host2SCS(bBuf + 5, bBuf + 6, Speed);

    return regWrite(ID, SMS_STS_ACC, bBuf, 7);
}
/**
  * @brief 执行异步写指令 (RegWrite) 中的命令  
  * @param None   
  * @retval 无返回值
  * @note   此函数用于执行之前通过RegWrite指令发送但未执行的命令。飞特SMS/STS系列舵机支持异步写指令，允许用户先发送多个RegWrite指令来设置目标位置、速度和加速度等参数，但这些指令不会立即执行。调用RegWriteAction函数后，所有之前发送的RegWrite指令将被同时执行，这对于需要同时控制多个舵机的应用场景非常有用，可以实现更协调的动作。需要注意的是，在调用RegWriteAction之前，确保所有需要执行的RegWrite指令都已经正确发送并且没有遗漏。  
  */
void RegWriteAction(void)
{
    regAction(0xfe);
}

/**
  * @brief  批量写位置指令（控制多个舵机）  
  * @param  ID[]: 舵机ID数组  
  * @param  IDN: 数组长度 
  * @retval 无返回值
  * @note   此函数通过调用syncWrite指令实现对多个舵机的批量控制。每个舵机的目标位置、速度和加速度信息被打包在一起发送，减少了通信延迟，提高了控制效率。需要注意的是，所有被控制的舵机必须同时在线并且响应，否则可能会导致部分舵机无法正确执行指令。 
  */
void SyncWritePosEx(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[])
{
    uint8_t offbuf[32 * 7];
    uint8_t i;
    uint16_t V;
    for (i = 0; i < IDN; i++)
    {
        if (Position[i] < 0)
        {
            Position[i] = -Position[i];
            Position[i] |= (1 << 15);
        }

        if (Speed)
        {
            V = Speed[i];
        }
        else
        {
            V = 0;
        }
        if (ACC)
        {
            offbuf[i * 7] = ACC[i];
        }
        else
        {
            offbuf[i * 7] = 0;
        }
        Host2SCS(offbuf + i * 7 + 1, offbuf + i * 7 + 2, Position[i]);
        Host2SCS(offbuf + i * 7 + 3, offbuf + i * 7 + 4, 0);
        Host2SCS(offbuf + i * 7 + 5, offbuf + i * 7 + 6, V);
    }
    syncWrite(ID, IDN, SMS_STS_ACC, offbuf, 7);
}

/**
  * @brief 设置舵机为轮模式 (连续旋转模式)
  * @param ID: 舵机ID  
  * @retval 成功返回1，失败返回-1  
  * @note   轮模式下，舵机不再控制位置，而是直接控制旋转速度和方向。设置为轮模式后，目标位置寄存器将被忽略，舵机将根据目标速度寄存器的值进行旋转。正值表示顺时针旋转，负值表示逆时针旋转。需要注意的是，在轮模式下，舵机无法保持特定位置，因此适用于需要连续旋转的应用场景，如机器人底盘驱动等。 
  */
int WheelMode(uint8_t ID)
{
    return writeByte(ID, SMS_STS_MODE, 1);
}

/**
  * @brief 设置舵机目标速度 (轮模式下使用)
  * @param ID: 舵机ID  
  * @param Speed: 目标速度
  * @param ACC: 加速度
  * @retval 成功返回1，失败返回-1
  * @note   此函数用于飞特SMS/STS系列舵机的轮 
  */
int WriteSpe(uint8_t ID, int16_t Speed, uint8_t ACC)
{
    uint8_t bBuf[2];
    if (Speed < 0)
    {
        Speed = -Speed;
        Speed |= (1 << 15);
    }
    bBuf[0] = ACC;
    genWrite(ID, SMS_STS_ACC, bBuf, 1);

    Host2SCS(bBuf + 0, bBuf + 1, Speed);

    genWrite(ID, SMS_STS_GOAL_SPEED_L, bBuf, 2);
    return 1;
}

/**
  * @brief  使能或关闭舵机扭矩  
  * @param ID: 舵机ID  
  * @param Enable: 0-关闭扭矩，1-使能扭矩，128-校准零点（当前位置设为新的零点）  
  * @retval 成功返回1，失败返回-1  
  * @note   使能扭矩后，舵机会根据目标位置进行运动，关闭扭矩后舵机将不受控制，可以被外力自由移动。校准零点功能会将当前位置设为新的零点，这对于需要重新定义舵机位置的用户来说非常有用。需要注意的是，校准零点会覆盖之前的零点设置，因此在使用此功能前请确保舵机处于正确的位置。 
  */
int EnableTorque(uint8_t ID, uint8_t Enable)
{
    return writeByte(ID, SMS_STS_TORQUE_ENABLE, Enable);
}

/**
  * @brief 解锁舵机EEPROM (允许修改参数)  
  * @param ID: 舵机ID  
  * @param  Enable: 0-解锁EEPROM，1-锁定EEPROM 
  * @retval 成功返回1，失败返回-1  
  * @note   解锁EEPROM后，舵机的参数可以被修改，这对于需要调整舵机参数以适应特定应用的用户来说非常有用。解锁后，用户可以通过写指令修改舵机的各种参数，如ID、波特率、工作模式等。需要注意的是，解锁EEPROM会增加误操作的风险，因此在不需要修改参数时建议锁定EEPROM以保护舵机的设置不被意外改变。 
  */
int unLockEprom(uint8_t ID)
{
    return writeByte(ID, SMS_STS_LOCK, 0);
}
/**
  * @brief 锁定舵机EEPROM (防止参数被意外修改)  
  * @param ID: 舵机ID  
  * @retval 成功返回1，失败返回-1  
  * @note   锁定EEPROM后，舵机的参数将无法被修改，除非再次调用unLockEprom函数解锁。这是为了防止在使用过程中误操作导致舵机参数被改变，从而影响舵机的性能和安全性。锁定后，只有通过特定的解锁指令才能修改参数，这增加了系统的安全性。 
  */
int LockEprom(uint8_t ID)
{
    return writeByte(ID, SMS_STS_LOCK, 1);
}

/**
  * @brief  校准舵机零点 (将当前位置设为新的零点)
  * @param ID: 舵机ID
  * @retval 成功返回1，失败返回-1
  * @note   此函数通过将舵机的扭矩使能值设置为128来实现零点校准。飞特SMS/STS系列舵机在扭矩使能寄存器中，值128表示将当前位置设为新的零点，并且保持当前的物理位置不变。调用此函数后，舵机的当前位置将被重新定义为新的零点位置，后续的目标位置将相对于这个新的零点进行计算和控制。 
  */
int CalibrationOfs(uint8_t ID)
{
    return writeByte(ID, SMS_STS_TORQUE_ENABLE, 128);
}
/**
  * @brief  获取舵机反馈信息
  * @param  ID: 舵机ID (若ID为-1则从上次FeedBack缓存中读)
  * @retval 成功返回1，失败返回-1，并将错误状态保存在全局变量Err中
  * @note   此函数会尝试从指定ID的舵机读取当前位置、速度、负载、电压、温度等状态信息，并存储在全局缓存Mem中。
  *         如果ID为-1，则直接从上次成功读取的缓存Mem中返回数据。读取失败时会设置Err=1，成功时Err=0。 
  */
int FeedBack(int ID)
{
    int nLen = Read(ID, SMS_STS_PRESENT_POSITION_L, Mem, sizeof(Mem));
    if (nLen != sizeof(Mem))
    {
        Err = 1;
        return -1;
    }
    Err = 0;
    return nLen;
}

// 读取舵机当前位置
// 参数：ID-舵机号 (若ID为-1则从上次FeedBack缓存中读)
// 返回：舵机当前的位置值 (处理了方向位)
int ReadPos(int ID)
{
    int Pos = -1;
    if (ID == -1)
    {
        Pos = Mem[SMS_STS_PRESENT_POSITION_H - SMS_STS_PRESENT_POSITION_L];
        Pos <<= 8;
        Pos |= Mem[SMS_STS_PRESENT_POSITION_L - SMS_STS_PRESENT_POSITION_L];
    }
    else
    {
        Err = 0;
        Pos = readWord(ID, SMS_STS_PRESENT_POSITION_L); // 通过串口从舵机读取
        if (Pos == -1)
        {
            Err = 1;
        }
    }
    // 解码符号位
    if (!Err && Pos & (1 << 15))
    {
        Pos = -(Pos & ~(1 << 15));
    }
    return Pos;
}
/**
  * @brief 读取舵机当前速度  
  * @param ID: 舵机ID (若ID为-1则从上次FeedBack缓存中读)  
  * @retval 舵机当前的速度值 (处理了方向位)  
  * @note   此函数会尝试从指定ID的舵机读取当前速度信息，并返回处理后的速度值。如果ID为-1，则直接从上次成功读取的缓存Mem中返回数据。读取失败时会设置Err=1，成功时Err=0。需要注意的是，返回的速度值已经处理了方向位，如果最高位为1，则表示速度为负值，函数会将其转换为对应的负数返回。 
  */
int ReadSpeed(int ID)
{
    int Speed = -1;
    if (ID == -1)
    {
        Speed = Mem[SMS_STS_PRESENT_SPEED_H - SMS_STS_PRESENT_POSITION_L];
        Speed <<= 8;
        Speed |= Mem[SMS_STS_PRESENT_SPEED_L - SMS_STS_PRESENT_POSITION_L];
    }
    else
    {
        Err = 0;
        Speed = readWord(ID, SMS_STS_PRESENT_SPEED_L);
        if (Speed == -1)
        {
            Err = 1;
            return -1;
        }
    }
    if (!Err && Speed & (1 << 15))
    {
        Speed = -(Speed & ~(1 << 15));
    }
    return Speed;
}
/**
  * @brief 读取舵机当前负载  
  * @param ID: 舵机ID (若ID为-1则从上次FeedBack缓存中读)  
  * @retval 舵机当前的负载值 (处理了方向位)  
  * @note   此函数会尝试从指定ID的舵机读取当前负载信息，并返回处理后的负载值。如果ID为-1，则直接从上次成功读取的缓存Mem中返回数据。读取失败时会设置Err=1，成功时Err=0。需要注意的是，返回的负载值已经处理了方向位，如果最高位为1，则表示负载为负值，函数会将其转换为对应的负数返回。  
  */
int ReadLoad(int ID)
{
    int Load = -1;
    if (ID == -1)
    {
        Load = Mem[SMS_STS_PRESENT_LOAD_H - SMS_STS_PRESENT_POSITION_L];
        Load <<= 8;
        Load |= Mem[SMS_STS_PRESENT_LOAD_L - SMS_STS_PRESENT_POSITION_L];
    }
    else
    {
        Err = 0;
        Load = readWord(ID, SMS_STS_PRESENT_LOAD_L);
        if (Load == -1)
        {
            Err = 1;
        }
    }
    if (!Err && Load & (1 << 10))
    {
        Load = -(Load & ~(1 << 10));
    }
    return Load;
}
/**
  * @brief 读取舵机当前电压  
  * @param ID: 舵机ID (若ID为-1则从上次FeedBack缓存中读)  
  * @retval 舵机当前的电压值  
  * @param None 
  * @retval 舵机当前的电压值  
  * @note   此函数会尝试从指定ID的舵机读取当前 
  */
int ReadVoltage(int ID)
{
    int Voltage = -1;
    if (ID == -1)
    {
        Voltage = Mem[SMS_STS_PRESENT_VOLTAGE - SMS_STS_PRESENT_POSITION_L];
    }
    else
    {
        Err = 0;
        Voltage = readByte(ID, SMS_STS_PRESENT_VOLTAGE);
        if (Voltage == -1)
        {
            Err = 1;
        }
    }
    return Voltage;
}
/**
  * @brief 读取舵机当前温度  
  * @param  ID: 舵机ID (若ID为-1则从上次FeedBack缓存中读) 
  * @retval 舵机当前的温度值  
  * @note   此函数会尝试从指定ID的舵机读取当前温度信息，并返回温度值。如果ID为-1，则直接从上次成功读取的缓存Mem中返回数据。读取失败时会设置Err=1，成功时Err=0。需要注意的是，返回的温度值是一个整数，表示当前的温度读数，单位通常为摄氏度。 
  */
int ReadTemper(int ID)
{
    int Temper = -1;
    if (ID == -1)
    {
        Temper = Mem[SMS_STS_PRESENT_TEMPERATURE - SMS_STS_PRESENT_POSITION_L];
    }
    else
    {
        Err = 0;
        Temper = readByte(ID, SMS_STS_PRESENT_TEMPERATURE);
        if (Temper == -1)
        {
            Err = 1;
        }
    }
    return Temper;
}
/**
  * @brief  读取舵机当前移动状态  
  * @param  ID: 舵机ID (若ID为-1则从上次FeedBack缓存中读) 
  * @retval 舵机当前的移动状态值 (0-停止，1-移动)  
  * @note   此函数会尝试从指定ID的舵机读取当前  
  */
int ReadMove(int ID)
{
    int Move = -1;
    if (ID == -1)
    {
        Move = Mem[SMS_STS_MOVING - SMS_STS_PRESENT_POSITION_L];
    }
    else
    {
        Err = 0;
        Move = readByte(ID, SMS_STS_MOVING);
        if (Move == -1)
        {
            Err = 1;
        }
    }
    return Move;
}
/**
  * @brief 读取舵机当前电流  
  * @param ID: 舵机ID (若ID为-1则从上次FeedBack缓存中读)  
  * @retval 舵机当前的电流值 (处理了方向位)  
  * @note   此函数会尝试从指定ID的舵机读取当前电流信息，并返回处理后的电流值。如果ID为-1，则直接从上次成功读取的缓存Mem中返回数据。读取失败时会设置Err=1，成功时Err=0。需要注意的是，返回的电流值已经处理了方向位，如果最高位为1，则表示电流为负值，函数会将其转换为对应的负数返回。 
  */
int ReadCurrent(int ID)
{
    int Current = -1;
    if (ID == -1)
    {
        Current = Mem[SMS_STS_PRESENT_CURRENT_H - SMS_STS_PRESENT_POSITION_L];
        Current <<= 8;
        Current |= Mem[SMS_STS_PRESENT_CURRENT_L - SMS_STS_PRESENT_POSITION_L];
    }
    else
    {
        Err = 0;
        Current = readWord(ID, SMS_STS_PRESENT_CURRENT_L);
        if (Current == -1)
        {
            Err = 1;
            return -1;
        }
    }
    if (!Err && Current & (1 << 15))
    {
        Current = -(Current & ~(1 << 15));
    }
    return Current;
}
