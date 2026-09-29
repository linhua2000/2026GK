#include "Key.h"                        // 用户头文件，定义按键事件标志位与键号
#include "main.h"                       // HAL：GPIO_InitTypeDef / HAL_GPIO_ReadPin

/*---------------------------------- 宏定义 ----------------------------------*/
// 按键物理状态
#define KEY_PRESSED     1               // 按键按下（四个键都是上拉输入，低电平有效）
#define KEY_UNPRESSED   0               // 按键释放

// 时间参数（单位：Key_Tick 调用次数；本工程每 5ms 调一次，括号里才是毫秒）
#define KEY_TIME_DOUBLE  40            // 双击有效间隔（200ms）
#define KEY_TIME_LONG    400           // 长按判定时间（2000ms）
#define KEY_TIME_REPEAT  20            // 长按重复触发间隔（100ms）

/*---------------------------------- 全局变量 ----------------------------------*/
/* 在 TIM6 中断里写、主循环里读，必须 volatile，否则 -O2 下读可能被提出循环 */
volatile uint8_t Key_Flag[KEY_COUNT];   // 每个按键的事件标志位（由Key_Check读取并清除）

/*---------------------------------- 内部数据 ----------------------------------*/
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} Key_Pin_t;

/* 四个键都是上拉输入，按下 = 低电平 */
static const Key_Pin_t s_key_pin[KEY_COUNT] =
{
    { GPIOC, GPIO_PIN_5 },              /* KEY_1 -> PC5 */
    { GPIOE, GPIO_PIN_7 },              /* KEY_2 -> PE7 */
    { GPIOE, GPIO_PIN_0 },              /* KEY_3 -> PE0 */
    { GPIOE, GPIO_PIN_8 },              /* KEY_4 -> PE8 */
};

/*---------------------------------- 函数实现 ----------------------------------*/
/**
 * @brief   按键硬件初始化
 * @param   无
 * @retval  无
 * @note    这四个脚在 CubeMX 里已经配成上拉输入（见 Core/Src/gpio.c），这里再配一遍
 *          是为了让驱动自包含 —— 将来 gpio.c 被重新生成也不影响本模块。
 */
void Key_Init(void)
{
//    GPIO_InitTypeDef GPIO_InitStructure = {0};
//    uint8_t i;

//    __HAL_RCC_GPIOC_CLK_ENABLE();
//    __HAL_RCC_GPIOE_CLK_ENABLE();

//    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;  // 输入模式
//    GPIO_InitStructure.Pull = GPIO_PULLUP;      // 上拉：按键未按下时引脚为高电平

//    for (i = 0; i < KEY_COUNT; i++)
//    {
//        GPIO_InitStructure.Pin = s_key_pin[i].pin;
//        HAL_GPIO_Init(s_key_pin[i].port, &GPIO_InitStructure);
//    }
}

/**
 * @brief   读取指定按键当前物理状态（原始电平）
 * @param   n: 按键编号 KEY_1 ~ KEY_4
 * @retval  KEY_PRESSED（按下）或 KEY_UNPRESSED（释放）
 */
uint8_t Key_GetState(uint8_t n)
{
    // 上拉输入且按键接地，按下时为0，释放时为1
    if (HAL_GPIO_ReadPin(s_key_pin[n].port, s_key_pin[n].pin) == GPIO_PIN_RESET)
    {
        return KEY_PRESSED;
    }
    return KEY_UNPRESSED;
}

/**
 * @brief   检查指定按键事件是否发生，并清除相应标志（HOLD标志除外）
 * @param   n: 按键编号 KEY_1 ~ KEY_4
 * @param   Flag: 要检查的事件标志（如KEY_DOWN, KEY_UP, KEY_SINGLE等）
 * @retval  1: 事件发生；0: 未发生
 */
uint8_t Key_Check(uint8_t n, uint8_t Flag)
{
    if (Key_Flag[n] & Flag)             // 如果对应标志位被置位
    {
        // 如果是HOLD标志，不清除它（因为HOLD表示长按过程中持续保持）
        // 其他标志在检测后需要清除，避免重复响应
        if (Flag != KEY_HOLD)
        {
            Key_Flag[n] &= ~Flag;       // 清除该标志位
        }
        return 1;                       // 事件发生
    }
    return 0;                           // 无事件
}

/**
 * @brief   按键状态机，需周期性调用（本工程每 5ms 调用一次，挂在 TIM6 中断里）
 *         该函数负责消抖、检测按键按下/释放、长按、双击、重复触发等事件
 *         并将结果记录在Key_Flag中
 */
void Key_Tick(void)
{
    // 静态变量，用于保存状态机的中间数据
    static uint8_t Count;               // 分频计数器，控制采样频率（2次调用采样一次 = 10ms）
    static uint8_t CurrState[KEY_COUNT], PrevState[KEY_COUNT]; // 当前和上一次采样到的按键物理状态
    static uint8_t S[KEY_COUNT];        // 状态机主状态：0=空闲，1=第一次按下，2=第一次释放后等待双击，3=双击确认中，4=长按重复触发
    static uint16_t Time[KEY_COUNT];    // 计时器（递减），用于各种超时判断
    uint8_t i;

    // 每次调用，先递减Time（如果大于0）
    for (i = 0; i < KEY_COUNT; i++)
    {
        if (Time[i] > 0)
        {
            Time[i]--;
        }
    }

    // 计数器自增，用于降低采样频率（去抖动）
    Count++;
    if (Count >= 2)                     // 每2次调用采样一次（调用周期5ms，则采样周期10ms）
    {
        Count = 0;                      // 重置计数器

        for (i = 0; i < KEY_COUNT; i++)
        {
            // 读取当前按键物理状态
            PrevState[i] = CurrState[i];        // 保存上次采样值
            CurrState[i] = Key_GetState(i);     // 获取当前采样值

            // 更新HOLD标志：只要按键处于按下状态，就一直置位HOLD标志
            if (CurrState[i] == KEY_PRESSED)
            {
                Key_Flag[i] |= KEY_HOLD;        // 按下时置位HOLD
            }
            else
            {
                Key_Flag[i] &= ~KEY_HOLD;       // 释放时清除HOLD
            }

            // 检测下降沿（按下瞬间）：当前按下，上次释放 -> 置位DOWN标志
            if (CurrState[i] == KEY_PRESSED && PrevState[i] == KEY_UNPRESSED)
            {
                Key_Flag[i] |= KEY_DOWN;
            }

            // 检测上升沿（释放瞬间）：当前释放，上次按下 -> 置位UP标志
            if (CurrState[i] == KEY_UNPRESSED && PrevState[i] == KEY_PRESSED)
            {
                Key_Flag[i] |= KEY_UP;
            }

            /*------------------- 高级事件状态机（单次、双击、长按、重复触发） -------------------*/
            // 状态机说明：
            // S=0：空闲，等待第一次按下
            // S=1：第一次按下已检测，等待释放或长按超时
            // S=2：第一次释放后，等待双击（第二个按下）或超时判定为单击
            // S=3：双击已确认，等待第二次释放
            // S=4：长按已触发，等待释放，期间重复产生REPEAT事件

            if (S[i] == 0)                  // 空闲状态
            {
                if (CurrState[i] == KEY_PRESSED)    // 检测到按键按下
                {
                    Time[i] = KEY_TIME_LONG;        // 设置长按定时器
                    S[i] = 1;                       // 进入状态1（第一次按下）
                }
            }
            else if (S[i] == 1)             // 第一次按下状态
            {
                if (CurrState[i] == KEY_UNPRESSED)  // 按键释放，可能产生单击或双击
                {
                    Time[i] = KEY_TIME_DOUBLE;      // 设置双击等待定时器
                    S[i] = 2;                       // 进入状态2（等待双击或超时判定单击）
                }
                else if (Time[i] == 0)          // 长按超时（按键一直按下超过KEY_TIME_LONG）
                {
                    Time[i] = KEY_TIME_REPEAT;      // 设置重复触发间隔
                    Key_Flag[i] |= KEY_LONG;        // 置位长按标志
                    S[i] = 4;                       // 进入状态4（长按重复触发模式）
                }
            }
            else if (S[i] == 2)             // 等待双击状态（第一次释放后）
            {
                if (CurrState[i] == KEY_PRESSED)    // 再次按下，产生双击
                {
                    Key_Flag[i] |= KEY_DOUBLE;      // 置位双击标志
                    S[i] = 3;                       // 进入状态3（等待第二次释放）
                }
                else if (Time[i] == 0)          // 等待超时，没有第二次按下，判定为单击
                {
                    Key_Flag[i] |= KEY_SINGLE;      // 置位单击标志
                    S[i] = 0;                       // 回到空闲状态
                }
            }
            else if (S[i] == 3)             // 双击确认后，等待第二次释放
            {
                if (CurrState[i] == KEY_UNPRESSED)  // 第二次释放
                {
                    S[i] = 0;                       // 回到空闲状态
                }
            }
            else if (S[i] == 4)             // 长按重复触发状态
            {
                if (CurrState[i] == KEY_UNPRESSED)  // 释放按键，结束长按模式
                {
                    S[i] = 0;                       // 回到空闲状态
                }
                else if (Time[i] == 0)          // 重复触发定时器超时（长按中每KEY_TIME_REPEAT触发一次）
                {
                    Time[i] = KEY_TIME_REPEAT;      // 重置定时器，准备下次触发
                    Key_Flag[i] |= KEY_REPEAT;      // 置位重复触发标志（可用于连续响应）
                    S[i] = 4;                       // 保持状态4
                }
            }
        }
    }
}
