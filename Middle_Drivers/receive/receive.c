#include "receive.h"
#include "usart.h"      /* huart4 */

/* 视觉串口: UART4 (PC10 TX / PC11 RX) */
#define VISION_ACK_BYTE 0x01   /* 收到一帧后回给视觉的确认字节 */

VisionData_t vision_data = {0};

static uint8_t  rx_byte = 0;             /* 中断接收到的单字节 */
static uint8_t  rx_buf[FRAME_MAX_LEN];   /* 接收缓冲, 取最大帧长(抓取 6 字节) */
static uint8_t  rx_idx = 0;              /* 已填充字节数 */
static uint8_t  rx_len = 0;              /* 当前帧期望长度, 由帧头决定 */
static uint32_t rx_last_tick = 0;        /* 最后一字节的 tick */

/* 根据帧头返回帧长, 非帧头返回 0 */
static uint8_t frame_len(uint8_t head)
{
    switch (head) {
        case FRAME_QR_HEAD:    return FRAME_QR_LEN;
        case FRAME_TRACK_HEAD: return FRAME_TRACK_LEN;
        case FRAME_TURN_HEAD:  return FRAME_TURN_LEN;
        case FRAME_GRAB_HEAD:  return FRAME_GRAB_LEN;
        default:               return 0;
    }
}

/* 根据帧头返回帧尾 */
static uint8_t frame_tail(uint8_t head)
{
    switch (head) {
        case FRAME_QR_HEAD:    return FRAME_QR_TAIL;
        case FRAME_TRACK_HEAD: return FRAME_TRACK_TAIL;
        case FRAME_TURN_HEAD:  return FRAME_TURN_TAIL;
        case FRAME_GRAB_HEAD:  return FRAME_GRAB_TAIL;
        default:               return 0;
    }
}

/* 收满一帧且帧尾校验通过后, 把数据写入 vision_data */
static void dispatch_frame(void)
{
    switch (rx_buf[0]) {
        case FRAME_QR_HEAD:
            vision_data.qr_x = (int8_t)rx_buf[1];
            vision_data.qr_y = (int8_t)rx_buf[2];
            vision_data.qr_z = (int8_t)rx_buf[3];
            vision_data.qr_flag = 1;
            break;
        case FRAME_TRACK_HEAD:
            vision_data.track_x = (int8_t)rx_buf[1];
            vision_data.track_y = (int8_t)rx_buf[2];
            vision_data.track_flag = 1;
            break;
        case FRAME_TURN_HEAD:
            vision_data.turn_flag = 1;
            break;
        case FRAME_GRAB_HEAD:
            /* 小端 int16: 低字节在前, 高字节在后 */
            vision_data.grab_x = (int16_t)(rx_buf[1] | ((uint16_t)rx_buf[2] << 8));
            vision_data.grab_y = (int16_t)(rx_buf[3] | ((uint16_t)rx_buf[4] << 8));
            vision_data.grab_flag = 1;
            break;
        default:
            break;
    }

    /* 帧尾校验通过, 数据已写入 vision_data, OLED 显示由主循环处理 */
}

/* 字节级状态机: 空闲找帧头 -> 按帧长填充 -> 收满校验帧尾 */
static void feed_byte(uint8_t ch)
{
    /* 超时保护: 上一字节距离太久, 之前的半截包作废 */
    uint32_t now = HAL_GetTick();
    if (rx_idx > 0 && (now - rx_last_tick) > VISION_RX_TIMEOUT) {
        rx_idx = 0;
    }
    rx_last_tick = now;

    /* 状态1: 空闲, 找帧头 */
    if (rx_idx == 0) {
        uint8_t len = frame_len(ch);
        if (len != 0) {
            rx_buf[0] = ch;
            rx_len = len;
            rx_idx = 1;
        }
        return;    /* 非帧头一律丢弃 */
    }

    /* 状态2: 填充包体 */
    rx_buf[rx_idx++] = ch;
    if (rx_idx < rx_len) {
        return;    /* 还没收满, 继续等下一字节 */
    }

    /* 状态3: 收满, 校验帧尾 */
    if (rx_buf[rx_len - 1] == frame_tail(rx_buf[0])) {
        dispatch_frame();
        rx_idx = 0;
        return;
    }

    /* 帧尾错误 = 同步丢失, 在 [1..rx_len-1] 里重找帧头,
     * 否则可能漏掉下一个真正的帧头 */
    uint8_t k;
    for (k = 1; k < rx_len; k++) {
        if (frame_len(rx_buf[k]) != 0) break;
    }

    if (k < rx_len) {
        /* 在中间找到了新帧头, 把它及其后的字节搬到开头 */
        uint8_t n = rx_len - k;
        for (uint8_t i = 0; i < n; i++) {
            rx_buf[i] = rx_buf[k + i];
        }
        rx_idx = n;
        rx_len = frame_len(rx_buf[0]);
        if (rx_idx >= rx_len) {
            /* 搬移后字节数已够一帧, 但帧尾未校验过, 直接丢弃重建 */
            rx_idx = 0;
        }
    } else {
        /* 整个缓冲里没有帧头, 彻底重来 */
        rx_idx = 0;
    }
}

/* 启动视觉串口接收中断 */
void Vision_UART_Init(void)
{
    HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
    HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
}

/* UART4 中断回调: 喂入收到的字节并重新武装接收中断。
 * 由 main.c 的 HAL_UART_RxCpltCallback 在 huart==&huart4 时调用。 */
void Vision_UART_RxCpltCallback(void)
{
    feed_byte(rx_byte);
    HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
}

/* 回一个字节 0x01 给视觉, 表示收到一帧 (测试用) */
void Vision_Send_Ack(void)
{
    uint8_t ack = VISION_ACK_BYTE;
    HAL_UART_Transmit(&huart4, &ack, 1, 10);
}

/* 发一帧 D8 cmd 8D 给视觉(帧头尾复用抓取帧) */
static void vision_send_grab(uint8_t cmd)
{
    uint8_t frame[3] = {FRAME_GRAB_HEAD, cmd, FRAME_GRAB_TAIL};
    HAL_UART_Transmit(&huart4, frame, 3, 10);
}

/* 抓完球转身后, 通知视觉切桶识别: D8 01 8D */
void Vision_Send_Switch_Bucket(void)
{
    vision_send_grab(0x01);
}

/* 放完球, 通知视觉完成: D8 02 8D */
void Vision_Send_Release_Done(void)
{
    vision_send_grab(0x02);
}

/* 注意: UART4 的中断入口 UART4_IRQHandler 已在 stm32f4xx_it.c 中定义,
 * 这里无需重复定义。 */



