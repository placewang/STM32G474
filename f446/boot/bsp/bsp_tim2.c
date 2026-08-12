
#include "tim.h"


typedef struct {
    uint32_t start_tick;   // 启动时刻的 TIM2 计数值
    uint32_t timeout_us;   // 超时时间（微秒）
    uint8_t is_running;    // 是否正在计时（1=计时中，0=停止/已到期）
    uint8_t expired;       // 是否已到期（1=已到期，需要查询后手动通过 start 复位）
} timer_t;

timer_t  TIM2_T1, TIM2_T2;

// 读取当前微秒时间戳
uint32_t TIM2_GetUs(void) {
    return __HAL_TIM_GET_COUNTER(&htim2);
}

// 启动（或重启）定时器 - 重置到期标志，重新开始计时
void bsp_tim2Start(timer_t *timer, uint32_t timeout_us) {
    timer->start_tick = TIM2_GetUs();
    timer->timeout_us = timeout_us;
    timer->is_running = 1;
    timer->expired = 0;      // 清除到期标志
}

// 查询定时器是否已到期（到期后保持 True，除非重新 start）
// 返回值：1=已到期，0=未到期或未运行
uint8_t bsp_tim2GetSta(timer_t *timer) {
    if (!timer->is_running) {
        // 如果未运行，返回之前保存的 expired 标志（用于保持到期状态）
        return timer->expired;
    }

    uint32_t now = TIM2_GetUs();
    uint32_t elapsed = now - timer->start_tick;

    if (elapsed >= timer->timeout_us) {
        timer->is_running = 0;   // 停止计时
//        timer->expired = 1;      // 标记为已到期
        return 1;
    }
    return 0;
}

// 可选：读取剩余时间（未到期时返回剩余微秒，到期或未运行时返回0）
uint32_t bsp_tim2_remaining(timer_t *timer) {
    if (!timer->is_running) return 0;
    uint32_t now = TIM2_GetUs();
    uint32_t elapsed = now - timer->start_tick;
    if (elapsed >= timer->timeout_us) return 0;
    return timer->timeout_us - elapsed;
}

