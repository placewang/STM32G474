#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef enum
{
    PID_MODE_POSITION = 0,                /* 位置式：返回限幅后的绝对输出 */
    PID_MODE_INCREMENTAL = 1              /* 增量式：返回本周期实际生效的输出增量 */
} PID_Mode_t;

typedef enum
{
    PID_STATUS_OK = 0,                    /* 操作成功 */
    PID_STATUS_INVALID_ARGUMENT = -1,     /* 句柄无效或输入为 NaN/无穷大 */
    PID_STATUS_TRACKING_LIMITED = 1       /* 无扰跟踪受输出或积分限幅约束 */
} PID_Status_t;

typedef struct 
{
    float Kp;
    float Ki;
    float Kd;
    float Ts;

    float out_max;
    float out_min;
    /* 输出最大变化率,单位为“输出单位/秒”<= 0 表示关闭限速 */
    float output_rate_max;
    float integral_max;
    float integral_min;

    float deadband;
    float integral_threshold;
    /* 微分低通系数：0 表示不滤波，越接近 1 滤波越强、响应越慢*/
    float derivative_filter;

    float target_val;
    float actual_val;
    float actual_last;
    float actual_prev;

    float err;
    float err_last;
    float err_prev;

    /* 误差对时间的积分，单位为“误差单位*秒”*/
    float integral;
    /* 测量值负导数的滤波结果；对测量微分可避免设定值阶跃冲击*/
    float derivative;

    float out;
    float last_delta;
    uint32_t initialized;
    uint32_t mode;
    uint32_t first_run;
} PID_Handle_t;

void PID_Init(PID_Handle_t *pid, PID_Mode_t mode, float Ts);
void PID_SetParam(PID_Handle_t *pid, float kp, float ki, float kd);
void PID_SetTarget(PID_Handle_t *pid, float target);
void PID_SetOutputLimit(PID_Handle_t *pid, float max, float min);
void PID_SetOutputRateLimit(PID_Handle_t *pid, float rate_max);
void PID_SetIntegralLimit(PID_Handle_t *pid, float max, float min);
void PID_SetDeadband(PID_Handle_t *pid, float deadband);
void PID_SetIntegralThreshold(PID_Handle_t *pid, float threshold);
void PID_SetDerivativeFilter(PID_Handle_t *pid, float filter);

/*
 * PID 统一计算入口。
 * 位置式返回限幅、限速后的绝对输出。
 * 增量式返回限幅、限速后实际生效的增量；调用方累加该返回值后，
 * 其执行器命令可与 pid->out 保持一致
 * actual_val 非有限数时不污染内部历史状态：位置式保持原输出，
 * 增量式返回 0 增量
 */
float PID_Calculate(PID_Handle_t *pid, float actual_val);

/* 清空运行状态但保留全部配置；下一次计算按冷启动处理 */
void PID_Reset(PID_Handle_t *pid);

/*
 * 按当前测量值和执行器命令重建运行状态，用于手动/自动切换或运行中恢复。
 * 位置式会反算积分状态以尽量实现无扰接管；增量式直接跟踪当前输出
 * 若当前输出越限，或位置式受积分限幅约束而无法精确反算，函数仍完成
 * 最接近的安全初始化，但返回 PID_STATUS_TRACKING_LIMITED
 */
PID_Status_t PID_ResetTracking(PID_Handle_t *pid,
                               float actual_val,
                               float current_output);

#endif
