/**
 * @file pid.h
 * @brief 通用PID控制器 - 支持位置式/增量式、抗积分饱和、输出限幅/限速、微分滤波
 * @note 所有接口均包含参数有效性检查，非有限值（NaN/Inf）被安全拒绝
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>

/**
 * @brief PID工作模式
 */
typedef enum
{
    PID_MODE_POSITION = 0,      /**< 位置式：直接输出限幅后的绝对控制量 */
    PID_MODE_INCREMENTAL = 1    /**< 增量式：输出本拍实际生效的增量，调用方需累加 */
} PID_Mode_t;

/**
 * @brief PID操作状态码
 */
typedef enum
{
    PID_STATUS_OK = 0,                  /**< 操作成功 */
    PID_STATUS_INVALID_ARGUMENT = -1,   /**< 句柄无效或输入包含NaN/Inf */
    PID_STATUS_TRACKING_LIMITED = 1     /**< 无扰跟踪受输出/积分限幅约束，未完全精确 */
} PID_Status_t;

/**
 * @brief PID控制器句柄 - 包含全部配置和运行状态
 * @note 用户应通过接口函数操作，避免直接修改内部字段
 */
typedef struct 
{
    /* ==================== 配置参数 ==================== */
    float Kp;                   /**< 比例增益 */
    float Ki;                   /**< 积分增益 */
    float Kd;                   /**< 微分增益 */
    float Ts;                   /**< 采样周期（秒），>0.0001 */

    float out_max;              /**< 输出上限 */
    float out_min;              /**< 输出下限 */
    float output_rate_max;      /**< 输出最大变化率（单位/秒），<=0关闭限速 */
    float integral_max;         /**< 积分上限 */
    float integral_min;         /**< 积分下限 */

    float deadband;             /**< 死区阈值，误差绝对值小于此值归零 */
    float integral_threshold;   /**< 积分分离阈值，误差超过此值暂停积分 */
    float derivative_filter;    /**< 微分低通系数 [0,1]，0关闭滤波，越接近1滤波越强 */

    /* ==================== 目标与反馈 ==================== */
    float target_val;           /**< 当前目标值（设定值） */
    float actual_val;           /**< 当前反馈值（测量值） */
    float actual_last;          /**< 上一拍反馈值 */
    float actual_prev;          /**< 上两拍反馈值 */

    /* ==================== 误差历史 ==================== */
    float err;                  /**< 当前误差（经死区处理） */
    float err_last;             /**< 上一拍误差 */
    float err_prev;             /**< 上两拍误差 */

    /* ==================== 内部状态 ==================== */
    float integral;             /**< 积分累积值（误差×时间） */
    float derivative;           /**< 滤波后的微分值（测量值微分） */

    float out;                  /**< 当前实际输出值（位置式绝对值/增量式累加基值） */
    float last_delta;           /**< 上一拍实际增量（仅增量式有效） */
    uint32_t initialized;       /**< 初始化标志，1=已初始化 */
    uint32_t mode;              /**< 工作模式，见 PID_Mode_t */
    uint32_t first_run;         /**< 首次运行标志，1=冷启动，微分项置零 */
} PID_Handle_t;

/* ==================== 配置接口 ==================== */

/**
 * @brief 初始化PID控制器
 * @param pid 句柄指针
 * @param mode 工作模式 @ref PID_Mode_t
 * @param Ts 采样周期（秒），非法值自动设为0.001s
 * @note 默认参数：输出±100、积分±100、微分滤波0.8、限速关闭
 */
void PID_Init(PID_Handle_t *pid, PID_Mode_t mode, float Ts);

/**
 * @brief 设置PID增益
 * @param kp 比例增益
 * @param ki 积分增益（接近0时自动清空积分）
 * @param kd 微分增益
 * @note 非有限参数被拒绝，防止NaN扩散
 */
void PID_SetParam(PID_Handle_t *pid, float kp, float ki, float kd);

/**
 * @brief 设置目标值（设定值）
 * @param target 目标值
 */
void PID_SetTarget(PID_Handle_t *pid, float target);

/**
 * @brief 设置输出限幅
 * @param max 上限
 * @param min 下限
 * @note 参数顺序自动纠正；限幅立即作用于当前输出
 */
void PID_SetOutputLimit(PID_Handle_t *pid, float max, float min);

/**
 * @brief 设置输出限速
 * @param rate_max 最大变化率（单位/秒），<=0关闭限速
 */
void PID_SetOutputRateLimit(PID_Handle_t *pid, float rate_max);

/**
 * @brief 设置积分限幅
 * @param max 上限
 * @param min 下限
 * @note 限幅立即作用于当前积分值
 */
void PID_SetIntegralLimit(PID_Handle_t *pid, float max, float min);

/**
 * @brief 设置死区阈值
 * @param deadband 非负阈值，内部强制≥0
 * @note 死区内误差归零，消除稳态微振
 */
void PID_SetDeadband(PID_Handle_t *pid, float deadband);

/**
 * @brief 设置积分分离阈值
 * @param threshold 误差绝对值小于此值才允许积分，<=0关闭积分分离
 * @note 防止启动阶段误差饱和导致积分深度饱和（windup）
 */
void PID_SetIntegralThreshold(PID_Handle_t *pid, float threshold);

/**
 * @brief 设置微分滤波系数
 * @param filter [0,1]，0关闭滤波，越接近1滤波越强
 * @note 超范围参数自动夹紧
 */
void PID_SetDerivativeFilter(PID_Handle_t *pid, float filter);

/* ==================== 核心计算接口 ==================== */

/**
 * @brief PID统一计算入口
 * @param pid 句柄
 * @param actual_val 当前反馈值（测量值）
 * @return 位置式：限幅/限速后的绝对输出值
 *         增量式：限幅/限速后实际生效的输出增量
 * @note 位置式：返回值可直接作为执行器命令
 *       增量式：调用方需累加返回值：cmd += PID_Calculate(pid, actual)
 * @warning actual_val为非有限数时不污染状态：
 *          位置式保持原输出，增量式返回0增量
 */
float PID_Calculate(PID_Handle_t *pid, float actual_val);

/* ==================== 状态管理接口 ==================== */

/**
 * @brief 完全复位 - 清空全部运行状态，保留配置
 * @note 下一次计算按冷启动处理（微分项不产生启动冲击）
 */
void PID_Reset(PID_Handle_t *pid);

/**
 * @brief 无扰跟踪 - 按当前测量值和执行器命令重建控制器状态
 * @param pid 句柄
 * @param actual_val 当前测量值
 * @param current_output 当前执行器命令（期望跟踪的输出）
 * @return @ref PID_Status_t
 * @details 位置式模式：反算积分项，使P+I输出逼近current_output
 *          增量式模式：直接设置out=current_output，积分清零
 * @note 用于手动/自动切换或运行中恢复，尽量减少对执行器的冲击
 * @warning 若当前输出越限，或位置式反算受积分限幅约束，
 *          函数仍完成最接近的安全初始化，但返回 PID_STATUS_TRACKING_LIMITED
 */
PID_Status_t PID_ResetTracking(PID_Handle_t *pid,
                               float actual_val,
                               float current_output);

#endif /* PID_H */

