/**
 * @file pid.c
 * @brief 通用PID控制器实现 - 支持位置式/增量式、积分分离、输出限幅、限速、微分滤波
 * @note 所有配置接口均包含参数有效性检查，避免NaN/Inf污染控制状态
 */

#include "pid.h"
#include <math.h>
#include <string.h>

#define PID_EPSILON 1e-12f          /* 浮点零判断阈值，用于积分使能判断 */
#define PID_REL_TOL 1e-6f           /* 相对容差，用于浮点比较 */

/**
 * @brief 三目限幅宏 - 将x限制在[low, high]区间
 * @warning 参数会多次求值，禁止传入带副作用的表达式
 */
#define CLAMP(x, low, high) ((x) > (high) ? (high) : ((x) < (low) ? (low) : (x)))

/**
 * @brief 判断单精度浮点数是否为有限值
 * @param x 待检查浮点数
 * @return 1=有限值，0=±Inf或NaN
 * @note 利用IEEE754指数位全1判定特殊值，避免math库依赖
 */
static inline int pid_isfinitef(float x)
{
    union {float f; uint32_t u;} value;
    value.f = x;
    return (value.u & UINT32_C(0x7F800000)) != UINT32_C(0x7F800000);
}

/**
 * @brief 验证句柄有效性
 * @return 1=有效，0=空指针或未初始化
 */
static int valid_handle(const PID_Handle_t *pid)
{
    return pid != NULL && pid->initialized != 0U;
}

/**
 * @brief 近似相等判断 - 混合相对/绝对容差
 * @details 自动适应数值量级：大数比相对误差，小数用绝对容差
 * @note 用于无扰切换时的状态一致性校验
 */
static int nearly_equal(float a, float b)
{
    float scale = fmaxf(1.0f, fmaxf(fabsf(a), fabsf(b)));
    return fabsf(a - b) <= PID_REL_TOL * scale;
}

/**
 * @brief 死区处理 - 消除小误差抖动
 * @details 误差在死区内归零；超出后扣除死区阈值，避免边界突变
 * @return 处理后的误差值
 */
static float apply_deadband(const PID_Handle_t *pid, float err)
{
    if (fabsf(err) <= pid->deadband) {
        return 0.0f;
    }
    return err - copysignf(pid->deadband, err);
}

/**
 * @brief 一阶IIR低通滤波 - 用于微分项降噪
 * @param pid 句柄（含滤波器状态 derivative）
 * @param raw_derivative 原始微分值
 * @return 滤波后微分值
 * @note 系数alpha越接近1滤波越强，0关闭滤波
 */
static float filter_derivative(PID_Handle_t *pid, float raw_derivative)
{
    float alpha = pid->derivative_filter;
    pid->derivative = alpha * pid->derivative + (1.0f - alpha) * raw_derivative;
    return pid->derivative;
}

/**
 * @brief 输出限速 - 限制每拍输出变化率
 * @param pid 句柄（含Ts采样周期）
 * @param requested 本拍理论输出
 * @param previous 上一拍实际输出
 * @return 限速后的输出值
 * @note output_rate_max<=0时关闭限速，返回requested原值
 */
static float apply_output_rate_limit(const PID_Handle_t *pid,
                                     float requested,
                                     float previous)
{
    float max_step;

    if (pid->output_rate_max <= 0.0f) {
        return requested;  /* 限速关闭 */
    }

    max_step = pid->output_rate_max * pid->Ts;
    if (!pid_isfinitef(max_step) || max_step <= 0.0f) {
        return requested;  /* 异常配置兜底 */
    }

    return CLAMP(requested, previous - max_step, previous + max_step);
}

/* ==================== 公开接口 ==================== */

/**
 * @brief PID控制器初始化
 * @param pid 句柄指针
 * @param mode PID_MODE_POSITION（位置式）或 PID_MODE_INCREMENTAL（增量式）
 * @param Ts 采样周期（秒），非法值时自动设为0.001s
 * @note 初始化默认参数：输出±100，积分限幅±100，微分滤波系数0.8
 */
void PID_Init(PID_Handle_t *pid, PID_Mode_t mode, float Ts)
{
    if (pid == NULL) {
        return;
    }

    memset(pid, 0, sizeof(*pid));

    pid->mode = (mode == PID_MODE_INCREMENTAL) ? PID_MODE_INCREMENTAL : PID_MODE_POSITION;
    pid->Ts = (pid_isfinitef(Ts) && Ts > 0.0001f) ? Ts : 0.001f;

    pid->out_max = 100.0f;
    pid->out_min = -100.0f;
    pid->output_rate_max = 0.0f;      /* 默认关闭限速 */
    pid->integral_max = 100.0f;       /* 与输出限幅同量级 */
    pid->integral_min = -100.0f;
    pid->integral_threshold = 1e12f;  /* 默认几乎无限大，积分分离不生效 */
    pid->derivative_filter = 0.8f;    /* 轻度滤波 */

    pid->initialized = 1U;
    pid->first_run = 1U;
}

/**
 * @brief 设置PID增益
 * @param kp 比例增益
 * @param ki 积分增益（接近0时自动清空积分）
 * @param kd 微分增益
 * @note 非有限值参数被直接拒绝，避免NaN扩散
 */
void PID_SetParam(PID_Handle_t *pid, float kp, float ki, float kd)
{
    if (!valid_handle(pid) || !pid_isfinitef(kp) || !pid_isfinitef(ki) || !pid_isfinitef(kd)) {
        return;
    }

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    /* Ki接近0时积分失效，清空累积状态避免死区积分残留 */
    if (fabsf(ki) < PID_EPSILON) {
        pid->integral = 0.0f;
    }
}

void PID_SetTarget(PID_Handle_t *pid, float target)
{
    if (!valid_handle(pid) || !pid_isfinitef(target)) {
        return;
    }
    pid->target_val = target;
}

/**
 * @brief 设置输出限幅 [min, max]
 * @note 参数顺序自动纠正；限幅缩小立即作用于当前输出
 */
void PID_SetOutputLimit(PID_Handle_t *pid, float max, float min)
{
    float temp;

    if (!valid_handle(pid) || !pid_isfinitef(max) || !pid_isfinitef(min)) {
        return;
    }

    if (max < min) {  /* 自动纠正调用顺序 */
        temp = max;
        max = min;
        min = temp;
    }

    pid->out_max = max;
    pid->out_min = min;
    pid->out = CLAMP(pid->out, min, max);  /* 立即收敛现有输出 */
}

/**
 * @brief 设置输出限速
 * @param rate_max 最大变化率（单位/秒），<=0时关闭限速
 */
void PID_SetOutputRateLimit(PID_Handle_t *pid, float rate_max)
{
    if (!valid_handle(pid) || !pid_isfinitef(rate_max)) {
        return;
    }
    pid->output_rate_max = (rate_max > 0.0f) ? rate_max : 0.0f;
}

/**
 * @brief 设置积分限幅
 * @note 立即作用于当前积分状态
 */
void PID_SetIntegralLimit(PID_Handle_t *pid, float max, float min)
{
    float temp;

    if (!valid_handle(pid) || !pid_isfinitef(max) || !pid_isfinitef(min)) {
        return;
    }

    if (max < min) {
        temp = max;
        max = min;
        min = temp;
    }

    pid->integral_max = max;
    pid->integral_min = min;
    pid->integral = CLAMP(pid->integral, min, max);
}

/**
 * @brief 设置死区
 * @param deadband 非负值，内部强制>=0
 * @note 死区内误差归零，有效消除稳态微振
 */
void PID_SetDeadband(PID_Handle_t *pid, float deadband)
{
    if (!valid_handle(pid) || !pid_isfinitef(deadband)) {
        return;
    }
    pid->deadband = (deadband > 0.0f) ? deadband : 0.0f;
}

/**
 * @brief 设置积分分离阈值
 * @param threshold 误差绝对值小于此值时才允许积分，<=0时关闭积分分离
 * @note 防止启动阶段误差饱和导致积分深度饱和（windup）
 */
void PID_SetIntegralThreshold(PID_Handle_t *pid, float threshold)
{
    if (!valid_handle(pid) || !pid_isfinitef(threshold)) {
        return;
    }
    pid->integral_threshold = (threshold > 0.0f) ? threshold : 1e12f;
}

/**
 * @brief 设置微分滤波系数
 * @param filter [0,1]，0关闭滤波，越接近1滤波越强
 * @note 超范围参数自动夹紧，保证滤波器稳定
 */
void PID_SetDerivativeFilter(PID_Handle_t *pid, float filter)
{
    if (!valid_handle(pid) || !pid_isfinitef(filter)) {
        return;
    }
    pid->derivative_filter = CLAMP(filter, 0.0f, 1.0f);
}

/* ==================== 核心算法 ==================== */

/**
 * @brief 位置式PID计算（内部函数）
 * @param pid 句柄
 * @param actual_val 当前反馈值
 * @return 限幅/限速后的绝对输出值
 * @details 算法流程：
 *          1. 误差=目标-反馈，经死区处理
 *          2. 对反馈微分（非目标微分），避免设定值阶跃冲击
 *          3. 微分经一阶低通滤波
 *          4. 误差小于阈值时允许积分累积（积分分离）
 *          5. 输出饱和检测：积分继续增大饱和方向时撤销本次积分（抗积分饱和）
 *          6. 输出限速
 *          7. 输出限幅
 * @note invalid actual_val时保持上次输出，不污染历史状态
 */
static float calculate_position(PID_Handle_t *pid, float actual_val)
{
    float filtered_derivative;
    float integral_old = pid->integral;
    float candidate_output;
    float rate_limited_output;
    float integral_delta_output;

    /* 无效反馈：保持原有输出，不更新任何状态 */
    if (!pid_isfinitef(actual_val)) {
        return pid->out;
    }

    pid->actual_val = actual_val;
    pid->err = apply_deadband(pid, pid->target_val - actual_val);

    /* 首次运行：微分置零，避免启动冲击 */
    if (pid->first_run != 0U) {
        filtered_derivative = 0.0f;
    } else {
        /* 对测量值微分：d(actual)/dt，负号后为 -d(actual)/dt */
        float raw_derivative = -(actual_val - pid->actual_last) / pid->Ts;
        filtered_derivative = filter_derivative(pid, raw_derivative);
    }

    /* 积分分离：仅当误差小于阈值且Ki有效时累积 */
    if (fabsf(pid->Ki) >= PID_EPSILON && fabsf(pid->err) < pid->integral_threshold) {
        pid->integral = CLAMP(pid->integral + pid->err * pid->Ts,
                              pid->integral_min, pid->integral_max);
    }

    /* 计算理论输出 */
    candidate_output = pid->Kp * pid->err
                     + pid->Ki * pid->integral
                     + pid->Kd * filtered_derivative;

    /* 抗积分饱和（Anti-Windup）：
     * 若输出已饱和且积分增量朝向饱和方向推进，则撤销本次积分累积 */
    integral_delta_output = pid->Ki * (pid->integral - integral_old);
    if ((candidate_output > pid->out_max && integral_delta_output > 0.0f)
        || (candidate_output < pid->out_min && integral_delta_output < 0.0f)) {
        pid->integral = integral_old;
        candidate_output -= integral_delta_output;
        integral_delta_output = 0.0f;
    }

    /* 输出限速 */
    rate_limited_output = apply_output_rate_limit(pid, candidate_output, pid->out);

    /* 限速也视为一种饱和约束：若限速导致输出无法达到理论值，则撤销积分 */
    if ((rate_limited_output < candidate_output && integral_delta_output > 0.0f)
        || (rate_limited_output > candidate_output && integral_delta_output < 0.0f)) {
        pid->integral = integral_old;
        candidate_output -= integral_delta_output;
        rate_limited_output = apply_output_rate_limit(pid, candidate_output, pid->out);
    }

    /* 最终输出限幅 */
    pid->out = CLAMP(rate_limited_output, pid->out_min, pid->out_max);

    /* 更新历史状态 */
    pid->err_prev = pid->err_last;
    pid->err_last = pid->err;
    pid->actual_prev = pid->actual_last;
    pid->actual_last = actual_val;
    pid->first_run = 0U;

    return pid->out;
}

/**
 * @brief 增量式PID计算（内部函数）
 * @param pid 句柄
 * @param actual_val 当前反馈值
 * @return 本拍实际生效的输出增量
 * @details 增量式输出 Δu = Kp*(e_k-e_{k-1}) + Ki*Ts*e_k + Kd*(ΔD)
 *          调用方需自行累加：u_k = u_{k-1} + Δu
 * @note invalid actual_val时返回0增量，不污染状态
 */
static float calculate_incremental(PID_Handle_t *pid, float actual_val)
{
    float old_output;
    float delta_output;
    float applied_delta;
    float derivative_delta = 0.0f;

    if (!pid_isfinitef(actual_val)) {
        return 0.0f;  /* 无效反馈：零增量，保持执行器当前位置 */
    }

    pid->actual_val = actual_val;
    pid->err = apply_deadband(pid, pid->target_val - actual_val);

    if (pid->first_run != 0U) {
        /* 首次运行：初始化测量历史，微分增量为零 */
        pid->actual_last = actual_val;
        pid->actual_prev = actual_val;
    } else {
        float previous_derivative = pid->derivative;
        float raw_derivative = -(actual_val - pid->actual_last) / pid->Ts;
        float current_derivative = filter_derivative(pid, raw_derivative);
        /* 微分增量：Kd * (D_k - D_{k-1}) */
        derivative_delta = pid->Kd * (current_derivative - previous_derivative);
    }

    /* 理论增量：P增量 + I增量 + D增量 */
    delta_output = pid->Kp * (pid->err - pid->err_last)
                 + pid->Ki * pid->Ts * pid->err
                 + derivative_delta;

    /* 应用限速：计算限速后实际能输出的增量 */
    old_output = pid->out;
    delta_output = apply_output_rate_limit(pid, old_output + delta_output, old_output) - old_output;

    /* 应用输出限幅，得到实际生效增量 */
    pid->out = CLAMP(old_output + delta_output, pid->out_min, pid->out_max);
    applied_delta = pid->out - old_output;

    /* 更新历史 */
    pid->err_prev = pid->err_last;
    pid->err_last = pid->err;
    pid->actual_prev = pid->actual_last;
    pid->actual_last = actual_val;
    pid->last_delta = applied_delta;
    pid->first_run = 0U;

    return applied_delta;
}

/**
 * @brief PID统一计算入口
 * @param pid 句柄
 * @param actual_val 当前反馈值
 * @return 位置式返回绝对输出，增量式返回增量
 * @note 位置式：调用方直接使用返回值作为执行器命令
 *       增量式：调用方累加返回值：cmd += PID_Calculate()
 */
float PID_Calculate(PID_Handle_t *pid, float actual_val)
{
    if (!valid_handle(pid)) {
        return 0.0f;
    }

    if (pid->mode == PID_MODE_POSITION) {
        return calculate_position(pid, actual_val);
    }

    if (pid->mode == PID_MODE_INCREMENTAL) {
        return calculate_incremental(pid, actual_val);
    }

    /* 未知模式兜底：保持当前输出不变 */
    return pid->out;
}

/**
 * @brief 完全复位 - 清空所有运行状态，保留配置
 * @note 下次计算按冷启动处理（first_run=1），微分项不产生启动冲击
 */
void PID_Reset(PID_Handle_t *pid)
{
    if (!valid_handle(pid)) {
        return;
    }

    pid->actual_val = 0.0f;
    pid->actual_last = 0.0f;
    pid->actual_prev = 0.0f;
    pid->err = 0.0f;
    pid->err_last = 0.0f;
    pid->err_prev = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->out = 0.0f;
    pid->last_delta = 0.0f;
    pid->first_run = 1U;
}

/**
 * @brief 无扰跟踪 - 按当前测量值和执行器命令重建控制器状态
 * @param pid 句柄
 * @param actual_val 当前测量值
 * @param current_output 当前执行器命令（期望跟踪的输出值）
 * @return PID_STATUS_OK: 完全无扰跟踪成功
 *         PID_STATUS_TRACKING_LIMITED: 输出被限幅或积分反算受限
 *         PID_STATUS_INVALID_ARGUMENT: 参数无效，状态未改变
 * @details 位置式模式：反算积分项，使 P+I 输出逼近 current_output
 *          增量式模式：直接设置 out=current_output，积分置零
 * @note 用于手动/自动切换或运行中恢复，尽量减少冲击
 */
PID_Status_t PID_ResetTracking(PID_Handle_t *pid,
                               float actual_val,
                               float current_output)
{
    float tracked_output;
    float requested_integral = 0.0f;
    float achievable_output;
    PID_Status_t status = PID_STATUS_OK;

    /* 参数有效性检查 */
    if (!valid_handle(pid) || !pid_isfinitef(actual_val) || !pid_isfinitef(current_output)) {
        return PID_STATUS_INVALID_ARGUMENT;
    }

    /* 执行器命令限幅夹紧 */
    tracked_output = CLAMP(current_output, pid->out_min, pid->out_max);
    if (!nearly_equal(tracked_output, current_output)) {
        status = PID_STATUS_TRACKING_LIMITED;  /* 命令本身越限 */
    }

    /* 初始化反馈和历史状态 */
    pid->actual_val = actual_val;
    pid->actual_last = actual_val;
    pid->actual_prev = actual_val;
    pid->err = apply_deadband(pid, pid->target_val - actual_val);
    pid->err_last = pid->err;
    pid->err_prev = pid->err;
    pid->derivative = 0.0f;
    pid->out = tracked_output;
    pid->last_delta = 0.0f;

    /* 位置式：反算积分项以实现无扰 */
    if (pid->mode == PID_MODE_POSITION && fabsf(pid->Ki) >= PID_EPSILON) {
        requested_integral = (tracked_output - pid->Kp * pid->err) / pid->Ki;
        pid->integral = CLAMP(requested_integral, pid->integral_min, pid->integral_max);
        if (!nearly_equal(pid->integral, requested_integral)) {
            status = PID_STATUS_TRACKING_LIMITED;  /* 积分被限幅 */
        }
    } else {
        pid->integral = 0.0f;  /* 增量式或Ki=0时积分清零 */
    }

    /* 验证反算精度 */
    if (pid->mode == PID_MODE_POSITION) {
        achievable_output = pid->Kp * pid->err + pid->Ki * pid->integral;
        if (fabsf(achievable_output - tracked_output) > 1e-5f) {
            status = PID_STATUS_TRACKING_LIMITED;  /* 反算精度不足 */
        }
    }

    pid->first_run = 0U;
    return status;
}

