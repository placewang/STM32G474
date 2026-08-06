#include "pid.h"

#include <math.h>
#include <string.h>

#define PID_EPSILON 1e-12f
#define CLAMP(x, low, high) ((x) > (high) ? (high) : ((x) < (low) ? (low) : (x)))

static inline int pid_isfinitef(float x)
{
    union {float f;uint32_t u;} value;

    /* keil 的 float 为 IEEE-754 binary32。直接检查指数位可避免部分
     * 指数全为 1 表示 NaN 或正负无穷，其余编码均为有限值。
     */
    value.f = x;
    return (value.u & UINT32_C(0x7F800000))
           != UINT32_C(0x7F800000);
}
/* 统一检查空指针和初始化标志，避免各公开接口重复写边界判断*/
static int valid_handle(const PID_Handle_t *pid)
{

    return pid != NULL && pid->initialized != 0U;
}
/* 相对+绝对容差，避免浮点舍入导致边界状态误报。 */
static int nearly_equal(float a, float b)
{
    float scale = fmaxf(1.0f, fmaxf(fabsf(a), fabsf(b)));


    return fabsf(a - b) <= 1e-6f * scale;
}
/* 死区内直接认为误差为零；死区外扣除阈值，避免边界处突然跳变*/
static float apply_deadband(const PID_Handle_t *pid, float err)
{
    if (fabsf(err) <= pid->deadband) {
        return 0.0f;
    }
    return err - copysignf(pid->deadband, err);
}
/* 一阶 IIR 低通：滤波状态保存在句柄中，不能跨控制器实例共享*/
static float filter_derivative(PID_Handle_t *pid, float raw_derivative)
{
    float alpha = pid->derivative_filter;


    pid->derivative = alpha * pid->derivative
                    + (1.0f - alpha) * raw_derivative;
    return pid->derivative;
}

static float apply_output_rate_limit(const PID_Handle_t *pid,
                                     float requested,
                                     float previous)
{
    float max_step;

    if (pid->output_rate_max <= 0.0f) {
        /* 默认关闭限速，保持传统 PID 的即时输出行为。 */
        return requested;
    }

    max_step = pid->output_rate_max * pid->Ts;
    if (!pid_isfinitef(max_step) || max_step <= 0.0f) {
        /* 防止异常配置或浮点溢出导致输出被错误锁死。 */
        return requested;
    }

    return CLAMP(requested, previous - max_step, previous + max_step);
}

void PID_Init(PID_Handle_t *pid, PID_Mode_t mode, float Ts)
{
    if (pid == NULL) {
        return;
    }

    memset(pid, 0, sizeof(*pid));

    /* 未识别的模式回退到位置式，避免进入未定义的计算分支。 */
    pid->mode = (mode == PID_MODE_INCREMENTAL)
              ? PID_MODE_INCREMENTAL
              : PID_MODE_POSITION;
    /* 采样周期非法时使用 1 ms 兜底，保证积分/微分不会除以零。 */
    pid->Ts = (pid_isfinitef(Ts) && Ts > 0.0001f) ? Ts : 0.001f;
	
    pid->out_max = 100.0f;
    pid->out_min = -100.0f;
    pid->output_rate_max = 0.0f;
    /* 积分状态的默认限幅采用与输出相同的量级，避免无界累积。 */
    pid->integral_max = 100.0f;
    pid->integral_min = -100.0f;
    pid->integral_threshold = 1e12f;
    pid->derivative_filter = 0.8f;

    pid->initialized = 1U;
    pid->first_run = 1U;
}

void PID_SetParam(PID_Handle_t *pid, float kp, float ki, float kd)
{
    /* 非有限参数直接拒绝，避免 NaN 沿计算链路扩散 */
    if (!valid_handle(pid) || !pid_isfinitef(kp) || !pid_isfinitef(ki) || !pid_isfinitef(kd)) {
        return;
    }

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

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

void PID_SetOutputLimit(PID_Handle_t *pid, float max, float min)
{
    float temp;

    if (!valid_handle(pid) || !pid_isfinitef(max) || !pid_isfinitef(min)) {
        return;
    }

    if (max < min) {
        /* 调用者参数顺序传反时自动纠正，而不是产生非法区间。 */
        temp = max;
        max = min;
        min = temp;
    }

    pid->out_max = max;
    pid->out_min = min;
    /* 运行中缩小限幅时立即收敛已有输出，避免下一拍突变到更远位置。 */
    pid->out = CLAMP(pid->out, min, max);
}

void PID_SetOutputRateLimit(PID_Handle_t *pid, float rate_max)
{
    /* 非有限值保持原配置；非正值用于显式关闭输出限速。 */
    if (!valid_handle(pid) || !pid_isfinitef(rate_max)) {
        return;
    }
    pid->output_rate_max = rate_max > 0.0f ? rate_max : 0.0f;
}

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
    /* 新限幅立即作用于已有积分状态。 */
    pid->integral = CLAMP(pid->integral, min, max);
}

void PID_SetDeadband(PID_Handle_t *pid, float deadband)
{
    if (!valid_handle(pid) || !pid_isfinitef(deadband)) {
        return;
    }
    pid->deadband = deadband > 0.0f ? deadband : 0.0f;
}

void PID_SetIntegralThreshold(PID_Handle_t *pid, float threshold)
{
    if (!valid_handle(pid) || !pid_isfinitef(threshold)) {
        return;
    }
    /* 非正阈值按“不启用积分分离”处理。 */
    pid->integral_threshold = threshold > 0.0f ? threshold : 1e12f;
}

void PID_SetDerivativeFilter(PID_Handle_t *pid, float filter)
{
    if (!valid_handle(pid) || !pid_isfinitef(filter)) {
        return;
    }
    /* 超范围参数自动夹紧，保证一阶滤波器始终稳定。 */
    pid->derivative_filter = CLAMP(filter, 0.0f, 1.0f);
}
  /* 位置式保持上一次安全输出*/
static float calculate_position(PID_Handle_t *pid, float actual_val)
{
    float filtered_derivative;
    float integral_old = pid->integral;
    float candidate_output;
    float rate_limited_output;
    float integral_delta_output;

    if (!pid_isfinitef(actual_val)) {
        /* 位置式保持上一次安全输出，并且不更新历史误差。 */
        return pid->out;
    }

    pid->actual_val = actual_val;
    pid->err = apply_deadband(pid, pid->target_val - actual_val);

    if (pid->first_run != 0U)
	{
        /* 首次没有可靠的测量历史，D 项置零，避免伪造启动冲击。 */
        filtered_derivative = 0.0f;  }
	else {
        /* 对测量值微分，避免目标值阶跃造成微分冲击。 */
        float raw_derivative = -(actual_val - pid->actual_last) / pid->Ts;
        filtered_derivative = filter_derivative(pid, raw_derivative);
    }

    if (fabsf(pid->Ki) >= PID_EPSILON
        && fabsf(pid->err) < pid->integral_threshold) {
        /* 积分按秒计量；误差过大时暂停积分，防止启动阶段积压。 */
        pid->integral = CLAMP(pid->integral + pid->err * pid->Ts,
                              pid->integral_min, pid->integral_max);
    }

    candidate_output = pid->Kp * pid->err
                     + pid->Ki * pid->integral
                     + pid->Kd * filtered_derivative;

    integral_delta_output = pid->Ki * (pid->integral - integral_old);
    if ((candidate_output > pid->out_max && integral_delta_output > 0.0f)
        || (candidate_output < pid->out_min && integral_delta_output < 0.0f)) {
        /* 输出已饱和且积分仍向饱和方向推动时，撤销本次积分。 */
        pid->integral = integral_old;
        candidate_output -= integral_delta_output;
        integral_delta_output = 0.0f;
    }

    rate_limited_output = apply_output_rate_limit(pid, candidate_output, pid->out);
    if ((rate_limited_output < candidate_output && integral_delta_output > 0.0f)
        || (rate_limited_output > candidate_output && integral_delta_output < 0.0f)) {
        /* 输出变化率受限也视作一种饱和，防止积分在限速期间继续堆积。 */
        pid->integral = integral_old;
        candidate_output -= integral_delta_output;
        rate_limited_output = apply_output_rate_limit(pid, candidate_output,
                                                      pid->out);
    }

    pid->out = CLAMP(rate_limited_output, pid->out_min, pid->out_max);
    pid->err_prev = pid->err_last;
    pid->err_last = pid->err;
    pid->actual_prev = pid->actual_last;
    pid->actual_last = actual_val;
    pid->first_run = 0U;

    return pid->out;
}

static float calculate_incremental(PID_Handle_t *pid, float actual_val)
{
    float old_output;
    float delta_output;
    float applied_delta;
    float derivative_delta = 0.0f;

    if (!pid_isfinitef(actual_val)) {
        /* 无效反馈不能生成增量，否则会破坏调用方维护的执行器命令。 */
        return 0.0f;
    }

    pid->actual_val = actual_val;
    pid->err = apply_deadband(pid, pid->target_val - actual_val);

    if (pid->first_run != 0U) 
	{
        /* 初始化测量历史使首次 D 项为零，但保留 err_last=0，
         * 让首次 P 项产生从内部输出零值出发的合理启动增量。 */
        pid->actual_last = actual_val;
        pid->actual_prev = actual_val; }
	else {
        float previous_derivative = pid->derivative;
        float raw_derivative = -(actual_val - pid->actual_last) / pid->Ts;
        float current_derivative = filter_derivative(pid, raw_derivative);
        derivative_delta = pid->Kd * (current_derivative - previous_derivative);
    }

    delta_output = pid->Kp * (pid->err - pid->err_last)
                 + pid->Ki * pid->Ts * pid->err
                 + derivative_delta;

    old_output = pid->out;
    /* 返回限幅/限速后真正生效的增量，而不是理论计算增量。 */
    delta_output = apply_output_rate_limit(pid, old_output + delta_output,
                                           old_output) - old_output;
    pid->out = CLAMP(old_output + delta_output, pid->out_min, pid->out_max);
    applied_delta = pid->out - old_output;

    pid->err_prev = pid->err_last;
    pid->err_last = pid->err;
    pid->actual_prev = pid->actual_last;
    pid->actual_last = actual_val;
    pid->last_delta = applied_delta;
    pid->first_run = 0U;

    return applied_delta;
}

float PID_Calculate(PID_Handle_t *pid, float actual_val)
{
    if (!valid_handle(pid)) {
        /* 无有效控制器时返回中性值，不尝试访问任何状态 */
        return 0.0f;
    }

    if (pid->mode == PID_MODE_POSITION) {
        return calculate_position(pid, actual_val);
    }
    if (pid->mode == PID_MODE_INCREMENTAL) {
        return calculate_incremental(pid, actual_val);
    }

    /* 结构体被外部误改时保持当前输出，禁止误入其他算法*/
    return pid->out;
}

void PID_Reset(PID_Handle_t *pid)
{
    if (!valid_handle(pid)) {
        /* Reset 对无效句柄保持幂等，不产生副作用 */
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

PID_Status_t PID_ResetTracking(PID_Handle_t *pid,
                               float actual_val,
                               float current_output)
{
    float tracked_output;
    float requested_integral = 0.0f;
    float achievable_output;
    PID_Status_t status = PID_STATUS_OK;

    if (!valid_handle(pid)
        || !pid_isfinitef(actual_val)
        || !pid_isfinitef(current_output)) 
	{
        /* 无效输入时不修改任何状态，交由调用方处理错误。 */
        return PID_STATUS_INVALID_ARGUMENT;
    }

    tracked_output = CLAMP(current_output, pid->out_min, pid->out_max);
    if (!nearly_equal(tracked_output, current_output)) 
	{
        /* 执行器命令本身越限时先夹紧，并通过返回状态通知调用方。 */
        status = PID_STATUS_TRACKING_LIMITED;
    }

    pid->actual_val = actual_val;
    pid->actual_last = actual_val;
    pid->actual_prev = actual_val;
    pid->err = apply_deadband(pid, pid->target_val - actual_val);
    pid->err_last = pid->err;
    pid->err_prev = pid->err;
    pid->derivative = 0.0f;
    pid->out = tracked_output;
    pid->last_delta = 0.0f;

    if (pid->mode == PID_MODE_POSITION && fabsf(pid->Ki) >= PID_EPSILON)
	{
        /* 通过 P/I 反算当前输出；Ki 很小时结果可能超出积分限幅*/
        requested_integral = (tracked_output - pid->Kp * pid->err) / pid->Ki;
        pid->integral = CLAMP(requested_integral, pid->integral_min,
                              pid->integral_max);
        if (!nearly_equal(pid->integral, requested_integral)) 
		{
            status = PID_STATUS_TRACKING_LIMITED;
        }
    }
	else 
	{
        pid->integral = 0.0f;
    }

    if (pid->mode == PID_MODE_POSITION) 
	{
        achievable_output = pid->Kp * pid->err + pid->Ki * pid->integral;
        if (fabsf(achievable_output - tracked_output) > 1e-5f) {
            /* 反算被限幅后无法精确无扰接管，但状态仍保持在安全范围。 */
            status = PID_STATUS_TRACKING_LIMITED;
        }
    }

    pid->first_run = 0U;
    return status;
}
