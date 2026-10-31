/**
 * @date 2026-10-26
 * @author Liang Haotian 18606604638@163.com
 * @file power_controller.c/h
 * @brief 底盘功率预测、动态功率分配与电流限制
 *
 * @details
 * 基于西交利物浦大学，香港科技大学的开源。基于四轮电机电流与转速建立功率模型，结合裁判系统功率上限、
 * 缓冲能量、超级电容剩余能量及底盘工作模式，动态确定功率预算。
 *
 * 主要功能：
 *  - 功率预测：采用 P = k0*I*ω + k1*ω² + k2*I² + k3 估算单轮功率。
 *  - 预算管理：根据裁判系统与超级电容状态调整底盘总功率上限。
 *  - 功率分配：融合功率占比与速度误差权重，为耗电轮分配功率预算，并将回馈功率计入可用预算。
 *  - 电流限制：通过功率模型反解允许电流，并进行总功率校验与收缩。
 *  - 在线辨识：利用实测功率和电机反馈，通过递推最小二乘法（RLS）更新模型损耗系数 k1、k2。
 *  - 状态反馈：提供限幅前后预测功率、模型参数及预算不可达标志。
 *
 * @note 使用前应调用 Power_Control_Init() 完成初始化。
 * @note 当模型预测的最小总功率仍超过预算时，输出最小功率点电流，并置位 budget_unreachable；此时无法保证预测功率满足上限。
 * 
 * @bug  在小陀螺时行进起步快速转向会超功率（未测是否能被裁判系统检测到）
 *       并且小陀螺时起步会向斜前方偏移，在小陀螺时给一个小一点的平移量会一直偏移，持续超功率
 */
#include "power_controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>
#include "Global_status.h"
#include "referee_system.h"
#include "robot_param.h"
#include "supercup.h"

#define RPM_TO_RAD 0.104719755f
#define CURRENT_LIMIT 16384.0f
#define ERROR_WEIGHT_START 15.0f
#define ERROR_WEIGHT_FULL 20.0f
#define POWER_LIMIT_ABS_MAX 250.0f
#define POWER_LIMIT_ABS_MIN 5.0f
#define REFEREE_INITIAL_POWER 50.0f
#define CAP_VALUE 6.67f
#define CAP_VOLTAGE_MAX 24.0f
#define RLS_LAMBDA 0.9999f
#define RLS_SPEED_SCALE 1000.0f
#define RLS_CURRENT_SCALE 10000.0f

static PowerControlStatus_t status;
static float model_k1 = POWER_CONTROL_K1_INITIAL;
static float model_k2 = POWER_CONTROL_K2_INITIAL;
static float rls_p[2][2];
static float last_referee_power = REFEREE_INITIAL_POWER;

/*********************** 内部调用tool ***********************/
/**
 * @brief 浮点限幅
 */
static float clampf(float value, float lower, float upper)
{
    return fminf(fmaxf(value, lower), upper);
}

/**
 * @brief 根据电流和转速预测单轮功率
 * @note P = k0·I·ω + k1·ω² + k2·I² + k3
 */
static float predict_power(float current, float speed_rpm)
{
    float omega = speed_rpm * RPM_TO_RAD;
    return POWER_CONTROL_K0 * current * omega + model_k1 * omega * omega +
           model_k2 * current * current + POWER_CONTROL_K3_PER_WHEEL;
}


/********************************************* init *****************************************/
/**
 * @brief 功率控制初始化
 */
void Power_Control_Init(void)
{
    memset(&status, 0, sizeof(status));
    memset(rls_p, 0, sizeof(rls_p));
    rls_p[0][0] = rls_p[1][1] = 1.0f;
    model_k1 = POWER_CONTROL_K1_INITIAL;
    model_k2 = POWER_CONTROL_K2_INITIAL;
    last_referee_power = REFEREE_INITIAL_POWER;
    status.k1 = model_k1;
    status.k2 = model_k2;
    status.rls_enabled = (POWER_CONTROL_RLS_ENABLED != 0);
}

/***************************************** PowerControl **************************************/
/**
 * @brief 底盘功率控制
 */
void Chassis_PowerControl(PowerControlWheel_t wheels[POWER_CONTROL_WHEEL_COUNT], float set_max_power)
{
    if (wheels == NULL)
        return;
    int referee_valid = isfinite(set_max_power) && set_max_power > 0.0f;
    if (referee_valid)
        last_referee_power = clampf(set_max_power, 0.0f, 255.0f);
    float referee_power = last_referee_power;// 使用最近一次有效裁判功率
    Supercap_SetPower((uint8_t)clampf(referee_power - 2.0f, 0.0f, 255.0f));

    int cap_online = cap.cap_power_Is_Online > 0 && isfinite(cap.remain_vol) && cap.remain_vol > 0.0f;
    float energy = cap_online ? Cap_GetEnergyPercent(cap.remain_vol) : 0.0f;

    //不同模式下的设置功率上限
    float total = 0.85f * referee_power;
    if (cap_online)
    {
        if (energy <= 0.2f)
            total = 0.75f * referee_power;
        else if (referee_valid && (Global.Cap.mode == FULL || Global.Chassis.mode == CROSS))
            total = (Global.Chassis.mode == CROSS) ? 300.0f : 5.0f * referee_power;
    }
    if (referee_valid)
    {
        //如果裁判系统反馈缓冲能量变少，进一步限制（但考虑到延时等原因，）
        if (Referee_data.Buffer_Energy <= 25)
            total = 0.3f * referee_power;
        else if (Referee_data.Buffer_Energy <= 45)
            total = 0.5f * referee_power;
        else if (Referee_data.Buffer_Energy <= 55)
            total = 0.8f * referee_power;
    }
    total = clampf(total, fminf(POWER_LIMIT_ABS_MIN, referee_power), POWER_LIMIT_ABS_MAX);

    if (cap_online)
        Power_Control_UpdateModel(wheels, cap.Chassis_power);//使用RLS

    Power_Control_Limit(wheels, total);//功率限制
}

/************************************* PowerControlTool ****************************************/
/**
 * @brief 求功率曲线在 [0, |current|] 上的最小功率点电流
 * @note P = k2·I² + k0·ω·I + k1·ω² + k3
 */
static float minimum_current(float current, float speed_rpm)
{
    float magnitude = fabsf(current);
    float direction = (current >= 0.0f) ? 1.0f : -1.0f;
    float b = POWER_CONTROL_K0 * speed_rpm * RPM_TO_RAD * direction;
    return direction * clampf(-b / (2.0f * model_k2), 0.0f, magnitude);
}


/**
 * @brief 将单轮电流限制到给定功率预算内
 * @note P = k2·I² + k0·ω·I + k1·ω² + k3
 */
static float limit_current(float current, float speed_rpm, float allowed_power)
{
    if (predict_power(current, speed_rpm) <= allowed_power)// 当前电流功率已满足预算
        return current;

    float direction = (current >= 0.0f) ? 1.0f : -1.0f;
    float omega = speed_rpm * RPM_TO_RAD;
    float b = POWER_CONTROL_K0 * omega * direction;
    float c = model_k1 * omega * omega + POWER_CONTROL_K3_PER_WHEEL - allowed_power;
    float delta = b * b - 4.0f * model_k2 * c;
    float minimum = fabsf(minimum_current(current, speed_rpm));
    if (delta <= 0.0f)
        return direction * minimum;

    float root;

    if (b >= 0.0f)
    {
        // 当 b >= 0，高正转速时 c 很小，delta 接近 b²，会丢失大量精度
        //分子有理化：(-b + sqrtf(delta)) / (2*k2) = -2c / (b + sqrtf(delta))
        float denominator = b + sqrtf(delta);
        root = (denominator > 0.0f) ? -2.0f * c / denominator : 0.0f;
    }
    else
    {
        root = (-b + sqrtf(delta)) / (2.0f * model_k2);
    }
    return direction * clampf(root, minimum, fabsf(current));
}

/**
 * @brief 四轮总功率限制主函数
 *
 */
void Power_Control_Limit(PowerControlWheel_t wheels[POWER_CONTROL_WHEEL_COUNT], float max_power)
{
    if (wheels == NULL)
        return;
    if (!isfinite(max_power) || max_power < 0.0f)
        max_power = 0.0f;

    float requested_power[POWER_CONTROL_WHEEL_COUNT];
    float error[POWER_CONTROL_WHEEL_COUNT];
    float minimum[POWER_CONTROL_WHEEL_COUNT];
    float sum_power = 0.0f;
    float sum_positive_power = 0.0f;
    float sum_error = 0.0f;
    float allocatable_power = max_power;
    float minimum_power = 0.0f;

    /*功率盘点*/
    for (int i = 0; i < POWER_CONTROL_WHEEL_COUNT; ++i)
    {
        PowerControlWheel_t *wheel = &wheels[i];

        //判断数据是否有效
        if (!isfinite(wheel->speed_rpm) || fabsf(wheel->speed_rpm) > 32768.0f)
        {
            wheel->speed_rpm = 0.0f;
            wheel->current = 0.0f;
        }
        float max_current = isfinite(wheel->max_current) ?
                            clampf(wheel->max_current, 0.0f, CURRENT_LIMIT) : 0.0f;
        wheel->current = isfinite(wheel->current) ?
                         clampf(wheel->current, -max_current, max_current) : 0.0f;
        float target = isfinite(wheel->target_speed_rpm) ?
                       clampf(wheel->target_speed_rpm, -32768.0f, 32768.0f) : wheel->speed_rpm;

        // 计算未限制前的功率信息
        error[i] = fabsf(target - wheel->speed_rpm) * RPM_TO_RAD / WHEEL_RATIO;  // 速度误差(用于权重)
        requested_power[i] = predict_power(wheel->current, wheel->speed_rpm);
        sum_power += requested_power[i];  // 总功率（带符号）
        minimum[i] = minimum_current(wheel->current, wheel->speed_rpm);
        minimum_power += predict_power(minimum[i], wheel->speed_rpm);  // 最小电流下的总功率
        if (requested_power[i] > 0.0f)
        {
            sum_positive_power += requested_power[i];  // 正功率（耗电）总和
            sum_error += error[i]; // 只累加“耗电轮”的误差
        }
        else
        {
            // 回馈功率增加可分配预算
            allocatable_power -= requested_power[i];
        }
    }

    status.power_limit = max_power;
    status.power_before_limit = sum_power;
    status.minimum_power = minimum_power;
    status.budget_unreachable = (minimum_power > max_power);

    /*总功率超限且至少有一个耗电轮，对每个轮子加权分配*/
    if (sum_power > max_power && sum_positive_power > 0.0f)
    {
        // 误差权重置信度
        float confidence = clampf((sum_error - ERROR_WEIGHT_START) /
                                  (ERROR_WEIGHT_FULL - ERROR_WEIGHT_START), 0.0f, 1.0f);
        for (int i = 0; i < POWER_CONTROL_WHEEL_COUNT; ++i)
        {
            if (requested_power[i] <= 0.0f)
                continue;
            float proportion = requested_power[i] / sum_positive_power;  // 功率占比
            float error_weight = (sum_error > 0.0f) ? error[i] / sum_error : proportion;  //误差占比，无误差时退化为功率占比
            float weight = confidence * error_weight + (1.0f - confidence) * proportion;  // 混合权重
            wheels[i].current = limit_current(wheels[i].current, wheels[i].speed_rpm, weight * allocatable_power);
        }
    }

    /*全局预算兜底*/
    float actual_power = 0.0f;
    for (int i = 0; i < POWER_CONTROL_WHEEL_COUNT; ++i)
        actual_power += predict_power(wheels[i].current, wheels[i].speed_rpm);

    if (actual_power > max_power)
    {
        // 设 I = minimum[i] + r * difference，m = minimum[i]、d = difference
        //则P(r) = k_0 * omega * (m + r * d) + k_1 * omega^2 + k_2 * (m + r * d)^2 + k_3
        // 整理得 P(r) = P(m) + (2 * k_2 * m + k_0 * omega) * d * r + k_2 * d^2 * r^2
        float a = 0.0f;
        float b = 0.0f;
        for (int i = 0; i < POWER_CONTROL_WHEEL_COUNT; ++i)
        {
            float difference = wheels[i].current - minimum[i];
            a += model_k2 * difference * difference;
            b += (2.0f * model_k2 * minimum[i] + POWER_CONTROL_K0 * wheels[i].speed_rpm * RPM_TO_RAD) * difference;
        }

        //r = (−b + sqrtf(b^2 + 4 * a * available)) / 2 * a = 2 * available / (b + sqrtf(b^2 + 4 * a * available))
        float ratio = 0.0f;
        if (minimum_power < max_power)
        {
            float available = max_power - minimum_power;
            float denominator = b + sqrtf(fmaxf(b * b + 4.0f * a * available, 0.0f));
            if (denominator > 0.0f)
                ratio = clampf(2.0f * available / denominator, 0.0f, 1.0f) * 0.999999f;
        }

        // 计算收缩后的功率
        actual_power = 0.0f;
        for (int i = 0; i < POWER_CONTROL_WHEEL_COUNT; ++i)
        {
            wheels[i].current = minimum[i] + ratio * (wheels[i].current - minimum[i]);
            actual_power += predict_power(wheels[i].current, wheels[i].speed_rpm);
        }
    }
    status.power_after_limit = actual_power;
}
/**************************************** RSL实时更新 ******************************************/
/**
 * @brief 用实测功率做 RLS
 * @note 功率计上线可使用
 */
void Power_Control_UpdateModel(const PowerControlWheel_t wheels[POWER_CONTROL_WHEEL_COUNT], float measured_power)
{
    if (!status.rls_enabled || wheels == NULL || !isfinite(measured_power) || fabsf(measured_power) <= 5.0f)
        return;

    float samples[2] = {0.0f, 0.0f};
    float effective_power = 0.0f;
    for (int i = 0; i < POWER_CONTROL_WHEEL_COUNT; ++i)
    {
        if (!isfinite(wheels[i].speed_rpm) || fabsf(wheels[i].speed_rpm) > 32768.0f ||
            !isfinite(wheels[i].feedback_current) || fabsf(wheels[i].feedback_current) > CURRENT_LIMIT)
            return;
        float omega = wheels[i].speed_rpm * RPM_TO_RAD;
        float normalized_current = wheels[i].feedback_current / RLS_CURRENT_SCALE;
        float normalized_speed = omega / RLS_SPEED_SCALE;
        samples[0] += normalized_speed * normalized_speed;
        samples[1] += normalized_current * normalized_current;
        effective_power += POWER_CONTROL_K0 * wheels[i].feedback_current * omega;
    }
    float loss = measured_power - effective_power - POWER_CONTROL_K3_PER_WHEEL * POWER_CONTROL_WHEEL_COUNT;
    if (!isfinite(loss) || loss < 0.0f || samples[0] + samples[1] < 1.0e-6f)
        return;

    float px[2] = {
        rls_p[0][0] * samples[0] + rls_p[0][1] * samples[1],
        rls_p[1][0] * samples[0] + rls_p[1][1] * samples[1]
    };
    float denominator = RLS_LAMBDA + samples[0] * px[0] + samples[1] * px[1];
    if (!isfinite(denominator) || denominator <= 0.0f)
        return;
    float gain[2] = {px[0] / denominator, px[1] / denominator};
    float residual = loss - model_k1 * RLS_SPEED_SCALE * RLS_SPEED_SCALE * samples[0] -
                     model_k2 * RLS_CURRENT_SCALE * RLS_CURRENT_SCALE * samples[1];
    float next_k1 = model_k1 + gain[0] * residual / (RLS_SPEED_SCALE * RLS_SPEED_SCALE);
    float next_k2 = model_k2 + gain[1] * residual / (RLS_CURRENT_SCALE * RLS_CURRENT_SCALE);
    if (!isfinite(next_k1) || !isfinite(next_k2) || next_k1 <= 0.0f || next_k2 <= 0.0f)
        return;
    for (int row = 0; row < 2; ++row)
        for (int column = 0; column < 2; ++column)
            rls_p[row][column] = (rls_p[row][column] - gain[row] * px[column]) / RLS_LAMBDA;
    model_k1 = status.k1 = fmaxf(next_k1, 1.0e-9f);
    model_k2 = status.k2 = fmaxf(next_k2, 1.0e-9f);
    ++status.rls_updates;
}

/*************************************** Cap相关 ********************************************/
/**
 * @brief 由电压计算超级电容存储能量
 */
float Cap_GetEnergy(float voltage)
{
    if (!isfinite(voltage) || voltage <= 0.0f)
        return 0.0f;
    return 0.5f * CAP_VALUE * voltage * voltage;
}

/**
 * @brief 由电压计算能量百分比
 */
float Cap_GetEnergyPercent(float voltage)
{
    if (!isfinite(voltage) || voltage <= 0.0f)
        return 0.0f;
    float ratio = voltage / CAP_VOLTAGE_MAX;
    return clampf(ratio * ratio, 0.0f, 1.0f);
}

/************************************* 外部调用函数 **************************************** */
/**
 * @brief 模块内部状态，对外只读
 * @note 外部可用const PowerControlStatus_t *power_status = Power_Control_GetStatus();调用
 */
const PowerControlStatus_t *Power_Control_GetStatus(void)
{
    return &status;
}

/**
 * @brief 外部设置 RLS 使能
 * @note 外部代码调用（可以不调用，在 power_controller.h 直接设置）
 *       非 0 视为启用
 */
void Power_Control_SetRLSEnabled(uint8_t enabled)
{
    status.rls_enabled = (enabled != 0);
}

/**
 * @brief 对外提供单轮预测功率
 */
float Power_Control_GetPredPower(float current, float speed_rpm)
{
    if (!isfinite(current) || !isfinite(speed_rpm))
        return 0.0f;
    return predict_power(current, speed_rpm);
}