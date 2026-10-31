#ifndef POWER_CONTROLLER_H
#define POWER_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POWER_CONTROL_WHEEL_COUNT 4

/* M3508 模型: omega = rotor RPM * 2*pi/60 (rad/s).
 * P = k0 * I * omega + k1 * omega^2 + k2 * I^2 + k3_per_wheel. */
#define POWER_CONTROL_K0 1.9233e-05f
#define POWER_CONTROL_K1_INITIAL 1.9202e-05f
#define POWER_CONTROL_K2_INITIAL 2.8908e-07f
#define POWER_CONTROL_K3_PER_WHEEL 1.15f
/*目前仅当cap.Chassis_power单独测量 四个 驱动轮时才可启用。*/
#ifndef POWER_CONTROL_RLS_ENABLED
#define POWER_CONTROL_RLS_ENABLED 1          /*非0启用 RLS*/
#endif

typedef struct
{
    float current;             /* In：转速PID输出；out：限制CAN电流。 */
    float speed_rpm;           /* 转子转速 RPM. */
    float target_speed_rpm;    /* 电机反馈的转子转速 RPM. */
    float max_current;         /* 速度PID lim_out, . */
    float feedback_current;    /* 测量的转矩电流, 用于 RLS. */
} PowerControlWheel_t;

typedef struct
{
    float power_limit;
    float power_before_limit;
    float power_after_limit;
    float minimum_power;       //无需反转/递增命令即可实现的最小值
    float k1;
    float k2;
    uint32_t rls_updates;
    uint8_t rls_enabled;
    uint8_t budget_unreachable; //速度/静态损失可能超出预算
} PowerControlStatus_t;

void Power_Control_Init(void);
float Power_Control_GetPredPower(float current, float speed_rpm);
void Power_Control_Limit(PowerControlWheel_t wheels[POWER_CONTROL_WHEEL_COUNT], float max_power);
const PowerControlStatus_t *Power_Control_GetStatus(void);
void Power_Control_SetRLSEnabled(uint8_t enabled);
void Power_Control_UpdateModel(const PowerControlWheel_t wheels[POWER_CONTROL_WHEEL_COUNT], float measured_power);
void Chassis_PowerControl(PowerControlWheel_t wheels[POWER_CONTROL_WHEEL_COUNT], float set_max_power);
float Cap_GetEnergy(float voltage);
float Cap_GetEnergyPercent(float voltage);

#ifdef __cplusplus
}
#endif

#endif
