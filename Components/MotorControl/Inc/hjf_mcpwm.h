#ifndef __HJF_MCPWM_H /* 防止头文件被重复包含。 */
#define __HJF_MCPWM_H /* 定义电机控制头文件保护宏。 */

#include "main.h" /* 引入工程公共定义和 HAL 基础接口。 */
#include "tim.h"  /* 引入 TIM1 和 TIM2 外设句柄。 */
#include "adc.h"  /* 引入 ADC2 外设句柄。 */

typedef enum /* 定义电机控制故障码，便于调试器和上层程序诊断。 */
{ /* 故障码枚举开始。 */
    BLDC_FAULT_NONE = 0U, /* 表示当前没有故障。 */
    BLDC_FAULT_INVALID_STEP, /* 表示六步换相步号无效。 */
    BLDC_FAULT_COM_EVENT, /* 表示 TIM1 COM 换步事件生成失败。 */
    BLDC_FAULT_CURRENT_CALIBRATION, /* 表示电流零点校准超时或失败。 */
    BLDC_FAULT_OVERCURRENT, /* 表示检测到相电流超过软件限制。 */
    BLDC_FAULT_BEMF_CALIBRATION, /* 表示无法得到有效反电动势积分。 */
    BLDC_FAULT_TRANSITION, /* 表示开环到闭环长期无法完成接管。 */
    BLDC_FAULT_LOST_SYNC, /* 表示积分闭环连续换步超时。 */
    BLDC_FAULT_VBUS_ADC, /* 表示母线电压 ADC 规则组采样失败。 */
    BLDC_FAULT_UNDERVOLTAGE, /* 表示母线电压低于 19 V。 */
    BLDC_FAULT_OVERVOLTAGE /* 表示母线电压高于 31 V。 */
} BLDC_Fault_t; /* 故障码枚举结束。 */

typedef enum /* 定义 VESC 风格输入限幅原因，数值允许按位组合。 */
{ /* 输入限幅原因枚举开始。 */
    BLDC_INPUT_LIMIT_NONE = 0U, /* 表示当前没有输入限幅。 */
    BLDC_INPUT_LIMIT_CURRENT = 1U, /* 表示母线输入电流上限正在限制目标。 */
    BLDC_INPUT_LIMIT_POWER = 2U, /* 表示母线输入功率上限正在限制目标。 */
    BLDC_INPUT_LIMIT_CURRENT_AND_POWER = 3U, /* 表示输入电流和功率上限同时生效。 */
    BLDC_INPUT_LIMIT_SOFT_OVERVOLTAGE = 4U /* 表示母线软过压正在按比例降低允许功率。 */
} BLDC_InputLimitReason_t; /* 输入限幅原因枚举结束。 */

typedef enum /* 定义 VESC 风格上层控制模式。 */
{ /* 上层控制模式枚举开始。 */
    BLDC_CONTROL_STOP = 0U, /* 表示停止输出。 */
    BLDC_CONTROL_CURRENT, /* 表示直接控制正向电机电流。 */
    BLDC_CONTROL_DUTY, /* 表示通过外环控制正向 PWM 占空比。 */
    BLDC_CONTROL_SPEED /* 表示通过外环控制正向电角转速。 */
} BLDC_ControlMode_t; /* 上层控制模式枚举结束。 */

typedef enum /* 定义安全换向状态，便于上层和调试器观察换向流程。 */
{ /* 安全换向状态枚举开始。 */
    BLDC_REVERSAL_IDLE = 0U, /* 表示当前没有正在执行的换向。 */
    BLDC_REVERSAL_RAMP_DOWN, /* 表示正在将当前方向的电流斜坡降到零。 */
    BLDC_REVERSAL_COAST_WAIT /* 表示桥臂已经关闭并等待转子滑行到低速。 */
} BLDC_ReversalState_t; /* 安全换向状态枚举结束。 */

typedef enum /* 定义统一输出限幅原因，数值允许按位组合。 */
{ /* 统一输出限幅原因枚举开始。 */
    BLDC_LIMIT_NONE = 0U, /* 表示当前没有任何输出限幅。 */
    BLDC_LIMIT_MOTOR_CURRENT = 1U, /* 表示电机电流上限正在限制命令。 */
    BLDC_LIMIT_INPUT_CURRENT = 2U, /* 表示估算母线输入电流上限正在限制命令。 */
    BLDC_LIMIT_INPUT_POWER = 4U, /* 表示估算母线输入功率上限正在限制命令。 */
    BLDC_LIMIT_DUTY = 8U, /* 表示 PWM 占空比上限正在限制输出。 */
    BLDC_LIMIT_ERPM = 16U, /* 表示电角转速配置上限正在限制命令。 */
    BLDC_LIMIT_MINIMUM_CURRENT = 32U, /* 表示最低稳定运行电流正在限制输出。 */
    BLDC_LIMIT_SOFT_OVERVOLTAGE = 64U /* 表示软过压降额正在限制电流和输入功率。 */
} BLDC_LimitReason_t; /* 统一输出限幅原因枚举结束。 */

void BLDC_test(void); /* 声明 VESC 风格无感方波 Integrate 测试入口。 */
void BLDC_Process(void); /* 在主循环中持续执行非阻塞后台管理。 */
void BLDC_Start(void); /* 请求从停止状态启动电机。 */
void BLDC_Stop(void); /* 请求电流斜坡到零后安全停机。 */
void BLDC_EmergencyStop(void); /* 立即关闭全部功率输出。 */
uint8_t BLDC_IsRunning(void); /* 返回是否已经进入积分闭环运行。 */
uint8_t BLDC_GetRestartCount(void); /* 返回当前连续软故障自动重启次数。 */
void BLDC_SetRunCurrentMa(int32_t command_ma); /* 设置 VESC 风格运行电流命令，低于释放阈值时执行软停机。 */
void BLDC_SetBrakeCurrentMa(uint32_t brake_current_ma); /* 禁止主动负转矩，任何非零制动请求都转换为滑行停机。 */
void BLDC_SetDutyPermille(uint16_t duty_permille); /* 设置当前方向的占空比幅值命令，单位为千分比。 */
void BLDC_SetSignedDutyPermille(int16_t duty_permille); /* 设置带方向符号的占空比命令，负值请求安全反转。 */
void BLDC_SetSpeedErpm(uint32_t speed_erpm); /* 设置当前方向的速度幅值命令，单位为电角转每分钟。 */
void BLDC_SetSignedSpeedErpm(int32_t speed_erpm); /* 设置带方向符号的速度命令，负值请求安全反转。 */
void BLDC_SetSpeedPidGains(uint32_t kp_x1000, uint32_t ki_x1000, uint32_t kd_x1000, uint16_t derivative_filter_permille); /* 设置速度 PID 增益和微分滤波，全部使用千分之一缩放。 */
BLDC_ControlMode_t BLDC_GetControlMode(void); /* 返回当前上层控制模式。 */
void BLDC_SetMotorCurrentLimitMa(int32_t limit_ma); /* 设置统一控制允许的最大正向电机电流。 */
void BLDC_SetDutyLimitsPermille(uint16_t minimum_permille, uint16_t maximum_permille); /* 设置电流 PI 输出占空比范围。 */
void BLDC_SetSpeedLimitsErpm(uint32_t minimum_erpm, uint32_t maximum_erpm); /* 设置无感速度模式允许的电角转速范围。 */
BLDC_LimitReason_t BLDC_GetLimitReason(void); /* 返回当前统一输出限幅原因位标志。 */
uint8_t BLDC_IsLimitActive(void); /* 返回当前是否存在任意统一输出限幅。 */
int32_t BLDC_GetUnifiedLimitedCurrentMa(void); /* 返回统一管理器最终允许的电机电流。 */
uint16_t BLDC_GetUnifiedLimitedDutyPermille(void); /* 返回统一管理器允许的最大占空比千分比。 */
uint32_t BLDC_GetUnifiedLimitedSpeedErpm(void); /* 返回统一管理器限幅后的速度目标。 */
void BLDC_SetCurrentRampMaPerSecond(uint32_t rise_ma_per_second, uint32_t fall_ma_per_second); /* 设置运行电流命令的上升和下降斜率。 */
int32_t BLDC_GetRequestedCurrentMa(void); /* 返回用户请求的运行电流命令，单位为毫安。 */
int32_t BLDC_GetAppliedCurrentMa(void); /* 返回经过外层斜坡后的运行电流命令，单位为毫安。 */
void BLDC_SetCurrentTargetMa(int32_t target_ma); /* 兼容旧接口，内部转交给运行电流命令接口。 */
void BLDC_SetInputCurrentLimitMa(uint32_t limit_ma); /* 设置估算母线输入电流上限，单位为毫安。 */
void BLDC_SetInputPowerLimitMw(uint32_t limit_mw); /* 设置母线输入功率上限，单位为毫瓦。 */
uint32_t BLDC_GetEstimatedInputCurrentMa(void); /* 返回按 VESC BLDC 公式估算的母线输入电流。 */
uint32_t BLDC_GetEstimatedInputPowerMw(void); /* 返回按母线电压计算的估算输入功率。 */
int32_t BLDC_GetEstimatedSignedInputCurrentMa(void); /* 返回带能量方向符号的估算母线电流，正值耗电、负值回充。 */
int32_t BLDC_GetEstimatedSignedInputPowerMw(void); /* 返回带能量方向符号的估算母线功率，正值耗电、负值回充。 */
uint8_t BLDC_IsRegenerationProtectionActive(void); /* 返回是否已经确认回充并执行禁止回充保护。 */
uint8_t BLDC_IsSoftOvervoltageActive(void); /* 返回母线软过压降额或等待释放是否正在生效。 */
uint16_t BLDC_GetSoftOvervoltageDeratePermille(void); /* 返回当前软过压允许功率的千分比。 */
uint32_t BLDC_GetRegenerationEventCount(void); /* 返回累计确认的持续回充事件次数。 */
uint32_t BLDC_GetSoftOvervoltageEventCount(void); /* 返回累计进入软过压区域的次数。 */
uint32_t BLDC_GetEnergyCoastStopCount(void); /* 返回回充或软过压触发的强制滑行停机次数。 */
uint32_t BLDC_GetNegativeTorqueRejectCount(void); /* 返回被转换为滑行停机的主动制动请求次数。 */
int32_t BLDC_GetPeakRegenerationCurrentMa(void); /* 返回上电以来最小的负母线电流估算值。 */
int32_t BLDC_GetPeakRegenerationPowerMw(void); /* 返回上电以来最小的负母线功率估算值。 */
int32_t BLDC_GetLimitedCurrentTargetMa(void); /* 返回输入电流和功率限幅后的电机电流目标。 */
BLDC_InputLimitReason_t BLDC_GetInputLimitReason(void); /* 返回当前输入限幅原因。 */
void BLDC_SetPhaseAdvanceMaxPermille(uint16_t max_permille); /* 设置最大积分阈值提前千分比，零表示关闭。 */
void BLDC_SetCurrentScalePermille(uint16_t u_permille, uint16_t v_permille, uint16_t w_permille); /* 分别设置 U、V、W 相电流增益千分比。 */
void BLDC_SetOvercurrentLimitMa(int32_t limit_ma); /* 设置运行时软件过流保护阈值，单位为毫安。 */
BLDC_Fault_t BLDC_GetFault(void); /* 返回当前电机控制故障码。 */
int32_t BLDC_GetMotorCurrentMa(void); /* 返回当前有效回流相电流，单位为毫安。 */
int32_t BLDC_GetFilteredCurrentMa(void); /* 返回低通滤波后的电机电流，单位为毫安。 */
int32_t BLDC_GetSignedMotorCurrentMa(void); /* 返回经过相极性归一化的带符号电机电流瞬时值。 */
int32_t BLDC_GetSignedFilteredCurrentMa(void); /* 返回经过低通滤波的带符号电机电流诊断值。 */
int32_t BLDC_GetSourcePhaseCurrentMa(void); /* 返回当前源相经过极性归一化的带符号电流。 */
int32_t BLDC_GetFloatingPhaseCurrentMa(void); /* 返回当前悬空相经过极性归一化的带符号电流。 */
uint32_t BLDC_GetCurrentPolarityErrorCount(void); /* 返回正常驱动期间显著负电流样本累计值。 */
uint8_t BLDC_SetCurrentPolarity(int8_t u_polarity, int8_t v_polarity, int8_t w_polarity); /* 仅在功率关闭时设置三相电流采样极性。 */
uint16_t BLDC_GetDuty(void); /* 返回当前占空比命令，满量程为 1200。 */
uint32_t BLDC_GetBusVoltageMv(void); /* 返回低通滤波后的母线电压，单位为毫伏。 */
uint8_t BLDC_SetDirection(int8_t direction); /* 仅在功率输出关闭时设置方向，负值选择反转，非负值选择正转。 */
int8_t BLDC_GetDirection(void); /* 返回当前方向状态，正一表示正转，负一表示反转。 */
int8_t BLDC_GetRequestedDirection(void); /* 返回最近一次有符号命令请求的目标方向。 */
BLDC_ReversalState_t BLDC_GetReversalState(void); /* 返回当前安全换向状态。 */
uint32_t BLDC_GetReversalCount(void); /* 返回已经完成的安全换向次数。 */
int32_t BLDC_GetSignedErpm(void); /* 返回带方向符号的滤波电角转速。 */
int32_t BLDC_GetTachometer(void); /* 返回带方向符号的累计换相步数。 */
uint32_t BLDC_GetTachometerAbs(void); /* 返回不区分方向的累计换相步数。 */
void BLDC_ResetTachometer(void); /* 原子清零有符号和绝对换相计数器。 */

#endif /* 结束电机控制头文件保护宏。 */
