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

void BLDC_test(void); /* 声明 VESC 风格无感方波 Integrate 测试入口。 */
void BLDC_SetCurrentTargetMa(int32_t target_ma); /* 设置正向运行目标电流，单位为毫安。 */
void BLDC_SetPhaseAdvanceMaxPermille(uint16_t max_permille); /* 设置最大积分阈值提前千分比，零表示关闭。 */
void BLDC_SetCurrentScalePermille(uint16_t u_permille, uint16_t v_permille, uint16_t w_permille); /* 分别设置 U、V、W 相电流增益千分比。 */
void BLDC_SetOvercurrentLimitMa(int32_t limit_ma); /* 设置运行时软件过流保护阈值，单位为毫安。 */
BLDC_Fault_t BLDC_GetFault(void); /* 返回当前电机控制故障码。 */
int32_t BLDC_GetMotorCurrentMa(void); /* 返回当前有效回流相电流，单位为毫安。 */
int32_t BLDC_GetFilteredCurrentMa(void); /* 返回低通滤波后的电机电流，单位为毫安。 */
uint16_t BLDC_GetDuty(void); /* 返回当前占空比命令，满量程为 1200。 */
uint32_t BLDC_GetBusVoltageMv(void); /* 返回低通滤波后的母线电压，单位为毫伏。 */

#endif /* 结束电机控制头文件保护宏。 */
