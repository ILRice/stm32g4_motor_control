/*
 ******************************************************************************
 * 文件名称：hjf_mcpwm.c
 * 功能说明：FD6288T + TIM1 的 VESC 风格无感方波 Integrate 控制。
 *
 * 当前程序执行流程：
 * 1. main.c 初始化 GPIO、DMA、ADC2、TIM1、TIM2，然后调用 BLDC_test()。
 * 2. BLDC_test() 启动三路内部运放，校准 ADC1/ADC2，并启动规则组和注入组。
 * 3. CALIBRATE 保持功率输出关闭，采集 1024 组三相电流零点并计算偏置。
 * 4. ALIGN 固定第 1 步 200 ms，使转子建立已知初始电角度。
 * 5. OPEN_LOOP 从 20 ms/步逐渐加速至 2 ms/步，并测量每步反电动势积分。
 * 6. 开环末段自动计算积分阈值，不需要预先填写电机反电动势常数。
 * 7. TRANSITION 保留 2 ms 强制换步兜底，同时尝试由积分阈值提前换步。
 * 8. 连续 6 次积分换步成功后进入 INTEGRATE，完全由反电动势积分换步。
 * 9. ADC1 采 U/W 相、ADC2 采 V 相，只有同一 PWM 周期的两组注入转换都完成后才处理电流。
 * 10. 三相分别使用独立增益系数换算电流，启动校准同时记录每相零点峰峰噪声。
 * 11. 每次换相后的前 4 组电流样本只做诊断，不送入 PI 和软件过流判断。
 * 12. 按当前六步状态选择持续导通的低桥回流相，并记录六个扇区各自的滤波电流。
 * 13. 电流给定从 0 平滑爬升到目标值，回流相电流经过一阶低通后送入 PI 控制器。
 * 14. PI 控制器在 20 kHz 配对电流回调中调节占空比，并执行积分和输出双重限幅。
 * 15. 任意一相连续两次超过可配置过流阈值时，立即关闭 TIM1 MOE 并记录过流故障。
 * 16. 功率级为非同步整流：源相高桥 PWM、回流相低桥常导通、第三相悬空。
 * 17. 三相 CCR 保持相同，桥臂模式和使能位通过 TIM1 COM 事件同时生效。
 * 18. ADC2 规则组在 OC4REF 上升沿采 BEMF，注入组在下降沿采 OPAMP2 电流。
 * 19. 每次成功换相把触发原因和关键数据写入 RAM 环形记录，不在实时控制中打印串口。
 * 20. ADC1 规则组每 10 ms 采集一次 VBUS，按 20 kΩ/1 kΩ 分压换算并低通滤波。
 * 21. 母线低于 19 V 或高于 31 V 时锁存故障并立即关闭 TIM1 主输出。
 * 22. 开环标定结束后保存基础磁链积分，并按母线电压和电角转速动态调整运行阈值。
 * 23. 仅在 INTEGRATE 状态按电角转速降低积分阈值，实现可限幅的转速相关换相提前。
 * 24. 提前量使用积分阈值千分比表示，不等同于机械角度或电角度数值。
 *
 * 首次调试建议监视：g_bldc.state、g_bldc.step、get_diff、
 * g_bldc.bemf_integral、g_bldc.integral_limit_unadvanced、g_bldc.integral_limit、
 * g_bldc.phase_advance_permille、g_bldc.electrical_rpm、
 * g_bldc.transition_valid_count、g_bldc.forced_count、g_current、
 * g_current_control、g_current_scale、g_bldc.duty、g_bldc.fault、g_comm_trace、
 * g_comm_trace_index、g_comm_trace_sequence、g_bus_voltage。
 * 首次上电务必使用限流电源、低母线电压，并确保可以立即断电。
 ******************************************************************************
 */

#include "hjf_mcpwm.h"                 /* 引入电机控制接口和 HAL 外设句柄。 */
#include "opamp.h"                     /* 引入三路内部运放句柄。 */
#include "stm32g4xx_ll_tim.h"          /* 引入定时器输出模式底层接口。 */
#include <stdint.h>                     /* 引入固定宽度整数类型。 */

#define BLDC_DUTY_SCALE                    1200U /* 定义占空比输入满量程。 */
#define BLDC_COMM_TRACE_LENGTH                64U /* 定义换相事件环形记录数量。 */
#define BLDC_TEST_DUTY                      250U /* 定义低电流启动时的 PI 初始占空比。 */
#define BLDC_PWM_FREQUENCY_HZ             20000U /* 定义快速控制回调频率为 20 kHz。 */
#define BLDC_ALIGN_TICKS                   4000U /* 定义 200 ms 定位时间。 */
#define BLDC_OPEN_LOOP_RAMP_STEP               2U /* 定义每步缩短两个 PWM 周期，保持原加速斜率。 */
#define BLDC_CALIBRATION_VALID_STEPS           12U /* 定义阈值标定有效步数。 */
#define BLDC_CALIBRATION_MAX_STEPS             72U /* 定义阈值标定最大尝试数。 */
#define BLDC_INTEGRAL_LIMIT_PERCENT            80U /* 定义积分阈值比例。 */
#define BLDC_MIN_CALIBRATION_INTEGRAL         500U /* 定义有效单步积分下限。 */
#define BLDC_MIN_INTEGRAL_LIMIT               400U /* 定义运行积分阈值下限。 */
#define BLDC_BEMF_DEADBAND                      10 /* 定义反电动势数字死区。 */
#define BLDC_BLANKING_TICKS                      6U /* 定义换步后 300 us 消隐时间。 */
#define BLDC_MIN_COMMUTATION_TICKS             12U /* 定义最短换步间隔为 600 us。 */
#define BLDC_TRANSITION_VALID_COUNT              6U /* 定义闭环接管成功次数。 */
#define BLDC_MAX_FORCED_COUNT                    3U /* 定义闭环最大连续超时数。 */
#define BLDC_TIMEOUT_MULTIPLIER                  2U /* 定义闭环超时倍数。 */
#define BLDC_TIMEOUT_MIN_TICKS                  60U /* 定义闭环超时下限为 3 ms。 */
#define BLDC_TIMEOUT_MAX_TICKS                 600U /* 定义闭环超时上限为 30 ms。 */
#define BLDC_ADC_SAMPLE_DELAY_COUNTS            20U /* 定义高桥导通后 ADC 延时。 */
#define BLDC_VALID_STEP_MIN                      1U /* 定义最小换相步号。 */
#define BLDC_VALID_STEP_MAX                      6U /* 定义最大换相步号。 */
#define BLDC_INTEGRAL_SATURATION        0x3FFFFFFFUL /* 定义积分器饱和值。 */
#define BLDC_CURRENT_CALIBRATION_SAMPLES       1024U /* 定义电流零点校准采样组数。 */
#define BLDC_CURRENT_CALIBRATION_TIMEOUT_TICKS 6000U /* 定义电流零点校准 300 ms 超时。 */
#define BLDC_CURRENT_GAIN_DENOMINATOR         1000000L /* 定义电流增益定点运算总分母。 */
#define BLDC_CURRENT_U_MA_PER_COUNT_X1000       22000L /* 定义 U 相每个 ADC 差值的标称电流增益。 */
#define BLDC_CURRENT_V_MA_PER_COUNT_X1000       22000L /* 定义 V 相每个 ADC 差值的标称电流增益。 */
#define BLDC_CURRENT_W_MA_PER_COUNT_X1000       22000L /* 定义 W 相每个 ADC 差值的标称电流增益。 */
#define BLDC_CURRENT_SCALE_DEFAULT_PERMILLE       1000U /* 定义三相电流增益默认校准系数。 */
#define BLDC_CURRENT_SCALE_MIN_PERMILLE            500U /* 定义允许设置的电流增益校准下限。 */
#define BLDC_CURRENT_SCALE_MAX_PERMILLE           2000U /* 定义允许设置的电流增益校准上限。 */
#define BLDC_CURRENT_ZERO_NOISE_LIMIT_COUNTS        80U /* 定义零电流校准允许的峰峰噪声上限。 */
#define BLDC_CURRENT_BLANKING_SAMPLES                4U /* 定义换相后跳过 200 us 的电流控制样本。 */
#define BLDC_CURRENT_STEP_FILTER_SHIFT                5U /* 定义六扇区诊断电流的三十二分之一滤波系数。 */
#define BLDC_OVERCURRENT_LIMIT_DEFAULT_MA      15000L /* 定义软件过流保护默认值为 15 A。 */
#define BLDC_OVERCURRENT_LIMIT_MIN_MA            500L /* 定义软件过流保护可设置下限为 0.5 A。 */
#define BLDC_OVERCURRENT_LIMIT_MAX_MA          15000L /* 定义软件过流保护可设置上限为 15 A。 */
#define BLDC_OVERCURRENT_CONFIRM_SAMPLES           2U /* 定义过流需要连续确认两次。 */
#define BLDC_CURRENT_OFFSET_MIN                 1400U /* 定义合理电流零点 ADC 下限。 */
#define BLDC_CURRENT_OFFSET_MAX                 2700U /* 定义合理电流零点 ADC 上限。 */
#define BLDC_CURRENT_TARGET_DEFAULT_MA          3000L /* 定义首次闭环测试目标电流为 3 A。 */
#define BLDC_CURRENT_RAMP_STEP_MA                  5L /* 定义每次采样最多增加 5 mA，保持原电流斜坡。 */
#define BLDC_CURRENT_FILTER_SHIFT                  4U /* 定义电流一阶低通系数为十六分之一。 */
#define BLDC_CURRENT_KP_DIVISOR                  100L /* 定义比例项每 100 mA 输出一个占空比单位。 */
#define BLDC_CURRENT_KI_DIVISOR               200000L /* 定义适配 20 kHz 回调的积分项缩放系数。 */
#define BLDC_ADC_REFERENCE_MV                   3300U /* 定义 ADC 模拟参考电压为 3300 mV。 */
#define BLDC_ADC_FULL_SCALE                     4095U /* 定义十二位 ADC 满量程计数。 */
#define BLDC_VBUS_DIVIDER_NUMERATOR               21U /* 定义 20 kΩ 加 1 kΩ 分压器的电压倍率。 */
#define BLDC_VBUS_DIVIDER_DENOMINATOR              1U /* 定义 1 kΩ 下臂对应的分压倍率分母。 */
#define BLDC_VBUS_CALIBRATION_PERMILLE           1000U /* 定义母线电压换算校准系数为千分之一千。 */
#define BLDC_VBUS_LOW_LIMIT_MV                  19000U /* 定义母线欠压保护阈值为 19 V。 */
#define BLDC_VBUS_HIGH_LIMIT_MV                 31000U /* 定义母线过压保护阈值为 31 V。 */
#define BLDC_VBUS_SAMPLE_PERIOD_MS                 10U /* 定义母线电压后台采样周期为 10 ms。 */
#define BLDC_VBUS_ADC_TIMEOUT_MS                    2U /* 定义 ADC1 规则组单次等待超时为 2 ms。 */
#define BLDC_VBUS_FILTER_SHIFT                       3U /* 定义母线电压低通系数为八分之一。 */
#define BLDC_VBUS_FAULT_CONFIRM_SAMPLES              3U /* 定义运行中电压故障连续确认三次。 */
#define BLDC_VBUS_ADC_ERROR_CONFIRM_SAMPLES          3U /* 定义 ADC 连续失败三次后停机。 */
#define BLDC_DYNAMIC_INTEGRAL_BASE_PERCENT          80U /* 定义标定阈值的百分之八十作为基础磁链项。 */
#define BLDC_DYNAMIC_INTEGRAL_MAX_PERCENT          160U /* 定义动态积分阈值最大为标定值的百分之一百六十。 */
#define BLDC_DYNAMIC_INTEGRAL_MIN_ERPM             1100U /* 定义耦合项计算使用的最低电角转速。 */
#define BLDC_PHASE_ADVANCE_START_ERPM              5000U /* 定义开始加入换相提前量的电角转速。 */
#define BLDC_PHASE_ADVANCE_FULL_ERPM              10000U /* 定义达到最大换相提前量的电角转速。 */
#define BLDC_PHASE_ADVANCE_DEFAULT_PERMILLE         100U /* 定义默认最大积分阈值提前量为百分之十。 */
#define BLDC_PHASE_ADVANCE_MAX_PERMILLE             200U /* 定义允许设置的最大积分阈值提前量为百分之二十。 */
#define BLDC_PHASE_ADVANCE_MIN_BASE_PERCENT          80U /* 定义提前后的阈值不得低于基础磁链项的百分之八十。 */
#define BLDC_PERMILLE_SCALE                         1000U /* 定义千分比定点运算满量程。 */

#define BLDC_CURRENT_TARGET_MIN_MA               300L /* 定义非零运行目标电流下限为 0.3 A。 */
#define BLDC_CURRENT_TARGET_MAX_MA             10000L /* 定义可设置目标电流上限为 10 A。 */
#define BLDC_CURRENT_DUTY_MIN                    100L /* 定义闭环运行最小占空比命令。 */
#define BLDC_CURRENT_DUTY_MAX                    600L /* 定义闭环运行最大占空比为百分之五十。 */
#define BLDC_OPEN_LOOP_START_TICKS          400U /* 定义开环初始 20 ms/步。 */
#define BLDC_OPEN_LOOP_END_TICKS             40U /* 定义开环最终 2 ms/步。 */

#define BLDC_ALL_HIGH_OUTPUTS \
    (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E) /* 定义三路高桥使能位。 */
#define BLDC_ALL_LOW_OUTPUTS \
    (TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE) /* 定义三路低桥使能位。 */
#define BLDC_ALL_OUTPUTS \
    (BLDC_ALL_HIGH_OUTPUTS | BLDC_ALL_LOW_OUTPUTS) /* 定义全部桥臂使能位。 */

typedef enum /* 定义无感控制状态。 */
{ /* 状态枚举开始。 */
    BLDC_STATE_STOP = 0U,               /* 表示功率输出停止。 */
    BLDC_STATE_CALIBRATE,               /* 表示正在校准三相电流零点。 */
    BLDC_STATE_ALIGN,                   /* 表示转子定位。 */
    BLDC_STATE_OPEN_LOOP,               /* 表示开环加速。 */
    BLDC_STATE_TRANSITION,              /* 表示闭环接管过渡。 */
    BLDC_STATE_INTEGRATE,               /* 表示积分闭环运行。 */
    BLDC_STATE_FAULT                    /* 表示故障停机。 */
} BLDC_State_t;                         /* 状态枚举结束。 */

typedef enum /* 定义换相事件触发原因。 */
{ /* 换相原因枚举开始。 */
    BLDC_COMM_REASON_ALIGN = 0U,        /* 表示定位结束后开始第一步开环换相。 */
    BLDC_COMM_REASON_OPEN_LOOP,         /* 表示开环定时器触发的强制换相。 */
    BLDC_COMM_REASON_BEMF,              /* 表示反电动势积分达到阈值触发换相。 */
    BLDC_COMM_REASON_TIMEOUT            /* 表示反电动势未及时触发而超时换相。 */
} BLDC_CommReason_t;                    /* 换相原因枚举结束。 */

typedef struct /* 定义反电动势采样数据。 */
{ /* 采样数据结构开始。 */
    volatile uint16_t u;                /* 保存 U 相 ADC 值。 */
    volatile uint16_t v;                /* 保存 V 相 ADC 值。 */
    volatile uint16_t w;                /* 保存 W 相 ADC 值。 */
    volatile uint8_t ready;             /* 标记采样已经有效。 */
    volatile uint32_t sample_count;     /* 记录采样组数。 */
} BEMF_Sample_t;                        /* 采样数据结构结束。 */

typedef struct /* 定义 BLDC 运行数据。 */
{ /* 运行数据结构开始。 */
    volatile BLDC_State_t state;        /* 保存当前状态。 */
    volatile uint8_t step;              /* 保存当前换相步。 */
    volatile uint16_t duty;             /* 保存当前占空比命令。 */
    volatile uint32_t pwm_pulse;        /* 保存 PWM 比较值。 */
    volatile uint32_t state_ticks;      /* 保存状态计时。 */
    volatile uint32_t commutation_ticks; /* 保存本步计时。 */
    volatile uint32_t last_commutation_ticks; /* 保存上一步周期。 */
    volatile uint32_t open_loop_ticks;  /* 保存开环换步周期。 */
    volatile uint32_t blanking_ticks;   /* 保存剩余消隐次数。 */
    volatile uint32_t bemf_integral;    /* 保存本步积分值。 */
    volatile uint32_t integral_limit;   /* 保存积分换步阈值。 */
    volatile uint32_t integral_limit_unadvanced; /* 保存未叠加换相提前量的动态积分阈值。 */
    volatile uint32_t integral_limit_base; /* 保存不随转速变化的基础磁链积分项。 */
    volatile uint32_t integral_coupling_reference; /* 保存标定工况下的母线耦合积分项。 */
    volatile uint32_t integral_limit_max; /* 保存动态积分阈值允许的最大值。 */
    volatile uint32_t integral_calibration_bus_mv; /* 保存积分标定时的母线电压。 */
    volatile uint32_t integral_calibration_erpm; /* 保存积分标定时的电角转速。 */
    volatile uint16_t phase_advance_permille; /* 保存当前实际应用的积分阈值提前千分比。 */
    volatile uint16_t phase_advance_max_permille; /* 保存用户允许的最大积分阈值提前千分比。 */
    volatile uint32_t calibration_sum;  /* 保存标定积分总和。 */
    volatile uint16_t calibration_count; /* 保存有效标定步数。 */
    volatile uint16_t calibration_attempts; /* 保存标定尝试步数。 */
    volatile uint8_t transition_valid_count; /* 保存连续积分成功数。 */
    volatile uint8_t forced_count;      /* 保存连续强制换步数。 */
    volatile uint32_t timeout_ticks;    /* 保存闭环换步超时。 */
    volatile uint32_t electrical_rpm;   /* 保存估算电角转速。 */
    volatile BLDC_Fault_t fault;        /* 保存最近一次故障原因。 */
} BLDC_Control_t;                       /* 运行数据结构结束。 */

typedef struct /* 定义三相电流采样和校准数据。 */
{ /* 三相电流数据结构开始。 */
    volatile uint16_t raw_u;            /* 保存 OPAMP1 对应的 U 相 ADC 原始值。 */
    volatile uint16_t raw_v;            /* 保存 OPAMP2 对应的 V 相 ADC 原始值。 */
    volatile uint16_t raw_w;            /* 保存 OPAMP3 对应的 W 相 ADC 原始值。 */
    volatile uint16_t offset_u;         /* 保存 U 相零电流 ADC 偏置。 */
    volatile uint16_t offset_v;         /* 保存 V 相零电流 ADC 偏置。 */
    volatile uint16_t offset_w;         /* 保存 W 相零电流 ADC 偏置。 */
    volatile uint16_t minimum_u;        /* 保存 U 相零点校准期间的最小 ADC 值。 */
    volatile uint16_t minimum_v;        /* 保存 V 相零点校准期间的最小 ADC 值。 */
    volatile uint16_t minimum_w;        /* 保存 W 相零点校准期间的最小 ADC 值。 */
    volatile uint16_t maximum_u;        /* 保存 U 相零点校准期间的最大 ADC 值。 */
    volatile uint16_t maximum_v;        /* 保存 V 相零点校准期间的最大 ADC 值。 */
    volatile uint16_t maximum_w;        /* 保存 W 相零点校准期间的最大 ADC 值。 */
    volatile uint16_t zero_noise_u;     /* 保存 U 相零点峰峰噪声计数。 */
    volatile uint16_t zero_noise_v;     /* 保存 V 相零点峰峰噪声计数。 */
    volatile uint16_t zero_noise_w;     /* 保存 W 相零点峰峰噪声计数。 */
    volatile int32_t current_u_ma;      /* 保存 U 相电流，单位为毫安。 */
    volatile int32_t current_v_ma;      /* 保存 V 相电流，单位为毫安。 */
    volatile int32_t current_w_ma;      /* 保存 W 相电流，单位为毫安。 */
    volatile int32_t motor_current_ma;  /* 保存当前回流相有效电流，单位为毫安。 */
    volatile int32_t floating_current_ma; /* 保存当前悬空相电流绝对值，单位为毫安。 */
    volatile int32_t step_current_ma[6]; /* 保存六个换相扇区各自的滤波回流电流。 */
    volatile uint32_t sum_u;            /* 保存 U 相零点校准累计值。 */
    volatile uint32_t sum_v;            /* 保存 V 相零点校准累计值。 */
    volatile uint32_t sum_w;            /* 保存 W 相零点校准累计值。 */
    volatile uint16_t count_adc1;       /* 保存 ADC1 注入序列校准采样数。 */
    volatile uint16_t count_adc2;       /* 保存 ADC2 注入序列校准采样数。 */
    volatile uint32_t paired_sample_count; /* 保存成功配对处理的三相电流样本数。 */
    volatile uint32_t sync_error_count; /* 保存某个 ADC 样本被覆盖前仍未配对的次数。 */
    volatile uint32_t blanked_sample_count; /* 保存换相消隐累计丢弃的电流控制样本数。 */
    volatile uint8_t calibrated;        /* 标记三相零点校准是否完成。 */
    volatile uint8_t overcurrent_count; /* 保存连续过流确认次数。 */
    volatile uint8_t adc1_ready;        /* 标记本 PWM 周期 ADC1 的 U/W 样本已经到达。 */
    volatile uint8_t adc2_ready;        /* 标记本 PWM 周期 ADC2 的 V 相样本已经到达。 */
    volatile uint8_t blanking_samples;  /* 保存换相后剩余的电流消隐样本数。 */
    volatile uint8_t zero_noise_ok;     /* 标记三相零点峰峰噪声均处于允许范围。 */
} BLDC_Current_t;                       /* 三相电流数据结构结束。 */

typedef struct /* 定义三相电流增益校准系数。 */
{ /* 电流增益校准结构开始。 */
    volatile uint16_t u_permille;       /* 保存 U 相电流增益千分比。 */
    volatile uint16_t v_permille;       /* 保存 V 相电流增益千分比。 */
    volatile uint16_t w_permille;       /* 保存 W 相电流增益千分比。 */
} BLDC_CurrentScale_t;                  /* 电流增益校准结构结束。 */

typedef struct /* 定义电流 PI 控制器运行数据。 */
{ /* 电流 PI 数据结构开始。 */
    volatile int32_t target_ma;        /* 保存用户设置的最终目标电流。 */
    volatile int32_t ramped_target_ma; /* 保存经过斜坡限制的当前目标电流。 */
    volatile int32_t filtered_current_ma; /* 保存低通滤波后的电机电流。 */
    volatile int32_t error_ma;         /* 保存目标电流与实际电流误差。 */
    volatile int32_t proportional_duty; /* 保存比例项输出。 */
    volatile int32_t integral_accumulator; /* 保存带缩放的积分累计值。 */
    volatile int32_t output_duty;      /* 保存 PI 最终占空比命令。 */
    volatile int32_t overcurrent_limit_ma; /* 保存运行时软件过流保护阈值。 */
    volatile uint8_t filter_initialized; /* 标记电流滤波器是否已经初始化。 */
} BLDC_CurrentControl_t;               /* 电流 PI 数据结构结束。 */

typedef struct /* 定义母线电压采样、滤波和保护数据。 */
{ /* 母线电压数据结构开始。 */
    volatile uint16_t raw;              /* 保存 VBUS_ADC 的十二位原始值。 */
    volatile uint16_t pot_raw;          /* 保存 ADC1 规则组第二顺位的 POT_ADC 原始值。 */
    volatile uint32_t measured_mv;      /* 保存本次换算得到的母线电压毫伏值。 */
    volatile uint32_t filtered_mv;      /* 保存低通滤波后的母线电压毫伏值。 */
    volatile uint32_t sample_count;     /* 保存成功完成的母线电压采样次数。 */
    volatile uint32_t last_sample_ms;   /* 保存上一次启动规则组采样的系统毫秒时间。 */
    volatile uint8_t initialized;       /* 标记母线电压滤波器是否已经初始化。 */
    volatile uint8_t low_count;         /* 保存连续欠压样本次数。 */
    volatile uint8_t high_count;        /* 保存连续过压样本次数。 */
    volatile uint8_t adc_error_count;   /* 保存 ADC1 规则组连续失败次数。 */
} BLDC_BusVoltage_t;                    /* 母线电压数据结构结束。 */

typedef struct /* 定义单次换相事件的调试快照。 */
{ /* 换相事件调试快照开始。 */
    volatile uint32_t sequence;         /* 保存事件连续序号，零表示该项无效。 */
    volatile uint32_t state_ticks;      /* 保存换相时当前状态已经运行的控制周期数。 */
    volatile uint32_t commutation_ticks; /* 保存本次换相前一步持续的控制周期数。 */
    volatile uint32_t bemf_integral;    /* 保存换相前的反电动势积分值。 */
    volatile uint32_t integral_limit;   /* 保存换相时使用的反电动势积分阈值。 */
    volatile uint32_t integral_limit_base; /* 保存换相时使用的基础磁链积分项。 */
    volatile uint32_t bus_voltage_mv;   /* 保存换相时低通滤波后的母线电压。 */
    volatile uint32_t timeout_ticks;    /* 保存换相时使用的闭环超时阈值。 */
    volatile int32_t bemf_diff;         /* 保存换相前最后一次反电动势差值。 */
    volatile int32_t motor_current_ma;  /* 保存换相时的电机电流，单位为毫安。 */
    volatile uint16_t duty;             /* 保存换相时的占空比命令。 */
    volatile uint8_t step_before;       /* 保存换相前的六步换相步号。 */
    volatile uint8_t step_after;        /* 保存换相后的六步换相步号。 */
    volatile uint8_t state;             /* 保存触发换相时的状态机状态。 */
    volatile uint8_t reason;            /* 保存本次换相的触发原因。 */
    volatile uint8_t forced_count;      /* 保存换相时连续强制换相次数。 */
    volatile uint8_t phase_advance_permille; /* 保存换相时实际应用的积分阈值提前千分比。 */
    volatile uint8_t reserved_1;        /* 保留一个字节以便后续增加调试字段。 */
    volatile uint8_t reserved_2;        /* 保留一个字节以便后续增加调试字段。 */
} BLDC_CommTrace_t;                     /* 单次换相事件调试快照结束。 */

static void BLDC_HardwarePrepare(void); /* 声明硬件准备函数。 */
static void BLDC_AllOutputsOff(void);   /* 声明关闭输出函数。 */
static void BLDC_SetDuty(uint16_t duty); /* 声明占空比设置函数。 */
static void BLDC_ApplyStep(uint8_t step); /* 声明换相步应用函数。 */
static void BLDC_Commutate(BLDC_CommReason_t reason); /* 声明带触发原因的前进一步函数。 */
static void BLDC_ClearCommTrace(void);  /* 声明换相调试记录清零函数。 */
static void BLDC_RecordCommutation(BLDC_CommReason_t reason, uint8_t step_before, uint8_t step_after, uint32_t elapsed_ticks); /* 声明换相事件记录函数。 */
static void BLDC_ResetBusVoltage(void); /* 声明母线电压运行数据清零函数。 */
static uint8_t BLDC_SampleBusVoltage(void); /* 声明 ADC1 规则组母线电压采样函数。 */
static void BLDC_CheckBusVoltage(uint8_t immediate); /* 声明母线欠压和过压保护检查函数。 */
static void BLDC_UpdateDynamicIntegralLimit(void); /* 声明 VESC 风格动态积分阈值更新函数。 */
static void BLDC_ServiceSlow(void);     /* 声明十毫秒后台服务函数。 */
static void BLDC_ResetCurrentData(void); /* 声明电流采样与诊断数据清零函数。 */
static void BLDC_TryProcessCurrentSample(void); /* 声明三相注入样本配对处理函数。 */
static int32_t BLDC_ConvertCurrentMa(int32_t adc_difference, int32_t nominal_gain_x1000, uint16_t scale_permille); /* 声明电流定点换算函数。 */
static int32_t BLDC_AbsoluteCurrentMa(int32_t current_ma); /* 声明电流绝对值函数。 */
static void BLDC_ControlTick(void);     /* 声明快速控制函数。 */
static void BLDC_AccumulateBemf(void);  /* 声明反电动势积分函数。 */
static void BLDC_FinishCurrentCalibration(void); /* 声明电流零点校准完成函数。 */
static void BLDC_UpdatePhaseCurrents(void); /* 声明三相电流换算和保护函数。 */
static void BLDC_RunCurrentControl(void); /* 声明电流滤波和 PI 控制函数。 */
static void BLDC_EnterFault(BLDC_Fault_t fault); /* 声明带故障码的停机函数。 */
static uint32_t BLDC_GetTimeoutTicks(uint32_t ticks); /* 声明超时计算函数。 */
static int32_t BLDC_GetBemfDiff(void);  /* 声明反电动势差值函数。 */

static uint16_t adc2_dma_buffer[3];     /* 保存 U、V、W 的 DMA 数据。 */
static BEMF_Sample_t g_bemf;            /* 保存反电动势调试数据。 */
static BLDC_Current_t g_current;         /* 保存三相电流采样和校准数据。 */
static BLDC_CurrentScale_t g_current_scale = /* 创建三相电流增益校准实例。 */
{ /* 三相电流增益校准初始化开始。 */
    .u_permille = BLDC_CURRENT_SCALE_DEFAULT_PERMILLE, /* 设置 U 相默认增益为标称值。 */
    .v_permille = BLDC_CURRENT_SCALE_DEFAULT_PERMILLE, /* 设置 V 相默认增益为标称值。 */
    .w_permille = BLDC_CURRENT_SCALE_DEFAULT_PERMILLE /* 设置 W 相默认增益为标称值。 */
}; /* 三相电流增益校准初始化结束。 */
static BLDC_BusVoltage_t g_bus_voltage; /* 保存母线电压采样、滤波和保护数据。 */
static BLDC_CurrentControl_t g_current_control = /* 创建电流 PI 控制器实例。 */
{ /* 电流 PI 控制器初始化开始。 */
    .target_ma = BLDC_CURRENT_TARGET_DEFAULT_MA, /* 设置首次测试目标电流。 */
    .output_duty = BLDC_TEST_DUTY,      /* 设置首次测试初始占空比。 */
    .overcurrent_limit_ma = BLDC_OVERCURRENT_LIMIT_DEFAULT_MA /* 设置默认软件过流保护阈值。 */
}; /* 电流 PI 控制器初始化结束。 */
static volatile int32_t get_diff;       /* 保存归一化反电动势差值。 */
static volatile BLDC_CommTrace_t g_comm_trace[BLDC_COMM_TRACE_LENGTH]; /* 保存最近六十四次换相事件。 */
static volatile uint16_t g_comm_trace_index; /* 保存下一次换相事件的写入位置。 */
static volatile uint32_t g_comm_trace_sequence; /* 保存换相事件累计序号。 */
static BLDC_Control_t g_bldc =          /* 创建控制器实例。 */
{ /* 控制器初始化开始。 */
    .state = BLDC_STATE_STOP,           /* 初始状态设为停止。 */
    .step = 1U,                         /* 初始步号设为 1。 */
    .open_loop_ticks = BLDC_OPEN_LOOP_START_TICKS, /* 设置初始开环周期。 */
    .timeout_ticks = BLDC_TIMEOUT_MAX_TICKS, /* 设置初始闭环超时。 */
    .phase_advance_max_permille = BLDC_PHASE_ADVANCE_DEFAULT_PERMILLE, /* 设置默认最大换相提前量。 */
    .fault = BLDC_FAULT_NONE            /* 初始故障码设为无故障。 */
}; /* 控制器初始化结束。 */

void BLDC_test(void)                    /* 实现 Integrate 测试入口。 */
{ /* 测试入口开始。 */
    BLDC_ClearCommTrace();              /* 清空上一次运行留下的换相调试记录。 */
    BLDC_ResetBusVoltage();             /* 清空上一次运行留下的母线电压数据。 */
    BLDC_ResetCurrentData();            /* 清空上一次运行留下的电流采样和诊断数据。 */
    g_bldc.fault = BLDC_FAULT_NONE;     /* 在硬件准备前清除上一次运行的故障码。 */
    g_bldc.state = BLDC_STATE_STOP;     /* 在完成母线电压检查前保持停止状态。 */
    g_bldc.integral_limit = 0U;         /* 清零上一次运行的动态积分阈值。 */
    g_bldc.integral_limit_unadvanced = 0U; /* 清零上一次运行的未提前动态积分阈值。 */
    g_bldc.integral_limit_base = 0U;    /* 清零上一次运行的基础磁链积分项。 */
    g_bldc.integral_coupling_reference = 0U; /* 清零上一次运行的母线耦合积分项。 */
    g_bldc.integral_limit_max = 0U;     /* 清零上一次运行的动态积分阈值上限。 */
    g_bldc.integral_calibration_bus_mv = 0U; /* 清零上一次积分标定母线电压。 */
    g_bldc.integral_calibration_erpm = 0U; /* 清零上一次积分标定电角转速。 */
    g_bldc.phase_advance_permille = 0U; /* 清零上一次运行的实际换相提前量。 */
    BLDC_HardwarePrepare();             /* 准备 ADC、DMA 和 TIM1。 */
    if (BLDC_SampleBusVoltage() == 0U)  /* 执行首次母线电压采样并检查 ADC 状态。 */
    { /* 首次母线电压采样失败处理开始。 */
        BLDC_EnterFault(BLDC_FAULT_VBUS_ADC); /* ADC1 规则组失败时禁止电机启动。 */
    } /* 首次母线电压采样失败处理结束。 */
    else                                /* 处理首次母线电压采样成功。 */
    { /* 首次母线电压保护检查开始。 */
        BLDC_CheckBusVoltage(1U);       /* 启动前立即执行 19 V 到 31 V 范围检查。 */
    } /* 首次母线电压保护检查结束。 */
    BLDC_SetDuty(BLDC_TEST_DUTY);       /* 设置测试占空比和采样点。 */
    g_current_control.ramped_target_ma = 0L; /* 从零开始平滑增加目标电流。 */
    g_current_control.filtered_current_ma = 0L; /* 清零上次运行的滤波电流。 */
    g_current_control.error_ma = 0L;    /* 清零上次运行的电流误差。 */
    g_current_control.proportional_duty = 0L; /* 清零上次运行的比例项。 */
    g_current_control.integral_accumulator = BLDC_TEST_DUTY * BLDC_CURRENT_KI_DIVISOR; /* 用初始占空比初始化积分项。 */
    g_current_control.output_duty = BLDC_TEST_DUTY; /* 保存初始占空比输出。 */
    g_current_control.filter_initialized = 0U; /* 要求首次有效电流重新初始化滤波器。 */
    if (g_bldc.state != BLDC_STATE_FAULT) /* 检查启动母线电压是否处于允许范围。 */
    { /* 允许启动处理开始。 */
        g_bldc.state = BLDC_STATE_CALIBRATE; /* 先在功率输出关闭时校准电流零点。 */
    } /* 允许启动处理结束。 */
    g_bldc.state_ticks = 0U;            /* 清零状态计时。 */
    __HAL_TIM_SET_COUNTER(&htim1, 0U);  /* 清零 TIM1 计数器。 */
    if (HAL_TIM_Base_Start(&htim1) != HAL_OK) /* 启动 TIM1。 */
    { /* TIM1 启动失败处理开始。 */
        Error_Handler();                /* 进入统一错误处理。 */
    } /* TIM1 启动失败处理结束。 */
    __HAL_TIM_MOE_DISABLE(&htim1);      /* 校准期间保持高级定时器主输出关闭。 */
    while (1)                           /* 保持后台任务运行。 */
    { /* 后台循环开始。 */
        BLDC_ServiceSlow();             /* 每十毫秒采样母线电压并更新动态积分阈值。 */
        if (g_bldc.state == BLDC_STATE_FAULT) /* 检查故障状态。 */
        { /* 故障状态处理开始。 */
            BLDC_AllOutputsOff();       /* 确保全部桥臂关闭。 */
        } /* 故障状态处理结束。 */
        __NOP();                        /* 保留稳定调试位置。 */
    } /* 后台循环结束。 */
} /* 测试入口结束。 */

static void BLDC_HardwarePrepare(void)  /* 实现硬件准备。 */
{ /* 硬件准备开始。 */
    BLDC_AllOutputsOff();               /* 首先关闭全部桥臂。 */
    if (HAL_OPAMP_SelfCalibrate(&hopamp1) != HAL_OK) /* 执行 OPAMP1 内部偏置校准。 */
    { /* OPAMP1 校准失败处理开始。 */
        Error_Handler();                /* 进入统一错误处理。 */
    } /* OPAMP1 校准失败处理结束。 */
    if (HAL_OPAMP_SelfCalibrate(&hopamp2) != HAL_OK) /* 执行 OPAMP2 内部偏置校准。 */
    { /* OPAMP2 校准失败处理开始。 */
        Error_Handler();                /* 进入统一错误处理。 */
    } /* OPAMP2 校准失败处理结束。 */
    if (HAL_OPAMP_SelfCalibrate(&hopamp3) != HAL_OK) /* 执行 OPAMP3 内部偏置校准。 */
    { /* OPAMP3 校准失败处理开始。 */
        Error_Handler();                /* 进入统一错误处理。 */
    } /* OPAMP3 校准失败处理结束。 */
    if (HAL_OPAMP_Start(&hopamp1) != HAL_OK) /* 启动 U 相电流检测运放。 */
    { /* OPAMP1 启动失败处理开始。 */
        Error_Handler();                /* 进入统一错误处理。 */
    } /* OPAMP1 启动失败处理结束。 */
    if (HAL_OPAMP_Start(&hopamp2) != HAL_OK) /* 启动 V 相电流检测运放。 */
    { /* OPAMP2 启动失败处理开始。 */
        Error_Handler();                /* 进入统一错误处理。 */
    } /* OPAMP2 启动失败处理结束。 */
    if (HAL_OPAMP_Start(&hopamp3) != HAL_OK) /* 启动 W 相电流检测运放。 */
    { /* OPAMP3 启动失败处理开始。 */
        Error_Handler();                /* 进入统一错误处理。 */
    } /* OPAMP3 启动失败处理结束。 */
    HAL_Delay(1U);                      /* 等待三路高速运放输出稳定。 */
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) /* 校准 ADC1。 */
    { /* ADC1 校准失败处理开始。 */
        Error_Handler();                /* 进入统一错误处理。 */
    } /* ADC1 校准失败处理结束。 */
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) /* 校准 ADC2。 */
    { /* ADC 校准失败处理开始。 */
        Error_Handler();                /* 进入统一错误处理。 */
    } /* ADC 校准失败处理结束。 */
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_dma_buffer, 3U) != HAL_OK) /* 启动 ADC2 DMA。 */
    { /* DMA 启动失败处理开始。 */
        Error_Handler();                /* 进入统一错误处理。 */
    } /* DMA 启动失败处理结束。 */
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) /* 启动 ADC1 两路注入电流采样。 */
    { /* ADC1 注入组启动失败处理开始。 */
        Error_Handler();                /* 进入统一错误处理。 */
    } /* ADC1 注入组启动失败处理结束。 */
    if (HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK) /* 启动 ADC2 单路注入电流采样。 */
    { /* ADC2 注入组启动失败处理开始。 */
        Error_Handler();                /* 进入统一错误处理。 */
    } /* ADC2 注入组启动失败处理结束。 */
    __HAL_TIM_SET_COUNTER(&htim2, 0U);  /* 清零 TIM2 计数器。 */
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK) /* 启动诊断用 TIM2。 */
    { /* TIM2 启动失败处理开始。 */
        Error_Handler();                /* 进入统一错误处理。 */
    } /* TIM2 启动失败处理结束。 */
    SET_BIT(htim1.Instance->CR2, TIM_CR2_CCPC); /* 启用输出控制预装载。 */
    __HAL_TIM_MOE_DISABLE(&htim1);      /* 保持 TIM1 主输出关闭。 */
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE | TIM_FLAG_COM); /* 清除残留标志。 */
} /* 硬件准备结束。 */

static void BLDC_ResetBusVoltage(void)  /* 实现母线电压运行数据清零。 */
{ /* 母线电压运行数据清零开始。 */
    g_bus_voltage.raw = 0U;             /* 清零 VBUS_ADC 原始值。 */
    g_bus_voltage.pot_raw = 0U;         /* 清零 POT_ADC 原始值。 */
    g_bus_voltage.measured_mv = 0U;     /* 清零本次母线电压换算值。 */
    g_bus_voltage.filtered_mv = 0U;     /* 清零母线电压低通滤波值。 */
    g_bus_voltage.sample_count = 0U;    /* 清零母线电压成功采样次数。 */
    g_bus_voltage.last_sample_ms = 0U;  /* 清零母线电压后台采样时间。 */
    g_bus_voltage.initialized = 0U;     /* 要求首次有效样本重新初始化滤波器。 */
    g_bus_voltage.low_count = 0U;       /* 清零连续欠压确认次数。 */
    g_bus_voltage.high_count = 0U;      /* 清零连续过压确认次数。 */
    g_bus_voltage.adc_error_count = 0U; /* 清零 ADC1 规则组连续错误次数。 */
} /* 母线电压运行数据清零结束。 */

static void BLDC_ResetCurrentData(void) /* 实现电流采样与诊断数据清零。 */
{ /* 电流采样与诊断数据清零开始。 */
    uint8_t step_index;                 /* 保存正在清零的六步诊断下标。 */
    g_current.raw_u = 0U;               /* 清零 U 相 ADC 原始值。 */
    g_current.raw_v = 0U;               /* 清零 V 相 ADC 原始值。 */
    g_current.raw_w = 0U;               /* 清零 W 相 ADC 原始值。 */
    g_current.offset_u = 0U;            /* 清零 U 相零电流偏置。 */
    g_current.offset_v = 0U;            /* 清零 V 相零电流偏置。 */
    g_current.offset_w = 0U;            /* 清零 W 相零电流偏置。 */
    g_current.minimum_u = BLDC_ADC_FULL_SCALE; /* 将 U 相校准最小值初始化到 ADC 满量程。 */
    g_current.minimum_v = BLDC_ADC_FULL_SCALE; /* 将 V 相校准最小值初始化到 ADC 满量程。 */
    g_current.minimum_w = BLDC_ADC_FULL_SCALE; /* 将 W 相校准最小值初始化到 ADC 满量程。 */
    g_current.maximum_u = 0U;           /* 清零 U 相校准最大值。 */
    g_current.maximum_v = 0U;           /* 清零 V 相校准最大值。 */
    g_current.maximum_w = 0U;           /* 清零 W 相校准最大值。 */
    g_current.zero_noise_u = 0U;        /* 清零 U 相零点峰峰噪声。 */
    g_current.zero_noise_v = 0U;        /* 清零 V 相零点峰峰噪声。 */
    g_current.zero_noise_w = 0U;        /* 清零 W 相零点峰峰噪声。 */
    g_current.current_u_ma = 0L;        /* 清零 U 相换算电流。 */
    g_current.current_v_ma = 0L;        /* 清零 V 相换算电流。 */
    g_current.current_w_ma = 0L;        /* 清零 W 相换算电流。 */
    g_current.motor_current_ma = 0L;    /* 清零有效回流相电流。 */
    g_current.floating_current_ma = 0L; /* 清零悬空相诊断电流。 */
    for (step_index = 0U; step_index < 6U; step_index++) /* 遍历六个换相扇区。 */
    { /* 单个换相扇区诊断清零开始。 */
        g_current.step_current_ma[step_index] = 0L; /* 清零本扇区滤波回流电流。 */
    } /* 单个换相扇区诊断清零结束。 */
    g_current.sum_u = 0U;               /* 清零 U 相零点累计值。 */
    g_current.sum_v = 0U;               /* 清零 V 相零点累计值。 */
    g_current.sum_w = 0U;               /* 清零 W 相零点累计值。 */
    g_current.count_adc1 = 0U;          /* 清零 ADC1 校准采样计数。 */
    g_current.count_adc2 = 0U;          /* 清零 ADC2 校准采样计数。 */
    g_current.paired_sample_count = 0U; /* 清零三相成功配对样本数。 */
    g_current.sync_error_count = 0U;    /* 清零三相样本同步错误计数。 */
    g_current.blanked_sample_count = 0U; /* 清零电流换相消隐样本累计值。 */
    g_current.calibrated = 0U;          /* 标记三相电流尚未完成零点校准。 */
    g_current.overcurrent_count = 0U;   /* 清零软件过流确认计数。 */
    g_current.adc1_ready = 0U;          /* 清除 ADC1 样本到达标志。 */
    g_current.adc2_ready = 0U;          /* 清除 ADC2 样本到达标志。 */
    g_current.blanking_samples = 0U;    /* 清零电流换相消隐剩余样本数。 */
    g_current.zero_noise_ok = 0U;       /* 标记零点峰峰噪声尚未通过检查。 */
} /* 电流采样与诊断数据清零结束。 */

static uint8_t BLDC_SampleBusVoltage(void) /* 实现 ADC1 规则组母线电压采样。 */
{ /* 母线电压采样开始。 */
    HAL_StatusTypeDef status;           /* 保存 HAL ADC 调用结果。 */
    uint32_t converted_mv;              /* 保存按照分压比换算的母线电压。 */
    uint32_t difference;                /* 保存低通滤波输入与输出的差值。 */
    uint64_t conversion_numerator;      /* 保存不会溢出的母线电压换算分子。 */
    status = HAL_ADC_Start(&hadc1);     /* 启动 ADC1 两顺位规则组软件转换。 */
    if (status != HAL_OK)               /* 检查 ADC1 规则组是否成功启动。 */
    { /* ADC1 规则组启动失败处理开始。 */
        return 0U;                      /* 返回采样失败并保留注入组运行状态。 */
    } /* ADC1 规则组启动失败处理结束。 */
    status = HAL_ADC_PollForConversion(&hadc1, BLDC_VBUS_ADC_TIMEOUT_MS); /* 等待第一顺位 VBUS_ADC 转换完成。 */
    if (status != HAL_OK)               /* 检查 VBUS_ADC 转换是否按时完成。 */
    { /* VBUS_ADC 转换失败处理开始。 */
        return 0U;                      /* 返回采样失败且不停止 ADC 注入组。 */
    } /* VBUS_ADC 转换失败处理结束。 */
    g_bus_voltage.raw = (uint16_t)HAL_ADC_GetValue(&hadc1); /* 读取第一顺位 VBUS_ADC 原始值。 */
    status = HAL_ADC_PollForConversion(&hadc1, BLDC_VBUS_ADC_TIMEOUT_MS); /* 等待第二顺位 POT_ADC 转换完成。 */
    if (status != HAL_OK)               /* 检查 POT_ADC 转换是否按时完成。 */
    { /* POT_ADC 转换失败处理开始。 */
        return 0U;                      /* 返回采样失败且不调用会停止注入组的 HAL_ADC_Stop。 */
    } /* POT_ADC 转换失败处理结束。 */
    g_bus_voltage.pot_raw = (uint16_t)HAL_ADC_GetValue(&hadc1); /* 读取第二顺位并清除本次规则组结果。 */
    conversion_numerator = (uint64_t)g_bus_voltage.raw * /* 使用原始 ADC 值开始计算真实母线电压。 */
                           (uint64_t)BLDC_ADC_REFERENCE_MV * /* 乘以 ADC 模拟参考电压。 */
                           (uint64_t)BLDC_VBUS_DIVIDER_NUMERATOR * /* 乘以 21:1 分压倍率分子。 */
                           (uint64_t)BLDC_VBUS_CALIBRATION_PERMILLE; /* 乘以可调整的校准系数。 */
    converted_mv = (uint32_t)(conversion_numerator / /* 将完整换算分子除以统一分母。 */
                   ((uint64_t)BLDC_ADC_FULL_SCALE * /* 除以十二位 ADC 满量程。 */
                    (uint64_t)BLDC_VBUS_DIVIDER_DENOMINATOR * /* 除以分压倍率分母。 */
                    1000ULL));          /* 除以千分比校准系数的基准。 */
    g_bus_voltage.measured_mv = converted_mv; /* 保存本次未经低通滤波的母线电压。 */
    if (g_bus_voltage.initialized == 0U) /* 检查是否为首次有效母线电压样本。 */
    { /* 母线电压滤波器初始化开始。 */
        g_bus_voltage.filtered_mv = converted_mv; /* 首次样本直接作为滤波初值。 */
        g_bus_voltage.initialized = 1U; /* 标记母线电压滤波器已经初始化。 */
    } /* 母线电压滤波器初始化结束。 */
    else if (converted_mv >= g_bus_voltage.filtered_mv) /* 检查母线电压是否正在上升。 */
    { /* 母线电压上升滤波开始。 */
        difference = converted_mv - g_bus_voltage.filtered_mv; /* 计算向上的电压差。 */
        g_bus_voltage.filtered_mv += difference >> BLDC_VBUS_FILTER_SHIFT; /* 以八分之一系数向上更新。 */
    } /* 母线电压上升滤波结束。 */
    else                                /* 处理母线电压下降。 */
    { /* 母线电压下降滤波开始。 */
        difference = g_bus_voltage.filtered_mv - converted_mv; /* 计算向下的电压差。 */
        g_bus_voltage.filtered_mv -= difference >> BLDC_VBUS_FILTER_SHIFT; /* 以八分之一系数向下更新。 */
    } /* 母线电压下降滤波结束。 */
    g_bus_voltage.sample_count++;       /* 增加母线电压成功采样次数。 */
    g_bus_voltage.adc_error_count = 0U; /* 成功采样后清零连续 ADC 错误次数。 */
    return 1U;                          /* 返回本次母线电压采样成功。 */
} /* 母线电压采样结束。 */

static void BLDC_CheckBusVoltage(uint8_t immediate) /* 实现母线欠压和过压保护检查。 */
{ /* 母线电压保护检查开始。 */
    if (g_bldc.state == BLDC_STATE_FAULT) /* 检查控制器是否已经锁存其他故障。 */
    { /* 已有故障处理开始。 */
        return;                         /* 保留首先发生的故障原因并结束检查。 */
    } /* 已有故障处理结束。 */
    if (g_bus_voltage.measured_mv < BLDC_VBUS_LOW_LIMIT_MV) /* 使用未滤波值检查母线电压是否低于 19 V。 */
    { /* 欠压样本处理开始。 */
        g_bus_voltage.high_count = 0U;  /* 欠压时清零连续过压确认次数。 */
        if (immediate != 0U)            /* 检查是否正在执行启动前立即保护。 */
        { /* 启动前欠压确认开始。 */
            g_bus_voltage.low_count = BLDC_VBUS_FAULT_CONFIRM_SAMPLES; /* 立即满足欠压确认次数。 */
        } /* 启动前欠压确认结束。 */
        else if (g_bus_voltage.low_count < BLDC_VBUS_FAULT_CONFIRM_SAMPLES) /* 检查欠压计数是否尚未饱和。 */
        { /* 运行中欠压计数开始。 */
            g_bus_voltage.low_count++;  /* 增加连续欠压确认次数。 */
        } /* 运行中欠压计数结束。 */
        if (g_bus_voltage.low_count >= BLDC_VBUS_FAULT_CONFIRM_SAMPLES) /* 检查欠压是否已经确认。 */
        { /* 欠压停机开始。 */
            BLDC_EnterFault(BLDC_FAULT_UNDERVOLTAGE); /* 锁存欠压故障并立即关闭功率输出。 */
        } /* 欠压停机结束。 */
    } /* 欠压样本处理结束。 */
    else if (g_bus_voltage.measured_mv > BLDC_VBUS_HIGH_LIMIT_MV) /* 使用未滤波值检查母线电压是否高于 31 V。 */
    { /* 过压样本处理开始。 */
        g_bus_voltage.low_count = 0U;   /* 过压时清零连续欠压确认次数。 */
        if (immediate != 0U)            /* 检查是否正在执行启动前立即保护。 */
        { /* 启动前过压确认开始。 */
            g_bus_voltage.high_count = BLDC_VBUS_FAULT_CONFIRM_SAMPLES; /* 立即满足过压确认次数。 */
        } /* 启动前过压确认结束。 */
        else if (g_bus_voltage.high_count < BLDC_VBUS_FAULT_CONFIRM_SAMPLES) /* 检查过压计数是否尚未饱和。 */
        { /* 运行中过压计数开始。 */
            g_bus_voltage.high_count++; /* 增加连续过压确认次数。 */
        } /* 运行中过压计数结束。 */
        if (g_bus_voltage.high_count >= BLDC_VBUS_FAULT_CONFIRM_SAMPLES) /* 检查过压是否已经确认。 */
        { /* 过压停机开始。 */
            BLDC_EnterFault(BLDC_FAULT_OVERVOLTAGE); /* 锁存过压故障并立即关闭功率输出。 */
        } /* 过压停机结束。 */
    } /* 过压样本处理结束。 */
    else                                /* 处理母线电压处于允许范围。 */
    { /* 正常母线电压处理开始。 */
        g_bus_voltage.low_count = 0U;   /* 清零连续欠压确认次数。 */
        g_bus_voltage.high_count = 0U;  /* 清零连续过压确认次数。 */
    } /* 正常母线电压处理结束。 */
} /* 母线电压保护检查结束。 */

static void BLDC_UpdateDynamicIntegralLimit(void) /* 实现 VESC 风格动态积分阈值更新。 */
{ /* 动态积分阈值更新开始。 */
    uint32_t speed_erpm;                /* 保存参与耦合项计算的电角转速。 */
    uint32_t running_limit;             /* 保存限幅前的运行积分阈值。 */
    uint32_t phase_advance_permille;    /* 保存按转速计算的积分阈值提前千分比。 */
    uint32_t phase_reduction;           /* 保存换相提前需要减去的积分阈值。 */
    uint32_t maximum_reduction;         /* 保存基础磁链安全下限允许的最大减小量。 */
    uint32_t minimum_advanced_limit;    /* 保存换相提前后的积分阈值安全下限。 */
    uint64_t coupling_numerator;        /* 保存母线耦合项计算分子。 */
    uint64_t coupling_denominator;      /* 保存母线耦合项计算分母。 */
    uint32_t coupling_term;             /* 保存随电压和转速变化的耦合积分项。 */
    if ((g_bldc.integral_limit_base == 0U) || /* 检查基础磁链积分项是否已经标定。 */
        (g_bldc.integral_calibration_bus_mv == 0U) || /* 检查标定母线电压是否有效。 */
        (g_bldc.integral_calibration_erpm == 0U)) /* 检查标定电角转速是否有效。 */
    { /* 动态积分参数尚未准备处理开始。 */
        return;                         /* 保持当前积分阈值并等待开环标定完成。 */
    } /* 动态积分参数尚未准备处理结束。 */
    speed_erpm = g_bldc.electrical_rpm; /* 读取当前估算电角转速。 */
    if (speed_erpm < BLDC_DYNAMIC_INTEGRAL_MIN_ERPM) /* 检查转速是否低于耦合计算下限。 */
    { /* 最低转速限制开始。 */
        speed_erpm = BLDC_DYNAMIC_INTEGRAL_MIN_ERPM; /* 使用最低电角转速避免除零和过大阈值。 */
    } /* 最低转速限制结束。 */
    coupling_numerator = (uint64_t)g_bldc.integral_coupling_reference * /* 使用标定工况耦合积分项。 */
                         (uint64_t)g_bus_voltage.filtered_mv * /* 按当前母线电压同比例调整。 */
                         (uint64_t)g_bldc.integral_calibration_erpm; /* 按标定电角转速建立反比基准。 */
    coupling_denominator = (uint64_t)g_bldc.integral_calibration_bus_mv * /* 除以标定母线电压。 */
                           (uint64_t)speed_erpm; /* 除以当前电角转速形成 VBUS/ERPM 耦合项。 */
    coupling_term = (uint32_t)(coupling_numerator / coupling_denominator); /* 计算动态母线耦合积分项。 */
    running_limit = g_bldc.integral_limit_base + coupling_term; /* 将基础磁链项和动态耦合项相加。 */
    if (running_limit < g_bldc.integral_limit_base) /* 检查无符号加法是否发生溢出。 */
    { /* 动态积分阈值溢出处理开始。 */
        running_limit = g_bldc.integral_limit_max; /* 溢出时直接限制到允许最大值。 */
    } /* 动态积分阈值溢出处理结束。 */
    if (running_limit > g_bldc.integral_limit_max) /* 检查动态积分阈值是否超过上限。 */
    { /* 动态积分阈值上限处理开始。 */
        running_limit = g_bldc.integral_limit_max; /* 限制到标定阈值的百分之一百六十。 */
    } /* 动态积分阈值上限处理结束。 */
    if (running_limit < g_bldc.integral_limit_base) /* 检查动态积分阈值是否低于基础磁链项。 */
    { /* 动态积分阈值下限处理开始。 */
        running_limit = g_bldc.integral_limit_base; /* 限制到基础磁链积分项。 */
    } /* 动态积分阈值下限处理结束。 */
    g_bldc.integral_limit_unadvanced = running_limit; /* 保存叠加换相提前量之前的动态阈值。 */
    phase_advance_permille = 0U;        /* 默认在启动和接管阶段不加入换相提前量。 */
    if ((g_bldc.state == BLDC_STATE_INTEGRATE) && /* 仅允许纯积分闭环使用换相提前。 */
        (speed_erpm > BLDC_PHASE_ADVANCE_START_ERPM) && /* 检查当前转速是否进入提前区间。 */
        (g_bldc.phase_advance_max_permille > 0U)) /* 检查用户是否启用了换相提前。 */
    { /* 转速相关换相提前计算开始。 */
        if (speed_erpm >= BLDC_PHASE_ADVANCE_FULL_ERPM) /* 检查是否达到最大提前转速。 */
        { /* 最大提前量处理开始。 */
            phase_advance_permille = g_bldc.phase_advance_max_permille; /* 使用用户设置的最大提前千分比。 */
        } /* 最大提前量处理结束。 */
        else                            /* 处理开始转速与最大转速之间的线性区间。 */
        { /* 线性提前量计算开始。 */
            phase_advance_permille = ((uint32_t)g_bldc.phase_advance_max_permille * /* 使用用户最大提前量作为线性终点。 */
                                       (speed_erpm - BLDC_PHASE_ADVANCE_START_ERPM)) / /* 计算超过起始转速的部分。 */
                                      (BLDC_PHASE_ADVANCE_FULL_ERPM - BLDC_PHASE_ADVANCE_START_ERPM); /* 按完整转速区间归一化。 */
        } /* 线性提前量计算结束。 */
    } /* 转速相关换相提前计算结束。 */
    minimum_advanced_limit = (g_bldc.integral_limit_base * /* 使用基础磁链积分项建立安全下限。 */
                              BLDC_PHASE_ADVANCE_MIN_BASE_PERCENT) / 100U; /* 允许最低下降到基础项的百分之八十。 */
    if (minimum_advanced_limit < BLDC_MIN_INTEGRAL_LIMIT) /* 检查提前阈值安全下限是否过小。 */
    { /* 提前阈值绝对下限处理开始。 */
        minimum_advanced_limit = BLDC_MIN_INTEGRAL_LIMIT; /* 保持原有最小可靠积分阈值。 */
    } /* 提前阈值绝对下限处理结束。 */
    if ((phase_advance_permille > 0U) && /* 检查本周期是否需要应用换相提前。 */
        (running_limit > minimum_advanced_limit)) /* 检查当前阈值是否仍有安全下降空间。 */
    { /* 积分阈值提前应用开始。 */
        phase_reduction = (uint32_t)(((uint64_t)running_limit * /* 使用未提前阈值作为计算基准。 */
                                      (uint64_t)phase_advance_permille) / BLDC_PERMILLE_SCALE); /* 按千分比计算阈值减小量。 */
        maximum_reduction = running_limit - minimum_advanced_limit; /* 计算安全下限允许的最大减小量。 */
        if (phase_reduction > maximum_reduction) /* 检查计算结果是否会越过安全下限。 */
        { /* 换相提前安全限幅开始。 */
            phase_reduction = maximum_reduction; /* 限制减小量以保持基础磁链安全余量。 */
        } /* 换相提前安全限幅结束。 */
        running_limit -= phase_reduction; /* 降低积分触发阈值以实现更早换相。 */
    } /* 积分阈值提前应用结束。 */
    g_bldc.phase_advance_permille = (uint16_t)phase_advance_permille; /* 保存当前实际请求的提前千分比。 */
    g_bldc.integral_limit = running_limit; /* 应用叠加换相提前后的运行积分阈值。 */
} /* 动态积分阈值更新结束。 */

static void BLDC_ServiceSlow(void)      /* 实现十毫秒后台服务。 */
{ /* 十毫秒后台服务开始。 */
    uint32_t now_ms;                    /* 保存当前 HAL 系统毫秒时间。 */
    now_ms = HAL_GetTick();             /* 读取当前系统毫秒时间。 */
    if ((now_ms - g_bus_voltage.last_sample_ms) < BLDC_VBUS_SAMPLE_PERIOD_MS) /* 检查十毫秒周期是否到达。 */
    { /* 后台服务尚未到期处理开始。 */
        return;                         /* 保持主循环非阻塞并等待下一周期。 */
    } /* 后台服务尚未到期处理结束。 */
    g_bus_voltage.last_sample_ms = now_ms; /* 保存本次后台服务时间。 */
    if (BLDC_SampleBusVoltage() == 0U)  /* 启动并读取一次 ADC1 规则组。 */
    { /* ADC1 规则组采样失败处理开始。 */
        if (g_bus_voltage.adc_error_count < BLDC_VBUS_ADC_ERROR_CONFIRM_SAMPLES) /* 检查错误计数是否尚未饱和。 */
        { /* ADC1 规则组错误计数开始。 */
            g_bus_voltage.adc_error_count++; /* 增加连续 ADC1 规则组错误次数。 */
        } /* ADC1 规则组错误计数结束。 */
        if ((g_bus_voltage.adc_error_count >= BLDC_VBUS_ADC_ERROR_CONFIRM_SAMPLES) && /* 检查 ADC 错误是否已经确认。 */
            (g_bldc.state != BLDC_STATE_FAULT)) /* 检查是否尚未锁存其他故障。 */
        { /* ADC1 规则组故障停机开始。 */
            BLDC_EnterFault(BLDC_FAULT_VBUS_ADC); /* 锁存母线 ADC 故障并立即停机。 */
        } /* ADC1 规则组故障停机结束。 */
        return;                         /* 本周期不再使用无效母线电压更新阈值。 */
    } /* ADC1 规则组采样失败处理结束。 */
    BLDC_CheckBusVoltage(0U);           /* 连续三次确认运行中的欠压或过压。 */
    if (g_bldc.state != BLDC_STATE_FAULT) /* 检查电压保护是否允许继续运行。 */
    { /* 动态积分阈值更新许可开始。 */
        BLDC_UpdateDynamicIntegralLimit(); /* 按最新电压和电角转速更新积分阈值。 */
    } /* 动态积分阈值更新许可结束。 */
} /* 十毫秒后台服务结束。 */

static void BLDC_AllOutputsOff(void)    /* 实现全部输出关闭。 */
{ /* 输出关闭开始。 */
    __HAL_TIM_MOE_DISABLE(&htim1);      /* 立即关闭主输出。 */
    CLEAR_BIT(htim1.Instance->CCER, BLDC_ALL_OUTPUTS); /* 清除全部输出使能位。 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U); /* 清零 U 相比较值。 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U); /* 清零 V 相比较值。 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U); /* 清零 W 相比较值。 */
} /* 输出关闭结束。 */

static void BLDC_SetDuty(uint16_t duty) /* 实现占空比和 ADC 触发设置。 */
{ /* 占空比设置开始。 */
    uint32_t period;                    /* 保存 PWM 周期计数。 */
    uint32_t pulse;                     /* 保存 PWM 比较值。 */
    uint32_t sample_pulse;              /* 保存 ADC 触发比较值。 */
    if (duty > BLDC_DUTY_SCALE)         /* 检查占空比上限。 */
    { /* 占空比限幅开始。 */
        duty = BLDC_DUTY_SCALE;         /* 限制到满量程。 */
    } /* 占空比限幅结束。 */
    period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U; /* 获取 PWM 周期。 */
    pulse = (period * (uint32_t)duty) / BLDC_DUTY_SCALE; /* 换算 PWM 比较值。 */
    sample_pulse = (pulse > BLDC_ADC_SAMPLE_DELAY_COUNTS) ? /* 判断导通窗口宽度。 */
                   (pulse - BLDC_ADC_SAMPLE_DELAY_COUNTS) : 1U; /* 计算下降计数段触发点。 */
    g_bldc.duty = duty;                 /* 保存占空比命令。 */
    g_bldc.pwm_pulse = pulse;           /* 保存 PWM 比较值。 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse); /* 设置 U 相比较值。 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse); /* 设置 V 相比较值。 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse); /* 设置 W 相比较值。 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, sample_pulse); /* 设置 ADC 触发点。 */
    if ((htim1.Instance->CR1 & TIM_CR1_CEN) == 0U) /* 判断 TIM1 尚未启动。 */
    { /* 启动前装载开始。 */
        if (HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE) != HAL_OK) /* 装载 CCR。 */
        { /* 更新事件失败处理开始。 */
            Error_Handler();            /* 进入统一错误处理。 */
        } /* 更新事件失败处理结束。 */
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE); /* 清除更新标志。 */
    } /* 启动前装载结束。 */
} /* 占空比设置结束。 */

static void BLDC_ApplyStep(uint8_t step) /* 实现 COM 原子换步。 */
{ /* 换相步应用开始。 */
    uint32_t source_channel;            /* 保存源相 LL 通道。 */
    uint32_t sink_low_output;           /* 保存回流相低桥位。 */
    uint32_t enabled_outputs;           /* 保存输出使能组合。 */
    if ((step < BLDC_VALID_STEP_MIN) || (step > BLDC_VALID_STEP_MAX)) /* 检查步号。 */
    { /* 无效步号处理开始。 */
        BLDC_EnterFault(BLDC_FAULT_INVALID_STEP); /* 记录无效步号并进入安全故障状态。 */
        return;                         /* 结束换步。 */
    } /* 无效步号处理结束。 */
    source_channel = LL_TIM_CHANNEL_CH1; /* 设置源相默认值。 */
    sink_low_output = TIM_CCER_CC2NE;   /* 设置回流相默认值。 */
    switch (step)                       /* 查询六步换相表。 */
    { /* 六步换相表开始。 */
        case 1U:                        /* 第 1 步：U+ PWM、V- 导通、W 悬空。 */
            source_channel = LL_TIM_CHANNEL_CH1; /* 选择 U 相源相。 */
            sink_low_output = TIM_CCER_CC2NE; /* 选择 V 相低桥。 */
            break;                      /* 完成第 1 步选择。 */
        case 2U:                        /* 第 2 步：U+ PWM、W- 导通、V 悬空。 */
            source_channel = LL_TIM_CHANNEL_CH1; /* 选择 U 相源相。 */
            sink_low_output = TIM_CCER_CC3NE; /* 选择 W 相低桥。 */
            break;                      /* 完成第 2 步选择。 */
        case 3U:                        /* 第 3 步：V+ PWM、W- 导通、U 悬空。 */
            source_channel = LL_TIM_CHANNEL_CH2; /* 选择 V 相源相。 */
            sink_low_output = TIM_CCER_CC3NE; /* 选择 W 相低桥。 */
            break;                      /* 完成第 3 步选择。 */
        case 4U:                        /* 第 4 步：V+ PWM、U- 导通、W 悬空。 */
            source_channel = LL_TIM_CHANNEL_CH2; /* 选择 V 相源相。 */
            sink_low_output = TIM_CCER_CC1NE; /* 选择 U 相低桥。 */
            break;                      /* 完成第 4 步选择。 */
        case 5U:                        /* 第 5 步：W+ PWM、U- 导通、V 悬空。 */
            source_channel = LL_TIM_CHANNEL_CH3; /* 选择 W 相源相。 */
            sink_low_output = TIM_CCER_CC1NE; /* 选择 U 相低桥。 */
            break;                      /* 完成第 5 步选择。 */
        case 6U:                        /* 第 6 步：W+ PWM、V- 导通、U 悬空。 */
            source_channel = LL_TIM_CHANNEL_CH3; /* 选择 W 相源相。 */
            sink_low_output = TIM_CCER_CC2NE; /* 选择 V 相低桥。 */
            break;                      /* 完成第 6 步选择。 */
        default:                        /* 处理理论不可达分支。 */
            BLDC_EnterFault(BLDC_FAULT_INVALID_STEP); /* 记录无效步号并进入安全故障状态。 */
            return;                     /* 结束换步。 */
    } /* 六步换相表结束。 */
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_FORCED_INACTIVE); /* 预装 U 高桥关闭。 */
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_FORCED_INACTIVE); /* 预装 V 高桥关闭。 */
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_FORCED_INACTIVE); /* 预装 W 高桥关闭。 */
    LL_TIM_OC_SetMode(TIM1, source_channel, LL_TIM_OCMODE_PWM1); /* 预装源相 PWM1。 */
    enabled_outputs = BLDC_ALL_HIGH_OUTPUTS | sink_low_output; /* 组合本步使能位。 */
    MODIFY_REG(htim1.Instance->CCER, BLDC_ALL_OUTPUTS, enabled_outputs); /* 预装本步使能位。 */
    if (HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_COM) != HAL_OK) /* 同时应用换步配置。 */
    { /* COM 事件失败处理开始。 */
        BLDC_EnterFault(BLDC_FAULT_COM_EVENT); /* 记录 COM 事件故障并立即安全停机。 */
        return;                         /* 结束换步。 */
    } /* COM 事件失败处理结束。 */
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_COM); /* 清除 COM 标志。 */
    g_bldc.step = step;                 /* 更新软件步号。 */
    g_current.blanking_samples = BLDC_CURRENT_BLANKING_SAMPLES; /* 换相后暂时禁止电流 PI 和软件过流判断。 */
} /* 换相步应用结束。 */

static void BLDC_ClearCommTrace(void)   /* 实现换相调试记录清零。 */
{ /* 换相调试记录清零开始。 */
    uint16_t index;                     /* 保存正在清零的记录下标。 */
    for (index = 0U; index < BLDC_COMM_TRACE_LENGTH; index++) /* 遍历全部环形记录。 */
    { /* 单项记录清零开始。 */
        g_comm_trace[index].sequence = 0U; /* 将事件序号清零并把该项标记为无效。 */
    } /* 单项记录清零结束。 */
    g_comm_trace_index = 0U;            /* 将下一次写入位置恢复到数组首项。 */
    g_comm_trace_sequence = 0U;         /* 将累计事件序号恢复为零。 */
    __DMB();                            /* 保证清零结果在电机开始运行前已经写入内存。 */
} /* 换相调试记录清零结束。 */

static void BLDC_RecordCommutation(BLDC_CommReason_t reason, uint8_t step_before, uint8_t step_after, uint32_t elapsed_ticks) /* 实现单次换相事件记录。 */
{ /* 单次换相事件记录开始。 */
    volatile BLDC_CommTrace_t *entry;   /* 保存当前环形记录项地址。 */
    uint16_t index;                     /* 保存当前环形记录项下标。 */
    uint32_t next_sequence;             /* 保存本次事件即将使用的连续序号。 */
    index = g_comm_trace_index;         /* 读取本次事件的写入位置。 */
    entry = &g_comm_trace[index];       /* 取得本次事件的记录项地址。 */
    entry->sequence = 0U;               /* 写入过程中先把该项标记为无效。 */
    entry->state_ticks = g_bldc.state_ticks; /* 记录当前状态已经持续的时间。 */
    entry->commutation_ticks = elapsed_ticks; /* 记录刚结束换相步的持续时间。 */
    entry->bemf_integral = g_bldc.bemf_integral; /* 记录换相前的反电动势积分值。 */
    entry->integral_limit = g_bldc.integral_limit; /* 记录换相时的积分触发阈值。 */
    entry->integral_limit_base = g_bldc.integral_limit_base; /* 记录换相时的基础磁链积分项。 */
    entry->bus_voltage_mv = g_bus_voltage.filtered_mv; /* 记录换相时低通滤波后的母线电压。 */
    entry->timeout_ticks = g_bldc.timeout_ticks; /* 记录换相时的闭环超时阈值。 */
    entry->bemf_diff = get_diff;        /* 记录换相前最后一次反电动势差值。 */
    entry->motor_current_ma = g_current.motor_current_ma; /* 记录换相时的实际电机电流。 */
    entry->duty = g_bldc.duty;          /* 记录换相时的占空比命令。 */
    entry->step_before = step_before;   /* 记录换相前的步号。 */
    entry->step_after = step_after;     /* 记录换相后的步号。 */
    entry->state = (uint8_t)g_bldc.state; /* 记录触发换相时的状态机状态。 */
    entry->reason = (uint8_t)reason;    /* 记录本次换相的触发原因。 */
    entry->forced_count = g_bldc.forced_count; /* 记录当前连续强制换相次数。 */
    entry->phase_advance_permille = (uint8_t)g_bldc.phase_advance_permille; /* 记录换相时实际使用的提前千分比。 */
    entry->reserved_1 = 0U;             /* 清零第二个保留字段。 */
    entry->reserved_2 = 0U;             /* 清零第三个保留字段。 */
    next_sequence = g_comm_trace_sequence + 1U; /* 生成新的连续事件序号。 */
    if (next_sequence == 0U)            /* 检查无符号序号是否已经回绕。 */
    { /* 序号回绕处理开始。 */
        next_sequence = 1U;             /* 跳过代表无效记录的零序号。 */
    } /* 序号回绕处理结束。 */
    __DMB();                            /* 保证快照字段先于有效事件序号写入内存。 */
    entry->sequence = next_sequence;    /* 最后写入序号并把该项标记为完整有效。 */
    g_comm_trace_sequence = next_sequence; /* 保存最新事件的连续序号。 */
    index++;                            /* 将写入位置移动到下一项。 */
    if (index >= BLDC_COMM_TRACE_LENGTH) /* 检查写入位置是否越过数组末尾。 */
    { /* 环形下标回绕处理开始。 */
        index = 0U;                     /* 回绕到数组首项继续覆盖最旧记录。 */
    } /* 环形下标回绕处理结束。 */
    g_comm_trace_index = index;         /* 保存下一次换相事件的写入位置。 */
} /* 单次换相事件记录结束。 */

static void BLDC_Commutate(BLDC_CommReason_t reason) /* 实现带触发原因的向前换相一步。 */
{ /* 前进一步开始。 */
    uint8_t step_before;                /* 保存换相前的步号。 */
    uint8_t next_step;                  /* 保存下一步号。 */
    uint32_t elapsed_ticks;             /* 保存本步运行周期。 */
    elapsed_ticks = g_bldc.commutation_ticks; /* 读取本步周期。 */
    step_before = g_bldc.step;          /* 读取换相前的步号。 */
    next_step = g_bldc.step + 1U;       /* 计算下一步号。 */
    if (next_step > BLDC_VALID_STEP_MAX) /* 检查是否超过第 6 步。 */
    { /* 步号回绕开始。 */
        next_step = BLDC_VALID_STEP_MIN; /* 回绕到第 1 步。 */
    } /* 步号回绕结束。 */
    BLDC_ApplyStep(next_step);          /* 应用下一换相步。 */
    if (g_bldc.state == BLDC_STATE_FAULT) /* 检查换步故障。 */
    { /* 换步故障处理开始。 */
        return;                         /* 保持故障并返回。 */
    } /* 换步故障处理结束。 */
    BLDC_RecordCommutation(reason, step_before, next_step, elapsed_ticks); /* 保存本次成功换相的完整快照。 */
    if (elapsed_ticks > 0U)             /* 检查测速周期有效。 */
    { /* 速度更新开始。 */
        g_bldc.last_commutation_ticks = elapsed_ticks; /* 保存换步周期。 */
        g_bldc.electrical_rpm = (BLDC_PWM_FREQUENCY_HZ * 10U) / elapsed_ticks; /* 估算电角转速。 */
        g_bldc.timeout_ticks = BLDC_GetTimeoutTicks(elapsed_ticks); /* 更新闭环超时。 */
    } /* 速度更新结束。 */
    g_bldc.commutation_ticks = 0U;      /* 清零本步计时。 */
    g_bldc.bemf_integral = 0U;          /* 清零本步积分。 */
    g_bldc.blanking_ticks = BLDC_BLANKING_TICKS; /* 设置换步消隐。 */
} /* 前进一步结束。 */

static void BLDC_AccumulateBemf(void)   /* 实现正向反电动势积分。 */
{ /* 积分处理开始。 */
    int32_t positive_diff;              /* 保存去死区差值。 */
    uint32_t increment;                 /* 保存本次积分增量。 */
    if (g_bldc.blanking_ticks > 0U)     /* 检查换步消隐。 */
    { /* 消隐处理开始。 */
        g_bldc.blanking_ticks--;        /* 消耗一次消隐计数。 */
        return;                         /* 丢弃本次采样。 */
    } /* 消隐处理结束。 */
    positive_diff = get_diff - BLDC_BEMF_DEADBAND; /* 去除数字死区。 */
    if (positive_diff <= 0)             /* 检查差值方向。 */
    { /* 无效差值处理开始。 */
        return;                         /* 负值和噪声不积分。 */
    } /* 无效差值处理结束。 */
    increment = (uint32_t)positive_diff; /* 转换为积分增量。 */
    if (g_bldc.bemf_integral <= (BLDC_INTEGRAL_SATURATION - increment)) /* 检查溢出。 */
    { /* 正常积分开始。 */
        g_bldc.bemf_integral += increment; /* 累加有效差值。 */
    } /* 正常积分结束。 */
    else                                /* 处理即将溢出的情况。 */
    { /* 饱和处理开始。 */
        g_bldc.bemf_integral = BLDC_INTEGRAL_SATURATION; /* 限制积分值。 */
    } /* 饱和处理结束。 */
} /* 积分处理结束。 */

static void BLDC_ControlTick(void)      /* 实现 20 kHz 快速状态机。 */
{ /* 快速状态机开始。 */
    uint32_t average;                   /* 保存平均单步积分。 */
    g_bldc.state_ticks++;               /* 累加状态时间。 */
    g_bldc.commutation_ticks++;         /* 累加换步时间。 */
    switch (g_bldc.state)               /* 执行当前状态逻辑。 */
    { /* 状态选择开始。 */
        case BLDC_STATE_CALIBRATE:      /* 处理三相电流零点校准状态。 */
            if ((g_current.count_adc1 >= BLDC_CURRENT_CALIBRATION_SAMPLES) && /* 检查 ADC1 样本数。 */
                (g_current.count_adc2 >= BLDC_CURRENT_CALIBRATION_SAMPLES)) /* 检查 ADC2 样本数。 */
            { /* 电流零点校准完成处理开始。 */
                BLDC_FinishCurrentCalibration(); /* 计算零点并开始转子定位。 */
            } /* 电流零点校准完成处理结束。 */
            else if (g_bldc.state_ticks >= BLDC_CURRENT_CALIBRATION_TIMEOUT_TICKS) /* 检查校准超时。 */
            { /* 电流零点校准超时处理开始。 */
                BLDC_EnterFault(BLDC_FAULT_CURRENT_CALIBRATION); /* 记录校准故障并保持输出关闭。 */
            } /* 电流零点校准超时处理结束。 */
            break;                      /* 结束电流零点校准状态。 */
        case BLDC_STATE_ALIGN:          /* 处理定位状态。 */
            if (g_bldc.state_ticks >= BLDC_ALIGN_TICKS) /* 检查定位完成。 */
            { /* 定位完成处理开始。 */
                g_bldc.state = BLDC_STATE_OPEN_LOOP; /* 进入开环状态。 */
                g_bldc.state_ticks = 0U; /* 清零状态计时。 */
                g_bldc.open_loop_ticks = BLDC_OPEN_LOOP_START_TICKS; /* 设置起始周期。 */
                g_bldc.calibration_sum = 0U; /* 清零标定总和。 */
                g_bldc.calibration_count = 0U; /* 清零有效步数。 */
                g_bldc.calibration_attempts = 0U; /* 清零尝试步数。 */
                BLDC_Commutate(BLDC_COMM_REASON_ALIGN); /* 记录定位结束并开始连续六步换相。 */
            } /* 定位完成处理结束。 */
            break;                      /* 结束定位状态。 */
        case BLDC_STATE_OPEN_LOOP:      /* 处理开环加速状态。 */
            BLDC_AccumulateBemf();      /* 测量当前步积分。 */
            if (g_bldc.commutation_ticks >= g_bldc.open_loop_ticks) /* 检查强制换步时刻。 */
            { /* 开环换步处理开始。 */
                if (g_bldc.open_loop_ticks <= BLDC_OPEN_LOOP_END_TICKS) /* 检查标定转速。 */
                { /* 阈值标定开始。 */
                    g_bldc.calibration_attempts++; /* 增加标定尝试数。 */
                    if (g_bldc.bemf_integral >= BLDC_MIN_CALIBRATION_INTEGRAL) /* 检查有效积分。 */
                    { /* 有效积分处理开始。 */
                        g_bldc.calibration_sum += g_bldc.bemf_integral; /* 累加有效积分。 */
                        g_bldc.calibration_count++; /* 增加有效步数。 */
                    } /* 有效积分处理结束。 */
                } /* 阈值标定结束。 */
                BLDC_Commutate(BLDC_COMM_REASON_OPEN_LOOP); /* 记录并执行开环定时强制换步。 */
                if (g_bldc.open_loop_ticks > BLDC_OPEN_LOOP_END_TICKS) /* 检查是否继续加速。 */
                { /* 开环加速处理开始。 */
                    g_bldc.open_loop_ticks -= BLDC_OPEN_LOOP_RAMP_STEP; /* 缩短换步周期。 */
                } /* 开环加速处理结束。 */
                else if (g_bldc.calibration_count >= BLDC_CALIBRATION_VALID_STEPS) /* 检查标定完成。 */
                { /* 标定完成处理开始。 */
                    average = g_bldc.calibration_sum / g_bldc.calibration_count; /* 计算平均积分。 */
                    g_bldc.integral_limit = (average * BLDC_INTEGRAL_LIMIT_PERCENT) / 100U; /* 计算阈值。 */
                    if (g_bldc.integral_limit < BLDC_MIN_INTEGRAL_LIMIT) /* 检查阈值下限。 */
                    { /* 阈值限幅开始。 */
                        g_bldc.integral_limit = BLDC_MIN_INTEGRAL_LIMIT; /* 应用最小阈值。 */
                    } /* 阈值限幅结束。 */
                    g_bldc.integral_limit_base = (g_bldc.integral_limit * /* 从标定阈值计算不随转速变化的磁链项。 */
                                                  BLDC_DYNAMIC_INTEGRAL_BASE_PERCENT) / 100U; /* 取标定阈值的百分之八十。 */
                    if (g_bldc.integral_limit_base < BLDC_MIN_INTEGRAL_LIMIT) /* 检查基础磁链项下限。 */
                    { /* 基础磁链项下限处理开始。 */
                        g_bldc.integral_limit_base = BLDC_MIN_INTEGRAL_LIMIT; /* 保持原有最小可靠积分阈值。 */
                    } /* 基础磁链项下限处理结束。 */
                    if (g_bldc.integral_limit_base > g_bldc.integral_limit) /* 检查基础项是否超过标定阈值。 */
                    { /* 基础磁链项上限处理开始。 */
                        g_bldc.integral_limit_base = g_bldc.integral_limit; /* 不允许基础项高于完整标定阈值。 */
                    } /* 基础磁链项上限处理结束。 */
                    g_bldc.integral_coupling_reference = g_bldc.integral_limit - /* 使用完整阈值减去基础磁链项。 */
                                                         g_bldc.integral_limit_base; /* 得到标定工况母线耦合项。 */
                    g_bldc.integral_limit_max = (g_bldc.integral_limit * /* 计算动态积分阈值最大值。 */
                                                 BLDC_DYNAMIC_INTEGRAL_MAX_PERCENT) / 100U; /* 限制为标定值的百分之一百六十。 */
                    if (g_bldc.integral_limit_max < g_bldc.integral_limit) /* 检查上限计算是否有效。 */
                    { /* 动态积分上限修正开始。 */
                        g_bldc.integral_limit_max = g_bldc.integral_limit; /* 至少允许使用完整标定阈值。 */
                    } /* 动态积分上限修正结束。 */
                    g_bldc.integral_calibration_bus_mv = g_bus_voltage.filtered_mv; /* 保存开环标定时的母线电压。 */
                    g_bldc.integral_calibration_erpm = g_bldc.electrical_rpm; /* 保存开环标定时的电角转速。 */
                    if (g_bldc.integral_calibration_erpm < BLDC_DYNAMIC_INTEGRAL_MIN_ERPM) /* 检查标定转速下限。 */
                    { /* 标定转速下限处理开始。 */
                        g_bldc.integral_calibration_erpm = BLDC_DYNAMIC_INTEGRAL_MIN_ERPM; /* 避免低转速导致耦合系数异常。 */
                    } /* 标定转速下限处理结束。 */
                    BLDC_UpdateDynamicIntegralLimit(); /* 立即用标定电压和转速生成首个运行阈值。 */
                    g_bldc.transition_valid_count = 0U; /* 清零接管成功数。 */
                    g_bldc.forced_count = 0U; /* 清零强制换步数。 */
                    g_bldc.state = BLDC_STATE_TRANSITION; /* 进入接管状态。 */
                    g_bldc.state_ticks = 0U; /* 清零状态时间。 */
                } /* 标定完成处理结束。 */
                else if (g_bldc.calibration_attempts >= BLDC_CALIBRATION_MAX_STEPS) /* 检查标定失败。 */
                { /* 标定失败处理开始。 */
                    BLDC_EnterFault(BLDC_FAULT_BEMF_CALIBRATION); /* 反电动势不足时记录故障并停机。 */
                } /* 标定失败处理结束。 */
                else                   /* 处理标定数据尚不足。 */
                { /* 保持转速开始。 */
                g_bldc.open_loop_ticks = BLDC_OPEN_LOOP_END_TICKS; /* 保持 2 ms/步。 */
                } /* 保持转速结束。 */
            } /* 开环换步处理结束。 */
            break;                      /* 结束开环状态。 */
        case BLDC_STATE_TRANSITION:     /* 处理闭环接管状态。 */
            BLDC_AccumulateBemf();      /* 累加反电动势。 */
            if ((g_bldc.bemf_integral >= g_bldc.integral_limit) && /* 检查积分阈值。 */
                (g_bldc.commutation_ticks >= BLDC_MIN_COMMUTATION_TICKS)) /* 检查最短周期。 */
            { /* 积分换步成功开始。 */
                g_bldc.transition_valid_count++; /* 增加连续成功数。 */
                g_bldc.forced_count = 0U; /* 清零强制换步数。 */
                BLDC_Commutate(BLDC_COMM_REASON_BEMF); /* 记录并执行接管阶段积分换步。 */
                if (g_bldc.transition_valid_count >= BLDC_TRANSITION_VALID_COUNT) /* 检查接管完成。 */
                { /* 接管完成处理开始。 */
                    g_bldc.state = BLDC_STATE_INTEGRATE; /* 进入积分闭环。 */
                    g_bldc.state_ticks = 0U; /* 清零状态时间。 */
                } /* 接管完成处理结束。 */
            } /* 积分换步成功结束。 */
            else if (g_bldc.commutation_ticks >= BLDC_OPEN_LOOP_END_TICKS) /* 检查兜底时刻。 */
            { /* 过渡强制换步开始。 */
                g_bldc.transition_valid_count = 0U; /* 清零连续成功数。 */
                g_bldc.forced_count++;  /* 增加强制换步数。 */
                BLDC_Commutate(BLDC_COMM_REASON_TIMEOUT); /* 记录并执行接管阶段超时兜底换步。 */
                if (g_bldc.forced_count >= BLDC_CALIBRATION_MAX_STEPS) /* 检查长期接管失败。 */
                { /* 接管失败处理开始。 */
                    BLDC_EnterFault(BLDC_FAULT_TRANSITION); /* 闭环接管长期失败时记录故障并停机。 */
                } /* 接管失败处理结束。 */
            } /* 过渡强制换步结束。 */
            break;                      /* 结束接管状态。 */
        case BLDC_STATE_INTEGRATE:      /* 处理纯积分闭环状态。 */
            BLDC_AccumulateBemf();      /* 累加反电动势。 */
            if ((g_bldc.bemf_integral >= g_bldc.integral_limit) && /* 检查积分阈值。 */
                (g_bldc.commutation_ticks >= BLDC_MIN_COMMUTATION_TICKS)) /* 检查最短周期。 */
            { /* 正常闭环换步开始。 */
                g_bldc.forced_count = 0U; /* 清零连续超时数。 */
                BLDC_Commutate(BLDC_COMM_REASON_BEMF); /* 记录并执行闭环阶段积分换步。 */
            } /* 正常闭环换步结束。 */
            else if (g_bldc.commutation_ticks >= g_bldc.timeout_ticks) /* 检查闭环超时。 */
            { /* 闭环超时处理开始。 */
                g_bldc.forced_count++;  /* 增加连续超时数。 */
                BLDC_Commutate(BLDC_COMM_REASON_TIMEOUT); /* 记录并执行闭环超时恢复换步。 */
                if (g_bldc.forced_count >= BLDC_MAX_FORCED_COUNT) /* 检查连续失步。 */
                { /* 连续失步处理开始。 */
                    BLDC_EnterFault(BLDC_FAULT_LOST_SYNC); /* 连续失步时记录故障并停机。 */
                } /* 连续失步处理结束。 */
            } /* 闭环超时处理结束。 */
            break;                      /* 结束积分闭环状态。 */
        case BLDC_STATE_STOP:           /* 处理停止状态。 */
        case BLDC_STATE_FAULT:          /* 处理故障状态。 */
        default:                        /* 处理未知状态。 */
            break;                      /* 停止和故障时不换步。 */
    } /* 状态选择结束。 */
} /* 快速状态机结束。 */

static void BLDC_FinishCurrentCalibration(void) /* 实现三相电流零点计算和运行启动。 */
{ /* 电流零点校准完成处理开始。 */
    g_current.offset_u = (uint16_t)(g_current.sum_u / BLDC_CURRENT_CALIBRATION_SAMPLES); /* 计算 U 相零点。 */
    g_current.offset_v = (uint16_t)(g_current.sum_v / BLDC_CURRENT_CALIBRATION_SAMPLES); /* 计算 V 相零点。 */
    g_current.offset_w = (uint16_t)(g_current.sum_w / BLDC_CURRENT_CALIBRATION_SAMPLES); /* 计算 W 相零点。 */
    g_current.zero_noise_u = g_current.maximum_u - g_current.minimum_u; /* 计算 U 相零点峰峰噪声。 */
    g_current.zero_noise_v = g_current.maximum_v - g_current.minimum_v; /* 计算 V 相零点峰峰噪声。 */
    g_current.zero_noise_w = g_current.maximum_w - g_current.minimum_w; /* 计算 W 相零点峰峰噪声。 */
    if ((g_current.offset_u < BLDC_CURRENT_OFFSET_MIN) || /* 检查 U 相零点下限。 */
        (g_current.offset_u > BLDC_CURRENT_OFFSET_MAX) || /* 检查 U 相零点上限。 */
        (g_current.offset_v < BLDC_CURRENT_OFFSET_MIN) || /* 检查 V 相零点下限。 */
        (g_current.offset_v > BLDC_CURRENT_OFFSET_MAX) || /* 检查 V 相零点上限。 */
        (g_current.offset_w < BLDC_CURRENT_OFFSET_MIN) || /* 检查 W 相零点下限。 */
        (g_current.offset_w > BLDC_CURRENT_OFFSET_MAX) || /* 检查 W 相零点上限。 */
        (g_current.zero_noise_u > BLDC_CURRENT_ZERO_NOISE_LIMIT_COUNTS) || /* 检查 U 相零点噪声。 */
        (g_current.zero_noise_v > BLDC_CURRENT_ZERO_NOISE_LIMIT_COUNTS) || /* 检查 V 相零点噪声。 */
        (g_current.zero_noise_w > BLDC_CURRENT_ZERO_NOISE_LIMIT_COUNTS)) /* 检查 W 相零点噪声。 */
    { /* 电流零点范围异常处理开始。 */
        BLDC_EnterFault(BLDC_FAULT_CURRENT_CALIBRATION); /* 记录电流采样硬件异常并停机。 */
        return;                         /* 不允许在无效零点下打开功率输出。 */
    } /* 电流零点范围异常处理结束。 */
    g_current.zero_noise_ok = 1U;       /* 标记三相零点偏置和峰峰噪声均已通过检查。 */
    g_current.calibrated = 1U;          /* 标记三相电流零点已经有效。 */
    g_current.overcurrent_count = 0U;   /* 清零过流连续确认计数。 */
    BLDC_ApplyStep(1U);                 /* 装载用于转子定位的第 1 换相步。 */
    if (g_bldc.state == BLDC_STATE_FAULT) /* 检查首个换相步是否装载失败。 */
    { /* 首个换相步失败处理开始。 */
        return;                         /* 保持故障状态和关闭输出。 */
    } /* 首个换相步失败处理结束。 */
    g_bldc.state_ticks = 0U;            /* 清零定位状态计时。 */
    g_bldc.commutation_ticks = 0U;      /* 清零首步换相计时。 */
    g_bldc.state = BLDC_STATE_ALIGN;    /* 进入转子定位状态。 */
    __HAL_TIM_MOE_ENABLE(&htim1);       /* 零点有效后才允许打开功率输出。 */
} /* 电流零点校准完成处理结束。 */

static int32_t BLDC_ConvertCurrentMa(int32_t adc_difference, int32_t nominal_gain_x1000, uint16_t scale_permille) /* 实现电流定点换算。 */
{ /* 电流定点换算开始。 */
    int64_t numerator;                  /* 保存带符号的高精度换算分子。 */
    numerator = (int64_t)adc_difference * /* 使用 ADC 相对零点差值开始计算。 */
                (int64_t)nominal_gain_x1000 * /* 乘以每计数标称毫安值的一千倍。 */
                (int64_t)scale_permille; /* 乘以本相实测校准千分比。 */
    if (numerator >= 0LL)               /* 检查电流换算分子是否为正。 */
    { /* 正电流四舍五入开始。 */
        numerator += (int64_t)BLDC_CURRENT_GAIN_DENOMINATOR / 2LL; /* 加入正数四舍五入量。 */
    } /* 正电流四舍五入结束。 */
    else                                /* 处理负电流换算分子。 */
    { /* 负电流四舍五入开始。 */
        numerator -= (int64_t)BLDC_CURRENT_GAIN_DENOMINATOR / 2LL; /* 加入负数四舍五入量。 */
    } /* 负电流四舍五入结束。 */
    return (int32_t)(numerator / (int64_t)BLDC_CURRENT_GAIN_DENOMINATOR); /* 返回换算后的真实毫安值。 */
} /* 电流定点换算结束。 */

static int32_t BLDC_AbsoluteCurrentMa(int32_t current_ma) /* 实现电流绝对值计算。 */
{ /* 电流绝对值计算开始。 */
    if (current_ma < 0L)                /* 检查电流方向是否为负。 */
    { /* 负电流处理开始。 */
        current_ma = -current_ma;       /* 转换为正的电流幅值。 */
    } /* 负电流处理结束。 */
    return current_ma;                  /* 返回电流幅值。 */
} /* 电流绝对值计算结束。 */

static void BLDC_TryProcessCurrentSample(void) /* 实现 ADC1 和 ADC2 注入样本配对。 */
{ /* 三相注入样本配对开始。 */
    if ((g_current.adc1_ready == 0U) || (g_current.adc2_ready == 0U)) /* 检查两组注入转换是否都已完成。 */
    { /* 三相样本尚未完整处理开始。 */
        return;                         /* 等待另一个 ADC 回调到达。 */
    } /* 三相样本尚未完整处理结束。 */
    g_current.adc1_ready = 0U;          /* 消耗本周期 ADC1 的 U/W 样本。 */
    g_current.adc2_ready = 0U;          /* 消耗本周期 ADC2 的 V 相样本。 */
    g_current.paired_sample_count++;    /* 记录一次完整的三相同步样本。 */
    if (g_current.calibrated != 0U)     /* 检查三相电流零点是否已经有效。 */
    { /* 已校准三相样本处理开始。 */
        BLDC_UpdatePhaseCurrents();     /* 只使用完整配对的数据换算电流并执行控制。 */
    } /* 已校准三相样本处理结束。 */
} /* 三相注入样本配对结束。 */

static void BLDC_UpdatePhaseCurrents(void) /* 实现三相电流换算和快速软件过流保护。 */
{ /* 三相电流更新开始。 */
    int32_t absolute_u_ma;              /* 保存 U 相电流绝对值。 */
    int32_t absolute_v_ma;              /* 保存 V 相电流绝对值。 */
    int32_t absolute_w_ma;              /* 保存 W 相电流绝对值。 */
    int32_t overcurrent_value_ma;       /* 保存用于快速保护的三相最大电流。 */
    int32_t step_filter_difference;     /* 保存当前扇区诊断滤波差值。 */
    uint8_t step_index;                 /* 保存当前换相扇区数组下标。 */
    g_current.current_u_ma = BLDC_ConvertCurrentMa((int32_t)g_current.raw_u - (int32_t)g_current.offset_u, /* 计算 U 相 ADC 相对零点差值。 */
                                                   BLDC_CURRENT_U_MA_PER_COUNT_X1000, /* 使用 U 相标称电流增益。 */
                                                   g_current_scale.u_permille); /* 应用 U 相实测校准千分比。 */
    g_current.current_v_ma = BLDC_ConvertCurrentMa((int32_t)g_current.raw_v - (int32_t)g_current.offset_v, /* 计算 V 相 ADC 相对零点差值。 */
                                                   BLDC_CURRENT_V_MA_PER_COUNT_X1000, /* 使用 V 相标称电流增益。 */
                                                   g_current_scale.v_permille); /* 应用 V 相实测校准千分比。 */
    g_current.current_w_ma = BLDC_ConvertCurrentMa((int32_t)g_current.raw_w - (int32_t)g_current.offset_w, /* 计算 W 相 ADC 相对零点差值。 */
                                                   BLDC_CURRENT_W_MA_PER_COUNT_X1000, /* 使用 W 相标称电流增益。 */
                                                   g_current_scale.w_permille); /* 应用 W 相实测校准千分比。 */
    absolute_u_ma = BLDC_AbsoluteCurrentMa(g_current.current_u_ma); /* 计算 U 相电流幅值。 */
    absolute_v_ma = BLDC_AbsoluteCurrentMa(g_current.current_v_ma); /* 计算 V 相电流幅值。 */
    absolute_w_ma = BLDC_AbsoluteCurrentMa(g_current.current_w_ma); /* 计算 W 相电流幅值。 */
    switch (g_bldc.step)               /* 根据非同步六步状态选择持续导通的回流相。 */
    { /* 回流相电流选择开始。 */
        case 1U:                        /* 第 1 步为 U+、V-、W 悬空。 */
            g_current.motor_current_ma = absolute_v_ma; /* 使用 V 相低桥回流电流幅值。 */
            g_current.floating_current_ma = absolute_w_ma; /* 保存 W 悬空相诊断电流。 */
            break;                      /* 结束第 1 步电流选择。 */
        case 2U:                        /* 第 2 步为 U+、W-、V 悬空。 */
            g_current.motor_current_ma = absolute_w_ma; /* 使用 W 相低桥回流电流幅值。 */
            g_current.floating_current_ma = absolute_v_ma; /* 保存 V 悬空相诊断电流。 */
            break;                      /* 结束第 2 步电流选择。 */
        case 3U:                        /* 第 3 步为 V+、W-、U 悬空。 */
            g_current.motor_current_ma = absolute_w_ma; /* 使用 W 相低桥回流电流幅值。 */
            g_current.floating_current_ma = absolute_u_ma; /* 保存 U 悬空相诊断电流。 */
            break;                      /* 结束第 3 步电流选择。 */
        case 4U:                        /* 第 4 步为 V+、U-、W 悬空。 */
            g_current.motor_current_ma = absolute_u_ma; /* 使用 U 相低桥回流电流幅值。 */
            g_current.floating_current_ma = absolute_w_ma; /* 保存 W 悬空相诊断电流。 */
            break;                      /* 结束第 4 步电流选择。 */
        case 5U:                        /* 第 5 步为 W+、U-、V 悬空。 */
            g_current.motor_current_ma = absolute_u_ma; /* 使用 U 相低桥回流电流幅值。 */
            g_current.floating_current_ma = absolute_v_ma; /* 保存 V 悬空相诊断电流。 */
            break;                      /* 结束第 5 步电流选择。 */
        case 6U:                        /* 第 6 步为 W+、V-、U 悬空。 */
            g_current.motor_current_ma = absolute_v_ma; /* 使用 V 相低桥回流电流幅值。 */
            g_current.floating_current_ma = absolute_u_ma; /* 保存 U 悬空相诊断电流。 */
            break;                      /* 结束第 6 步电流选择。 */
        default:                        /* 处理理论不可达的无效步号。 */
            g_current.motor_current_ma = 0L; /* 无效步号不输出电流值。 */
            g_current.floating_current_ma = 0L; /* 无效步号不输出悬空相诊断值。 */
            return;                     /* 结束本次电流保护计算。 */
    } /* 回流相电流选择结束。 */
    if (g_current.blanking_samples > 0U) /* 检查是否处于换相后的电流消隐窗口。 */
    { /* 电流换相消隐开始。 */
        g_current.blanking_samples--;   /* 消耗一个电流消隐样本。 */
        g_current.blanked_sample_count++; /* 累加被消隐的电流控制样本数。 */
        g_current.overcurrent_count = 0U; /* 消隐期间不保留软件过流连续计数。 */
        return;                         /* 本样本不进入电流 PI 和软件过流判断。 */
    } /* 电流换相消隐结束。 */
    step_index = g_bldc.step - 1U;      /* 将一到六步转换为零到五数组下标。 */
    if (g_current.step_current_ma[step_index] == 0L) /* 检查本扇区是否首次得到有效电流。 */
    { /* 扇区诊断电流初始化开始。 */
        g_current.step_current_ma[step_index] = g_current.motor_current_ma; /* 使用首个有效样本初始化本扇区滤波值。 */
    } /* 扇区诊断电流初始化结束。 */
    else                                /* 处理本扇区后续有效电流样本。 */
    { /* 扇区诊断电流滤波开始。 */
        step_filter_difference = g_current.motor_current_ma - g_current.step_current_ma[step_index]; /* 计算扇区电流滤波差值。 */
        g_current.step_current_ma[step_index] += step_filter_difference >> BLDC_CURRENT_STEP_FILTER_SHIFT; /* 以三十二分之一系数更新扇区电流。 */
    } /* 扇区诊断电流滤波结束。 */
    if ((g_bldc.state == BLDC_STATE_ALIGN) || /* 判断定位状态是否已经输出功率。 */
        (g_bldc.state == BLDC_STATE_OPEN_LOOP) || /* 判断开环状态是否已经输出功率。 */
        (g_bldc.state == BLDC_STATE_TRANSITION) || /* 判断接管状态是否已经输出功率。 */
        (g_bldc.state == BLDC_STATE_INTEGRATE)) /* 判断闭环状态是否已经输出功率。 */
    { /* 运行状态过流检查开始。 */
        overcurrent_value_ma = absolute_u_ma; /* 首先使用 U 相电流作为三相最大值。 */
        if (absolute_v_ma > overcurrent_value_ma) /* 检查 V 相电流是否更大。 */
        { /* V 相最大值更新开始。 */
            overcurrent_value_ma = absolute_v_ma; /* 将软件保护电流更新为 V 相。 */
        } /* V 相最大值更新结束。 */
        if (absolute_w_ma > overcurrent_value_ma) /* 检查 W 相电流是否更大。 */
        { /* W 相最大值更新开始。 */
            overcurrent_value_ma = absolute_w_ma; /* 将软件保护电流更新为 W 相。 */
        } /* W 相最大值更新结束。 */
        if (overcurrent_value_ma >= g_current_control.overcurrent_limit_ma) /* 检查任意一相是否超过运行时保护阈值。 */
        { /* 过流确认处理开始。 */
            if (g_current.overcurrent_count < BLDC_OVERCURRENT_CONFIRM_SAMPLES) /* 防止确认计数溢出。 */
            { /* 过流确认计数开始。 */
                g_current.overcurrent_count++; /* 累加连续过流采样次数。 */
            } /* 过流确认计数结束。 */
            if (g_current.overcurrent_count >= BLDC_OVERCURRENT_CONFIRM_SAMPLES) /* 检查过流是否连续成立。 */
            { /* 确认过流处理开始。 */
                BLDC_EnterFault(BLDC_FAULT_OVERCURRENT); /* 立即关闭输出并记录过流故障。 */
            } /* 确认过流处理结束。 */
        } /* 过流确认处理结束。 */
        else                            /* 处理本次电流未超过限制的情况。 */
        { /* 过流恢复计数处理开始。 */
            g_current.overcurrent_count = 0U; /* 非连续过流时重新开始确认。 */
        } /* 过流恢复计数处理结束。 */
        if (g_bldc.state != BLDC_STATE_FAULT) /* 检查快速保护没有在本次采样中触发。 */
        { /* 电流闭环执行开始。 */
            BLDC_RunCurrentControl();  /* 根据滤波电流执行一次 PI 占空比调节。 */
        } /* 电流闭环执行结束。 */
    } /* 运行状态过流检查结束。 */
    else                                /* 处理没有输出功率的状态。 */
    { /* 非运行状态处理开始。 */
        g_current.overcurrent_count = 0U; /* 非运行状态不执行过流确认。 */
    } /* 非运行状态处理结束。 */
} /* 三相电流更新结束。 */

static void BLDC_RunCurrentControl(void) /* 实现 20 kHz 电流滤波、给定斜坡和 PI 控制。 */
{ /* 电流闭环控制开始。 */
    int32_t measured_current_ma;       /* 保存用于控制的非负电流测量值。 */
    int32_t tentative_duty;            /* 保存限幅前的 PI 占空比。 */
    int32_t integral_minimum;          /* 保存积分项下限。 */
    int32_t integral_maximum;          /* 保存积分项上限。 */
    measured_current_ma = g_current.motor_current_ma; /* 读取当前回流相电流。 */
    if (measured_current_ma < 0L)      /* 检查换相尖峰或零点噪声造成的负值。 */
    { /* 负电流处理开始。 */
        measured_current_ma = 0L;      /* 正向驱动阶段不让负值错误地增大占空比。 */
    } /* 负电流处理结束。 */
    if (g_current_control.filter_initialized == 0U) /* 检查滤波器是否第一次得到有效电流。 */
    { /* 滤波器初始化开始。 */
        g_current_control.filtered_current_ma = measured_current_ma; /* 直接装入首个测量值以避免启动延迟。 */
        g_current_control.filter_initialized = 1U; /* 标记滤波器已经初始化。 */
    } /* 滤波器初始化结束。 */
    else                                /* 处理后续连续电流采样。 */
    { /* 一阶低通滤波开始。 */
        g_current_control.filtered_current_ma += /* 累加本次滤波修正量。 */
            (measured_current_ma - g_current_control.filtered_current_ma) >> BLDC_CURRENT_FILTER_SHIFT; /* 使用十六分之一系数抑制换相噪声。 */
    } /* 一阶低通滤波结束。 */
    if (g_current_control.ramped_target_ma < g_current_control.target_ma) /* 检查目标电流是否需要向上爬升。 */
    { /* 目标电流上升斜坡开始。 */
        g_current_control.ramped_target_ma += BLDC_CURRENT_RAMP_STEP_MA; /* 每个采样周期增加固定电流。 */
        if (g_current_control.ramped_target_ma > g_current_control.target_ma) /* 检查斜坡是否越过最终目标。 */
        { /* 上升斜坡限幅开始。 */
            g_current_control.ramped_target_ma = g_current_control.target_ma; /* 精确停在最终目标。 */
        } /* 上升斜坡限幅结束。 */
    } /* 目标电流上升斜坡结束。 */
    else if (g_current_control.ramped_target_ma > g_current_control.target_ma) /* 检查目标电流是否需要向下变化。 */
    { /* 目标电流下降斜坡开始。 */
        g_current_control.ramped_target_ma -= BLDC_CURRENT_RAMP_STEP_MA; /* 每个采样周期降低固定电流。 */
        if (g_current_control.ramped_target_ma < g_current_control.target_ma) /* 检查下降斜坡是否越过最终目标。 */
        { /* 下降斜坡限幅开始。 */
            g_current_control.ramped_target_ma = g_current_control.target_ma; /* 精确停在最终目标。 */
        } /* 下降斜坡限幅结束。 */
    } /* 目标电流下降斜坡结束。 */
    else                                /* 处理斜坡已经到达目标的情况。 */
    { /* 目标保持开始。 */
        __NOP();                        /* 保持当前斜坡目标不变。 */
    } /* 目标保持结束。 */
    g_current_control.error_ma = g_current_control.ramped_target_ma - g_current_control.filtered_current_ma; /* 计算电流闭环误差。 */
    g_current_control.proportional_duty = g_current_control.error_ma / BLDC_CURRENT_KP_DIVISOR; /* 计算比例项占空比。 */
    tentative_duty = (g_current_control.integral_accumulator / BLDC_CURRENT_KI_DIVISOR) + /* 换算当前积分项。 */
                     g_current_control.proportional_duty; /* 合成限幅前 PI 输出。 */
    if (!(((tentative_duty >= BLDC_CURRENT_DUTY_MAX) && (g_current_control.error_ma > 0L)) || /* 检查上限方向积分饱和。 */
          ((tentative_duty <= BLDC_CURRENT_DUTY_MIN) && (g_current_control.error_ma < 0L)))) /* 检查下限方向积分饱和。 */
    { /* 条件积分开始。 */
        g_current_control.integral_accumulator += g_current_control.error_ma; /* 仅在不会加重饱和时积分。 */
    } /* 条件积分结束。 */
    integral_minimum = BLDC_CURRENT_DUTY_MIN * BLDC_CURRENT_KI_DIVISOR; /* 计算积分器下限。 */
    integral_maximum = BLDC_CURRENT_DUTY_MAX * BLDC_CURRENT_KI_DIVISOR; /* 计算积分器上限。 */
    if (g_current_control.integral_accumulator < integral_minimum) /* 检查积分器下溢。 */
    { /* 积分器下限处理开始。 */
        g_current_control.integral_accumulator = integral_minimum; /* 限制积分器到最小运行占空比。 */
    } /* 积分器下限处理结束。 */
    if (g_current_control.integral_accumulator > integral_maximum) /* 检查积分器上溢。 */
    { /* 积分器上限处理开始。 */
        g_current_control.integral_accumulator = integral_maximum; /* 限制积分器到最大运行占空比。 */
    } /* 积分器上限处理结束。 */
    tentative_duty = (g_current_control.integral_accumulator / BLDC_CURRENT_KI_DIVISOR) + /* 使用更新后的积分项。 */
                     g_current_control.proportional_duty; /* 重新合成 PI 输出。 */
    if (tentative_duty < BLDC_CURRENT_DUTY_MIN) /* 检查最终输出下限。 */
    { /* 输出下限处理开始。 */
        tentative_duty = BLDC_CURRENT_DUTY_MIN; /* 应用最小运行占空比。 */
    } /* 输出下限处理结束。 */
    if (tentative_duty > BLDC_CURRENT_DUTY_MAX) /* 检查最终输出上限。 */
    { /* 输出上限处理开始。 */
        tentative_duty = BLDC_CURRENT_DUTY_MAX; /* 应用最大测试占空比。 */
    } /* 输出上限处理结束。 */
    g_current_control.output_duty = tentative_duty; /* 保存便于调试器观察的 PI 输出。 */
    BLDC_SetDuty((uint16_t)tentative_duty); /* 将 PI 输出应用到三相 PWM 和 ADC 采样点。 */
} /* 电流闭环控制结束。 */

static uint32_t BLDC_GetTimeoutTicks(uint32_t ticks) /* 实现换步超时计算。 */
{ /* 超时计算开始。 */
    uint32_t timeout;                   /* 保存计算结果。 */
    timeout = ticks * BLDC_TIMEOUT_MULTIPLIER; /* 计算两倍换步周期。 */
    if (timeout < BLDC_TIMEOUT_MIN_TICKS) /* 检查超时下限。 */
    { /* 下限处理开始。 */
        timeout = BLDC_TIMEOUT_MIN_TICKS; /* 应用超时下限。 */
    } /* 下限处理结束。 */
    if (timeout > BLDC_TIMEOUT_MAX_TICKS) /* 检查超时上限。 */
    { /* 上限处理开始。 */
        timeout = BLDC_TIMEOUT_MAX_TICKS; /* 应用超时上限。 */
    } /* 上限处理结束。 */
    return timeout;                     /* 返回最终超时值。 */
} /* 超时计算结束。 */

static void BLDC_EnterFault(BLDC_Fault_t fault) /* 实现带故障码的安全停机。 */
{ /* 故障停机开始。 */
    g_bldc.fault = fault;               /* 保存导致停机的明确故障原因。 */
    g_bldc.state = BLDC_STATE_FAULT;    /* 记录故障状态。 */
    BLDC_AllOutputsOff();               /* 立即关闭全部桥臂。 */
} /* 故障停机结束。 */

BLDC_Fault_t BLDC_GetFault(void)        /* 实现当前故障码读取接口。 */
{ /* 故障码读取开始。 */
    return g_bldc.fault;                /* 返回最近一次电机控制故障。 */
} /* 故障码读取结束。 */

void BLDC_SetCurrentTargetMa(int32_t target_ma) /* 实现正向运行目标电流设置接口。 */
{ /* 目标电流设置开始。 */
    if (target_ma < BLDC_CURRENT_TARGET_MIN_MA) /* 检查目标电流下限。 */
    { /* 目标电流下限处理开始。 */
        target_ma = BLDC_CURRENT_TARGET_MIN_MA; /* 限制到能够维持无感运行的最小测试电流。 */
    } /* 目标电流下限处理结束。 */
    if (target_ma > BLDC_CURRENT_TARGET_MAX_MA) /* 检查目标电流上限。 */
    { /* 目标电流上限处理开始。 */
        target_ma = BLDC_CURRENT_TARGET_MAX_MA; /* 限制到软件过流值以下。 */
    } /* 目标电流上限处理结束。 */
    g_current_control.target_ma = target_ma; /* 原子更新后续斜坡使用的最终目标。 */
} /* 目标电流设置结束。 */

void BLDC_SetPhaseAdvanceMaxPermille(uint16_t max_permille) /* 实现最大积分阈值提前千分比设置接口。 */
{ /* 最大换相提前量设置开始。 */
    if (max_permille > BLDC_PHASE_ADVANCE_MAX_PERMILLE) /* 检查用户设置是否超过安全上限。 */
    { /* 最大换相提前量上限处理开始。 */
        max_permille = BLDC_PHASE_ADVANCE_MAX_PERMILLE; /* 限制到百分之二十的最大允许值。 */
    } /* 最大换相提前量上限处理结束。 */
    g_bldc.phase_advance_max_permille = max_permille; /* 原子更新后续动态阈值使用的最大提前量。 */
} /* 最大换相提前量设置结束。 */

void BLDC_SetCurrentScalePermille(uint16_t u_permille, uint16_t v_permille, uint16_t w_permille) /* 实现三相电流增益独立校准接口。 */
{ /* 三相电流增益校准设置开始。 */
    if (u_permille < BLDC_CURRENT_SCALE_MIN_PERMILLE) /* 检查 U 相校准系数下限。 */
    { /* U 相校准系数下限处理开始。 */
        u_permille = BLDC_CURRENT_SCALE_MIN_PERMILLE; /* 限制 U 相校准系数到百分之五十。 */
    } /* U 相校准系数下限处理结束。 */
    if (u_permille > BLDC_CURRENT_SCALE_MAX_PERMILLE) /* 检查 U 相校准系数上限。 */
    { /* U 相校准系数上限处理开始。 */
        u_permille = BLDC_CURRENT_SCALE_MAX_PERMILLE; /* 限制 U 相校准系数到百分之二百。 */
    } /* U 相校准系数上限处理结束。 */
    if (v_permille < BLDC_CURRENT_SCALE_MIN_PERMILLE) /* 检查 V 相校准系数下限。 */
    { /* V 相校准系数下限处理开始。 */
        v_permille = BLDC_CURRENT_SCALE_MIN_PERMILLE; /* 限制 V 相校准系数到百分之五十。 */
    } /* V 相校准系数下限处理结束。 */
    if (v_permille > BLDC_CURRENT_SCALE_MAX_PERMILLE) /* 检查 V 相校准系数上限。 */
    { /* V 相校准系数上限处理开始。 */
        v_permille = BLDC_CURRENT_SCALE_MAX_PERMILLE; /* 限制 V 相校准系数到百分之二百。 */
    } /* V 相校准系数上限处理结束。 */
    if (w_permille < BLDC_CURRENT_SCALE_MIN_PERMILLE) /* 检查 W 相校准系数下限。 */
    { /* W 相校准系数下限处理开始。 */
        w_permille = BLDC_CURRENT_SCALE_MIN_PERMILLE; /* 限制 W 相校准系数到百分之五十。 */
    } /* W 相校准系数下限处理结束。 */
    if (w_permille > BLDC_CURRENT_SCALE_MAX_PERMILLE) /* 检查 W 相校准系数上限。 */
    { /* W 相校准系数上限处理开始。 */
        w_permille = BLDC_CURRENT_SCALE_MAX_PERMILLE; /* 限制 W 相校准系数到百分之二百。 */
    } /* W 相校准系数上限处理结束。 */
    g_current_scale.u_permille = u_permille; /* 原子更新 U 相电流增益千分比。 */
    g_current_scale.v_permille = v_permille; /* 原子更新 V 相电流增益千分比。 */
    g_current_scale.w_permille = w_permille; /* 原子更新 W 相电流增益千分比。 */
} /* 三相电流增益校准设置结束。 */

void BLDC_SetOvercurrentLimitMa(int32_t limit_ma) /* 实现运行时软件过流阈值设置接口。 */
{ /* 软件过流阈值设置开始。 */
    if (limit_ma < BLDC_OVERCURRENT_LIMIT_MIN_MA) /* 检查软件过流阈值下限。 */
    { /* 软件过流阈值下限处理开始。 */
        limit_ma = BLDC_OVERCURRENT_LIMIT_MIN_MA; /* 限制到最低五百毫安。 */
    } /* 软件过流阈值下限处理结束。 */
    if (limit_ma > BLDC_OVERCURRENT_LIMIT_MAX_MA) /* 检查软件过流阈值上限。 */
    { /* 软件过流阈值上限处理开始。 */
        limit_ma = BLDC_OVERCURRENT_LIMIT_MAX_MA; /* 限制到最高十五安。 */
    } /* 软件过流阈值上限处理结束。 */
    g_current_control.overcurrent_limit_ma = limit_ma; /* 原子更新快速软件过流保护阈值。 */
} /* 软件过流阈值设置结束。 */

int32_t BLDC_GetMotorCurrentMa(void)    /* 实现当前有效电机电流读取接口。 */
{ /* 电机电流读取开始。 */
    return g_current.motor_current_ma;  /* 返回当前回流相电流，单位为毫安。 */
} /* 电机电流读取结束。 */

int32_t BLDC_GetFilteredCurrentMa(void) /* 实现滤波电机电流读取接口。 */
{ /* 滤波电流读取开始。 */
    return g_current_control.filtered_current_ma; /* 返回送入 PI 的滤波电流。 */
} /* 滤波电流读取结束。 */

uint16_t BLDC_GetDuty(void)             /* 实现当前占空比命令读取接口。 */
{ /* 占空比读取开始。 */
    return g_bldc.duty;                 /* 返回满量程为 1200 的占空比命令。 */
} /* 占空比读取结束。 */

uint32_t BLDC_GetBusVoltageMv(void)     /* 实现母线电压读取接口。 */
{ /* 母线电压读取开始。 */
    return g_bus_voltage.filtered_mv;   /* 返回低通滤波后的母线电压毫伏值。 */
} /* 母线电压读取结束。 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) /* 实现 ADC DMA 完成回调。 */
{ /* DMA 回调开始。 */
    if (hadc->Instance == ADC2)         /* 仅处理 ADC2。 */
    { /* ADC2 处理开始。 */
        g_bemf.u = adc2_dma_buffer[0];  /* 更新 U 相数据。 */
        g_bemf.v = adc2_dma_buffer[1];  /* 更新 V 相数据。 */
        g_bemf.w = adc2_dma_buffer[2];  /* 更新 W 相数据。 */
        g_bemf.ready = 1U;              /* 标记采样有效。 */
        g_bemf.sample_count++;          /* 增加采样计数。 */
        get_diff = BLDC_GetBemfDiff();  /* 计算有符号差值。 */
        BLDC_ControlTick();             /* 推进快速控制状态机。 */
    } /* ADC2 处理结束。 */
} /* DMA 回调结束。 */

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) /* 实现 ADC 注入电流采样完成回调。 */
{ /* ADC 注入电流回调开始。 */
    if (hadc->Instance == ADC1)         /* 处理 ADC1 的 U 相和 W 相注入序列。 */
    { /* ADC1 注入序列处理开始。 */
        if (g_current.adc1_ready != 0U) /* 检查上一组 ADC1 样本是否尚未与 ADC2 配对。 */
        { /* ADC1 样本覆盖诊断开始。 */
            g_current.sync_error_count++; /* 记录一次 ADC1 与 ADC2 注入样本不同步。 */
        } /* ADC1 样本覆盖诊断结束。 */
        g_current.raw_u = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1); /* 读取 OPAMP1 的 U 相电流。 */
        g_current.raw_w = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2); /* 读取 OPAMP3 的 W 相电流。 */
        if ((g_bldc.state == BLDC_STATE_CALIBRATE) && /* 检查当前是否正在校准电流零点。 */
            (g_current.count_adc1 < BLDC_CURRENT_CALIBRATION_SAMPLES)) /* 检查 ADC1 校准样本是否足够。 */
        { /* ADC1 零点累计开始。 */
            g_current.sum_u += g_current.raw_u; /* 累加 U 相零电流 ADC 值。 */
            g_current.sum_w += g_current.raw_w; /* 累加 W 相零电流 ADC 值。 */
            if (g_current.raw_u < g_current.minimum_u) /* 检查 U 相零点新最小值。 */
            { /* U 相零点最小值更新开始。 */
                g_current.minimum_u = g_current.raw_u; /* 保存 U 相零点新最小值。 */
            } /* U 相零点最小值更新结束。 */
            if (g_current.raw_u > g_current.maximum_u) /* 检查 U 相零点新最大值。 */
            { /* U 相零点最大值更新开始。 */
                g_current.maximum_u = g_current.raw_u; /* 保存 U 相零点新最大值。 */
            } /* U 相零点最大值更新结束。 */
            if (g_current.raw_w < g_current.minimum_w) /* 检查 W 相零点新最小值。 */
            { /* W 相零点最小值更新开始。 */
                g_current.minimum_w = g_current.raw_w; /* 保存 W 相零点新最小值。 */
            } /* W 相零点最小值更新结束。 */
            if (g_current.raw_w > g_current.maximum_w) /* 检查 W 相零点新最大值。 */
            { /* W 相零点最大值更新开始。 */
                g_current.maximum_w = g_current.raw_w; /* 保存 W 相零点新最大值。 */
            } /* W 相零点最大值更新结束。 */
            g_current.count_adc1++;     /* 增加 ADC1 注入序列采样计数。 */
        } /* ADC1 零点累计结束。 */
        g_current.adc1_ready = 1U;      /* 标记本 PWM 周期 U/W 相样本已经到达。 */
        BLDC_TryProcessCurrentSample(); /* 尝试与同周期 V 相样本配对后执行控制。 */
    } /* ADC1 注入序列处理结束。 */
    else if (hadc->Instance == ADC2)    /* 处理 ADC2 的 V 相注入转换。 */
    { /* ADC2 注入转换处理开始。 */
        if (g_current.adc2_ready != 0U) /* 检查上一组 ADC2 样本是否尚未与 ADC1 配对。 */
        { /* ADC2 样本覆盖诊断开始。 */
            g_current.sync_error_count++; /* 记录一次 ADC2 与 ADC1 注入样本不同步。 */
        } /* ADC2 样本覆盖诊断结束。 */
        g_current.raw_v = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1); /* 读取 OPAMP2 的 V 相电流。 */
        if ((g_bldc.state == BLDC_STATE_CALIBRATE) && /* 检查当前是否正在校准电流零点。 */
            (g_current.count_adc2 < BLDC_CURRENT_CALIBRATION_SAMPLES)) /* 检查 ADC2 校准样本是否足够。 */
        { /* ADC2 零点累计开始。 */
            g_current.sum_v += g_current.raw_v; /* 累加 V 相零电流 ADC 值。 */
            if (g_current.raw_v < g_current.minimum_v) /* 检查 V 相零点新最小值。 */
            { /* V 相零点最小值更新开始。 */
                g_current.minimum_v = g_current.raw_v; /* 保存 V 相零点新最小值。 */
            } /* V 相零点最小值更新结束。 */
            if (g_current.raw_v > g_current.maximum_v) /* 检查 V 相零点新最大值。 */
            { /* V 相零点最大值更新开始。 */
                g_current.maximum_v = g_current.raw_v; /* 保存 V 相零点新最大值。 */
            } /* V 相零点最大值更新结束。 */
            g_current.count_adc2++;     /* 增加 ADC2 注入转换采样计数。 */
        } /* ADC2 零点累计结束。 */
        g_current.adc2_ready = 1U;      /* 标记本 PWM 周期 V 相样本已经到达。 */
        BLDC_TryProcessCurrentSample(); /* 尝试与同周期 U/W 相样本配对后执行控制。 */
    } /* ADC2 注入转换处理结束。 */
    else                                /* 处理其他 ADC 实例。 */
    { /* 其他 ADC 处理开始。 */
        __NOP();                        /* 其他 ADC 不属于本电机电流采样。 */
    } /* 其他 ADC 处理结束。 */
} /* ADC 注入电流回调结束。 */

static int32_t BLDC_GetBemfDiff(void)   /* 实现反电动势差值计算。 */
{ /* 差值计算开始。 */
    int32_t vzero;                      /* 保存虚拟中性点。 */
    int32_t ph_u;                       /* 保存 U 相差值。 */
    int32_t ph_v;                       /* 保存 V 相差值。 */
    int32_t ph_w;                       /* 保存 W 相差值。 */
    int32_t v_diff;                     /* 保存归一化差值。 */
    vzero = ((int32_t)g_bemf.u + (int32_t)g_bemf.v + (int32_t)g_bemf.w) / 3; /* 计算虚拟中性点。 */
    ph_u = (int32_t)g_bemf.u - vzero;  /* 计算 U 相相对值。 */
    ph_v = (int32_t)g_bemf.v - vzero;  /* 计算 V 相相对值。 */
    ph_w = (int32_t)g_bemf.w - vzero;  /* 计算 W 相相对值。 */
    v_diff = 0;                         /* 默认差值为零。 */
    switch (g_bldc.step)                /* 选择当前悬空相和方向。 */
    { /* 差值选择开始。 */
        case 1U:                        /* 第 1 步检测 W 相下降。 */
            v_diff = -ph_w;             /* 将正确方向归一为正值。 */
            break;                      /* 结束第 1 步。 */
        case 2U:                        /* 第 2 步检测 V 相上升。 */
            v_diff = ph_v;              /* 将正确方向归一为正值。 */
            break;                      /* 结束第 2 步。 */
        case 3U:                        /* 第 3 步检测 U 相下降。 */
            v_diff = -ph_u;             /* 将正确方向归一为正值。 */
            break;                      /* 结束第 3 步。 */
        case 4U:                        /* 第 4 步检测 W 相上升。 */
            v_diff = ph_w;              /* 将正确方向归一为正值。 */
            break;                      /* 结束第 4 步。 */
        case 5U:                        /* 第 5 步检测 V 相下降。 */
            v_diff = -ph_v;             /* 将正确方向归一为正值。 */
            break;                      /* 结束第 5 步。 */
        case 6U:                        /* 第 6 步检测 U 相上升。 */
            v_diff = ph_u;              /* 将正确方向归一为正值。 */
            break;                      /* 结束第 6 步。 */
        default:                        /* 处理无效步号。 */
            v_diff = 0;                 /* 无效步号不积分。 */
            break;                      /* 结束默认分支。 */
    } /* 差值选择结束。 */
    return v_diff;                      /* 返回归一化差值。 */
} /* 差值计算结束。 */
