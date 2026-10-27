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
 * 5. OPEN_LOOP 从 20 ms/步逐渐加速至 5 ms/步，并测量每步反电动势积分。
 * 6. 开环末段自动计算积分阈值，不需要预先填写电机反电动势常数。
 * 7. TRANSITION 保留 5 ms 强制换步兜底，同时尝试由积分阈值提前换步。
 * 8. 连续 6 次积分换步成功后进入 INTEGRATE，完全由反电动势积分换步。
 * 9. ADC 注入回调换算三相电流，并按当前换相步选择实际回流相电流。
 * 10. 电流给定从 0 平滑爬升到 3 A，回流相电流经过一阶低通后送入 PI 控制器。
 * 11. PI 控制器在 10 kHz 电流采样回调中调节占空比，并执行积分和输出双重限幅。
 * 12. 有效回流相连续两次超过 15 A 时，立即关闭 TIM1 MOE 并记录过流故障。
 * 13. 功率级为非同步整流：源相高桥 PWM、回流相低桥常导通、第三相悬空。
 * 14. 三相 CCR 保持相同，桥臂模式和使能位通过 TIM1 COM 事件同时生效。
 * 15. ADC2 规则组在 OC4REF 上升沿采 BEMF，注入组在下降沿采 OPAMP2 电流。
 *
 * 首次调试建议监视：g_bldc.state、g_bldc.step、get_diff、
 * g_bldc.bemf_integral、g_bldc.integral_limit、g_bldc.electrical_rpm、
 * g_bldc.transition_valid_count、g_bldc.forced_count、g_current、
 * g_current_control、g_bldc.duty、g_bldc.fault。
 * 首次上电务必使用限流电源、低母线电压，并确保可以立即断电。
 ******************************************************************************
 */

#include "hjf_mcpwm.h"                 /* 引入电机控制接口和 HAL 外设句柄。 */
#include "opamp.h"                     /* 引入三路内部运放句柄。 */
#include "stm32g4xx_ll_tim.h"          /* 引入定时器输出模式底层接口。 */
#include <stdint.h>                     /* 引入固定宽度整数类型。 */

#define BLDC_DUTY_SCALE                    1200U /* 定义占空比输入满量程。 */
#define BLDC_TEST_DUTY                      100U /* 定义首次测试占空比。 */
#define BLDC_PWM_FREQUENCY_HZ             10000U /* 定义快速控制回调频率。 */
#define BLDC_ALIGN_TICKS                   2000U /* 定义 200 ms 定位时间。 */
#define BLDC_OPEN_LOOP_RAMP_STEP               1U /* 定义每步缩短一个 PWM 周期。 */
#define BLDC_CALIBRATION_VALID_STEPS           12U /* 定义阈值标定有效步数。 */
#define BLDC_CALIBRATION_MAX_STEPS             72U /* 定义阈值标定最大尝试数。 */
#define BLDC_INTEGRAL_LIMIT_PERCENT            80U /* 定义积分阈值比例。 */
#define BLDC_MIN_CALIBRATION_INTEGRAL         500U /* 定义有效单步积分下限。 */
#define BLDC_MIN_INTEGRAL_LIMIT               400U /* 定义运行积分阈值下限。 */
#define BLDC_BEMF_DEADBAND                      10 /* 定义反电动势数字死区。 */
#define BLDC_BLANKING_TICKS                      3U /* 定义换步后消隐次数。 */
#define BLDC_MIN_COMMUTATION_TICKS              8U /* 定义最短换步间隔。 */
#define BLDC_TRANSITION_VALID_COUNT              6U /* 定义闭环接管成功次数。 */
#define BLDC_MAX_FORCED_COUNT                    3U /* 定义闭环最大连续超时数。 */
#define BLDC_TIMEOUT_MULTIPLIER                  2U /* 定义闭环超时倍数。 */
#define BLDC_TIMEOUT_MIN_TICKS                  30U /* 定义闭环超时下限。 */
#define BLDC_TIMEOUT_MAX_TICKS                 300U /* 定义闭环超时上限。 */
#define BLDC_ADC_SAMPLE_DELAY_COUNTS            20U /* 定义高桥导通后 ADC 延时。 */
#define BLDC_VALID_STEP_MIN                      1U /* 定义最小换相步号。 */
#define BLDC_VALID_STEP_MAX                      6U /* 定义最大换相步号。 */
#define BLDC_INTEGRAL_SATURATION        0x3FFFFFFFUL /* 定义积分器饱和值。 */
#define BLDC_CURRENT_CALIBRATION_SAMPLES       1024U /* 定义电流零点校准采样组数。 */
#define BLDC_CURRENT_CALIBRATION_TIMEOUT_TICKS 3000U /* 定义电流零点校准 300 ms 超时。 */
#define BLDC_CURRENT_MA_PER_ADC_COUNT             22L /* 定义每个 ADC 差值约等于 22 mA。 */
#define BLDC_OVERCURRENT_LIMIT_MA              15000L /* 定义首次测试软件过流值为 15 A。 */
#define BLDC_OVERCURRENT_CONFIRM_SAMPLES           2U /* 定义过流需要连续确认两次。 */
#define BLDC_CURRENT_OFFSET_MIN                 1400U /* 定义合理电流零点 ADC 下限。 */
#define BLDC_CURRENT_OFFSET_MAX                 2700U /* 定义合理电流零点 ADC 上限。 */
#define BLDC_CURRENT_TARGET_DEFAULT_MA          3000L /* 定义首次闭环测试目标电流为 3 A。 */
#define BLDC_CURRENT_RAMP_STEP_MA                 10L /* 定义每次采样最多增加 10 mA。 */
#define BLDC_CURRENT_FILTER_SHIFT                  3U /* 定义电流一阶低通系数为八分之一。 */
#define BLDC_CURRENT_KP_DIVISOR                  100L /* 定义比例项每 100 mA 输出一个占空比单位。 */
#define BLDC_CURRENT_KI_DIVISOR               100000L /* 定义积分项缩放系数。 */

#define BLDC_CURRENT_TARGET_MIN_MA               300L /* 定义非零运行目标电流下限为 0.3 A。 */
#define BLDC_CURRENT_TARGET_MAX_MA             10000L /* 定义可设置目标电流上限为 10 A。 */
#define BLDC_CURRENT_DUTY_MIN                    100L /* 定义闭环运行最小占空比命令。 */
#define BLDC_CURRENT_DUTY_MAX                    600L /* 定义闭环运行最大占空比为百分之五十。 */
#define BLDC_OPEN_LOOP_START_TICKS          200U /* 定义开环初始 20 ms/步。 */
#define BLDC_OPEN_LOOP_END_TICKS             20U /* 定义开环最终 2 ms/步。 */

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
    volatile int32_t current_u_ma;      /* 保存 U 相电流，单位为毫安。 */
    volatile int32_t current_v_ma;      /* 保存 V 相电流，单位为毫安。 */
    volatile int32_t current_w_ma;      /* 保存 W 相电流，单位为毫安。 */
    volatile int32_t motor_current_ma;  /* 保存当前回流相有效电流，单位为毫安。 */
    volatile uint32_t sum_u;            /* 保存 U 相零点校准累计值。 */
    volatile uint32_t sum_v;            /* 保存 V 相零点校准累计值。 */
    volatile uint32_t sum_w;            /* 保存 W 相零点校准累计值。 */
    volatile uint16_t count_adc1;       /* 保存 ADC1 注入序列校准采样数。 */
    volatile uint16_t count_adc2;       /* 保存 ADC2 注入序列校准采样数。 */
    volatile uint8_t calibrated;        /* 标记三相零点校准是否完成。 */
    volatile uint8_t overcurrent_count; /* 保存连续过流确认次数。 */
} BLDC_Current_t;                       /* 三相电流数据结构结束。 */

typedef struct /* 定义电流 PI 控制器运行数据。 */
{ /* 电流 PI 数据结构开始。 */
    volatile int32_t target_ma;        /* 保存用户设置的最终目标电流。 */
    volatile int32_t ramped_target_ma; /* 保存经过斜坡限制的当前目标电流。 */
    volatile int32_t filtered_current_ma; /* 保存低通滤波后的电机电流。 */
    volatile int32_t error_ma;         /* 保存目标电流与实际电流误差。 */
    volatile int32_t proportional_duty; /* 保存比例项输出。 */
    volatile int32_t integral_accumulator; /* 保存带缩放的积分累计值。 */
    volatile int32_t output_duty;      /* 保存 PI 最终占空比命令。 */
    volatile uint8_t filter_initialized; /* 标记电流滤波器是否已经初始化。 */
} BLDC_CurrentControl_t;               /* 电流 PI 数据结构结束。 */

static void BLDC_HardwarePrepare(void); /* 声明硬件准备函数。 */
static void BLDC_AllOutputsOff(void);   /* 声明关闭输出函数。 */
static void BLDC_SetDuty(uint16_t duty); /* 声明占空比设置函数。 */
static void BLDC_ApplyStep(uint8_t step); /* 声明换相步应用函数。 */
static void BLDC_Commutate(void);       /* 声明前进一步函数。 */
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
static BLDC_CurrentControl_t g_current_control = /* 创建电流 PI 控制器实例。 */
{ /* 电流 PI 控制器初始化开始。 */
    .target_ma = BLDC_CURRENT_TARGET_DEFAULT_MA, /* 设置首次测试目标电流。 */
    .output_duty = BLDC_TEST_DUTY       /* 设置首次测试初始占空比。 */
}; /* 电流 PI 控制器初始化结束。 */
static volatile int32_t get_diff;       /* 保存归一化反电动势差值。 */
static BLDC_Control_t g_bldc =          /* 创建控制器实例。 */
{ /* 控制器初始化开始。 */
    .state = BLDC_STATE_STOP,           /* 初始状态设为停止。 */
    .step = 1U,                         /* 初始步号设为 1。 */
    .open_loop_ticks = BLDC_OPEN_LOOP_START_TICKS, /* 设置初始开环周期。 */
    .timeout_ticks = BLDC_TIMEOUT_MAX_TICKS, /* 设置初始闭环超时。 */
    .fault = BLDC_FAULT_NONE            /* 初始故障码设为无故障。 */
}; /* 控制器初始化结束。 */

void BLDC_test(void)                    /* 实现 Integrate 测试入口。 */
{ /* 测试入口开始。 */
    BLDC_HardwarePrepare();             /* 准备 ADC、DMA 和 TIM1。 */
    BLDC_SetDuty(BLDC_TEST_DUTY);       /* 设置测试占空比和采样点。 */
    g_current_control.ramped_target_ma = 0L; /* 从零开始平滑增加目标电流。 */
    g_current_control.filtered_current_ma = 0L; /* 清零上次运行的滤波电流。 */
    g_current_control.error_ma = 0L;    /* 清零上次运行的电流误差。 */
    g_current_control.proportional_duty = 0L; /* 清零上次运行的比例项。 */
    g_current_control.integral_accumulator = BLDC_TEST_DUTY * BLDC_CURRENT_KI_DIVISOR; /* 用初始占空比初始化积分项。 */
    g_current_control.output_duty = BLDC_TEST_DUTY; /* 保存初始占空比输出。 */
    g_current_control.filter_initialized = 0U; /* 要求首次有效电流重新初始化滤波器。 */
    g_bldc.fault = BLDC_FAULT_NONE;     /* 清除上一次运行留下的故障码。 */
    g_bldc.state = BLDC_STATE_CALIBRATE; /* 先在功率输出关闭时校准电流零点。 */
    g_bldc.state_ticks = 0U;            /* 清零状态计时。 */
    __HAL_TIM_SET_COUNTER(&htim1, 0U);  /* 清零 TIM1 计数器。 */
    if (HAL_TIM_Base_Start(&htim1) != HAL_OK) /* 启动 TIM1。 */
    { /* TIM1 启动失败处理开始。 */
        Error_Handler();                /* 进入统一错误处理。 */
    } /* TIM1 启动失败处理结束。 */
    __HAL_TIM_MOE_DISABLE(&htim1);      /* 校准期间保持高级定时器主输出关闭。 */
    while (1)                           /* 保持后台任务运行。 */
    { /* 后台循环开始。 */
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
} /* 换相步应用结束。 */

static void BLDC_Commutate(void)        /* 实现向前换相一步。 */
{ /* 前进一步开始。 */
    uint8_t next_step;                  /* 保存下一步号。 */
    uint32_t elapsed_ticks;             /* 保存本步运行周期。 */
    elapsed_ticks = g_bldc.commutation_ticks; /* 读取本步周期。 */
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

static void BLDC_ControlTick(void)      /* 实现 10 kHz 快速状态机。 */
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
                BLDC_Commutate();       /* 开始连续六步换相。 */
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
                BLDC_Commutate();       /* 执行强制换步。 */
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
                    g_bldc.open_loop_ticks = BLDC_OPEN_LOOP_END_TICKS; /* 保持 5 ms/步。 */
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
                BLDC_Commutate();       /* 根据积分换步。 */
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
                BLDC_Commutate();       /* 以 5 ms/步兜底。 */
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
                BLDC_Commutate();       /* 根据积分换步。 */
            } /* 正常闭环换步结束。 */
            else if (g_bldc.commutation_ticks >= g_bldc.timeout_ticks) /* 检查闭环超时。 */
            { /* 闭环超时处理开始。 */
                g_bldc.forced_count++;  /* 增加连续超时数。 */
                BLDC_Commutate();       /* 强制换一步尝试恢复。 */
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
    if ((g_current.offset_u < BLDC_CURRENT_OFFSET_MIN) || /* 检查 U 相零点下限。 */
        (g_current.offset_u > BLDC_CURRENT_OFFSET_MAX) || /* 检查 U 相零点上限。 */
        (g_current.offset_v < BLDC_CURRENT_OFFSET_MIN) || /* 检查 V 相零点下限。 */
        (g_current.offset_v > BLDC_CURRENT_OFFSET_MAX) || /* 检查 V 相零点上限。 */
        (g_current.offset_w < BLDC_CURRENT_OFFSET_MIN) || /* 检查 W 相零点下限。 */
        (g_current.offset_w > BLDC_CURRENT_OFFSET_MAX)) /* 检查 W 相零点上限。 */
    { /* 电流零点范围异常处理开始。 */
        BLDC_EnterFault(BLDC_FAULT_CURRENT_CALIBRATION); /* 记录电流采样硬件异常并停机。 */
        return;                         /* 不允许在无效零点下打开功率输出。 */
    } /* 电流零点范围异常处理结束。 */
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

static void BLDC_UpdatePhaseCurrents(void) /* 实现三相电流换算和快速软件过流保护。 */
{ /* 三相电流更新开始。 */
    int32_t absolute_current_ma;        /* 保存当前回流相电流绝对值。 */
    g_current.current_u_ma = ((int32_t)g_current.raw_u - (int32_t)g_current.offset_u) * BLDC_CURRENT_MA_PER_ADC_COUNT; /* 换算 U 相电流。 */
    g_current.current_v_ma = ((int32_t)g_current.raw_v - (int32_t)g_current.offset_v) * BLDC_CURRENT_MA_PER_ADC_COUNT; /* 换算 V 相电流。 */
    g_current.current_w_ma = ((int32_t)g_current.raw_w - (int32_t)g_current.offset_w) * BLDC_CURRENT_MA_PER_ADC_COUNT; /* 换算 W 相电流。 */
    switch (g_bldc.step)               /* 根据非同步六步状态选择持续导通的回流相。 */
    { /* 回流相电流选择开始。 */
        case 1U:                        /* 第 1 步的回流相为 V 相。 */
        case 6U:                        /* 第 6 步的回流相也为 V 相。 */
            g_current.motor_current_ma = g_current.current_v_ma; /* 使用 V 相采样作为电机电流。 */
            break;                      /* 结束 V 相回流选择。 */
        case 2U:                        /* 第 2 步的回流相为 W 相。 */
        case 3U:                        /* 第 3 步的回流相也为 W 相。 */
            g_current.motor_current_ma = g_current.current_w_ma; /* 使用 W 相采样作为电机电流。 */
            break;                      /* 结束 W 相回流选择。 */
        case 4U:                        /* 第 4 步的回流相为 U 相。 */
        case 5U:                        /* 第 5 步的回流相也为 U 相。 */
            g_current.motor_current_ma = g_current.current_u_ma; /* 使用 U 相采样作为电机电流。 */
            break;                      /* 结束 U 相回流选择。 */
        default:                        /* 处理理论不可达的无效步号。 */
            g_current.motor_current_ma = 0L; /* 无效步号不输出电流值。 */
            return;                     /* 结束本次电流保护计算。 */
    } /* 回流相电流选择结束。 */
    if ((g_bldc.state == BLDC_STATE_ALIGN) || /* 判断定位状态是否已经输出功率。 */
        (g_bldc.state == BLDC_STATE_OPEN_LOOP) || /* 判断开环状态是否已经输出功率。 */
        (g_bldc.state == BLDC_STATE_TRANSITION) || /* 判断接管状态是否已经输出功率。 */
        (g_bldc.state == BLDC_STATE_INTEGRATE)) /* 判断闭环状态是否已经输出功率。 */
    { /* 运行状态过流检查开始。 */
        absolute_current_ma = g_current.motor_current_ma; /* 复制当前回流相电流。 */
        if (absolute_current_ma < 0L)   /* 检查当前电流是否为负值。 */
        { /* 电流绝对值处理开始。 */
            absolute_current_ma = -absolute_current_ma; /* 将负电流转换为绝对值。 */
        } /* 电流绝对值处理结束。 */
        if (absolute_current_ma >= BLDC_OVERCURRENT_LIMIT_MA) /* 检查是否超过 15 A 软件限制。 */
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

static void BLDC_RunCurrentControl(void) /* 实现 10 kHz 电流滤波、给定斜坡和 PI 控制。 */
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
            (measured_current_ma - g_current_control.filtered_current_ma) >> BLDC_CURRENT_FILTER_SHIFT; /* 使用八分之一系数抑制换相噪声。 */
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
        g_current.raw_u = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1); /* 读取 OPAMP1 的 U 相电流。 */
        g_current.raw_w = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2); /* 读取 OPAMP3 的 W 相电流。 */
        if ((g_bldc.state == BLDC_STATE_CALIBRATE) && /* 检查当前是否正在校准电流零点。 */
            (g_current.count_adc1 < BLDC_CURRENT_CALIBRATION_SAMPLES)) /* 检查 ADC1 校准样本是否足够。 */
        { /* ADC1 零点累计开始。 */
            g_current.sum_u += g_current.raw_u; /* 累加 U 相零电流 ADC 值。 */
            g_current.sum_w += g_current.raw_w; /* 累加 W 相零电流 ADC 值。 */
            g_current.count_adc1++;     /* 增加 ADC1 注入序列采样计数。 */
        } /* ADC1 零点累计结束。 */
        if (g_current.calibrated != 0U) /* 检查三相电流零点是否已经有效。 */
        { /* 运行电流处理开始。 */
            BLDC_UpdatePhaseCurrents(); /* 使用最新三相数据换算电流并检查过流。 */
        } /* 运行电流处理结束。 */
    } /* ADC1 注入序列处理结束。 */
    else if (hadc->Instance == ADC2)    /* 处理 ADC2 的 V 相注入转换。 */
    { /* ADC2 注入转换处理开始。 */
        g_current.raw_v = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1); /* 读取 OPAMP2 的 V 相电流。 */
        if ((g_bldc.state == BLDC_STATE_CALIBRATE) && /* 检查当前是否正在校准电流零点。 */
            (g_current.count_adc2 < BLDC_CURRENT_CALIBRATION_SAMPLES)) /* 检查 ADC2 校准样本是否足够。 */
        { /* ADC2 零点累计开始。 */
            g_current.sum_v += g_current.raw_v; /* 累加 V 相零电流 ADC 值。 */
            g_current.count_adc2++;     /* 增加 ADC2 注入转换采样计数。 */
        } /* ADC2 零点累计结束。 */
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
