/*
 ******************************************************************************
 * 文件名称：hjf_mcpwm.c
 * 功能说明：FD6288T + TIM1 的 VESC 风格无感方波 Integrate 控制。
 *
 * 当前程序执行流程：
 * 1. main.c 初始化外设后调用 BLDC_test()，该函数只初始化控制硬件并立即返回。
 * 2. CALIBRATE 保持全部桥臂关闭，采集 1024 组三相电流零点，完成后进入 STOP。
 * 3. main.c 调用电流、占空比或速度接口写入命令，有效命令会自动请求启动。
 * 4. main.c 的 while 循环持续调用 BLDC_Process()，执行命令斜坡、电压保护、启停和故障恢复管理。
 * 5. ALIGN 固定第 1 步 200 ms，使转子建立已知初始电角度。
 * 6. OPEN_LOOP 从 6 ms/步快速加速至 2 ms/步，并测量每步反电动势积分。
 * 7. 开环末段自动计算积分阈值，不需要预先填写电机反电动势常数。
 * 8. TRANSITION 保留 2 ms 强制换步兜底，同时尝试由积分阈值提前换步。
 * 9. 连续 6 次积分换步成功后进入 INTEGRATE，完全由反电动势积分换步。
 * 10. BLDC_Stop() 清除上层运行命令并将内部电流目标斜坡到零，然后关闭全部桥臂。
 * 11. BLDC_EmergencyStop() 不等待电流斜坡，立即清除 TIM1 MOE 和全部桥臂使能。
 * 12. 反电动势标定失败、接管失败和闭环失步属于软故障，等待 250 ms 后自动重启。
 * 13. 软故障最多自动重启 3 次，每次适量提高启动占空比，闭环稳定 2 秒后清零计数。
 * 14. 电流校准、过流、母线和换相硬件故障保持锁存，不允许后台自动重新通电。
 * 15. ADC1 采 U/W 相、ADC2 采 V 相，同一 PWM 周期的两组注入转换完成后才处理电流。
 * 16. 三相分别使用独立增益系数换算电流，启动校准同时记录每相零点峰峰噪声。
 * 17. 每次换相后的前 4 组电流样本只做诊断，不送入 PI 和软件过流判断。
 * 18. 按六步状态选择持续导通的低桥回流相，并记录六个扇区各自的滤波电流。
 * 19. 定位、开环、接管和闭环最初 300 ms 至少使用 500 mA，避免低电流接管后原地抖动。
 * 20. 闭环稳定时间结束后，电流从 500 mA 平滑降到当前模式目标，回流相电流送入 PI 控制器。
 * 21. PI 控制器在 20 kHz 配对电流回调中调节占空比，并执行积分和输出双重限幅。
 * 22. 任意一相连续两次超过过流阈值时，立即关闭 TIM1 MOE 并锁存过流故障。
 * 23. 功率级为非同步整流：源相高桥 PWM、回流相低桥常导通、第三相悬空。
 * 24. 三相 CCR 保持相同，桥臂模式和使能位通过 TIM1 COM 事件同时生效。
 * 25. ADC2 规则组在 OC4REF 上升沿采 BEMF，注入组在下降沿采 OPAMP2 电流。
 * 26. 每次成功换相把触发原因和关键数据写入 RAM 环形记录，不在实时控制中打印串口。
 * 27. ADC1 规则组每 10 ms 采集一次 VBUS，按 20 kΩ/1 kΩ 分压换算并低通滤波。
 * 28. 运行中母线低于 19 V 或高于 31 V 时锁存硬故障并立即关闭 TIM1 主输出。
 * 29. 开环标定结束后保存基础磁链积分，并按母线电压和电角转速动态调整运行阈值。
 * 30. 仅在 INTEGRATE 状态按电角转速降低积分阈值，实现可限幅的转速相关换相提前。
 * 31. 提前量使用积分阈值千分比表示，不等同于机械角度或电角度数值。
 * 32. 运行电流命令独立保存请求值和斜坡值，默认按 500 mA/s 上升、1000 mA/s 下降。
 * 33. 电流模式有效命令下限为 100 mA，低于该值视为释放电机并执行软停机。
 * 34. 启动阶段的 500 mA 可靠接管电流独立于运行命令，闭环保持结束后才交给外层斜坡值。
 * 35. 按 VESC BLDC 公式用滤波电机电流乘以实际占空比估算母线输入电流。
 * 36. 输入功率上限先除以母线电压得到等效输入电流上限，再反算允许的电机电流。
 * 37. 输入限幅仅在 INTEGRATE 稳定保持三百毫秒后参与，避免削弱定位、开环和闭环接管。
 * 38. 若输入限制要求低于当前模式最低电流，则执行软停机而不维持不可控的更低电流。
 * 39. 上层统一支持停止、电流、占空比和电角转速四种控制模式，命令幅值保持非负并由方向接口选择转向。
 * 40. 电流模式直接生成运行电流命令，占空比和速度模式使用一千赫兹外环生成电流命令。
 * 41. 占空比和速度外环的输出仍经过命令斜坡、电流 PI、输入电流和输入功率限制。
 * 42. 模式切换时使用当前运行电流预置外环积分项，减小切换瞬间的转矩突变。
 * 43. 统一限幅管理器同时仲裁电机电流、母线电流、输入功率、PWM 占空比和电角转速。
 * 44. 每种限制使用独立位标志记录，多个限制同时存在时可以在同一个原因变量中观察。
 * 45. 速度命令先限制到实机验证范围，再进入速度 PI，避免不可达目标长期推高电流。
 * 46. 速度外环检测到 PWM 或输入限制饱和时，积分器跟踪实际可实现电流并快速退出饱和。
 * 47. 电流 PI 的最小和最大占空比改为运行时配置，但默认仍保持原有安全测试范围。
 * 48. 电流模式严格保持用户转矩命令，速度和占空比外环则允许闭环电流最低降到 100 mA。
 * 49. 默认使用速度模式运行，空载时由速度 PI 自动降低电流而不是持续强制四百毫安转矩。
 * 50. INTEGRATE 换相超时先仿照 VESC 前进两个换相步恢复转矩，连续失败后才进入失步重启。
 * 51. 转速估算累计完整六次正常换相的总周期，避免单个扇区周期取整造成电角转速跳变。
 * 52. 六步原始电角转速经过十分之一系数低通滤波后供速度 PI 和动态积分阈值共同使用。
 * 53. 跨两步失步恢复会清除尚未完成的测速窗口，避免异常恢复周期污染后续转速估算。
 * 54. 速度外环使用一毫秒显式时间基准的可配置 PID，默认增益保持第十一步已经验证的响应。
 * 55. 速度 PID 的比例、积分、原始微分、滤波微分和限幅前后输出全部保存在 RAM 中供监视。
 * 56. 速度积分器允许释放到零毫安，但最终正向输出仍保持一百毫安，不启用负电流或再生制动。
 * 57. 速度 PID 遇到电流、输入功率或 PWM 上限时回算可实现积分项，避免负载期间继续积分饱和。
 * 58. 方向状态独立保存在运动跟踪器中，只有全部功率输出关闭时才允许修改正反转方向。
 * 59. 六步滤波转速乘以方向符号生成有符号 ERPM，正转为正数，反转为负数。
 * 60. 每次成功换相更新有符号和绝对换相计数器，跨两步失步恢复按实际两步累计。
 * 61. 换相计数器跨普通停机和自动重启保留，只有初始化或显式清零接口才会归零。
 * 62. 最近一次启动的定位、开环、接管和闭环进入时刻全部以毫秒保存，监视刷新漏帧后仍可追溯。
 * 63. 最近一次启动同时保存开环换相、接管积分换相和接管超时换相累计次数。
 * 64. 第十三步建立方向与位置诊断基础，正转实测的启动换相总数与有符号里程计完全一致。
 * 65. 反转复用同一组六个功率状态，但步号按 1、6、5、4、3、2 的顺序递减并循环。
 * 66. 反转时将悬空相反电动势极性整体取反，使积分器仍然只累计已经归一化的正值。
 * 67. 当前只允许停机后选择方向，不允许带电直接反向，也不启用负电流或再生制动。
 * 68. 第十四步让电流命令直接使用正负号，并增加带符号占空比和带符号速度命令接口。
 * 69. 运行中收到相反方向命令时先暂停原外环，将运行电流按照配置的下降斜率降到零。
 * 70. 电流降零后关闭全部桥臂，至少滑行五百毫秒并连续二十毫秒检查三相 BEMF 峰峰值。
 * 71. 若转子三秒内仍未满足 BEMF 静止阈值，则记录一次等待超时并继续使用定位流程安全重启。
 * 72. 新方向生效后自动恢复换向期间保存的电流、占空比或速度命令，不要求上层重复发送。
 * 73. 换向过程中可以更新最终目标，显式软停机、紧急停机或任何故障都会取消待执行换向。
 * 74. 第十四步仍然只进行先停后反转，不进行反接制动、负转矩控制或母线再生制动。
 * 75. 第十五步保留三相零点校准后的原始正负电流，不再让诊断链在选相前丢失电流方向。
 * 76. 三相电流极性可在功率关闭时分别配置，默认正一保持现有硬件换算方向不变。
 * 77. 每个六步扇区同时选择源相、回流相和悬空相，回流相建立统一带符号电机电流。
 * 78. 原 motor_current_ma 继续保存带符号电流绝对值，现有电流 PI、过流和限流行为保持不变。
 * 79. 带符号电机电流建立独立低通、六扇区滤波、正负峰值和显著负样本累计诊断。
 * 80. 带符号滤波电流乘以实际占空比估算正负母线电流和功率，负值仅表示潜在回充方向。
 * 81. 软件过流仍取三相绝对最大值，任何极性的过流都不能绕过原有快速保护。
 * 82. 第十五步不改变桥臂驱动且不启用制动，实机确认极性后再实现负转矩电流闭环。
 * 83. 第十六步针对不能吸收回充能量的电池，明确禁止主动负转矩和再生制动。
 * 84. 非零制动电流请求只记录拒绝次数并执行原有滑行停机，不产生反向桥臂矢量。
 * 85. 带符号滤波电流乘占空比得到的负母线电流连续二十毫秒后才确认真实回充。
 * 86. 回充检测只在 INTEGRATE 稳定保持阶段生效，启动和换相瞬态不会触发保护。
 * 87. 确认回充后立即撤销运行命令、关闭 TIM1 全部输出并禁止自动重新启动。
 * 88. 母线达到二十九伏后在一伏区间内线性降低允许电流，相当于逐步降低输入功率。
 * 89. 母线达到三十伏时不再尝试调节占空比，而是立即关闭桥臂进入自由滑行。
 * 90. 母线下降到二十八伏以下才解除软过压启动禁止，避免阈值附近反复启停。
 * 91. 三十一伏硬过压仍保持锁存，并且桥臂关闭后也继续检查体二极管回充造成的过压。
 * 92. 回充次数、软过压次数、滑行停机次数、负峰值和降额比例全部保存在 RAM 供监视。
 * 93. 软件无法消耗机械能，高速外力拖动仍需依靠硬件钳位或能量泄放电路保证绝对安全。
 *
 * 首次调试建议监视：g_bldc.state、g_bldc.step、get_diff、
 * g_bldc.bemf_integral、g_bldc.integral_limit_unadvanced、g_bldc.integral_limit、
 * g_bldc.phase_advance_permille、g_bldc.raw_electrical_rpm、g_bldc.electrical_rpm、
 * g_bldc.speed_period_sum_ticks、g_bldc.speed_commutation_count、g_bldc.speed_sample_count、
 * g_bldc.transition_valid_count、g_bldc.forced_count、g_current、
 * g_current_control、g_current_scale、g_bldc.duty、g_bldc.fault、g_comm_trace、
 * g_comm_trace_index、g_comm_trace_sequence、g_bus_voltage、g_run_manager、g_control_command、
 * g_current_command、g_speed_pid、g_motion、g_reversal、g_current_polarity、g_input_limit、
 * g_output_limit、g_energy_protection。
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
#define BLDC_SPEED_AVERAGE_COMMUTATIONS       6U /* 定义每个测速窗口累计完整六次换相。 */
#define BLDC_SPEED_FILTER_DIVISOR             10L /* 定义转速低通滤波每次修正原始差值的十分之一。 */
#define BLDC_SPEED_FILTER_Q_SHIFT             16U /* 定义转速低通内部使用十六位小数定点格式。 */
#define BLDC_ALIGN_TICKS                   4000U /* 定义 200 ms 定位时间。 */
#define BLDC_OPEN_LOOP_RAMP_STEP               4U /* 定义每步缩短四个控制周期，使开环加速时间约为八十毫秒。 */
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
#define BLDC_CURRENT_POLARITY_THRESHOLD_MA          100L /* 定义判断带符号电流极性的最小有效幅值。 */
#define BLDC_OVERCURRENT_LIMIT_DEFAULT_MA      15000L /* 定义软件过流保护默认值为 15 A。 */
#define BLDC_OVERCURRENT_LIMIT_MIN_MA            500L /* 定义软件过流保护可设置下限为 0.5 A。 */
#define BLDC_OVERCURRENT_LIMIT_MAX_MA          15000L /* 定义软件过流保护可设置上限为 15 A。 */
#define BLDC_OVERCURRENT_CONFIRM_SAMPLES           2U /* 定义过流需要连续确认两次。 */
#define BLDC_CURRENT_OFFSET_MIN                 1400U /* 定义合理电流零点 ADC 下限。 */
#define BLDC_CURRENT_OFFSET_MAX                 2700U /* 定义合理电流零点 ADC 上限。 */
#define BLDC_CURRENT_RAMP_STEP_MA                  5L /* 定义每次采样最多增加 5 mA，保持原电流斜坡。 */
#define BLDC_STARTUP_CURRENT_MIN_MA               500L /* 定义定位、开环和接管阶段的最低电流为五百毫安。 */
#define BLDC_CLOSED_LOOP_CURRENT_HOLD_TICKS      6000U /* 定义进入闭环后继续保持启动电流三百毫秒。 */
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
#define BLDC_VBUS_SOFT_DERATE_START_MV          29000U /* 定义母线从二十九伏开始线性降低允许功率。 */
#define BLDC_VBUS_SOFT_COAST_STOP_MV            30000U /* 定义母线达到三十伏时立即关闭桥臂滑行。 */
#define BLDC_VBUS_SOFT_RELEASE_MV               28000U /* 定义母线降到二十八伏后解除软过压启动禁止。 */
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

#define BLDC_CURRENT_TARGET_MAX_MA             10000L /* 定义可设置目标电流上限为 10 A。 */
#define BLDC_CURRENT_COMMAND_RELEASE_MA            100L /* 定义电流模式低于一百毫安时释放电机。 */
#define BLDC_CLOSED_LOOP_CURRENT_MIN_MA            100L /* 定义速度和占空比外环最低闭环电流为一百毫安。 */
#define BLDC_CURRENT_RISE_DEFAULT_MA_PER_SECOND   500U /* 定义运行电流命令默认每秒上升五百毫安。 */
#define BLDC_CURRENT_FALL_DEFAULT_MA_PER_SECOND  1000U /* 定义运行电流命令默认每秒下降一千毫安。 */
#define BLDC_CURRENT_RAMP_MIN_MA_PER_SECOND       100U /* 定义可设置电流命令斜率下限为每秒一百毫安。 */
#define BLDC_CURRENT_RAMP_MAX_MA_PER_SECOND     20000U /* 定义可设置电流命令斜率上限为每秒二十安。 */
#define BLDC_CURRENT_COMMAND_TIME_BASE_MS        1000U /* 定义电流命令斜率从每秒换算到毫秒的时间基准。 */
#define BLDC_CURRENT_COMMAND_MAX_ELAPSED_MS       100U /* 定义单次后台迟滞最多补偿一百毫秒。 */
#define BLDC_INPUT_CURRENT_LIMIT_DEFAULT_MA        600U /* 定义默认母线输入电流上限为六百毫安。 */
#define BLDC_INPUT_CURRENT_LIMIT_MIN_MA            100U /* 定义可设置母线输入电流上限下限为一百毫安。 */
#define BLDC_INPUT_CURRENT_LIMIT_MAX_MA          20000U /* 定义可设置母线输入电流上限上限为二十安。 */
#define BLDC_INPUT_POWER_LIMIT_DEFAULT_MW        15000U /* 定义默认母线输入功率上限为十五瓦。 */
#define BLDC_INPUT_POWER_LIMIT_MIN_MW             1000U /* 定义可设置输入功率上限下限为一瓦。 */
#define BLDC_INPUT_POWER_LIMIT_MAX_MW          1000000U /* 定义可设置输入功率上限上限为一千瓦。 */
#define BLDC_REGEN_INPUT_CURRENT_THRESHOLD_MA       50L /* 定义确认回充使用的负母线电流绝对阈值为五十毫安。 */
#define BLDC_REGEN_CONFIRM_TIME_MS                    20U /* 定义负母线电流连续二十毫秒才确认回充。 */
#define BLDC_CONTROL_PERIOD_MS                       1U /* 定义占空比和速度外环执行周期为一毫秒。 */
#define BLDC_CONTROL_CURRENT_MARGIN_MA             100L /* 定义外环目标距离软件过流值保留一百毫安。 */
#define BLDC_MOTOR_CURRENT_LIMIT_DEFAULT_MA        1100L /* 定义统一管理器默认最大电机电流为一点一安。 */
#define BLDC_MOTOR_CURRENT_LIMIT_MIN_MA             100L /* 定义统一电机电流限制可设置下限为一百毫安。 */
#define BLDC_MOTOR_CURRENT_LIMIT_MAX_MA           10000L /* 定义统一电机电流限制可设置上限为十安。 */
#define BLDC_DUTY_COMMAND_MIN_PERMILLE             100U /* 定义占空比模式最低有效命令为百分之十。 */
#define BLDC_DUTY_COMMAND_MAX_PERMILLE             500U /* 定义占空比模式最高命令为百分之五十。 */
#define BLDC_DUTY_LIMIT_MIN_DEFAULT_COUNTS          100U /* 定义电流 PI 默认最小占空比计数。 */
#define BLDC_DUTY_LIMIT_MAX_DEFAULT_COUNTS          600U /* 定义电流 PI 默认最大占空比计数为百分之五十。 */
#define BLDC_DUTY_LOOP_KP_MA_PER_COUNT               2L /* 定义占空比误差每计数输出两毫安比例电流。 */
#define BLDC_DUTY_LOOP_KI_DIVISOR                    50L /* 定义一千赫兹占空比积分项缩放系数。 */
#define BLDC_SPEED_COMMAND_MIN_ERPM                1500U /* 定义速度模式最低有效电角转速。 */
#define BLDC_SPEED_COMMAND_MAX_ERPM               20000U /* 定义速度模式最高电角转速。 */
#define BLDC_SPEED_LIMIT_MIN_DEFAULT_ERPM          1500U /* 定义统一管理器默认最低有效电角转速。 */
#define BLDC_SPEED_LIMIT_MAX_DEFAULT_ERPM          6500U /* 定义实机验证后的默认最高电角转速。 */
#define BLDC_SPEED_PID_SCALE                       1000L /* 定义速度 PID 增益和积分内部状态的千分之一缩放。 */
#define BLDC_SPEED_PID_KP_DEFAULT_X1000              50U /* 定义默认比例增益为每 ERPM 输出零点零五毫安。 */
#define BLDC_SPEED_PID_KI_DEFAULT_X1000            1000U /* 定义默认积分增益为每 ERPM 每秒一毫安。 */
#define BLDC_SPEED_PID_KD_DEFAULT_X1000               0U /* 定义默认关闭微分项，待实机验证后再增加。 */
#define BLDC_SPEED_PID_D_FILTER_DEFAULT_PERMILLE     200U /* 定义默认微分低通每次修正原始差值的百分之二十。 */
#define BLDC_SPEED_PID_KP_MAX_X1000                 1000U /* 定义比例增益可配置安全上限。 */
#define BLDC_SPEED_PID_KI_MAX_X1000                10000U /* 定义积分增益可配置安全上限。 */
#define BLDC_SPEED_PID_KD_MAX_X1000                  100U /* 定义微分增益可配置安全上限。 */
#define BLDC_SPEED_PID_MAX_ELAPSED_MS                  10U /* 定义速度 PID 单次最多补偿十毫秒后台迟滞。 */
#define BLDC_CURRENT_DUTY_MIN                    100L /* 定义闭环运行最小占空比命令。 */
#define BLDC_CURRENT_DUTY_MAX                    600L /* 定义闭环运行最大占空比为百分之五十。 */
#define BLDC_OPEN_LOOP_START_TICKS          120U /* 定义开环初始 6 ms/步，兼顾快速启动与闭环接管裕量。 */
#define BLDC_OPEN_LOOP_END_TICKS             40U /* 定义开环最终 2 ms/步。 */
#define BLDC_RESTART_MAX_COUNT                 3U /* 定义软故障最多自动重启三次。 */
#define BLDC_RESTART_DELAY_MS                250U /* 定义软故障重启前等待二百五十毫秒。 */
#define BLDC_RESTART_STABLE_MS              2000U /* 定义闭环稳定两秒后清除重启计数。 */
#define BLDC_REVERSAL_COAST_MIN_MS            500U /* 定义换向关闭桥臂后的最短滑行等待时间。 */
#define BLDC_REVERSAL_COAST_MAX_MS           3000U /* 定义换向等待转子减速的最大时间。 */
#define BLDC_REVERSAL_BEMF_QUIET_COUNTS        80U /* 定义允许重新定位的三相反电动势最大峰峰值。 */
#define BLDC_REVERSAL_QUIET_CONFIRM_MS          20U /* 定义反电动势连续安静二十毫秒后允许反向启动。 */
#define BLDC_RESTART_DUTY_STEP                25U /* 定义每次重启增加的初始占空比。 */
#define BLDC_RESTART_DUTY_MAX                350U /* 定义自动重启初始占空比上限。 */

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

typedef enum /* 定义运行管理器停机原因。 */
{ /* 停机原因枚举开始。 */
    BLDC_STOP_REASON_NONE = 0U,         /* 表示尚未发生停机。 */
    BLDC_STOP_REASON_USER,              /* 表示用户请求软停机。 */
    BLDC_STOP_REASON_EMERGENCY,         /* 表示用户请求紧急停机。 */
    BLDC_STOP_REASON_SOFT_FAULT_RETRY,  /* 表示软故障正在等待自动恢复。 */
    BLDC_STOP_REASON_RETRY_EXHAUSTED,   /* 表示软故障自动重启次数已经耗尽。 */
    BLDC_STOP_REASON_HARD_FAULT,        /* 表示不可自动恢复的硬故障停机。 */
    BLDC_STOP_REASON_DIRECTION_CHANGE,  /* 表示有符号命令正在执行安全换向停机。 */
    BLDC_STOP_REASON_REGEN_PROTECTION,  /* 表示持续回充触发立即关闭桥臂。 */
    BLDC_STOP_REASON_SOFT_OVERVOLTAGE   /* 表示母线达到软过压停机阈值。 */
} BLDC_StopReason_t;                    /* 停机原因枚举结束。 */

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
    volatile uint16_t two_step_recovery_count; /* 保存本次启动累计执行的跨两步恢复次数。 */
    volatile uint32_t timeout_ticks;    /* 保存闭环换步超时。 */
    volatile uint32_t raw_electrical_rpm; /* 保存最近一个完整六步窗口计算的原始电角转速。 */
    volatile uint32_t electrical_rpm;   /* 保存供控制器使用的低通滤波电角转速。 */
    volatile uint32_t speed_period_sum_ticks; /* 保存当前测速窗口累计的换相周期。 */
    volatile uint32_t speed_commutation_count; /* 保存当前测速窗口已经累计的正常换相次数。 */
    volatile uint32_t speed_sample_count; /* 保存已经生成的完整六步转速样本数量。 */
    volatile uint32_t speed_filter_q16; /* 保存带十六位小数的低通滤波转速内部状态。 */
    volatile BLDC_Fault_t fault;        /* 保存最近一次故障原因。 */
} BLDC_Control_t;                       /* 运行数据结构结束。 */

typedef struct /* 定义启停和自动恢复运行管理器。 */
{ /* 运行管理器结构开始。 */
    volatile uint8_t initialized;       /* 标记底层硬件和快速控制回调已经启动。 */
    volatile uint8_t run_command;       /* 保存用户运行命令，一表示请求运行。 */
    volatile uint8_t stop_pending;      /* 标记正在执行电流斜坡软停机。 */
    volatile uint8_t restart_count;     /* 保存当前连续软故障自动重启次数。 */
    volatile uint8_t restart_exhausted; /* 标记自动重启次数已经耗尽。 */
    volatile uint8_t last_state;        /* 保存后台上一次观察到的控制状态。 */
    volatile BLDC_StopReason_t stop_reason; /* 保存最近一次停机或恢复原因。 */
    volatile BLDC_Fault_t last_fault;   /* 保存最近一次故障，成功重启后仍保留诊断信息。 */
    volatile uint32_t restart_due_ms;   /* 保存允许执行下一次自动重启的毫秒时刻。 */
    volatile uint32_t integrate_enter_ms; /* 保存最近一次进入积分闭环的毫秒时刻。 */
    volatile uint32_t start_count;      /* 保存累计成功发起启动流程的次数。 */
    volatile uint32_t stop_count;       /* 保存累计完成安全停机的次数。 */
} BLDC_RunManager_t;                    /* 运行管理器结构结束。 */

typedef struct /* 定义第十四步安全换向管理和粘性诊断数据。 */
{ /* 安全换向管理结构开始。 */
    volatile BLDC_ReversalState_t state; /* 保存空闲、减流或滑行等待状态。 */
    volatile BLDC_ControlMode_t pending_mode; /* 保存换向完成后需要恢复的控制模式。 */
    volatile int8_t requested_direction; /* 保存最近一次有符号命令请求的目标方向。 */
    volatile uint8_t quiet_confirm_ms;  /* 保存反电动势连续满足静止阈值的毫秒数。 */
    volatile int32_t pending_current_ma; /* 保存待恢复的正幅值电流命令。 */
    volatile uint16_t pending_duty_permille; /* 保存待恢复的正幅值占空比命令。 */
    volatile uint32_t pending_speed_erpm; /* 保存待恢复的正幅值速度命令。 */
    volatile uint32_t request_ms;       /* 保存本次收到反向命令的系统时刻。 */
    volatile uint32_t coast_enter_ms;   /* 保存本次关闭桥臂开始滑行的系统时刻。 */
    volatile uint32_t last_quiet_check_ms; /* 保存最近一次执行反电动势静止确认的时刻。 */
    volatile uint32_t completed_count;  /* 保存已经完成停机并重启的换向次数。 */
    volatile uint32_t canceled_count;   /* 保存被停止命令或故障取消的换向次数。 */
    volatile uint32_t timeout_count;    /* 保存达到最大滑行时间后继续重启的次数。 */
} BLDC_Reversal_t;                      /* 安全换向管理结构结束。 */

typedef struct /* 定义 VESC 风格方向、位置累计和启动粘性诊断数据。 */
{ /* 运动跟踪和启动诊断数据结构开始。 */
    volatile int8_t direction;          /* 保存当前方向，正一表示已验证的正向换相顺序。 */
    volatile int32_t signed_erpm;       /* 保存乘以方向符号后的滤波电角转速。 */
    volatile int32_t tachometer;        /* 保存带方向符号的累计换相步数。 */
    volatile uint32_t tachometer_abs;   /* 保存不区分方向的累计换相步数。 */
    volatile uint32_t startup_attempt;  /* 保存这些粘性诊断对应的累计启动序号。 */
    volatile uint32_t align_enter_ms;   /* 保存最近一次进入定位状态的系统毫秒时刻。 */
    volatile uint32_t open_loop_enter_ms; /* 保存最近一次进入开环状态的系统毫秒时刻。 */
    volatile uint32_t transition_enter_ms; /* 保存最近一次进入接管状态的系统毫秒时刻。 */
    volatile uint32_t integrate_enter_ms; /* 保存最近一次进入积分闭环的系统毫秒时刻。 */
    volatile uint32_t open_loop_commutations; /* 保存最近一次启动的开环强制换相次数。 */
    volatile uint32_t transition_bemf_commutations; /* 保存最近一次接管的积分成功换相次数。 */
    volatile uint32_t transition_forced_commutations; /* 保存最近一次接管的超时兜底换相次数。 */
} BLDC_Motion_t;                        /* 运动跟踪和启动诊断数据结构结束。 */

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
    volatile int32_t motor_current_ma;  /* 保存当前回流相电流幅值，继续供现有正向电流 PI 使用。 */
    volatile int32_t signed_motor_current_ma; /* 保存经过相极性归一化的回流相带符号电流。 */
    volatile int32_t signed_filtered_current_ma; /* 保存独立低通后的带符号电机电流诊断值。 */
    volatile int32_t source_current_ma; /* 保存当前源相经过极性归一化的带符号电流。 */
    volatile int32_t floating_current_ma; /* 保存当前悬空相经过极性归一化的带符号电流。 */
    volatile int32_t step_current_ma[6]; /* 保存六个换相扇区各自的滤波电流幅值。 */
    volatile int32_t step_signed_current_ma[6]; /* 保存六个换相扇区各自的滤波带符号电流。 */
    volatile int32_t signed_positive_peak_ma; /* 保存本次启动带符号电流的最大正峰值。 */
    volatile int32_t signed_negative_peak_ma; /* 保存本次启动带符号电流的最小负峰值。 */
    volatile uint32_t sum_u;            /* 保存 U 相零点校准累计值。 */
    volatile uint32_t sum_v;            /* 保存 V 相零点校准累计值。 */
    volatile uint32_t sum_w;            /* 保存 W 相零点校准累计值。 */
    volatile uint16_t count_adc1;       /* 保存 ADC1 注入序列校准采样数。 */
    volatile uint16_t count_adc2;       /* 保存 ADC2 注入序列校准采样数。 */
    volatile uint32_t paired_sample_count; /* 保存成功配对处理的三相电流样本数。 */
    volatile uint32_t sync_error_count; /* 保存某个 ADC 样本被覆盖前仍未配对的次数。 */
    volatile uint32_t blanked_sample_count; /* 保存换相消隐累计丢弃的电流控制样本数。 */
    volatile uint32_t polarity_positive_count; /* 保存显著正电流有效样本累计值。 */
    volatile uint32_t polarity_error_count; /* 保存正常驱动期间显著负电流样本累计值。 */
    volatile uint8_t calibrated;        /* 标记三相零点校准是否完成。 */
    volatile uint8_t overcurrent_count; /* 保存连续过流确认次数。 */
    volatile uint8_t adc1_ready;        /* 标记本 PWM 周期 ADC1 的 U/W 样本已经到达。 */
    volatile uint8_t adc2_ready;        /* 标记本 PWM 周期 ADC2 的 V 相样本已经到达。 */
    volatile uint8_t blanking_samples;  /* 保存换相后剩余的电流消隐样本数。 */
    volatile uint8_t zero_noise_ok;     /* 标记三相零点峰峰噪声均处于允许范围。 */
} BLDC_Current_t;                       /* 三相电流数据结构结束。 */

typedef struct /* 定义三相电流采样方向归一化系数。 */
{ /* 三相电流采样方向归一化结构开始。 */
    volatile int8_t u;                  /* 保存 U 相采样极性，数值只能为正一或负一。 */
    volatile int8_t v;                  /* 保存 V 相采样极性，数值只能为正一或负一。 */
    volatile int8_t w;                  /* 保存 W 相采样极性，数值只能为正一或负一。 */
} BLDC_CurrentPolarity_t;               /* 三相电流采样方向归一化结构结束。 */

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

typedef struct /* 定义 VESC 风格运行电流命令管理数据。 */
{ /* 运行电流命令管理结构开始。 */
    volatile int32_t requested_ma;      /* 保存用户最近一次请求的运行电流。 */
    volatile int32_t applied_ma;        /* 保存经过外层斜坡后送往电流 PI 的命令。 */
    volatile uint32_t rise_ma_per_second; /* 保存运行电流命令的上升斜率。 */
    volatile uint32_t fall_ma_per_second; /* 保存运行电流命令的下降斜率。 */
    volatile uint32_t last_update_ms;   /* 保存上一次执行命令斜坡的系统毫秒时间。 */
    volatile uint32_t ramp_remainder;   /* 保存整数斜坡除法尚未使用的千分余量。 */
    volatile int8_t ramp_direction;     /* 保存当前斜坡方向，一为上升，负一为下降。 */
} BLDC_CurrentCommand_t;                /* 运行电流命令管理结构结束。 */

typedef struct /* 定义 VESC 风格统一上层控制命令数据。 */
{ /* 统一上层控制命令结构开始。 */
    volatile BLDC_ControlMode_t mode;   /* 保存当前停止、电流、占空比或速度模式。 */
    volatile int32_t current_command_ma; /* 保存电流模式用户命令。 */
    volatile uint16_t duty_command_permille; /* 保存占空比模式千分比命令。 */
    volatile uint16_t duty_target_counts; /* 保存换算到一千二百分度的占空比目标。 */
    volatile uint32_t speed_command_erpm; /* 保存速度模式电角转速命令。 */
    volatile int32_t duty_error_counts; /* 保存占空比外环误差。 */
    volatile int32_t speed_error_erpm; /* 保存速度外环误差。 */
    volatile int32_t output_current_ma; /* 保存上层模式最终生成的电流命令。 */
    volatile int32_t duty_integral_accumulator; /* 保存占空比外环带缩放积分项。 */
    volatile int32_t speed_integral_accumulator; /* 保存速度外环带缩放积分项。 */
    volatile uint32_t last_update_ms;  /* 保存上层控制外环上一次执行时刻。 */
} BLDC_ControlCommand_t;                /* 统一上层控制命令结构结束。 */

typedef struct /* 定义 VESC 风格速度 PID 配置和运行诊断数据。 */
{ /* 速度 PID 数据结构开始。 */
    volatile uint32_t kp_x1000;         /* 保存比例增益千分之一缩放值。 */
    volatile uint32_t ki_x1000;         /* 保存积分增益千分之一缩放值。 */
    volatile uint32_t kd_x1000;         /* 保存微分增益千分之一缩放值。 */
    volatile uint16_t derivative_filter_permille; /* 保存微分低通滤波修正千分比。 */
    volatile int32_t proportional_ma;   /* 保存速度 PID 比例项输出毫安值。 */
    volatile int32_t integral_ma;       /* 保存速度 PID 积分项输出毫安值。 */
    volatile int32_t derivative_raw_ma; /* 保存速度 PID 未滤波微分项毫安值。 */
    volatile int32_t derivative_filtered_ma; /* 保存速度 PID 低通滤波后的微分项毫安值。 */
    volatile int32_t output_unclamped_ma; /* 保存速度 PID 限幅前电流输出。 */
    volatile int32_t output_limited_ma; /* 保存统一正向电流限制后的速度 PID 输出。 */
    volatile int32_t previous_error_erpm; /* 保存上一个控制周期的速度误差。 */
    volatile int32_t integral_remainder; /* 保存积分定点除法尚未使用的千分余量。 */
    volatile uint32_t elapsed_ms;       /* 保存最近一次速度 PID 实际使用的时间步长。 */
    volatile uint8_t initialized;       /* 标记速度 PID 微分历史是否已经建立。 */
} BLDC_SpeedPid_t;                      /* 速度 PID 数据结构结束。 */

typedef struct /* 定义 VESC 风格母线输入电流和功率限幅数据。 */
{ /* 母线输入限幅结构开始。 */
    volatile uint32_t current_limit_ma; /* 保存用户设置的最大母线输入电流。 */
    volatile uint32_t power_limit_mw;   /* 保存用户设置的最大母线输入功率。 */
    volatile uint32_t estimated_current_ma; /* 保存由电机电流和占空比估算的输入电流。 */
    volatile uint32_t estimated_power_mw; /* 保存由输入电流和母线电压估算的输入功率。 */
    volatile int32_t estimated_signed_current_ma; /* 保存带能量方向符号的估算母线电流。 */
    volatile int32_t estimated_signed_power_mw; /* 保存带能量方向符号的估算母线功率。 */
    volatile uint32_t power_current_limit_ma; /* 保存功率上限换算得到的输入电流上限。 */
    volatile uint32_t effective_current_limit_ma; /* 保存电流与功率限制中的较小输入电流上限。 */
    volatile int32_t limited_target_ma; /* 保存输入限制处理后的电机电流目标。 */
    volatile BLDC_InputLimitReason_t reason; /* 保存当前实际生效的输入限幅原因。 */
    volatile uint8_t active;            /* 标记输入限制当前是否正在降低目标。 */
    volatile uint8_t stop_requested;    /* 标记输入限制已因目标过低请求软停机。 */
} BLDC_InputLimit_t;                    /* 母线输入限幅结构结束。 */

typedef struct /* 定义 VESC 风格统一输出限幅和饱和诊断数据。 */
{ /* 统一输出限幅结构开始。 */
    volatile int32_t motor_current_limit_ma; /* 保存用户配置的最大正向电机电流。 */
    volatile uint16_t duty_min_counts; /* 保存电流 PI 允许的最小占空比计数。 */
    volatile uint16_t duty_max_counts; /* 保存电流 PI 允许的最大占空比计数。 */
    volatile uint32_t speed_min_erpm; /* 保存速度模式允许的最低电角转速。 */
    volatile uint32_t speed_max_erpm; /* 保存速度模式允许的最高电角转速。 */
    volatile int32_t requested_current_ma; /* 保存限幅前的电机电流请求。 */
    volatile int32_t limited_current_ma; /* 保存统一限幅后的电机电流请求。 */
    volatile uint16_t requested_duty_permille; /* 保存用户原始占空比千分比请求。 */
    volatile uint16_t limited_duty_permille; /* 保存配置限幅后的占空比千分比目标。 */
    volatile uint32_t requested_speed_erpm; /* 保存用户原始电角转速请求。 */
    volatile uint32_t limited_speed_erpm; /* 保存配置限幅后的电角转速目标。 */
    volatile BLDC_LimitReason_t command_reason; /* 保存命令进入控制器前发生的配置限幅原因。 */
    volatile BLDC_LimitReason_t active_reason; /* 保存当前全部生效限制的组合位标志。 */
    volatile uint8_t active; /* 标记当前是否存在任意统一输出限幅。 */
    volatile uint8_t current_saturated; /* 标记电机电流已经到达允许上限。 */
    volatile uint8_t duty_saturated; /* 标记电流 PI 已经到达占空比上限。 */
    volatile uint8_t speed_saturated; /* 标记速度命令已经到达配置上限。 */
} BLDC_OutputLimit_t;                   /* 统一输出限幅结构结束。 */

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

typedef struct /* 定义第十六步禁止回充和软过压保护诊断数据。 */
{ /* 禁止回充和软过压保护结构开始。 */
    volatile uint32_t regen_candidate_since_ms; /* 保存本次连续负母线电流的起始时刻。 */
    volatile uint32_t last_event_ms;   /* 保存最近一次回充或软过压事件时刻。 */
    volatile uint32_t regen_event_count; /* 保存累计确认的持续回充事件次数。 */
    volatile uint32_t soft_overvoltage_event_count; /* 保存累计进入软过压区域的次数。 */
    volatile uint32_t coast_stop_count; /* 保存保护触发立即滑行停机的累计次数。 */
    volatile uint32_t negative_torque_reject_count; /* 保存主动制动请求被拒绝的累计次数。 */
    volatile int32_t peak_regen_current_ma; /* 保存上电以来最小的负母线电流估算值。 */
    volatile int32_t peak_regen_power_mw; /* 保存上电以来最小的负母线功率估算值。 */
    volatile uint16_t soft_derate_permille; /* 保存软过压当前允许功率的千分比。 */
    volatile uint8_t regen_candidate_active; /* 标记负母线电流是否正在连续确认。 */
    volatile uint8_t regen_protection_active; /* 标记已经确认回充并执行保护。 */
    volatile uint8_t soft_overvoltage_active; /* 标记软过压降额或释放回差正在生效。 */
    volatile uint8_t soft_stop_requested; /* 标记母线已经达到强制滑行阈值。 */
    volatile uint8_t coast_stop_active; /* 标记最近一次停止由能量保护触发。 */
    volatile uint8_t restart_inhibited; /* 标记母线恢复前禁止新的运行命令。 */
} BLDC_EnergyProtection_t;              /* 禁止回充和软过压保护结构结束。 */

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
static void BLDC_CommutateSteps(BLDC_CommReason_t reason, uint8_t steps); /* 声明带触发原因的指定步数换相函数。 */
static void BLDC_ResetSpeedEstimator(void); /* 声明多步转速估算器复位函数。 */
static void BLDC_UpdateSpeedEstimator(uint32_t elapsed_ticks, uint8_t steps); /* 声明多步转速估算器更新函数。 */
static void BLDC_UpdateMotionTracking(BLDC_CommReason_t reason, uint8_t steps); /* 声明方向位置和启动换相诊断更新函数。 */
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
static void BLDC_ProcessCurrentCommand(void); /* 声明毫秒级运行电流命令斜坡函数。 */
static void BLDC_UpdateInputLimits(void); /* 声明 VESC 风格输入电流和功率限幅函数。 */
static void BLDC_ProcessEnergyProtection(void); /* 声明禁止回充和软过压保护状态处理函数。 */
static void BLDC_ForceEnergyCoastStop(BLDC_StopReason_t reason); /* 声明能量保护立即关闭桥臂函数。 */
static void BLDC_ProcessControlCommand(void); /* 声明统一上层控制模式处理函数。 */
static int32_t BLDC_GetControlCurrentMaximumMa(void); /* 声明上层控制允许电流上限计算函数。 */
static int32_t BLDC_GetControlCurrentMinimumMa(void); /* 声明当前控制模式最低有效电流计算函数。 */
static void BLDC_ResetSpeedPidRuntime(int32_t seed_current_ma); /* 声明速度 PID 运行状态和积分预置函数。 */
static void BLDC_PreloadControlIntegrators(void); /* 声明模式切换积分预置函数。 */
static void BLDC_EnterFault(BLDC_Fault_t fault); /* 声明带故障码的停机函数。 */
static uint8_t BLDC_IsPowerState(void); /* 声明功率输出状态判断函数。 */
static uint8_t BLDC_IsRecoverableFault(BLDC_Fault_t fault); /* 声明软故障类型判断函数。 */
static uint8_t BLDC_IsBusVoltageReady(void); /* 声明启动母线电压许可判断函数。 */
static void BLDC_BeginStartup(void);    /* 声明定位和开环启动准备函数。 */
static void BLDC_CompleteStop(BLDC_StopReason_t reason); /* 声明安全停机完成函数。 */
static void BLDC_ProcessManager(void);  /* 声明启停和自动恢复后台管理函数。 */
static uint8_t BLDC_RequestSignedCommand(BLDC_ControlMode_t mode, int8_t direction, int32_t current_ma, uint16_t duty_permille, uint32_t speed_erpm); /* 声明有符号命令方向仲裁函数。 */
static void BLDC_ProcessReversal(uint32_t now_ms); /* 声明安全换向减流、滑行和重启状态机。 */
static void BLDC_CancelReversal(void);  /* 声明安全换向取消和待执行命令清理函数。 */
static uint16_t BLDC_GetBemfSpread(void); /* 声明三相反电动势峰峰值计算函数。 */
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
static BLDC_CurrentPolarity_t g_current_polarity = /* 创建三相电流方向归一化配置实例。 */
{ /* 三相电流方向归一化配置初始化开始。 */
    .u = 1,                             /* 默认保持 U 相 ADC 相对零点差值方向不变。 */
    .v = 1,                             /* 默认保持 V 相 ADC 相对零点差值方向不变。 */
    .w = 1                              /* 默认保持 W 相 ADC 相对零点差值方向不变。 */
}; /* 三相电流方向归一化配置初始化结束。 */
static BLDC_BusVoltage_t g_bus_voltage; /* 保存母线电压采样、滤波和保护数据。 */
static BLDC_EnergyProtection_t g_energy_protection = /* 创建禁止回充和软过压保护实例。 */
{ /* 禁止回充和软过压保护初始化开始。 */
    .soft_derate_permille = BLDC_PERMILLE_SCALE /* 上电默认允许百分之一百的正常输入功率。 */
}; /* 禁止回充和软过压保护初始化结束。 */
static BLDC_RunManager_t g_run_manager; /* 保存启停命令、恢复计数和停机诊断信息。 */
static BLDC_Reversal_t g_reversal =     /* 创建安全换向管理和粘性诊断实例。 */
{ /* 安全换向管理初始化开始。 */
    .state = BLDC_REVERSAL_IDLE,        /* 上电默认没有正在执行的换向。 */
    .pending_mode = BLDC_CONTROL_STOP,  /* 上电默认没有等待恢复的控制模式。 */
    .requested_direction = 1            /* 上电默认目标方向为正转。 */
}; /* 安全换向管理初始化结束。 */
static BLDC_Motion_t g_motion =         /* 创建方向、位置累计和启动粘性诊断实例。 */
{ /* 运动跟踪和启动诊断初始化开始。 */
    .direction = 1,                    /* 上电默认选择已经完成实机验证的正向换相方向。 */
    .signed_erpm = 0L,                 /* 上电时有符号电角转速为零。 */
    .tachometer = 0L,                  /* 上电时有符号累计换相步数为零。 */
    .tachometer_abs = 0U               /* 上电时绝对累计换相步数为零。 */
}; /* 运动跟踪和启动诊断初始化结束。 */
static BLDC_CurrentControl_t g_current_control = /* 创建电流 PI 控制器实例。 */
{ /* 电流 PI 控制器初始化开始。 */
    .target_ma = 0L,                   /* 上电默认不向电流 PI 请求输出。 */
    .output_duty = BLDC_TEST_DUTY,      /* 设置首次测试初始占空比。 */
    .overcurrent_limit_ma = BLDC_OVERCURRENT_LIMIT_DEFAULT_MA /* 设置默认软件过流保护阈值。 */
}; /* 电流 PI 控制器初始化结束。 */
static BLDC_CurrentCommand_t g_current_command = /* 创建运行电流命令管理实例。 */
{ /* 运行电流命令管理初始化开始。 */
    .requested_ma = 0L,                /* 上电默认没有运行电流请求。 */
    .applied_ma = 0L,                  /* 上电默认送往电流 PI 的命令为零。 */
    .rise_ma_per_second = BLDC_CURRENT_RISE_DEFAULT_MA_PER_SECOND, /* 设置默认上升斜率。 */
    .fall_ma_per_second = BLDC_CURRENT_FALL_DEFAULT_MA_PER_SECOND, /* 设置默认下降斜率。 */
    .ramp_direction = 0                /* 上电默认没有正在执行的斜坡方向。 */
}; /* 运行电流命令管理初始化结束。 */
static BLDC_ControlCommand_t g_control_command = /* 创建统一上层控制命令实例。 */
{ /* 统一上层控制命令初始化开始。 */
    .mode = BLDC_CONTROL_STOP,         /* 上电默认保持停止模式。 */
    .current_command_ma = 0L,          /* 上电默认电流模式命令为零。 */
    .duty_command_permille = 0U,       /* 上电默认占空比模式命令为零。 */
    .duty_target_counts = 0U,          /* 上电默认内部占空比目标为零。 */
    .speed_command_erpm = 0U,          /* 上电默认速度模式命令为零。 */
    .output_current_ma = 0L            /* 上电默认上层输出电流为零。 */
}; /* 统一上层控制命令初始化结束。 */
static BLDC_SpeedPid_t g_speed_pid =   /* 创建速度 PID 配置和运行诊断实例。 */
{ /* 速度 PID 初始化开始。 */
    .kp_x1000 = BLDC_SPEED_PID_KP_DEFAULT_X1000, /* 设置默认比例增益。 */
    .ki_x1000 = BLDC_SPEED_PID_KI_DEFAULT_X1000, /* 设置默认积分增益。 */
    .kd_x1000 = BLDC_SPEED_PID_KD_DEFAULT_X1000, /* 默认关闭尚未实机调参的微分增益。 */
    .derivative_filter_permille = BLDC_SPEED_PID_D_FILTER_DEFAULT_PERMILLE, /* 设置默认微分滤波强度。 */
    .initialized = 0U                  /* 上电要求首个速度样本建立微分历史。 */
}; /* 速度 PID 初始化结束。 */
static BLDC_InputLimit_t g_input_limit = /* 创建母线输入电流和功率限幅实例。 */
{ /* 母线输入限幅初始化开始。 */
    .current_limit_ma = BLDC_INPUT_CURRENT_LIMIT_DEFAULT_MA, /* 设置默认六百毫安输入电流上限。 */
    .power_limit_mw = BLDC_INPUT_POWER_LIMIT_DEFAULT_MW, /* 设置默认十五瓦输入功率上限。 */
    .effective_current_limit_ma = BLDC_INPUT_CURRENT_LIMIT_DEFAULT_MA, /* 设置初始有效输入电流上限。 */
    .limited_target_ma = 0L,           /* 上电默认不向电流 PI 请求输出。 */
    .reason = BLDC_INPUT_LIMIT_NONE,   /* 上电默认没有输入限幅原因。 */
    .active = 0U,                      /* 上电默认输入限幅没有生效。 */
    .stop_requested = 0U               /* 上电默认没有限幅软停机请求。 */
}; /* 母线输入限幅初始化结束。 */
static BLDC_OutputLimit_t g_output_limit = /* 创建统一输出限幅和饱和诊断实例。 */
{ /* 统一输出限幅初始化开始。 */
    .motor_current_limit_ma = BLDC_MOTOR_CURRENT_LIMIT_DEFAULT_MA, /* 设置默认最大电机电流为一点一安。 */
    .duty_min_counts = BLDC_DUTY_LIMIT_MIN_DEFAULT_COUNTS, /* 保持原电流 PI 最小占空比。 */
    .duty_max_counts = BLDC_DUTY_LIMIT_MAX_DEFAULT_COUNTS, /* 保持原电流 PI 百分之五十最大占空比。 */
    .speed_min_erpm = BLDC_SPEED_LIMIT_MIN_DEFAULT_ERPM, /* 设置默认最低无感速度命令。 */
    .speed_max_erpm = BLDC_SPEED_LIMIT_MAX_DEFAULT_ERPM, /* 设置实机验证后的默认最高速度。 */
    .active_reason = BLDC_LIMIT_NONE, /* 上电默认没有统一输出限幅。 */
    .command_reason = BLDC_LIMIT_NONE, /* 上电默认没有命令配置限幅。 */
    .active = 0U, /* 上电默认统一输出限幅未生效。 */
    .current_saturated = 0U, /* 上电默认电机电流没有饱和。 */
    .duty_saturated = 0U, /* 上电默认 PWM 占空比没有饱和。 */
    .speed_saturated = 0U /* 上电默认速度命令没有饱和。 */
}; /* 统一输出限幅初始化结束。 */
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
    BLDC_ResetSpeedEstimator();         /* 清空上一次运行留下的六步转速估算数据。 */
    g_motion.direction = 1;             /* 初始化时默认选择正向，随后允许在功率关闭状态切换方向。 */
    g_motion.signed_erpm = 0L;          /* 清零有符号滤波电角转速。 */
    g_motion.tachometer = 0L;           /* 清零有符号累计换相步数。 */
    g_motion.tachometer_abs = 0U;       /* 清零绝对累计换相步数。 */
    g_motion.startup_attempt = 0U;      /* 清零启动粘性诊断对应的启动序号。 */
    g_motion.align_enter_ms = 0U;       /* 清零最近一次定位进入时刻。 */
    g_motion.open_loop_enter_ms = 0U;   /* 清零最近一次开环进入时刻。 */
    g_motion.transition_enter_ms = 0U;  /* 清零最近一次接管进入时刻。 */
    g_motion.integrate_enter_ms = 0U;   /* 清零最近一次积分闭环进入时刻。 */
    g_motion.open_loop_commutations = 0U; /* 清零最近一次开环强制换相次数。 */
    g_motion.transition_bemf_commutations = 0U; /* 清零最近一次接管积分换相次数。 */
    g_motion.transition_forced_commutations = 0U; /* 清零最近一次接管超时换相次数。 */
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
    g_bldc.two_step_recovery_count = 0U; /* 清零上一次运行的跨两步恢复累计次数。 */
    g_run_manager.initialized = 0U;     /* 初始化过程中暂不接受启动流程。 */
    g_run_manager.run_command = 0U;     /* 上电初始化默认保持停止命令。 */
    g_run_manager.stop_pending = 0U;    /* 清除上一次软停机等待标志。 */
    g_run_manager.restart_count = 0U;   /* 清零连续自动重启次数。 */
    g_run_manager.restart_exhausted = 0U; /* 清除自动重启耗尽标志。 */
    g_run_manager.last_state = (uint8_t)BLDC_STATE_STOP; /* 设置后台初始观察状态。 */
    g_run_manager.stop_reason = BLDC_STOP_REASON_NONE; /* 清除上一次停机原因。 */
    g_run_manager.last_fault = BLDC_FAULT_NONE; /* 清除上一次故障诊断。 */
    g_run_manager.restart_due_ms = 0U;  /* 清零自动重启允许时刻。 */
    g_run_manager.integrate_enter_ms = 0U; /* 清零闭环稳定计时起点。 */
    g_run_manager.start_count = 0U;     /* 清零累计启动次数。 */
    g_run_manager.stop_count = 0U;      /* 清零累计停机次数。 */
    g_reversal.state = BLDC_REVERSAL_IDLE; /* 初始化时清除可能遗留的换向流程。 */
    g_reversal.pending_mode = BLDC_CONTROL_STOP; /* 初始化时清除等待恢复的控制模式。 */
    g_reversal.requested_direction = 1; /* 初始化时默认有符号命令请求正转。 */
    g_reversal.quiet_confirm_ms = 0U;   /* 清零反电动势静止连续确认时间。 */
    g_reversal.pending_current_ma = 0L; /* 清零等待恢复的电流命令。 */
    g_reversal.pending_duty_permille = 0U; /* 清零等待恢复的占空比命令。 */
    g_reversal.pending_speed_erpm = 0U; /* 清零等待恢复的速度命令。 */
    g_reversal.request_ms = 0U;         /* 清零最近一次换向请求时刻。 */
    g_reversal.coast_enter_ms = 0U;     /* 清零最近一次滑行开始时刻。 */
    g_reversal.last_quiet_check_ms = 0U; /* 清零最近一次静止判定时刻。 */
    g_reversal.completed_count = 0U;    /* 清零已经完成的换向次数。 */
    g_reversal.canceled_count = 0U;     /* 清零已经取消的换向次数。 */
    g_reversal.timeout_count = 0U;      /* 清零滑行等待超时次数。 */
    g_current_command.requested_ma = 0L; /* 初始化时清除可能遗留的运行电流请求。 */
    g_current_command.applied_ma = 0L;  /* 初始化时将外层斜坡命令恢复为零。 */
    g_current_command.last_update_ms = HAL_GetTick(); /* 从当前系统时刻开始计算命令斜坡。 */
    g_current_command.ramp_remainder = 0U; /* 清除上一次斜坡留下的整数余量。 */
    g_current_command.ramp_direction = 0; /* 清除上一次斜坡方向。 */
    g_current_control.target_ma = 0L;   /* 初始化时禁止旧目标值意外请求电流。 */
    g_control_command.mode = BLDC_CONTROL_STOP; /* 初始化时强制恢复停止控制模式。 */
    g_control_command.current_command_ma = 0L; /* 清除上一次电流模式命令。 */
    g_control_command.duty_command_permille = 0U; /* 清除上一次占空比模式命令。 */
    g_control_command.duty_target_counts = 0U; /* 清除上一次内部占空比目标。 */
    g_control_command.speed_command_erpm = 0U; /* 清除上一次速度模式命令。 */
    g_control_command.duty_error_counts = 0L; /* 清除上一次占空比外环误差。 */
    g_control_command.speed_error_erpm = 0L; /* 清除上一次速度外环误差。 */
    g_control_command.output_current_ma = 0L; /* 清除上一次上层输出电流。 */
    g_control_command.duty_integral_accumulator = 0L; /* 清除上一次占空比积分项。 */
    g_control_command.speed_integral_accumulator = 0L; /* 清除上一次速度积分项。 */
    BLDC_ResetSpeedPidRuntime(0L);      /* 清除速度 PID 比例积分微分历史和诊断值。 */
    g_control_command.last_update_ms = HAL_GetTick(); /* 从当前时刻开始执行上层外环。 */
    g_input_limit.estimated_current_ma = 0U; /* 清除上一次运行的估算输入电流。 */
    g_input_limit.estimated_power_mw = 0U; /* 清除上一次运行的估算输入功率。 */
    g_input_limit.estimated_signed_current_ma = 0L; /* 清除上一次运行的带符号估算输入电流。 */
    g_input_limit.estimated_signed_power_mw = 0L; /* 清除上一次运行的带符号估算输入功率。 */
    g_input_limit.power_current_limit_ma = g_input_limit.current_limit_ma; /* 初始化功率等效电流上限。 */
    g_input_limit.effective_current_limit_ma = g_input_limit.current_limit_ma; /* 初始化实际输入电流上限。 */
    g_input_limit.limited_target_ma = 0L; /* 清除上一次运行的限幅后电流目标。 */
    g_input_limit.reason = BLDC_INPUT_LIMIT_NONE; /* 清除上一次运行的输入限幅原因。 */
    g_input_limit.active = 0U;         /* 清除上一次运行的输入限幅状态。 */
    g_input_limit.stop_requested = 0U; /* 清除上一次运行的限幅软停机状态。 */
    g_energy_protection.regen_candidate_since_ms = 0U; /* 清除上一次连续回充候选起点。 */
    g_energy_protection.last_event_ms = 0U; /* 清除最近一次能量保护事件时刻。 */
    g_energy_protection.regen_event_count = 0U; /* 清零累计持续回充事件次数。 */
    g_energy_protection.soft_overvoltage_event_count = 0U; /* 清零累计软过压事件次数。 */
    g_energy_protection.coast_stop_count = 0U; /* 清零累计能量保护滑行停机次数。 */
    g_energy_protection.negative_torque_reject_count = 0U; /* 清零主动制动请求拒绝次数。 */
    g_energy_protection.peak_regen_current_ma = 0L; /* 清零上电以来负母线电流峰值。 */
    g_energy_protection.peak_regen_power_mw = 0L; /* 清零上电以来负母线功率峰值。 */
    g_energy_protection.soft_derate_permille = BLDC_PERMILLE_SCALE; /* 初始化为百分之一百允许功率。 */
    g_energy_protection.regen_candidate_active = 0U; /* 清除连续回充确认状态。 */
    g_energy_protection.regen_protection_active = 0U; /* 清除已经确认的回充保护状态。 */
    g_energy_protection.soft_overvoltage_active = 0U; /* 清除软过压状态。 */
    g_energy_protection.soft_stop_requested = 0U; /* 清除软过压强制停机请求。 */
    g_energy_protection.coast_stop_active = 0U; /* 清除能量保护滑行停机状态。 */
    g_energy_protection.restart_inhibited = 0U; /* 初始化时允许安全母线电压下启动。 */
    g_output_limit.requested_current_ma = 0L; /* 清除上一次统一管理器的原始电流请求。 */
    g_output_limit.limited_current_ma = 0L; /* 清除上一次统一管理器的限幅电流。 */
    g_output_limit.requested_duty_permille = 0U; /* 清除上一次原始占空比请求。 */
    g_output_limit.limited_duty_permille = 0U; /* 清除上一次限幅后占空比目标。 */
    g_output_limit.requested_speed_erpm = 0U; /* 清除上一次原始速度请求。 */
    g_output_limit.limited_speed_erpm = 0U; /* 清除上一次限幅后速度目标。 */
    g_output_limit.command_reason = BLDC_LIMIT_NONE; /* 清除上一次命令配置限幅原因。 */
    g_output_limit.active_reason = BLDC_LIMIT_NONE; /* 清除上一次运行饱和原因组合。 */
    g_output_limit.active = 0U;         /* 清除上一次统一限幅活动标志。 */
    g_output_limit.current_saturated = 0U; /* 清除上一次电流饱和标志。 */
    g_output_limit.duty_saturated = 0U; /* 清除上一次占空比饱和标志。 */
    g_output_limit.speed_saturated = 0U; /* 清除上一次速度饱和标志。 */
    BLDC_HardwarePrepare();             /* 准备 ADC、DMA 和 TIM1。 */
    if (BLDC_SampleBusVoltage() == 0U)  /* 执行首次母线电压采样并检查 ADC 状态。 */
    { /* 首次母线电压采样失败处理开始。 */
        BLDC_EnterFault(BLDC_FAULT_VBUS_ADC); /* ADC1 规则组失败时禁止电机启动。 */
    } /* 首次母线电压采样失败处理结束。 */
    else                                /* 处理首次母线电压采样成功。 */
    { /* 首次母线电压记录完成处理开始。 */
        __NOP();                        /* 空闲初始化阶段只记录电压，收到启动命令后再判断许可。 */
    } /* 首次母线电压记录完成处理结束。 */
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
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1); /* 校准期间无条件保持高级定时器主输出关闭。 */
    g_run_manager.initialized = 1U;     /* 标记快速采样和后台管理已经可以运行。 */
} /* 测试入口结束。 */

void BLDC_Process(void)                /* 实现非阻塞后台服务入口。 */
{ /* 非阻塞后台服务开始。 */
    if (g_run_manager.initialized == 0U) /* 检查底层初始化是否已经完成。 */
    { /* 尚未初始化处理开始。 */
        return;                         /* 初始化完成前不执行启停管理。 */
    } /* 尚未初始化处理结束。 */
    BLDC_ServiceSlow();                 /* 每十毫秒采样母线电压并更新动态积分阈值。 */
    BLDC_ProcessControlCommand();       /* 按当前电流、占空比或速度模式生成电流命令。 */
    BLDC_ProcessCurrentCommand();       /* 按用户配置的上升和下降斜率更新电流命令。 */
    BLDC_ProcessEnergyProtection();     /* 检查持续回充和软过压并在需要时立即滑行停机。 */
    BLDC_ProcessManager();              /* 执行运行命令、软停机和自动恢复管理。 */
} /* 非阻塞后台服务结束。 */

static int32_t BLDC_GetControlCurrentMaximumMa(void) /* 实现上层控制允许电流上限计算。 */
{ /* 上层控制允许电流上限计算开始。 */
    int32_t maximum_current_ma;         /* 保存考虑软件过流余量后的最大电流。 */
    maximum_current_ma = g_current_control.overcurrent_limit_ma - BLDC_CONTROL_CURRENT_MARGIN_MA; /* 从软件过流值减去安全余量。 */
    if (maximum_current_ma < BLDC_CLOSED_LOOP_CURRENT_MIN_MA) /* 检查计算结果是否低于闭环外环最低电流。 */
    { /* 上层电流上限过低处理开始。 */
        maximum_current_ma = BLDC_CLOSED_LOOP_CURRENT_MIN_MA; /* 至少保留一百毫安闭环外环控制范围。 */
    } /* 上层电流上限过低处理结束。 */
    if (maximum_current_ma > BLDC_CURRENT_TARGET_MAX_MA) /* 检查计算结果是否超过电流 PI 软件上限。 */
    { /* 上层电流上限过高处理开始。 */
        maximum_current_ma = BLDC_CURRENT_TARGET_MAX_MA; /* 限制到电流 PI 最大允许目标。 */
    } /* 上层电流上限过高处理结束。 */
    if (maximum_current_ma > g_output_limit.motor_current_limit_ma) /* 检查用户配置的电机电流上限是否更严格。 */
    { /* 统一电机电流上限处理开始。 */
        maximum_current_ma = g_output_limit.motor_current_limit_ma; /* 选择统一管理器配置的较小电机电流上限。 */
    } /* 统一电机电流上限处理结束。 */
    return maximum_current_ma;          /* 返回当前上层模式允许的最大电流。 */
} /* 上层控制允许电流上限计算结束。 */

static int32_t BLDC_GetControlCurrentMinimumMa(void) /* 实现当前控制模式最低有效电流计算。 */
{ /* 当前控制模式最低有效电流计算开始。 */
    if (g_control_command.mode == BLDC_CONTROL_CURRENT) /* 检查当前是否为直接电流模式。 */
    { /* 直接电流模式最低值处理开始。 */
        return BLDC_CURRENT_COMMAND_RELEASE_MA; /* 电流模式只应用用户命令释放阈值。 */
    } /* 直接电流模式最低值处理结束。 */
    if ((g_control_command.mode == BLDC_CONTROL_DUTY) || /* 检查当前是否为占空比外环模式。 */
        (g_control_command.mode == BLDC_CONTROL_SPEED)) /* 检查当前是否为速度外环模式。 */
    { /* 外环模式最低值处理开始。 */
        return BLDC_CLOSED_LOOP_CURRENT_MIN_MA; /* 外环保留一百毫安最低闭环控制能力。 */
    } /* 外环模式最低值处理结束。 */
    return 0L;                          /* 停止模式不要求任何最低电流。 */
} /* 当前控制模式最低有效电流计算结束。 */

static void BLDC_ResetSpeedPidRuntime(int32_t seed_current_ma) /* 实现速度 PID 运行状态复位和积分预置。 */
{ /* 速度 PID 运行状态复位开始。 */
    if (seed_current_ma < 0L)           /* 检查积分预置是否请求了当前未支持的负电流。 */
    { /* 负积分预置处理开始。 */
        seed_current_ma = 0L;           /* 正向非同步整流模式最低只允许预置到零。 */
    } /* 负积分预置处理结束。 */
    if (seed_current_ma > BLDC_GetControlCurrentMaximumMa()) /* 检查积分预置是否超过统一电流上限。 */
    { /* 积分预置上限处理开始。 */
        seed_current_ma = BLDC_GetControlCurrentMaximumMa(); /* 限制积分预置到当前可实现最大电流。 */
    } /* 积分预置上限处理结束。 */
    g_control_command.speed_integral_accumulator = seed_current_ma * BLDC_SPEED_PID_SCALE; /* 用当前电流建立千分之一毫安积分内部状态。 */
    g_speed_pid.proportional_ma = 0L;   /* 清零上一次速度比例项诊断值。 */
    g_speed_pid.integral_ma = seed_current_ma; /* 同步保存当前速度积分项毫安值。 */
    g_speed_pid.derivative_raw_ma = 0L; /* 清零上一次未滤波微分项。 */
    g_speed_pid.derivative_filtered_ma = 0L; /* 清零上一次滤波微分项。 */
    g_speed_pid.output_unclamped_ma = seed_current_ma; /* 将限幅前输出同步到积分预置值。 */
    g_speed_pid.output_limited_ma = seed_current_ma; /* 将限幅后输出同步到积分预置值。 */
    g_speed_pid.previous_error_erpm = 0L; /* 清零旧速度误差历史。 */
    g_speed_pid.integral_remainder = 0L; /* 清零旧积分定点余量。 */
    g_speed_pid.elapsed_ms = 0U;        /* 清零尚未执行的速度 PID 时间步长。 */
    g_speed_pid.initialized = 0U;       /* 要求下一个有效速度周期重新建立微分历史。 */
} /* 速度 PID 运行状态复位结束。 */

static void BLDC_PreloadControlIntegrators(void) /* 实现模式切换外环积分预置。 */
{ /* 模式切换外环积分预置开始。 */
    int32_t seed_current_ma;            /* 保存用于无扰切换的当前电流种子。 */
    seed_current_ma = g_current_command.applied_ma; /* 优先使用当前经过斜坡的运行电流。 */
    if (seed_current_ma < BLDC_CLOSED_LOOP_CURRENT_MIN_MA) /* 检查当前电流是否低于外环闭环下限。 */
    { /* 模式切换电流种子下限处理开始。 */
        seed_current_ma = BLDC_CLOSED_LOOP_CURRENT_MIN_MA; /* 停止或低电流阶段使用一百毫安作为外环初值。 */
    } /* 模式切换电流种子下限处理结束。 */
    if (seed_current_ma > BLDC_GetControlCurrentMaximumMa()) /* 检查电流种子是否超过上层允许上限。 */
    { /* 模式切换电流种子上限处理开始。 */
        seed_current_ma = BLDC_GetControlCurrentMaximumMa(); /* 限制积分预置到当前允许上限。 */
    } /* 模式切换电流种子上限处理结束。 */
    g_control_command.duty_integral_accumulator = seed_current_ma * BLDC_DUTY_LOOP_KI_DIVISOR; /* 用当前电流预置占空比积分项。 */
    BLDC_ResetSpeedPidRuntime(seed_current_ma); /* 使用当前电流无扰预置速度 PID 并清除旧微分历史。 */
    g_control_command.output_current_ma = seed_current_ma; /* 同步上层输出诊断值。 */
} /* 模式切换外环积分预置结束。 */

static void BLDC_ProcessControlCommand(void) /* 实现统一上层控制模式处理。 */
{ /* 统一上层控制模式处理开始。 */
    uint32_t now_ms;                    /* 保存当前系统毫秒时刻。 */
    uint32_t elapsed_ms;                /* 保存本次外层控制实际经历的毫秒数。 */
    int32_t maximum_current_ma;         /* 保存当前允许的最大电机电流。 */
    int32_t minimum_current_ma;         /* 保存当前控制模式允许的最低电机电流。 */
    int32_t output_current_ma;          /* 保存本次上层模式生成的电流目标。 */
    int32_t proportional_current_ma;    /* 保存占空比或速度外环比例电流。 */
    int32_t integral_minimum;           /* 保存当前外环积分项下限。 */
    int32_t integral_maximum;           /* 保存当前外环积分项上限。 */
    int32_t tracking_current_ma;        /* 保存下游饱和时外环积分器跟踪的可实现电流。 */
    int32_t measured_tracking_ma;       /* 保存根据实际电流得到的 PWM 饱和跟踪值。 */
    BLDC_LimitReason_t active_reason;   /* 保存本周期组合得到的全部限幅原因。 */
    uint8_t downstream_upper_saturated; /* 标记输入限制或 PWM 上限正在阻止继续增加输出。 */
    now_ms = HAL_GetTick();             /* 读取本次上层控制处理时刻。 */
    elapsed_ms = now_ms - g_control_command.last_update_ms; /* 计算无符号回绕安全的实际控制间隔。 */
    if (elapsed_ms < BLDC_CONTROL_PERIOD_MS) /* 检查一毫秒外环周期是否到达。 */
    { /* 上层外环周期未到处理开始。 */
        return;                         /* 保持上一周期生成的电流命令。 */
    } /* 上层外环周期未到处理结束。 */
    g_control_command.last_update_ms = now_ms; /* 记录本次已经执行上层控制。 */
    if (elapsed_ms > BLDC_SPEED_PID_MAX_ELAPSED_MS) /* 检查调试暂停或后台迟滞是否造成过大时间步长。 */
    { /* 速度 PID 时间步长上限处理开始。 */
        elapsed_ms = BLDC_SPEED_PID_MAX_ELAPSED_MS; /* 限制单次积分变化，避免恢复运行时电流突跳。 */
    } /* 速度 PID 时间步长上限处理结束。 */
    maximum_current_ma = BLDC_GetControlCurrentMaximumMa(); /* 获取考虑过流余量的电流上限。 */
    minimum_current_ma = BLDC_GetControlCurrentMinimumMa(); /* 获取当前模式对应的最低有效电流。 */
    active_reason = g_output_limit.command_reason; /* 首先保留命令进入控制器前发生的配置限幅。 */
    tracking_current_ma = maximum_current_ma; /* 默认让外环积分器跟踪最大允许电机电流。 */
    downstream_upper_saturated = 0U;    /* 默认下游没有阻止外环继续增加电流。 */
    g_output_limit.current_saturated = 0U; /* 默认本周期电机电流没有达到上限。 */
    g_output_limit.duty_saturated = 0U; /* 默认本周期 PWM 占空比没有达到上限。 */
    g_output_limit.speed_saturated = 0U; /* 默认本周期速度命令没有达到上限。 */
    if (g_input_limit.active != 0U)     /* 检查上一周期输入限幅是否正在降低电流目标。 */
    { /* 输入限制统一原因映射开始。 */
        downstream_upper_saturated = 1U; /* 输入限制存在时禁止外环继续正向积分。 */
        if ((g_input_limit.reason & BLDC_INPUT_LIMIT_CURRENT) != 0U) /* 检查是否包含母线输入电流限制。 */
        { /* 母线输入电流原因映射开始。 */
            active_reason = (BLDC_LimitReason_t)((uint32_t)active_reason | (uint32_t)BLDC_LIMIT_INPUT_CURRENT); /* 加入输入电流限制位。 */
        } /* 母线输入电流原因映射结束。 */
        if ((g_input_limit.reason & BLDC_INPUT_LIMIT_POWER) != 0U) /* 检查是否包含母线输入功率限制。 */
        { /* 母线输入功率原因映射开始。 */
            active_reason = (BLDC_LimitReason_t)((uint32_t)active_reason | (uint32_t)BLDC_LIMIT_INPUT_POWER); /* 加入输入功率限制位。 */
        } /* 母线输入功率原因映射结束。 */
        if ((g_input_limit.reason & BLDC_INPUT_LIMIT_SOFT_OVERVOLTAGE) != 0U) /* 检查是否包含母线软过压降额。 */
        { /* 母线软过压原因映射开始。 */
            active_reason = (BLDC_LimitReason_t)((uint32_t)active_reason | (uint32_t)BLDC_LIMIT_SOFT_OVERVOLTAGE); /* 加入软过压降额限制位。 */
        } /* 母线软过压原因映射结束。 */
        if ((g_input_limit.limited_target_ma >= minimum_current_ma) && /* 检查输入限制值仍允许当前模式运行。 */
            (g_input_limit.limited_target_ma < tracking_current_ma)) /* 检查输入限制是否比当前跟踪值更严格。 */
        { /* 输入限制跟踪电流选择开始。 */
            tracking_current_ma = g_input_limit.limited_target_ma; /* 让外环积分器跟踪输入限制真正允许的电机电流。 */
        } /* 输入限制跟踪电流选择结束。 */
    } /* 输入限制统一原因映射结束。 */
    if ((g_current_control.output_duty >= (int32_t)g_output_limit.duty_max_counts) && /* 检查电流 PI 是否已经到达配置 PWM 上限。 */
        (g_current_control.error_ma > 0L) && /* 检查电流环是否仍然要求继续增加 PWM。 */
        (g_bldc.state == BLDC_STATE_INTEGRATE)) /* 只在积分闭环稳定运行时判定 PWM 饱和。 */
    { /* PWM 上限饱和处理开始。 */
        active_reason = (BLDC_LimitReason_t)((uint32_t)active_reason | (uint32_t)BLDC_LIMIT_DUTY); /* 加入 PWM 占空比限制位。 */
        g_output_limit.duty_saturated = 1U; /* 标记电流 PI 已经受到 PWM 上限限制。 */
        downstream_upper_saturated = 1U; /* 禁止速度或占空比外环继续正向积累。 */
        measured_tracking_ma = g_current_control.filtered_current_ma; /* 使用当前实际滤波电流作为可实现能力参考。 */
        if (measured_tracking_ma < 0L)  /* 检查滤波电流是否因采样噪声成为负值。 */
        { /* 负跟踪电流处理开始。 */
            measured_tracking_ma = -measured_tracking_ma; /* 统一使用正向电机电流绝对值。 */
        } /* 负跟踪电流处理结束。 */
        if (measured_tracking_ma < minimum_current_ma) /* 检查跟踪值是否低于当前模式最低电流。 */
        { /* PWM 饱和跟踪下限处理开始。 */
            measured_tracking_ma = minimum_current_ma; /* 至少保持当前模式允许的一百毫安控制下限。 */
        } /* PWM 饱和跟踪下限处理结束。 */
        if (measured_tracking_ma > maximum_current_ma) /* 检查跟踪值是否超过统一电流上限。 */
        { /* PWM 饱和跟踪上限处理开始。 */
            measured_tracking_ma = maximum_current_ma; /* 限制跟踪值到当前允许最大电流。 */
        } /* PWM 饱和跟踪上限处理结束。 */
        if (measured_tracking_ma < tracking_current_ma) /* 检查 PWM 实际能力是否比其他限制更严格。 */
        { /* PWM 饱和跟踪值选择开始。 */
            tracking_current_ma = measured_tracking_ma; /* 选择更小的实际可实现电流。 */
        } /* PWM 饱和跟踪值选择结束。 */
    } /* PWM 上限饱和处理结束。 */
    output_current_ma = 0L;             /* 默认停止模式输出零电流。 */
    if (g_control_command.mode == BLDC_CONTROL_CURRENT) /* 检查是否处于直接电流模式。 */
    { /* 直接电流模式处理开始。 */
        if ((g_bldc.state != BLDC_STATE_INTEGRATE) || /* 检查是否仍处于定位、开环或闭环接管。 */
            (g_bldc.state_ticks < BLDC_CLOSED_LOOP_CURRENT_HOLD_TICKS)) /* 检查闭环三百毫秒保持是否结束。 */
        { /* 电流模式启动保持开始。 */
            output_current_ma = BLDC_STARTUP_CURRENT_MIN_MA; /* 启动和接管保持阶段明确请求五百毫安。 */
        } /* 电流模式启动保持结束。 */
        else                            /* 处理闭环启动保持已经结束的情况。 */
        { /* 电流模式用户命令处理开始。 */
            output_current_ma = g_control_command.current_command_ma; /* 严格使用用户电流命令并交给外层下降斜坡。 */
        } /* 电流模式用户命令处理结束。 */
    } /* 直接电流模式处理结束。 */
    else if (g_control_command.mode == BLDC_CONTROL_DUTY) /* 检查是否处于占空比外环模式。 */
    { /* 占空比外环模式处理开始。 */
        if ((g_bldc.state != BLDC_STATE_INTEGRATE) || /* 检查是否仍处于定位、开环或闭环接管。 */
            (g_bldc.state_ticks < BLDC_CLOSED_LOOP_CURRENT_HOLD_TICKS)) /* 检查闭环三百毫秒保持是否结束。 */
        { /* 占空比外环启动等待开始。 */
            output_current_ma = BLDC_STARTUP_CURRENT_MIN_MA; /* 启动和接管保持阶段直接请求五百毫安。 */
            g_control_command.duty_integral_accumulator = output_current_ma * BLDC_DUTY_LOOP_KI_DIVISOR; /* 持续预置积分项避免接管突变。 */
            g_control_command.duty_error_counts = 0L; /* 启动期间不计算没有意义的占空比误差。 */
        } /* 占空比外环启动等待结束。 */
        else                            /* 处理已经允许闭合占空比外环的情况。 */
        { /* 占空比 PI 处理开始。 */
            g_control_command.duty_error_counts = (int32_t)g_control_command.duty_target_counts - (int32_t)g_bldc.duty; /* 计算目标和实际占空比误差。 */
            proportional_current_ma = g_control_command.duty_error_counts * BLDC_DUTY_LOOP_KP_MA_PER_COUNT; /* 将占空比误差换算为比例电流。 */
            integral_minimum = BLDC_CLOSED_LOOP_CURRENT_MIN_MA * BLDC_DUTY_LOOP_KI_DIVISOR; /* 计算占空比积分项下限。 */
            integral_maximum = maximum_current_ma * BLDC_DUTY_LOOP_KI_DIVISOR; /* 计算占空比积分项上限。 */
            output_current_ma = (g_control_command.duty_integral_accumulator / BLDC_DUTY_LOOP_KI_DIVISOR) + proportional_current_ma; /* 计算限幅前占空比 PI 输出。 */
            if (((output_current_ma >= maximum_current_ma) || /* 检查占空比外环自身是否达到电流上限。 */
                 (downstream_upper_saturated != 0U)) && /* 检查下游是否已被输入或 PWM 限制。 */
                (g_control_command.duty_error_counts > 0L)) /* 仅处理继续要求增加输出的误差方向。 */
            { /* 占空比上限跟踪抗饱和开始。 */
                g_control_command.duty_integral_accumulator = /* 直接回算能够得到可实现电流的积分项。 */
                    (tracking_current_ma - proportional_current_ma) * BLDC_DUTY_LOOP_KI_DIVISOR; /* 让 PI 输出跟随下游允许值。 */
            } /* 占空比上限跟踪抗饱和结束。 */
            else if ((output_current_ma <= BLDC_CLOSED_LOOP_CURRENT_MIN_MA) && /* 检查占空比外环是否到达最低闭环电流。 */
                     (g_control_command.duty_error_counts < 0L)) /* 检查误差是否仍要求继续降低输出。 */
            { /* 占空比下限跟踪抗饱和开始。 */
                g_control_command.duty_integral_accumulator = /* 回算最低稳定电流对应的积分项。 */
                    (BLDC_CLOSED_LOOP_CURRENT_MIN_MA - proportional_current_ma) * BLDC_DUTY_LOOP_KI_DIVISOR; /* 防止积分器继续向下积累。 */
                active_reason = (BLDC_LimitReason_t)((uint32_t)active_reason | (uint32_t)BLDC_LIMIT_MINIMUM_CURRENT); /* 加入最低稳定电流限制位。 */
            } /* 占空比下限跟踪抗饱和结束。 */
            else                        /* 处理没有发生外环饱和的正常情况。 */
            { /* 占空比正常积分开始。 */
                g_control_command.duty_integral_accumulator += g_control_command.duty_error_counts; /* 仅在不会加重饱和时积分。 */
            } /* 占空比正常积分结束。 */
            if (g_control_command.duty_integral_accumulator < integral_minimum) /* 检查占空比积分项下溢。 */
            { /* 占空比积分项下限处理开始。 */
                g_control_command.duty_integral_accumulator = integral_minimum; /* 限制积分项到稳定运行最低电流。 */
            } /* 占空比积分项下限处理结束。 */
            if (g_control_command.duty_integral_accumulator > integral_maximum) /* 检查占空比积分项上溢。 */
            { /* 占空比积分项上限处理开始。 */
                g_control_command.duty_integral_accumulator = integral_maximum; /* 限制积分项到当前允许最大电流。 */
            } /* 占空比积分项上限处理结束。 */
            output_current_ma = (g_control_command.duty_integral_accumulator / BLDC_DUTY_LOOP_KI_DIVISOR) + proportional_current_ma; /* 使用更新积分项重新计算输出。 */
        } /* 占空比 PI 处理结束。 */
    } /* 占空比外环模式处理结束。 */
    else if (g_control_command.mode == BLDC_CONTROL_SPEED) /* 检查是否处于电角转速外环模式。 */
    { /* 电角转速外环模式处理开始。 */
        if ((g_bldc.state != BLDC_STATE_INTEGRATE) || /* 检查是否仍处于无感启动和接管过程。 */
            (g_bldc.state_ticks < BLDC_CLOSED_LOOP_CURRENT_HOLD_TICKS)) /* 检查闭环三百毫秒保持是否结束。 */
        { /* 速度外环启动等待开始。 */
            output_current_ma = BLDC_STARTUP_CURRENT_MIN_MA; /* 启动和接管保持阶段直接请求五百毫安。 */
            BLDC_ResetSpeedPidRuntime(output_current_ma); /* 持续预置五百毫安积分并等待有效滤波转速。 */
            g_control_command.speed_error_erpm = 0L; /* 启动期间不使用尚未稳定的转速误差。 */
        } /* 速度外环启动等待结束。 */
        else                            /* 处理已经允许闭合速度外环的情况。 */
        { /* 速度 PID 处理开始。 */
            int64_t proportional_numerator; /* 保存速度比例项定点计算分子。 */
            int64_t derivative_numerator; /* 保存速度微分项定点计算分子。 */
            int64_t derivative_filter_delta; /* 保存微分低通目标与当前值的差。 */
            int64_t integral_numerator; /* 保存包含历史余量的速度积分计算分子。 */
            int64_t integral_candidate; /* 保存限幅前的速度积分内部状态候选值。 */
            int64_t integral_maximum_scaled; /* 保存最大电流对应的积分内部状态上限。 */
            int32_t derivative_error_erpm; /* 保存本周期与上周期之间的速度误差变化。 */
            g_control_command.speed_error_erpm = (int32_t)g_control_command.speed_command_erpm - (int32_t)g_bldc.electrical_rpm; /* 计算目标与实际电角转速误差。 */
            proportional_numerator = (int64_t)g_control_command.speed_error_erpm * (int64_t)g_speed_pid.kp_x1000; /* 使用千分之一比例增益计算比例项分子。 */
            proportional_current_ma = (int32_t)(proportional_numerator / BLDC_SPEED_PID_SCALE); /* 将比例项换算为整数毫安。 */
            g_speed_pid.proportional_ma = proportional_current_ma; /* 保存比例项供实时监视。 */
            g_speed_pid.elapsed_ms = elapsed_ms; /* 保存本周期实际使用的 PID 时间步长。 */
            if (g_speed_pid.initialized == 0U) /* 检查速度微分历史是否已经建立。 */
            { /* 首个有效速度 PID 周期处理开始。 */
                g_speed_pid.previous_error_erpm = g_control_command.speed_error_erpm; /* 用当前误差建立历史避免首次微分冲击。 */
                g_speed_pid.derivative_raw_ma = 0L; /* 首周期不生成原始微分输出。 */
                g_speed_pid.derivative_filtered_ma = 0L; /* 首周期不生成滤波微分输出。 */
                g_speed_pid.initialized = 1U; /* 标记后续周期可以计算误差变化。 */
            } /* 首个有效速度 PID 周期处理结束。 */
            else                        /* 处理已经具备上一周期误差的情况。 */
            { /* 速度微分项计算开始。 */
                derivative_error_erpm = g_control_command.speed_error_erpm - g_speed_pid.previous_error_erpm; /* 计算误差在本周期内的变化量。 */
                derivative_numerator = (int64_t)derivative_error_erpm * (int64_t)g_speed_pid.kd_x1000; /* 将误差变化乘以微分增益千分值。 */
                g_speed_pid.derivative_raw_ma = (int32_t)(derivative_numerator / (int64_t)elapsed_ms); /* 按实际毫秒间隔换算原始微分电流。 */
                derivative_filter_delta = (int64_t)g_speed_pid.derivative_raw_ma - (int64_t)g_speed_pid.derivative_filtered_ma; /* 计算原始微分与滤波值的差。 */
                g_speed_pid.derivative_filtered_ma += (int32_t)((derivative_filter_delta * /* 使用用户滤波修正千分比。 */
                    (int64_t)g_speed_pid.derivative_filter_permille) / BLDC_PERMILLE_SCALE); /* 对微分项执行一阶低通滤波。 */
            } /* 速度微分项计算结束。 */
            g_speed_pid.previous_error_erpm = g_control_command.speed_error_erpm; /* 保存本周期误差供下一周期计算微分。 */
            g_speed_pid.integral_ma = g_control_command.speed_integral_accumulator / BLDC_SPEED_PID_SCALE; /* 读取更新前积分项毫安值。 */
            output_current_ma = proportional_current_ma + /* 合成更新积分前的比例项。 */
                                g_speed_pid.integral_ma + /* 合成更新积分前的积分项。 */
                                g_speed_pid.derivative_filtered_ma; /* 合成滤波微分项。 */
            integral_maximum_scaled = (int64_t)maximum_current_ma * BLDC_SPEED_PID_SCALE; /* 计算速度积分内部状态上限。 */
            if (((output_current_ma >= maximum_current_ma) || /* 检查速度外环自身是否达到电流上限。 */
                 (downstream_upper_saturated != 0U)) && /* 检查下游是否已被输入或 PWM 限制。 */
                (g_control_command.speed_error_erpm > 0L)) /* 仅处理继续要求增加速度的误差方向。 */
            { /* 速度上限跟踪抗饱和开始。 */
                integral_candidate = (int64_t)(tracking_current_ma - /* 从下游实际允许电流开始回算。 */
                    proportional_current_ma - g_speed_pid.derivative_filtered_ma) * BLDC_SPEED_PID_SCALE; /* 扣除比例和微分项得到可实现积分状态。 */
                g_speed_pid.integral_remainder = 0L; /* 饱和回算后清除旧积分定点余量。 */
            } /* 速度上限跟踪抗饱和结束。 */
            else                        /* 处理没有发生速度外环饱和的正常情况。 */
            { /* 速度正常积分开始。 */
                integral_numerator = ((int64_t)g_control_command.speed_error_erpm * /* 使用当前滤波速度误差。 */
                    (int64_t)g_speed_pid.ki_x1000 * (int64_t)elapsed_ms) + /* 乘以积分增益和显式毫秒时间步长。 */
                    (int64_t)g_speed_pid.integral_remainder; /* 加入上周期未满一个内部单位的余量。 */
                integral_candidate = (int64_t)g_control_command.speed_integral_accumulator + /* 从当前积分状态开始累加。 */
                    (integral_numerator / BLDC_SPEED_PID_SCALE); /* 将积分分子换算为千分之一毫安内部增量。 */
                g_speed_pid.integral_remainder = (int32_t)(integral_numerator % BLDC_SPEED_PID_SCALE); /* 保存尚未使用的带符号积分余量。 */
            } /* 速度正常积分结束。 */
            if (integral_candidate < 0L) /* 检查正向非同步整流速度积分是否低于零。 */
            { /* 速度积分项零下限处理开始。 */
                integral_candidate = 0L; /* 允许积分释放到零，但禁止形成负电流和再生制动。 */
                g_speed_pid.integral_remainder = 0L; /* 到达积分零下限后丢弃继续向负方向的余量。 */
            } /* 速度积分项零下限处理结束。 */
            if (integral_candidate > integral_maximum_scaled) /* 检查速度积分项是否超过当前电流上限。 */
            { /* 速度积分项上限处理开始。 */
                integral_candidate = integral_maximum_scaled; /* 限制速度积分状态到当前允许最大电流。 */
                g_speed_pid.integral_remainder = 0L; /* 到达积分上限后丢弃继续向正方向的余量。 */
            } /* 速度积分项上限处理结束。 */
            g_control_command.speed_integral_accumulator = (int32_t)integral_candidate; /* 保存限幅后的速度积分内部状态。 */
            g_speed_pid.integral_ma = g_control_command.speed_integral_accumulator / BLDC_SPEED_PID_SCALE; /* 更新积分项毫安诊断值。 */
            output_current_ma = proportional_current_ma + /* 合成最终比例项。 */
                                g_speed_pid.integral_ma + /* 合成最终积分项。 */
                                g_speed_pid.derivative_filtered_ma; /* 合成最终滤波微分项。 */
            g_speed_pid.output_unclamped_ma = output_current_ma; /* 保存最终统一限幅前的速度 PID 输出。 */
        } /* 速度 PID 处理结束。 */
    } /* 电角转速外环模式处理结束。 */
    else                                /* 处理停止模式或非法模式。 */
    { /* 停止模式处理开始。 */
        output_current_ma = 0L;         /* 停止模式不生成电流命令。 */
    } /* 停止模式处理结束。 */
    g_output_limit.requested_current_ma = output_current_ma; /* 保存最终统一电流限幅前的外环请求。 */
    if (output_current_ma > maximum_current_ma) /* 检查上层输出是否超过当前允许最大电流。 */
    { /* 上层输出上限处理开始。 */
        output_current_ma = maximum_current_ma; /* 应用过流保护余量对应的最大电流。 */
        active_reason = (BLDC_LimitReason_t)((uint32_t)active_reason | (uint32_t)BLDC_LIMIT_MOTOR_CURRENT); /* 加入电机电流限制位。 */
        g_output_limit.current_saturated = 1U; /* 标记电机电流请求已经达到上限。 */
    } /* 上层输出上限处理结束。 */
    else if ((g_control_command.mode != BLDC_CONTROL_STOP) && /* 检查是否处于任意运行模式。 */
             (output_current_ma >= maximum_current_ma)) /* 检查输出是否刚好等于最大允许电流。 */
    { /* 电机电流边界诊断开始。 */
        active_reason = (BLDC_LimitReason_t)((uint32_t)active_reason | (uint32_t)BLDC_LIMIT_MOTOR_CURRENT); /* 记录电机电流已经到达边界。 */
        g_output_limit.current_saturated = 1U; /* 标记电机电流处于饱和边界。 */
    } /* 电机电流边界诊断结束。 */
    if ((g_control_command.mode != BLDC_CONTROL_STOP) && /* 检查当前是否为某种运行模式。 */
        (output_current_ma < minimum_current_ma)) /* 检查运行模式输出是否低于当前模式下限。 */
    { /* 上层运行电流下限处理开始。 */
        output_current_ma = minimum_current_ma; /* 运行模式保持各自定义的一百毫安最低电流。 */
        active_reason = (BLDC_LimitReason_t)((uint32_t)active_reason | (uint32_t)BLDC_LIMIT_MINIMUM_CURRENT); /* 加入最低稳定电流限制位。 */
    } /* 上层运行电流下限处理结束。 */
    if ((active_reason & BLDC_LIMIT_ERPM) != 0U) /* 检查速度配置上限是否正在生效。 */
    { /* 速度饱和标志更新开始。 */
        g_output_limit.speed_saturated = 1U; /* 标记用户速度命令已经被配置范围限制。 */
    } /* 速度饱和标志更新结束。 */
    g_output_limit.limited_current_ma = output_current_ma; /* 保存统一管理器最终允许的电机电流。 */
    g_output_limit.active_reason = active_reason; /* 保存本周期全部限幅原因组合。 */
    g_output_limit.active = (active_reason != BLDC_LIMIT_NONE) ? 1U : 0U; /* 根据原因组合更新统一限幅活动标志。 */
    g_control_command.output_current_ma = output_current_ma; /* 保存上层模式最终输出电流。 */
    if (g_control_command.mode == BLDC_CONTROL_SPEED) /* 检查当前输出是否来自速度 PID。 */
    { /* 速度 PID 最终输出诊断更新开始。 */
        g_speed_pid.output_limited_ma = output_current_ma; /* 保存统一限幅后的速度 PID 最终电流。 */
    } /* 速度 PID 最终输出诊断更新结束。 */
    else                                /* 处理当前不是速度控制模式的情况。 */
    { /* 非速度模式诊断清理开始。 */
        g_speed_pid.output_limited_ma = 0L; /* 清零不再有效的速度 PID 限幅后输出。 */
    } /* 非速度模式诊断清理结束。 */
    g_current_command.requested_ma = output_current_ma; /* 将上层输出交给已有毫秒级电流斜坡。 */
} /* 统一上层控制模式处理结束。 */

static void BLDC_ProcessCurrentCommand(void) /* 实现毫秒级运行电流命令斜坡。 */
{ /* 运行电流命令斜坡开始。 */
    uint32_t now_ms;                    /* 保存当前系统毫秒时间。 */
    uint32_t elapsed_ms;                /* 保存距离上一次斜坡更新的毫秒数。 */
    uint32_t ramp_rate;                 /* 保存当前方向使用的每秒电流斜率。 */
    uint32_t ramp_numerator;            /* 保存包含历史余量的斜坡计算分子。 */
    int32_t ramp_step_ma;               /* 保存本次允许变化的整数毫安数。 */
    int8_t new_direction;               /* 保存本次目标对应的斜坡方向。 */
    now_ms = HAL_GetTick();             /* 读取本次后台处理的系统时刻。 */
    elapsed_ms = now_ms - g_current_command.last_update_ms; /* 计算无符号回绕安全的时间差。 */
    if (elapsed_ms == 0U)               /* 检查是否仍处于同一个系统毫秒。 */
    { /* 同毫秒重复调用处理开始。 */
        return;                         /* 每毫秒最多更新一次外层命令斜坡。 */
    } /* 同毫秒重复调用处理结束。 */
    g_current_command.last_update_ms = now_ms; /* 保存已经处理到的最新系统时刻。 */
    if (elapsed_ms > BLDC_CURRENT_COMMAND_MAX_ELAPSED_MS) /* 检查后台是否长时间没有得到执行。 */
    { /* 后台长时间迟滞处理开始。 */
        elapsed_ms = BLDC_CURRENT_COMMAND_MAX_ELAPSED_MS; /* 限制单次跳变量避免突然增加转矩。 */
    } /* 后台长时间迟滞处理结束。 */
    if (g_current_command.applied_ma < g_current_command.requested_ma) /* 检查电流命令是否需要上升。 */
    { /* 电流命令上升处理开始。 */
        new_direction = 1;              /* 标记本次执行上升斜坡。 */
        ramp_rate = g_current_command.rise_ma_per_second; /* 选择用户配置的上升斜率。 */
    } /* 电流命令上升处理结束。 */
    else if (g_current_command.applied_ma > g_current_command.requested_ma) /* 检查电流命令是否需要下降。 */
    { /* 电流命令下降处理开始。 */
        new_direction = -1;             /* 标记本次执行下降斜坡。 */
        ramp_rate = g_current_command.fall_ma_per_second; /* 选择用户配置的下降斜率。 */
    } /* 电流命令下降处理结束。 */
    else                                /* 处理外层命令已经到达用户请求的情况。 */
    { /* 电流命令保持处理开始。 */
        g_current_command.ramp_remainder = 0U; /* 到达目标后不保留亚毫安余量。 */
        g_current_command.ramp_direction = 0; /* 标记当前没有正在执行的斜坡。 */
        BLDC_UpdateInputLimits();       /* 即使命令不变也根据最新占空比更新输入限幅。 */
        return;                         /* 本周期不需要继续计算斜坡。 */
    } /* 电流命令保持处理结束。 */
    if (new_direction != g_current_command.ramp_direction) /* 检查命令是否刚刚改变方向。 */
    { /* 斜坡方向改变处理开始。 */
        g_current_command.ramp_remainder = 0U; /* 丢弃旧方向留下的亚毫安余量。 */
        g_current_command.ramp_direction = new_direction; /* 保存新的斜坡方向。 */
    } /* 斜坡方向改变处理结束。 */
    ramp_numerator = (ramp_rate * elapsed_ms) + g_current_command.ramp_remainder; /* 累加本次斜坡量和历史余量。 */
    ramp_step_ma = (int32_t)(ramp_numerator / BLDC_CURRENT_COMMAND_TIME_BASE_MS); /* 换算本次可变化的整数毫安数。 */
    g_current_command.ramp_remainder = ramp_numerator % BLDC_CURRENT_COMMAND_TIME_BASE_MS; /* 保存尚不足一毫安的余量。 */
    if (ramp_step_ma == 0L)             /* 检查当前累计时间是否足以改变一毫安。 */
    { /* 亚毫安斜坡累计处理开始。 */
        BLDC_UpdateInputLimits();       /* 斜坡不足一毫安时仍更新输入电流和功率限制。 */
        return;                         /* 保留余量并等待后续毫秒继续累计。 */
    } /* 亚毫安斜坡累计处理结束。 */
    if (new_direction > 0)              /* 检查当前是否执行上升斜坡。 */
    { /* 上升斜坡应用开始。 */
        g_current_command.applied_ma += ramp_step_ma; /* 按计算结果增加外层电流命令。 */
        if (g_current_command.applied_ma > g_current_command.requested_ma) /* 检查上升斜坡是否越过请求值。 */
        { /* 上升斜坡越界处理开始。 */
            g_current_command.applied_ma = g_current_command.requested_ma; /* 精确停在用户请求值。 */
        } /* 上升斜坡越界处理结束。 */
    } /* 上升斜坡应用结束。 */
    else                                /* 处理当前执行下降斜坡的情况。 */
    { /* 下降斜坡应用开始。 */
        g_current_command.applied_ma -= ramp_step_ma; /* 按计算结果降低外层电流命令。 */
        if (g_current_command.applied_ma < g_current_command.requested_ma) /* 检查下降斜坡是否越过请求值。 */
        { /* 下降斜坡越界处理开始。 */
            g_current_command.applied_ma = g_current_command.requested_ma; /* 精确停在用户请求值。 */
        } /* 下降斜坡越界处理结束。 */
    } /* 下降斜坡应用结束。 */
    BLDC_UpdateInputLimits();           /* 将外层命令经过输入电流和功率限制后送入电流 PI。 */
} /* 运行电流命令斜坡结束。 */

static void BLDC_UpdateInputLimits(void) /* 实现 VESC 风格输入电流和功率限幅。 */
{ /* 输入电流和功率限幅开始。 */
    int32_t motor_current_ma;           /* 保存用于输入电流估算的非负滤波电机电流。 */
    int32_t limited_target_ma;          /* 保存本次计算得到的限幅后电机电流目标。 */
    int32_t minimum_current_ma;          /* 保存当前控制模式允许的最低电机电流。 */
    uint32_t duty;                      /* 保存满量程为一千二百的实际占空比。 */
    uint32_t bus_voltage_mv;            /* 保存低通滤波后的母线电压。 */
    uint32_t current_allowed_motor_ma;  /* 保存输入电流上限允许的电机电流。 */
    uint32_t power_allowed_motor_ma;    /* 保存输入功率上限允许的电机电流。 */
    uint32_t selected_allowed_motor_ma; /* 保存两个输入限制允许电机电流的较小值。 */
    uint32_t soft_allowed_motor_ma;     /* 保存软过压降额允许的电机电流。 */
    uint32_t soft_voltage_span_mv;      /* 保存软过压线性降额区间宽度。 */
    uint32_t soft_voltage_remaining_mv; /* 保存距离软过压强制滑行阈值的电压余量。 */
    uint16_t soft_derate_permille;      /* 保存当前母线电压对应的允许功率千分比。 */
    BLDC_InputLimitReason_t selected_reason; /* 保存本次较严格限制对应的原因。 */
    motor_current_ma = g_current_control.filtered_current_ma; /* 读取已经低通滤波的电机电流。 */
    if (motor_current_ma < 0L)          /* 检查换相噪声是否产生负电流值。 */
    { /* 负电流估算处理开始。 */
        motor_current_ma = -motor_current_ma; /* 输入电流诊断使用电机电流绝对值。 */
    } /* 负电流估算处理结束。 */
    duty = (uint32_t)g_bldc.duty;       /* 读取当前实际应用的占空比命令。 */
    bus_voltage_mv = g_bus_voltage.filtered_mv; /* 读取当前低通滤波母线电压。 */
    g_input_limit.estimated_signed_current_ma = (int32_t)(((int64_t)g_current.signed_filtered_current_ma * /* 使用带符号滤波电流建立能量方向诊断。 */
                                                  (int64_t)duty) / (int64_t)BLDC_DUTY_SCALE); /* 按 VESC BLDC 公式估算正负母线电流。 */
    g_input_limit.estimated_signed_power_mw = (int32_t)(((int64_t)g_input_limit.estimated_signed_current_ma * /* 使用带符号母线电流建立功率乘积。 */
                                                (int64_t)bus_voltage_mv) / 1000LL); /* 将毫安乘毫伏换算成带符号毫瓦。 */
    g_input_limit.estimated_current_ma = (uint32_t)(((int64_t)motor_current_ma * /* 使用六十四位乘法防止中间结果溢出。 */
                                          (int64_t)duty) / (int64_t)BLDC_DUTY_SCALE); /* 按 VESC BLDC 公式计算输入电流。 */
    g_input_limit.estimated_power_mw = (uint32_t)(((uint64_t)g_input_limit.estimated_current_ma * /* 使用估算输入电流建立功率乘积。 */
                                        (uint64_t)bus_voltage_mv) / 1000ULL); /* 将毫安乘毫伏换算成毫瓦。 */
    if (bus_voltage_mv > 0U)            /* 检查母线电压是否可以作为除数。 */
    { /* 功率上限换算开始。 */
        g_input_limit.power_current_limit_ma = (uint32_t)(((uint64_t)g_input_limit.power_limit_mw * 1000ULL) / /* 将毫瓦换算为毫安乘毫伏。 */
                                               (uint64_t)bus_voltage_mv); /* 按 P 除以 U 得到功率对应输入电流。 */
    } /* 功率上限换算结束。 */
    else                                /* 处理尚未得到有效母线电压的情况。 */
    { /* 无有效母线电压处理开始。 */
        g_input_limit.power_current_limit_ma = g_input_limit.current_limit_ma; /* 暂时只使用配置的输入电流上限。 */
    } /* 无有效母线电压处理结束。 */
    g_input_limit.effective_current_limit_ma = g_input_limit.current_limit_ma; /* 默认使用配置的输入电流上限。 */
    if (g_input_limit.power_current_limit_ma < g_input_limit.effective_current_limit_ma) /* 检查功率上限是否更严格。 */
    { /* 功率限制更严格处理开始。 */
        g_input_limit.effective_current_limit_ma = g_input_limit.power_current_limit_ma; /* 选择功率对应的较小输入电流。 */
    } /* 功率限制更严格处理结束。 */
    limited_target_ma = g_current_command.applied_ma; /* 默认不改变经过命令斜坡的电机电流。 */
    minimum_current_ma = BLDC_GetControlCurrentMinimumMa(); /* 获取输入限制不能跨越的当前模式电流下限。 */
    g_input_limit.reason = BLDC_INPUT_LIMIT_NONE; /* 默认本周期没有输入限幅原因。 */
    g_input_limit.active = 0U;         /* 默认本周期没有降低电机电流目标。 */
    soft_derate_permille = BLDC_PERMILLE_SCALE; /* 默认软过压允许百分之一百的正常功率。 */
    if (g_run_manager.run_command != 0U) /* 检查当前是否仍处于有效运行请求。 */
    { /* 新运行周期诊断清理开始。 */
        g_input_limit.stop_requested = 0U; /* 新运行周期允许重新记录一次限幅软停机。 */
    } /* 新运行周期诊断清理结束。 */
    if ((g_bldc.state == BLDC_STATE_INTEGRATE) && /* 检查是否已经进入反电动势积分闭环。 */
        (g_bldc.state_ticks >= BLDC_CLOSED_LOOP_CURRENT_HOLD_TICKS) && /* 检查闭环三百毫秒启动保持是否结束。 */
        (duty > 0U) &&                 /* 检查占空比能否用于反算允许电机电流。 */
        (g_run_manager.run_command != 0U)) /* 检查用户是否仍然请求运行。 */
    { /* 闭环输入限幅计算开始。 */
        current_allowed_motor_ma = (uint32_t)(((uint64_t)g_input_limit.current_limit_ma * /* 建立输入电流上限乘占空比满量程。 */
                                               (uint64_t)BLDC_DUTY_SCALE) / (uint64_t)duty); /* 反算输入电流上限允许的电机电流。 */
        power_allowed_motor_ma = (uint32_t)(((uint64_t)g_input_limit.power_current_limit_ma * /* 建立功率等效输入电流乘占空比满量程。 */
                                             (uint64_t)BLDC_DUTY_SCALE) / (uint64_t)duty); /* 反算功率上限允许的电机电流。 */
        selected_allowed_motor_ma = current_allowed_motor_ma; /* 默认输入电流上限更严格。 */
        selected_reason = BLDC_INPUT_LIMIT_CURRENT; /* 默认记录输入电流限制原因。 */
        if (power_allowed_motor_ma < selected_allowed_motor_ma) /* 检查功率限制允许值是否更小。 */
        { /* 功率允许值更小处理开始。 */
            selected_allowed_motor_ma = power_allowed_motor_ma; /* 选择功率限制允许的电机电流。 */
            selected_reason = BLDC_INPUT_LIMIT_POWER; /* 记录功率限制为实际原因。 */
        } /* 功率允许值更小处理结束。 */
        else if (power_allowed_motor_ma == selected_allowed_motor_ma) /* 检查两个限制是否得到相同允许值。 */
        { /* 两个限制同时生效处理开始。 */
            selected_reason = BLDC_INPUT_LIMIT_CURRENT_AND_POWER; /* 记录电流和功率限制同时生效。 */
        } /* 两个限制同时生效处理结束。 */
        else                            /* 处理输入电流限制单独更严格的情况。 */
        { /* 输入电流限制单独生效处理开始。 */
            __NOP();                    /* 保留已经选择的输入电流限制原因。 */
        } /* 输入电流限制单独生效处理结束。 */
        if ((uint32_t)limited_target_ma > selected_allowed_motor_ma) /* 检查用户电流命令是否超过输入限制允许值。 */
        { /* 输入限制降低目标处理开始。 */
            limited_target_ma = (int32_t)selected_allowed_motor_ma; /* 将电机电流目标限制到允许值。 */
            g_input_limit.reason = selected_reason; /* 保存实际降低目标的限制原因。 */
            g_input_limit.active = 1U; /* 标记输入限幅当前已经生效。 */
            if (limited_target_ma < minimum_current_ma) /* 检查允许值是否低于当前模式最低电流。 */
            { /* 限幅目标过低处理开始。 */
                limited_target_ma = 0L; /* 将不稳定的低电流目标转换为零电流。 */
                g_input_limit.stop_requested = 1U; /* 标记本次停机由输入限制过低触发。 */
                BLDC_Stop();            /* 使用原有电流降零流程安全关闭功率输出。 */
            } /* 限幅目标过低处理结束。 */
        } /* 输入限制降低目标处理结束。 */
    } /* 闭环输入限幅计算结束。 */
    if ((g_bldc.state == BLDC_STATE_INTEGRATE) && /* 检查软过压降额是否处于稳定闭环阶段。 */
        (g_bldc.state_ticks >= BLDC_CLOSED_LOOP_CURRENT_HOLD_TICKS) && /* 检查启动可靠接管保持已经结束。 */
        (g_run_manager.run_command != 0U) && /* 检查用户仍然请求电机运行。 */
        (bus_voltage_mv >= BLDC_VBUS_SOFT_DERATE_START_MV)) /* 检查母线已经进入二十九伏软过压区域。 */
    { /* 软过压线性降额开始。 */
        soft_voltage_span_mv = BLDC_VBUS_SOFT_COAST_STOP_MV - BLDC_VBUS_SOFT_DERATE_START_MV; /* 计算二十九到三十伏降额区间。 */
        if (bus_voltage_mv >= BLDC_VBUS_SOFT_COAST_STOP_MV) /* 检查母线是否已经达到立即滑行阈值。 */
        { /* 软过压强制滑行请求开始。 */
            soft_derate_permille = 0U; /* 三十伏及以上不再允许继续输出功率。 */
            g_energy_protection.soft_stop_requested = 1U; /* 请求主循环立即关闭全部桥臂。 */
        } /* 软过压强制滑行请求结束。 */
        else                            /* 处理二十九到三十伏之间的线性降额区。 */
        { /* 软过压比例计算开始。 */
            soft_voltage_remaining_mv = BLDC_VBUS_SOFT_COAST_STOP_MV - bus_voltage_mv; /* 计算距离三十伏还剩多少电压余量。 */
            soft_derate_permille = (uint16_t)(((uint64_t)soft_voltage_remaining_mv * /* 将剩余电压余量换算为千分比。 */
                                                (uint64_t)BLDC_PERMILLE_SCALE) / (uint64_t)soft_voltage_span_mv); /* 二十九伏为一千、三十伏为零。 */
        } /* 软过压比例计算结束。 */
        soft_allowed_motor_ma = (uint32_t)(((uint64_t)(uint32_t)g_current_command.applied_ma * /* 使用当前斜坡后电流作为降额基准。 */
                                             (uint64_t)soft_derate_permille) / (uint64_t)BLDC_PERMILLE_SCALE); /* 按允许功率比例换算电机电流。 */
        if ((uint32_t)limited_target_ma > soft_allowed_motor_ma) /* 检查软过压限制是否比现有限制更严格。 */
        { /* 软过压降低目标开始。 */
            limited_target_ma = (int32_t)soft_allowed_motor_ma; /* 将电机电流按母线电压比例逐渐降低。 */
            g_input_limit.reason = (BLDC_InputLimitReason_t)((uint32_t)g_input_limit.reason | /* 保留已经存在的输入电流和功率限制原因。 */
                                   (uint32_t)BLDC_INPUT_LIMIT_SOFT_OVERVOLTAGE); /* 加入软过压降额原因。 */
            g_input_limit.active = 1U; /* 标记本周期软过压正在限制输出。 */
            if (limited_target_ma < minimum_current_ma) /* 检查降额后电流是否已经低于无感稳定下限。 */
            { /* 软过压低于稳定电流处理开始。 */
                limited_target_ma = 0L; /* 禁止保持不可控的过低电流继续换相。 */
                g_energy_protection.soft_stop_requested = 1U; /* 请求关闭桥臂而不是产生负转矩制动。 */
            } /* 软过压低于稳定电流处理结束。 */
        } /* 软过压降低目标结束。 */
    } /* 软过压线性降额结束。 */
    g_energy_protection.soft_derate_permille = soft_derate_permille; /* 保存当前允许功率比例供实时监视。 */
    if (g_run_manager.run_command == 0U) /* 检查运行命令是否已经被用户或输入限制撤销。 */
    { /* 停止状态目标同步开始。 */
        limited_target_ma = 0L;         /* 停止状态始终向快速电流 PI 提供零目标。 */
    } /* 停止状态目标同步结束。 */
    g_input_limit.limited_target_ma = limited_target_ma; /* 保存最终限幅目标供实时监视。 */
    g_current_control.target_ma = limited_target_ma; /* 原子更新快速电流 PI 使用的最终目标。 */
} /* 输入电流和功率限幅结束。 */

static void BLDC_ForceEnergyCoastStop(BLDC_StopReason_t reason) /* 实现能量保护立即关闭桥臂。 */
{ /* 能量保护立即关闭桥臂开始。 */
    if (g_energy_protection.coast_stop_active == 0U) /* 检查本次能量事件是否尚未记录滑行停机。 */
    { /* 能量保护停机首次记录开始。 */
        g_energy_protection.coast_stop_count++; /* 累计一次回充或软过压触发的滑行停机。 */
    } /* 能量保护停机首次记录结束。 */
    g_energy_protection.coast_stop_active = 1U; /* 标记最近一次停机由能量保护触发。 */
    g_energy_protection.restart_inhibited = 1U; /* 母线恢复到二十八伏前禁止重新打开桥臂。 */
    g_energy_protection.last_event_ms = HAL_GetTick(); /* 保存本次立即滑行停机发生时刻。 */
    BLDC_EmergencyStop();               /* 复用已验证接口立即撤销命令并关闭 TIM1 全部输出。 */
    g_run_manager.stop_reason = reason; /* 用回充或软过压原因覆盖通用紧急停机诊断。 */
} /* 能量保护立即关闭桥臂结束。 */

static void BLDC_ProcessEnergyProtection(void) /* 实现禁止回充和软过压保护状态处理。 */
{ /* 禁止回充和软过压保护状态处理开始。 */
    uint32_t now_ms;                    /* 保存当前系统毫秒时刻。 */
    int32_t signed_input_current_ma;    /* 保存本周期带符号估算母线电流。 */
    int32_t signed_input_power_mw;      /* 保存本周期带符号估算母线功率。 */
    now_ms = HAL_GetTick();             /* 读取本次保护处理时刻。 */
    signed_input_current_ma = g_input_limit.estimated_signed_current_ma; /* 读取正值耗电、负值回充的估算电流。 */
    signed_input_power_mw = g_input_limit.estimated_signed_power_mw; /* 读取正值耗电、负值回充的估算功率。 */
    if (signed_input_current_ma < g_energy_protection.peak_regen_current_ma) /* 检查是否出现更大的负母线电流。 */
    { /* 负母线电流峰值更新开始。 */
        g_energy_protection.peak_regen_current_ma = signed_input_current_ma; /* 保存上电以来最小的负母线电流。 */
    } /* 负母线电流峰值更新结束。 */
    if (signed_input_power_mw < g_energy_protection.peak_regen_power_mw) /* 检查是否出现更大的负母线功率。 */
    { /* 负母线功率峰值更新开始。 */
        g_energy_protection.peak_regen_power_mw = signed_input_power_mw; /* 保存上电以来最小的负母线功率。 */
    } /* 负母线功率峰值更新结束。 */
    if (g_bus_voltage.filtered_mv >= BLDC_VBUS_SOFT_DERATE_START_MV) /* 检查母线是否进入软过压区域。 */
    { /* 软过压区域进入处理开始。 */
        if (g_energy_protection.soft_overvoltage_active == 0U) /* 检查是否为本次首次跨入软过压区域。 */
        { /* 软过压事件首次记录开始。 */
            g_energy_protection.soft_overvoltage_event_count++; /* 累计一次软过压区域进入事件。 */
            g_energy_protection.last_event_ms = now_ms; /* 保存本次软过压事件时刻。 */
        } /* 软过压事件首次记录结束。 */
        g_energy_protection.soft_overvoltage_active = 1U; /* 保持软过压活动标志直到电压降至释放阈值。 */
        g_energy_protection.restart_inhibited = 1U; /* 软过压期间禁止新的启动请求打开桥臂。 */
    } /* 软过压区域进入处理结束。 */
    else if (g_bus_voltage.filtered_mv <= BLDC_VBUS_SOFT_RELEASE_MV) /* 检查母线是否已经低于释放回差阈值。 */
    { /* 软过压释放处理开始。 */
        g_energy_protection.soft_overvoltage_active = 0U; /* 解除软过压活动标志。 */
        g_energy_protection.restart_inhibited = 0U; /* 允许用户在安全电压下重新发送运行命令。 */
        if (BLDC_IsPowerState() == 0U)  /* 检查当前是否已经关闭全部功率输出。 */
        { /* 停机状态降额复位开始。 */
            g_energy_protection.soft_derate_permille = BLDC_PERMILLE_SCALE; /* 恢复百分之一百允许功率诊断。 */
            g_energy_protection.soft_stop_requested = 0U; /* 清除已经完成的软过压停机请求。 */
        } /* 停机状态降额复位结束。 */
    } /* 软过压释放处理结束。 */
    else                                /* 处理二十八到二十九伏回差保持区。 */
    { /* 软过压回差保持开始。 */
        __NOP();                        /* 保持原活动和启动禁止状态以避免阈值附近反复启停。 */
    } /* 软过压回差保持结束。 */
    if ((g_energy_protection.soft_stop_requested != 0U) || /* 检查线性降额是否已经低于稳定运行电流。 */
        ((g_bus_voltage.measured_mv >= BLDC_VBUS_SOFT_COAST_STOP_MV) && /* 检查未滤波母线电压是否达到三十伏。 */
         (BLDC_IsPowerState() != 0U))) /* 只在桥臂仍输出时执行一次立即滑行停机。 */
    { /* 软过压立即滑行处理开始。 */
        if (g_energy_protection.soft_overvoltage_active == 0U) /* 检查快速电压突升是否越过了滤波事件记录。 */
        { /* 快速软过压事件补充记录开始。 */
            g_energy_protection.soft_overvoltage_event_count++; /* 为直接达到三十伏的事件补充累计次数。 */
            g_energy_protection.last_event_ms = now_ms; /* 保存快速软过压事件发生时刻。 */
        } /* 快速软过压事件补充记录结束。 */
        g_energy_protection.soft_overvoltage_active = 1U; /* 锁存软过压状态直到母线降至释放阈值。 */
        g_energy_protection.soft_stop_requested = 0U; /* 在执行停机前消费本次软过压请求。 */
        BLDC_ForceEnergyCoastStop(BLDC_STOP_REASON_SOFT_OVERVOLTAGE); /* 立即关闭桥臂并禁止自动重启。 */
        return;                         /* 停机完成后不再处理本周期回充候选。 */
    } /* 软过压立即滑行处理结束。 */
    if ((g_bldc.state == BLDC_STATE_INTEGRATE) && /* 检查是否处于稳定反电动势积分闭环。 */
        (g_bldc.state_ticks >= BLDC_CLOSED_LOOP_CURRENT_HOLD_TICKS) && /* 检查闭环启动保持阶段已经结束。 */
        (g_run_manager.run_command != 0U) && /* 检查用户仍然请求输出正向转矩。 */
        (signed_input_current_ma <= -BLDC_REGEN_INPUT_CURRENT_THRESHOLD_MA)) /* 检查估算母线电流是否持续指向回充。 */
    { /* 持续回充候选处理开始。 */
        if (g_energy_protection.regen_candidate_active == 0U) /* 检查是否为本次第一个显著负电流样本。 */
        { /* 回充候选计时开始。 */
            g_energy_protection.regen_candidate_active = 1U; /* 标记正在连续确认回充。 */
            g_energy_protection.regen_candidate_since_ms = now_ms; /* 保存连续负电流起点。 */
        } /* 回充候选计时结束。 */
        else if ((now_ms - g_energy_protection.regen_candidate_since_ms) >= BLDC_REGEN_CONFIRM_TIME_MS) /* 检查负电流是否已连续二十毫秒。 */
        { /* 持续回充确认处理开始。 */
            g_energy_protection.regen_candidate_active = 0U; /* 结束本次候选计时。 */
            g_energy_protection.regen_protection_active = 1U; /* 锁存本次回充保护活动诊断。 */
            g_energy_protection.regen_event_count++; /* 累计一次真实持续回充事件。 */
            g_energy_protection.last_event_ms = now_ms; /* 保存本次回充确认时刻。 */
            BLDC_ForceEnergyCoastStop(BLDC_STOP_REASON_REGEN_PROTECTION); /* 立即关闭桥臂而不请求负转矩。 */
            return;                     /* 回充保护完成后结束本周期处理。 */
        } /* 持续回充确认处理结束。 */
        else                            /* 处理仍在二十毫秒确认窗口内的负电流。 */
        { /* 回充候选保持开始。 */
            __NOP();                    /* 保持候选计时并继续观察下一毫秒。 */
        } /* 回充候选保持结束。 */
    } /* 持续回充候选处理结束。 */
    else                                /* 处理负电流不连续、尚未闭环或已经停机。 */
    { /* 回充候选取消开始。 */
        g_energy_protection.regen_candidate_active = 0U; /* 短暂换相负尖峰不会继续累计确认时间。 */
        g_energy_protection.regen_candidate_since_ms = 0U; /* 清除已经失效的候选起点。 */
    } /* 回充候选取消结束。 */
} /* 禁止回充和软过压保护状态处理结束。 */

static uint8_t BLDC_IsPowerState(void) /* 实现功率输出状态判断。 */
{ /* 功率输出状态判断开始。 */
    if ((g_bldc.state == BLDC_STATE_ALIGN) || /* 检查是否处于转子定位状态。 */
        (g_bldc.state == BLDC_STATE_OPEN_LOOP) || /* 检查是否处于开环加速状态。 */
        (g_bldc.state == BLDC_STATE_TRANSITION) || /* 检查是否处于闭环接管状态。 */
        (g_bldc.state == BLDC_STATE_INTEGRATE)) /* 检查是否处于积分闭环状态。 */
    { /* 功率输出状态处理开始。 */
        return 1U;                      /* 返回当前确实存在桥臂功率输出。 */
    } /* 功率输出状态处理结束。 */
    return 0U;                          /* 返回当前没有桥臂功率输出。 */
} /* 功率输出状态判断结束。 */

static uint8_t BLDC_IsRecoverableFault(BLDC_Fault_t fault) /* 实现软故障类型判断。 */
{ /* 软故障类型判断开始。 */
    if ((fault == BLDC_FAULT_BEMF_CALIBRATION) || /* 检查是否为反电动势标定失败。 */
        (fault == BLDC_FAULT_TRANSITION) || /* 检查是否为开环接管失败。 */
        (fault == BLDC_FAULT_LOST_SYNC))   /* 检查是否为闭环失步。 */
    { /* 可恢复软故障处理开始。 */
        return 1U;                      /* 允许运行管理器尝试有限次数重启。 */
    } /* 可恢复软故障处理结束。 */
    return 0U;                          /* 其余故障必须锁存并由用户检查。 */
} /* 软故障类型判断结束。 */

static uint8_t BLDC_IsBusVoltageReady(void) /* 实现启动母线电压许可判断。 */
{ /* 启动母线电压许可判断开始。 */
    if ((g_bus_voltage.initialized != 0U) && /* 检查母线电压是否已经得到有效样本。 */
        (g_bus_voltage.measured_mv >= BLDC_VBUS_LOW_LIMIT_MV) && /* 检查母线电压不低于十九伏。 */
        (g_bus_voltage.measured_mv < BLDC_VBUS_SOFT_DERATE_START_MV) && /* 检查新启动不会直接进入软过压降额区域。 */
        (g_energy_protection.restart_inhibited == 0U)) /* 检查上一次回充或软过压保护已经解除启动禁止。 */
    { /* 母线电压允许启动处理开始。 */
        return 1U;                      /* 返回当前母线电压允许打开功率输出。 */
    } /* 母线电压允许启动处理结束。 */
    return 0U;                          /* 返回当前母线电压不允许启动。 */
} /* 启动母线电压许可判断结束。 */

static void BLDC_BeginStartup(void)    /* 实现定位和开环启动准备。 */
{ /* 定位和开环启动准备开始。 */
    uint32_t now_ms;                    /* 保存本次实际开始启动的系统毫秒时刻。 */
    uint16_t start_duty;                /* 保存本次启动使用的初始占空比。 */
    uint8_t step_index;                 /* 保存六个扇区诊断数组下标。 */
    if ((g_run_manager.initialized == 0U) || /* 检查底层初始化是否完成。 */
        (g_current.calibrated == 0U) || /* 检查三相电流零点是否有效。 */
        (g_run_manager.run_command == 0U) || /* 检查用户是否仍然请求运行。 */
        (BLDC_IsBusVoltageReady() == 0U)) /* 检查母线电压是否允许启动。 */
    { /* 启动条件不满足处理开始。 */
        return;                         /* 保持全部桥臂关闭并等待条件满足。 */
    } /* 启动条件不满足处理结束。 */
    now_ms = HAL_GetTick();             /* 在全部启动许可满足后锁存本次启动基准时刻。 */
    g_energy_protection.regen_candidate_since_ms = 0U; /* 新启动前清除旧的连续回充确认起点。 */
    g_energy_protection.regen_candidate_active = 0U; /* 新启动前清除旧的回充候选状态。 */
    g_energy_protection.regen_protection_active = 0U; /* 新启动前清除上一次回充保护活动标志。 */
    g_energy_protection.soft_stop_requested = 0U; /* 新启动前清除上一次软过压停机请求。 */
    g_energy_protection.coast_stop_active = 0U; /* 新启动前清除上一次能量保护滑行标志。 */
    if (g_motion.direction < 0)         /* 检查停机期间选择的方向是否为反转。 */
    { /* 反转方向规范化开始。 */
        g_motion.direction = -1;        /* 将任意负方向值规范为负一。 */
    } /* 反转方向规范化结束。 */
    else                                /* 处理正转或尚未设置方向的情况。 */
    { /* 正转方向规范化开始。 */
        g_motion.direction = 1;         /* 将零和任意正方向值规范为正一。 */
    } /* 正转方向规范化结束。 */
    g_motion.align_enter_ms = now_ms;   /* 记录本次进入转子定位的精确毫秒时刻。 */
    g_motion.open_loop_enter_ms = 0U;   /* 清除上一次启动留下的开环进入时刻。 */
    g_motion.transition_enter_ms = 0U;  /* 清除上一次启动留下的接管进入时刻。 */
    g_motion.integrate_enter_ms = 0U;   /* 清除上一次启动留下的闭环进入时刻。 */
    g_motion.open_loop_commutations = 0U; /* 清除上一次启动的开环强制换相总数。 */
    g_motion.transition_bemf_commutations = 0U; /* 清除上一次启动的接管积分换相总数。 */
    g_motion.transition_forced_commutations = 0U; /* 清除上一次启动的接管超时换相总数。 */
    start_duty = (uint16_t)(BLDC_TEST_DUTY + /* 使用基础启动占空比建立本次初值。 */
                 ((uint16_t)g_run_manager.restart_count * BLDC_RESTART_DUTY_STEP)); /* 每次软故障重试适量提高启动转矩。 */
    if (start_duty > BLDC_RESTART_DUTY_MAX) /* 检查重试启动占空比是否超过安全上限。 */
    { /* 重试启动占空比上限处理开始。 */
        start_duty = BLDC_RESTART_DUTY_MAX; /* 限制自动恢复时的最大启动占空比。 */
    } /* 重试启动占空比上限处理结束。 */
    g_bldc.state = BLDC_STATE_STOP;     /* 在装载首个换相步前保持停止状态。 */
    g_bldc.fault = BLDC_FAULT_NONE;     /* 清除已经允许重试的软故障状态。 */
    g_bldc.step = 1U;                   /* 每次启动统一从第一换相步定位。 */
    g_bldc.state_ticks = 0U;            /* 清零启动状态计时。 */
    g_bldc.commutation_ticks = 0U;      /* 清零当前换相步计时。 */
    g_bldc.last_commutation_ticks = 0U; /* 清零上一步换相周期。 */
    g_bldc.open_loop_ticks = BLDC_OPEN_LOOP_START_TICKS; /* 恢复开环启动的初始慢速周期。 */
    g_bldc.blanking_ticks = 0U;         /* 清零上一次反电动势消隐计数。 */
    g_bldc.bemf_integral = 0U;          /* 清零上一次反电动势积分值。 */
    g_bldc.integral_limit = 0U;         /* 清零上一次运行积分阈值。 */
    g_bldc.integral_limit_unadvanced = 0U; /* 清零上一次未提前的积分阈值。 */
    g_bldc.integral_limit_base = 0U;    /* 要求本次开环重新标定基础磁链项。 */
    g_bldc.integral_coupling_reference = 0U; /* 要求本次开环重新标定母线耦合项。 */
    g_bldc.integral_limit_max = 0U;     /* 清零动态积分阈值上限。 */
    g_bldc.integral_calibration_bus_mv = 0U; /* 清零积分标定母线电压。 */
    g_bldc.integral_calibration_erpm = 0U; /* 清零积分标定电角转速。 */
    g_bldc.phase_advance_permille = 0U; /* 启动阶段关闭换相提前。 */
    g_bldc.calibration_sum = 0U;        /* 清零本次积分标定累计值。 */
    g_bldc.calibration_count = 0U;      /* 清零本次有效标定步数。 */
    g_bldc.calibration_attempts = 0U;   /* 清零本次标定尝试次数。 */
    g_bldc.transition_valid_count = 0U; /* 清零闭环接管连续成功次数。 */
    g_bldc.forced_count = 0U;           /* 清零闭环连续强制换相次数。 */
    g_bldc.two_step_recovery_count = 0U; /* 清零本次启动的跨两步恢复累计次数。 */
    g_bldc.timeout_ticks = BLDC_TIMEOUT_MAX_TICKS; /* 恢复启动阶段的最大换相超时。 */
    BLDC_ResetSpeedEstimator();         /* 清零原始转速、滤波转速和未完成的六步测速窗口。 */
    g_current_control.ramped_target_ma = 0L; /* 从零开始建立本次电流命令。 */
    g_current_control.filtered_current_ma = 0L; /* 清零上一次滤波电流。 */
    g_current_control.error_ma = 0L;    /* 清零上一次电流闭环误差。 */
    g_current_control.proportional_duty = 0L; /* 清零上一次比例项。 */
    g_current_control.integral_accumulator = (int32_t)start_duty * BLDC_CURRENT_KI_DIVISOR; /* 用启动占空比预置积分项。 */
    g_current_control.output_duty = (int32_t)start_duty; /* 保存本次启动的当前占空比。 */
    g_current_control.filter_initialized = 0U; /* 要求首次有效样本重新初始化电流滤波器。 */
    g_current.overcurrent_count = 0U;   /* 清零软件过流连续确认计数。 */
    g_current.blanking_samples = 0U;    /* 清零上一次换相电流消隐计数。 */
    g_current.signed_motor_current_ma = 0L; /* 清零本次启动的带符号瞬时电流。 */
    g_current.signed_filtered_current_ma = 0L; /* 清零本次启动的带符号滤波电流。 */
    g_current.signed_positive_peak_ma = 0L; /* 清零本次启动的正电流峰值。 */
    g_current.signed_negative_peak_ma = 0L; /* 清零本次启动的负电流峰值。 */
    g_current.polarity_positive_count = 0U; /* 清零本次启动的显著正电流样本数。 */
    g_current.polarity_error_count = 0U; /* 清零本次启动的显著负电流样本数。 */
    for (step_index = 0U; step_index < 6U; step_index++) /* 遍历六个换相扇区诊断值。 */
    { /* 六扇区诊断值清零开始。 */
        g_current.step_current_ma[step_index] = 0L; /* 清除上一次运行留下的扇区滤波电流。 */
        g_current.step_signed_current_ma[step_index] = 0L; /* 清除上一次运行留下的扇区带符号滤波电流。 */
    } /* 六扇区诊断值清零结束。 */
    BLDC_SetDuty(start_duty);           /* 应用启动占空比并同步更新 ADC 采样点。 */
    BLDC_ApplyStep(1U);                 /* 装载第一换相步用于转子定位。 */
    if (g_bldc.state == BLDC_STATE_FAULT) /* 检查第一换相步装载是否失败。 */
    { /* 第一换相步装载失败处理开始。 */
        return;                         /* 保留硬故障并禁止打开主输出。 */
    } /* 第一换相步装载失败处理结束。 */
    g_bldc.state = BLDC_STATE_ALIGN;    /* 进入转子定位状态。 */
    g_run_manager.stop_pending = 0U;    /* 清除可能遗留的软停机等待标志。 */
    g_run_manager.last_state = (uint8_t)BLDC_STATE_ALIGN; /* 同步后台观察状态避免误判跳变。 */
    g_run_manager.start_count++;        /* 记录一次实际发起的启动流程。 */
    g_motion.startup_attempt = g_run_manager.start_count; /* 将粘性诊断绑定到已经实际发起的累计启动序号。 */
    __HAL_TIM_MOE_ENABLE(&htim1);       /* 所有启动条件满足后才打开高级定时器主输出。 */
} /* 定位和开环启动准备结束。 */

static void BLDC_CompleteStop(BLDC_StopReason_t reason) /* 实现安全停机完成处理。 */
{ /* 安全停机完成处理开始。 */
    g_bldc.state = BLDC_STATE_STOP;     /* 先退出快速控制功率状态。 */
    BLDC_AllOutputsOff();               /* 关闭主输出并清除全部桥臂使能。 */
    g_bldc.step = 1U;                   /* 将停止后的观察步号恢复为第一步。 */
    g_bldc.duty = 0U;                   /* 将诊断占空比同步为零。 */
    g_bldc.pwm_pulse = 0U;              /* 将诊断比较值同步为零。 */
    g_bldc.state_ticks = 0U;            /* 清零停止后的状态计时。 */
    g_bldc.commutation_ticks = 0U;      /* 清零停止后的换相计时。 */
    g_bldc.last_commutation_ticks = 0U; /* 清零停止后的历史换相周期。 */
    BLDC_ResetSpeedEstimator();         /* 停止后清零原始转速、滤波转速和测速窗口。 */
    g_bldc.phase_advance_permille = 0U; /* 停止后清零实际换相提前量。 */
    g_bldc.bemf_integral = 0U;          /* 停止后清零反电动势积分值。 */
    g_current_control.ramped_target_ma = 0L; /* 将斜坡电流目标归零。 */
    g_current_control.filtered_current_ma = 0L; /* 清零停止后的滤波电流。 */
    g_current_control.error_ma = 0L;    /* 清零停止后的电流误差。 */
    g_current_control.proportional_duty = 0L; /* 清零停止后的比例项。 */
    g_current_control.output_duty = 0L; /* 将诊断 PI 输出同步为零。 */
    g_current_control.integral_accumulator = 0L; /* 清零停止后的积分项。 */
    g_current_control.filter_initialized = 0U; /* 要求下次启动重新初始化电流滤波器。 */
    g_current.signed_motor_current_ma = 0L; /* 停机后清零不再有效的带符号瞬时电流。 */
    g_current.signed_filtered_current_ma = 0L; /* 停机后清零不再有效的带符号滤波电流。 */
    g_current.source_current_ma = 0L;   /* 停机后清零不再有效的源相诊断电流。 */
    g_current.floating_current_ma = 0L; /* 停机后清零不再有效的悬空相诊断电流。 */
    BLDC_ResetSpeedPidRuntime(0L);      /* 停止后清零速度 PID 运行状态并保留用户配置增益。 */
    g_run_manager.stop_pending = 0U;    /* 清除软停机等待标志。 */
    g_run_manager.stop_reason = reason; /* 保存本次停机原因用于调试。 */
    g_run_manager.last_state = (uint8_t)BLDC_STATE_STOP; /* 同步后台观察状态。 */
    g_run_manager.stop_count++;         /* 记录一次完成的安全停机。 */
} /* 安全停机完成处理结束。 */

static uint16_t BLDC_GetBemfSpread(void) /* 实现三相反电动势峰峰值计算。 */
{ /* 三相反电动势峰峰值计算开始。 */
    uint16_t minimum;                   /* 保存三相反电动势原始值中的最小值。 */
    uint16_t maximum;                   /* 保存三相反电动势原始值中的最大值。 */
    minimum = g_bemf.u;                 /* 使用 U 相建立最小值初值。 */
    maximum = g_bemf.u;                 /* 使用 U 相建立最大值初值。 */
    if (g_bemf.v < minimum)             /* 检查 V 相是否低于当前最小值。 */
    { /* V 相最小值更新开始。 */
        minimum = g_bemf.v;             /* 将 V 相保存为新的最小值。 */
    } /* V 相最小值更新结束。 */
    if (g_bemf.w < minimum)             /* 检查 W 相是否低于当前最小值。 */
    { /* W 相最小值更新开始。 */
        minimum = g_bemf.w;             /* 将 W 相保存为新的最小值。 */
    } /* W 相最小值更新结束。 */
    if (g_bemf.v > maximum)             /* 检查 V 相是否高于当前最大值。 */
    { /* V 相最大值更新开始。 */
        maximum = g_bemf.v;             /* 将 V 相保存为新的最大值。 */
    } /* V 相最大值更新结束。 */
    if (g_bemf.w > maximum)             /* 检查 W 相是否高于当前最大值。 */
    { /* W 相最大值更新开始。 */
        maximum = g_bemf.w;             /* 将 W 相保存为新的最大值。 */
    } /* W 相最大值更新结束。 */
    return (uint16_t)(maximum - minimum); /* 返回不受虚拟中性点影响的三相峰峰值。 */
} /* 三相反电动势峰峰值计算结束。 */

static void BLDC_CancelReversal(void)   /* 实现安全换向取消和待执行命令清理。 */
{ /* 安全换向取消开始。 */
    if (g_reversal.state != BLDC_REVERSAL_IDLE) /* 检查是否确实取消了尚未完成的换向。 */
    { /* 有效换向取消计数开始。 */
        g_reversal.canceled_count++;    /* 累计一次被用户停机或故障打断的换向。 */
    } /* 有效换向取消计数结束。 */
    g_reversal.state = BLDC_REVERSAL_IDLE; /* 恢复换向管理器空闲状态。 */
    g_reversal.pending_mode = BLDC_CONTROL_STOP; /* 清除等待恢复的控制模式。 */
    g_reversal.requested_direction = g_motion.direction; /* 将目标方向恢复为当前实际方向。 */
    g_reversal.quiet_confirm_ms = 0U;   /* 清零反电动势静止连续确认时间。 */
    g_reversal.pending_current_ma = 0L; /* 清除等待恢复的电流命令。 */
    g_reversal.pending_duty_permille = 0U; /* 清除等待恢复的占空比命令。 */
    g_reversal.pending_speed_erpm = 0U; /* 清除等待恢复的速度命令。 */
} /* 安全换向取消结束。 */

static uint8_t BLDC_RequestSignedCommand(BLDC_ControlMode_t mode, int8_t direction, int32_t current_ma, uint16_t duty_permille, uint32_t speed_erpm) /* 实现有符号命令方向仲裁。 */
{ /* 有符号命令方向仲裁开始。 */
    if (direction < 0)                  /* 检查命令是否请求反转。 */
    { /* 反转方向规范化开始。 */
        direction = -1;                /* 将任意负值规范为负一。 */
    } /* 反转方向规范化结束。 */
    else                                /* 处理正转和零方向值。 */
    { /* 正转方向规范化开始。 */
        direction = 1;                 /* 将非负值统一规范为正一。 */
    } /* 正转方向规范化结束。 */
    g_reversal.pending_mode = mode;     /* 保存换向完成后需要恢复的控制模式。 */
    g_reversal.requested_direction = direction; /* 保存最近一次有符号命令目标方向。 */
    g_reversal.pending_current_ma = current_ma; /* 保存待执行的正幅值电流命令。 */
    g_reversal.pending_duty_permille = duty_permille; /* 保存待执行的正幅值占空比命令。 */
    g_reversal.pending_speed_erpm = speed_erpm; /* 保存待执行的正幅值速度命令。 */
    if (g_reversal.state != BLDC_REVERSAL_IDLE) /* 检查是否已经在执行上一条换向命令。 */
    { /* 换向过程中命令更新开始。 */
        return 1U;                      /* 只更新待执行目标并继续当前安全停机流程。 */
    } /* 换向过程中命令更新结束。 */
    if (direction == g_motion.direction) /* 检查目标方向是否与当前换相方向一致。 */
    { /* 同方向命令处理开始。 */
        return 0U;                      /* 允许调用者立即应用新的命令幅值。 */
    } /* 同方向命令处理结束。 */
    if (BLDC_IsPowerState() != 0U)      /* 检查方向不同时桥臂是否仍在输出功率。 */
    { /* 运行中安全换向请求开始。 */
        g_reversal.state = BLDC_REVERSAL_RAMP_DOWN; /* 先进入电流斜坡降零阶段。 */
        g_reversal.request_ms = HAL_GetTick(); /* 记录本次换向请求的精确时刻。 */
        g_reversal.coast_enter_ms = 0U; /* 清除上一次滑行等待起点。 */
        g_reversal.last_quiet_check_ms = 0U; /* 清除上一次静止判定时刻。 */
        g_reversal.quiet_confirm_ms = 0U; /* 清除上一次反电动势静止确认结果。 */
        g_control_command.mode = BLDC_CONTROL_STOP; /* 暂停原外环，防止速度或占空比环重新抬高电流。 */
        g_control_command.output_current_ma = 0L; /* 将外环输出诊断同步为零。 */
        g_current_command.requested_ma = 0L; /* 请求现有运行电流按照下降斜率回到零。 */
        g_run_manager.run_command = 0U; /* 阻止当前方向发生失步后自动重新启动。 */
        g_run_manager.stop_pending = 1U; /* 标记当前正在执行换向软停机。 */
        g_run_manager.stop_reason = BLDC_STOP_REASON_DIRECTION_CHANGE; /* 记录本次停机由换向触发。 */
        return 1U;                      /* 通知调用者命令已经暂存而非立即执行。 */
    } /* 运行中安全换向请求结束。 */
    if (BLDC_SetDirection(direction) == 0U) /* 检查无功率状态下方向设置是否意外失败。 */
    { /* 方向设置失败处理开始。 */
        return 1U;                      /* 保留命令但禁止在方向不明确时启动。 */
    } /* 方向设置失败处理结束。 */
    return 0U;                          /* 返回方向已经安全切换，可以立即应用命令。 */
} /* 有符号命令方向仲裁结束。 */

static void BLDC_ProcessReversal(uint32_t now_ms) /* 实现安全换向减流、滑行和重启状态机。 */
{ /* 安全换向状态机开始。 */
    uint32_t coast_elapsed_ms;          /* 保存桥臂关闭后的滑行等待时间。 */
    BLDC_ControlMode_t pending_mode;    /* 保存即将恢复的控制模式局部副本。 */
    int8_t requested_direction;         /* 保存即将应用的目标方向局部副本。 */
    int32_t pending_current_ma;         /* 保存即将恢复的电流命令局部副本。 */
    uint16_t pending_duty_permille;     /* 保存即将恢复的占空比命令局部副本。 */
    uint32_t pending_speed_erpm;        /* 保存即将恢复的速度命令局部副本。 */
    if (g_bldc.state == BLDC_STATE_FAULT) /* 检查换向期间是否发生了任何软硬件故障。 */
    { /* 换向故障取消开始。 */
        BLDC_CancelReversal();          /* 故障优先并放弃本次自动反向重启。 */
        return;                         /* 保持故障状态等待原有故障处理流程。 */
    } /* 换向故障取消结束。 */
    if (g_reversal.state == BLDC_REVERSAL_RAMP_DOWN) /* 检查是否仍处于当前方向减流阶段。 */
    { /* 换向减流阶段开始。 */
        g_run_manager.run_command = 0U; /* 持续禁止当前方向自动恢复。 */
        g_run_manager.stop_pending = 1U; /* 保持换向软停机标志。 */
        g_current_command.requested_ma = 0L; /* 持续要求外层电流斜坡降到零。 */
        if ((BLDC_IsPowerState() != 0U) && /* 检查功率状态是否仍然存在。 */
            (g_current_command.applied_ma <= 0L) && /* 检查外层命令斜坡是否已经到零。 */
            (g_current_control.ramped_target_ma <= 0L)) /* 检查快速电流目标是否也已经到零。 */
        { /* 换向减流完成开始。 */
            BLDC_CompleteStop(BLDC_STOP_REASON_DIRECTION_CHANGE); /* 关闭主输出并清理本方向控制状态。 */
        } /* 换向减流完成结束。 */
        if (g_bldc.state == BLDC_STATE_STOP) /* 检查桥臂是否已经完成安全关闭。 */
        { /* 换向滑行阶段进入开始。 */
            g_reversal.state = BLDC_REVERSAL_COAST_WAIT; /* 进入转子自然滑行和 BEMF 静止确认阶段。 */
            g_reversal.coast_enter_ms = now_ms; /* 记录桥臂关闭后的滑行起点。 */
            g_reversal.last_quiet_check_ms = now_ms; /* 从当前毫秒开始执行静止确认。 */
            g_reversal.quiet_confirm_ms = 0U; /* 清零连续反电动势安静时间。 */
        } /* 换向滑行阶段进入结束。 */
        return;                         /* 本周期不执行新方向启动。 */
    } /* 换向减流阶段结束。 */
    if (g_reversal.state != BLDC_REVERSAL_COAST_WAIT) /* 检查是否为理论不可达的换向状态。 */
    { /* 非法换向状态处理开始。 */
        BLDC_CancelReversal();          /* 恢复为空闲状态并禁止意外启动。 */
        return;                         /* 结束本次状态机处理。 */
    } /* 非法换向状态处理结束。 */
    BLDC_AllOutputsOff();               /* 滑行等待期间每次处理都确保全部桥臂保持关闭。 */
    coast_elapsed_ms = now_ms - g_reversal.coast_enter_ms; /* 计算无符号回绕安全的滑行时间。 */
    if ((coast_elapsed_ms >= BLDC_REVERSAL_COAST_MIN_MS) && /* 检查最短无输出滑行时间是否结束。 */
        (now_ms != g_reversal.last_quiet_check_ms)) /* 检查本毫秒是否尚未执行静止判定。 */
    { /* 每毫秒反电动势静止确认开始。 */
        g_reversal.last_quiet_check_ms = now_ms; /* 记录本毫秒已经完成静止判定。 */
        if (BLDC_GetBemfSpread() <= BLDC_REVERSAL_BEMF_QUIET_COUNTS) /* 检查三相反电动势峰峰值是否足够低。 */
        { /* 反电动势安静确认开始。 */
            if (g_reversal.quiet_confirm_ms < UINT8_MAX) /* 检查连续确认时间是否尚未饱和。 */
            { /* 连续确认时间累计开始。 */
                g_reversal.quiet_confirm_ms++; /* 累计一个满足静止阈值的毫秒。 */
            } /* 连续确认时间累计结束。 */
        } /* 反电动势安静确认结束。 */
        else                            /* 处理电机仍有明显反电动势的情况。 */
        { /* 反电动势活动处理开始。 */
            g_reversal.quiet_confirm_ms = 0U; /* 重新开始连续二十毫秒静止确认。 */
        } /* 反电动势活动处理结束。 */
    } /* 每毫秒反电动势静止确认结束。 */
    if ((g_reversal.quiet_confirm_ms < BLDC_REVERSAL_QUIET_CONFIRM_MS) && /* 检查尚未连续满足静止条件。 */
        (coast_elapsed_ms < BLDC_REVERSAL_COAST_MAX_MS)) /* 检查最大滑行等待时间尚未到达。 */
    { /* 继续滑行等待开始。 */
        return;                         /* 保持桥臂关闭并等待转速进一步下降。 */
    } /* 继续滑行等待结束。 */
    if (coast_elapsed_ms >= BLDC_REVERSAL_COAST_MAX_MS) /* 检查是否由最大等待时间触发继续启动。 */
    { /* 滑行等待超时诊断开始。 */
        g_reversal.timeout_count++;     /* 记录一次没有提前满足 BEMF 静止判据的换向。 */
    } /* 滑行等待超时诊断结束。 */
    pending_mode = g_reversal.pending_mode; /* 在切换为空闲前保存待恢复控制模式。 */
    requested_direction = g_reversal.requested_direction; /* 保存最新目标方向。 */
    pending_current_ma = g_reversal.pending_current_ma; /* 保存最新待恢复电流命令。 */
    pending_duty_permille = g_reversal.pending_duty_permille; /* 保存最新待恢复占空比命令。 */
    pending_speed_erpm = g_reversal.pending_speed_erpm; /* 保存最新待恢复速度命令。 */
    g_reversal.state = BLDC_REVERSAL_IDLE; /* 允许方向设置和公开命令接口重新工作。 */
    if (BLDC_SetDirection(requested_direction) == 0U) /* 检查关闭桥臂后的方向切换是否成功。 */
    { /* 换向方向设置失败开始。 */
        BLDC_CancelReversal();          /* 清除待执行命令并保持安全停止。 */
        return;                         /* 禁止在方向设置失败后重新打开桥臂。 */
    } /* 换向方向设置失败结束。 */
    g_reversal.completed_count++;       /* 记录一次完成减流、滑行和方向切换的换向。 */
    if (pending_mode == BLDC_CONTROL_CURRENT) /* 检查是否需要恢复直接电流模式。 */
    { /* 电流模式恢复开始。 */
        BLDC_SetRunCurrentMa((requested_direction < 0) ? -pending_current_ma : pending_current_ma); /* 使用原符号恢复电流命令。 */
    } /* 电流模式恢复结束。 */
    else if (pending_mode == BLDC_CONTROL_DUTY) /* 检查是否需要恢复占空比模式。 */
    { /* 占空比模式恢复开始。 */
        BLDC_SetSignedDutyPermille((requested_direction < 0) ? -(int16_t)pending_duty_permille : (int16_t)pending_duty_permille); /* 使用原符号恢复占空比命令。 */
    } /* 占空比模式恢复结束。 */
    else if (pending_mode == BLDC_CONTROL_SPEED) /* 检查是否需要恢复速度模式。 */
    { /* 速度模式恢复开始。 */
        BLDC_SetSignedSpeedErpm((requested_direction < 0) ? -(int32_t)pending_speed_erpm : (int32_t)pending_speed_erpm); /* 使用原符号恢复速度命令。 */
    } /* 速度模式恢复结束。 */
    else                                /* 处理等待期间命令被清除的情况。 */
    { /* 无待恢复命令处理开始。 */
        BLDC_CancelReversal();          /* 保持电机停止并清除无效待执行数据。 */
    } /* 无待恢复命令处理结束。 */
} /* 安全换向状态机结束。 */

static void BLDC_ProcessManager(void)  /* 实现启停和自动恢复后台管理。 */
{ /* 启停和自动恢复后台管理开始。 */
    uint32_t now_ms;                    /* 保存当前系统毫秒时间。 */
    now_ms = HAL_GetTick();             /* 读取本次后台处理时间。 */
    if (g_run_manager.last_state != (uint8_t)g_bldc.state) /* 检查快速控制状态是否发生变化。 */
    { /* 控制状态变化处理开始。 */
        if (g_bldc.state == BLDC_STATE_INTEGRATE) /* 检查是否刚进入积分闭环。 */
        { /* 积分闭环稳定计时开始。 */
            g_run_manager.integrate_enter_ms = now_ms; /* 记录本次闭环接管完成时刻。 */
        } /* 积分闭环稳定计时结束。 */
        g_run_manager.last_state = (uint8_t)g_bldc.state; /* 保存后台已经观察到的新状态。 */
    } /* 控制状态变化处理结束。 */
    if (g_reversal.state != BLDC_REVERSAL_IDLE) /* 检查有符号命令是否正在执行安全换向。 */
    { /* 安全换向优先处理开始。 */
        BLDC_ProcessReversal(now_ms);   /* 执行减流、关闭桥臂、滑行和反向重启。 */
        return;                         /* 换向期间禁止普通启动和故障自动恢复并行执行。 */
    } /* 安全换向优先处理结束。 */
    if ((g_bldc.state == BLDC_STATE_INTEGRATE) && /* 检查当前是否稳定处于积分闭环。 */
        (g_run_manager.restart_count > 0U) && /* 检查本轮是否曾经自动重启。 */
        ((now_ms - g_run_manager.integrate_enter_ms) >= BLDC_RESTART_STABLE_MS)) /* 检查闭环是否已经稳定两秒。 */
    { /* 自动恢复成功处理开始。 */
        g_run_manager.restart_count = 0U; /* 稳定运行后清零连续重启次数。 */
        g_run_manager.restart_exhausted = 0U; /* 清除自动重启耗尽标志。 */
        g_run_manager.stop_reason = BLDC_STOP_REASON_NONE; /* 标记本轮软故障已经恢复。 */
    } /* 自动恢复成功处理结束。 */
    if (g_run_manager.run_command == 0U) /* 检查用户是否请求停止。 */
    { /* 用户停止命令处理开始。 */
        if (BLDC_IsPowerState() != 0U)  /* 检查桥臂当前是否仍在输出功率。 */
        { /* 运行中软停机处理开始。 */
            g_run_manager.stop_pending = 1U; /* 标记电流目标正在向零斜坡。 */
            if (g_current_control.ramped_target_ma <= 0L) /* 检查电流目标是否已经平滑降到零。 */
            { /* 软停机斜坡完成处理开始。 */
                BLDC_CompleteStop(BLDC_STOP_REASON_USER); /* 关闭全部输出并进入停止状态。 */
            } /* 软停机斜坡完成处理结束。 */
        } /* 运行中软停机处理结束。 */
        return;                         /* 停止命令下不执行启动或自动恢复。 */
    } /* 用户停止命令处理结束。 */
    if (g_bldc.state == BLDC_STATE_CALIBRATE) /* 检查上电电流零点校准是否仍在进行。 */
    { /* 电流零点校准等待开始。 */
        return;                         /* 等待快速回调完成零点校准。 */
    } /* 电流零点校准等待结束。 */
    if (g_bldc.state == BLDC_STATE_STOP) /* 检查是否已经具备发起全新启动的状态。 */
    { /* 全新启动请求处理开始。 */
        if (g_current.calibrated == 0U) /* 检查电流零点校准是否有效。 */
        { /* 电流零点无效处理开始。 */
            return;                     /* 禁止在电流零点无效时打开桥臂。 */
        } /* 电流零点无效处理结束。 */
        if (BLDC_IsBusVoltageReady() == 0U) /* 检查启动前母线电压是否处于安全范围。 */
        { /* 启动母线电压异常处理开始。 */
            if (g_bus_voltage.measured_mv < BLDC_VBUS_LOW_LIMIT_MV) /* 判断当前是否为欠压。 */
            { /* 启动欠压处理开始。 */
                BLDC_EnterFault(BLDC_FAULT_UNDERVOLTAGE); /* 锁存启动欠压硬故障。 */
            } /* 启动欠压处理结束。 */
            else if (g_bus_voltage.measured_mv > BLDC_VBUS_HIGH_LIMIT_MV) /* 检查是否已经超过三十一伏硬过压阈值。 */
            { /* 启动硬过压处理开始。 */
                BLDC_EnterFault(BLDC_FAULT_OVERVOLTAGE); /* 锁存启动过压硬故障。 */
            } /* 启动硬过压处理结束。 */
            else                        /* 处理软过压回差区、回充保护锁定或尚未得到有效采样。 */
            { /* 启动保护等待开始。 */
                __NOP();                /* 保持桥臂关闭并等待电压恢复或用户重新发送命令。 */
            } /* 启动保护等待结束。 */
            return;                     /* 电压异常时不发起转子定位。 */
        } /* 启动母线电压异常处理结束。 */
        BLDC_BeginStartup();            /* 从转子定位开始发起一次启动流程。 */
        return;                         /* 本周期启动处理完成。 */
    } /* 全新启动请求处理结束。 */
    if (g_bldc.state == BLDC_STATE_FAULT) /* 检查是否正在等待软故障自动恢复。 */
    { /* 故障状态后台处理开始。 */
        if (BLDC_IsRecoverableFault(g_bldc.fault) == 0U) /* 检查当前故障是否允许自动恢复。 */
        { /* 硬故障锁存处理开始。 */
            return;                     /* 保持桥臂关闭并等待人工检查。 */
        } /* 硬故障锁存处理结束。 */
        if (g_run_manager.restart_count >= BLDC_RESTART_MAX_COUNT) /* 检查软故障重试次数是否耗尽。 */
        { /* 自动重试耗尽处理开始。 */
            g_run_manager.restart_exhausted = 1U; /* 标记本轮不能继续自动重启。 */
            g_run_manager.stop_reason = BLDC_STOP_REASON_RETRY_EXHAUSTED; /* 保存重试耗尽停机原因。 */
            return;                     /* 保持故障状态和全部桥臂关闭。 */
        } /* 自动重试耗尽处理结束。 */
        if ((int32_t)(now_ms - g_run_manager.restart_due_ms) < 0L) /* 检查二百五十毫秒冷却等待是否结束。 */
        { /* 自动重启尚未到期处理开始。 */
            return;                     /* 等待功率级和电机电流衰减。 */
        } /* 自动重启尚未到期处理结束。 */
        if (BLDC_IsBusVoltageReady() == 0U) /* 检查重启前母线电压是否仍然安全。 */
        { /* 自动重启电压异常处理开始。 */
            return;                     /* 电压恢复前不增加重试次数。 */
        } /* 自动重启电压异常处理结束。 */
        g_run_manager.restart_count++;  /* 记录即将发起的一次自动重启。 */
        BLDC_BeginStartup();            /* 使用逐次增强的启动占空比重新启动。 */
    } /* 故障状态后台处理结束。 */
} /* 启停和自动恢复后台管理结束。 */

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
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1); /* 硬件准备期间无条件保持 TIM1 主输出关闭。 */
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
    g_current.motor_current_ma = 0L;    /* 清零有效回流相电流幅值。 */
    g_current.signed_motor_current_ma = 0L; /* 清零带符号回流相电流。 */
    g_current.signed_filtered_current_ma = 0L; /* 清零带符号滤波电流。 */
    g_current.source_current_ma = 0L;   /* 清零源相带符号诊断电流。 */
    g_current.floating_current_ma = 0L; /* 清零悬空相带符号诊断电流。 */
    g_current.signed_positive_peak_ma = 0L; /* 清零带符号电流正峰值。 */
    g_current.signed_negative_peak_ma = 0L; /* 清零带符号电流负峰值。 */
    for (step_index = 0U; step_index < 6U; step_index++) /* 遍历六个换相扇区。 */
    { /* 单个换相扇区诊断清零开始。 */
        g_current.step_current_ma[step_index] = 0L; /* 清零本扇区滤波回流电流。 */
        g_current.step_signed_current_ma[step_index] = 0L; /* 清零本扇区滤波带符号电流。 */
    } /* 单个换相扇区诊断清零结束。 */
    g_current.sum_u = 0U;               /* 清零 U 相零点累计值。 */
    g_current.sum_v = 0U;               /* 清零 V 相零点累计值。 */
    g_current.sum_w = 0U;               /* 清零 W 相零点累计值。 */
    g_current.count_adc1 = 0U;          /* 清零 ADC1 校准采样计数。 */
    g_current.count_adc2 = 0U;          /* 清零 ADC2 校准采样计数。 */
    g_current.paired_sample_count = 0U; /* 清零三相成功配对样本数。 */
    g_current.sync_error_count = 0U;    /* 清零三相样本同步错误计数。 */
    g_current.blanked_sample_count = 0U; /* 清零电流换相消隐样本累计值。 */
    g_current.polarity_positive_count = 0U; /* 清零显著正电流样本累计值。 */
    g_current.polarity_error_count = 0U; /* 清零显著负电流样本累计值。 */
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
    if (g_bus_voltage.measured_mv > BLDC_VBUS_HIGH_LIMIT_MV) /* 无论桥臂是否关闭都检查母线是否高于三十一伏。 */
    { /* 全状态过压样本处理开始。 */
        g_bus_voltage.low_count = 0U;   /* 过压时清零连续欠压确认次数。 */
        if (immediate != 0U)            /* 检查是否正在执行启动前立即保护。 */
        { /* 启动前过压确认开始。 */
            g_bus_voltage.high_count = BLDC_VBUS_FAULT_CONFIRM_SAMPLES; /* 立即满足过压确认次数。 */
        } /* 启动前过压确认结束。 */
        else if (g_bus_voltage.high_count < BLDC_VBUS_FAULT_CONFIRM_SAMPLES) /* 检查过压计数是否尚未饱和。 */
        { /* 运行或滑行中过压计数开始。 */
            g_bus_voltage.high_count++; /* 增加连续过压确认次数。 */
        } /* 运行或滑行中过压计数结束。 */
        if (g_bus_voltage.high_count >= BLDC_VBUS_FAULT_CONFIRM_SAMPLES) /* 检查过压是否已经确认。 */
        { /* 全状态过压停机开始。 */
            BLDC_EnterFault(BLDC_FAULT_OVERVOLTAGE); /* 锁存过压故障并确保全部功率输出关闭。 */
        } /* 全状态过压停机结束。 */
        return;                         /* 过压样本不再进入普通功率状态检查。 */
    } /* 全状态过压样本处理结束。 */
    if (BLDC_IsPowerState() == 0U)      /* 检查当前是否没有功率输出。 */
    { /* 空闲状态电压保护处理开始。 */
        g_bus_voltage.low_count = 0U;   /* 空闲时不累计运行欠压确认次数。 */
        g_bus_voltage.high_count = 0U;  /* 空闲时不累计运行过压确认次数。 */
        return;                         /* 启动电压许可由运行管理器单独判断。 */
    } /* 空闲状态电压保护处理结束。 */
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
            (BLDC_IsPowerState() != 0U) && /* 只在功率输出期间将采样错误升级为停机故障。 */
            (g_bldc.state != BLDC_STATE_FAULT)) /* 检查是否尚未锁存其他故障。 */
        { /* ADC1 规则组故障停机开始。 */
            BLDC_EnterFault(BLDC_FAULT_VBUS_ADC); /* 锁存母线 ADC 故障并立即停机。 */
        } /* ADC1 规则组故障停机结束。 */
        return;                         /* 本周期不再使用无效母线电压更新阈值。 */
    } /* ADC1 规则组采样失败处理结束。 */
    BLDC_CheckBusVoltage(0U);           /* 连续三次确认运行中的欠压或过压。 */
    if (BLDC_IsPowerState() != 0U)       /* 检查当前是否需要更新运行积分阈值。 */
    { /* 动态积分阈值更新许可开始。 */
        BLDC_UpdateDynamicIntegralLimit(); /* 按最新电压和电角转速更新积分阈值。 */
    } /* 动态积分阈值更新许可结束。 */
} /* 十毫秒后台服务结束。 */

static void BLDC_AllOutputsOff(void)    /* 实现全部输出关闭。 */
{ /* 输出关闭开始。 */
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1); /* 不受 CCER 通道状态限制并立即关闭主输出。 */
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
    BLDC_CommutateSteps(reason, 1U);     /* 将普通换相统一交给指定步数实现。 */
} /* 前进一步结束。 */

static void BLDC_ResetSpeedEstimator(void) /* 实现多步转速估算器复位。 */
{ /* 多步转速估算器复位开始。 */
    g_bldc.raw_electrical_rpm = 0U;     /* 清零最近一个六步窗口的原始电角转速。 */
    g_bldc.electrical_rpm = 0U;         /* 清零提供给控制器的滤波电角转速。 */
    g_motion.signed_erpm = 0L;          /* 清零带方向符号的滤波电角转速。 */
    g_bldc.speed_period_sum_ticks = 0U; /* 清零尚未完成测速窗口的周期总和。 */
    g_bldc.speed_commutation_count = 0U; /* 清零尚未完成测速窗口的换相次数。 */
    g_bldc.speed_sample_count = 0U;     /* 清零已经生成的完整六步转速样本数量。 */
    g_bldc.speed_filter_q16 = 0U;       /* 清零带小数精度的转速滤波内部状态。 */
} /* 多步转速估算器复位结束。 */

static void BLDC_UpdateSpeedEstimator(uint32_t elapsed_ticks, uint8_t steps) /* 实现 VESC 风格多换相周期转速估算。 */
{ /* 多换相周期转速估算开始。 */
    uint64_t raw_speed_numerator;       /* 保存原始电角转速计算分子。 */
    uint64_t filter_target_q16;         /* 保存本次原始转速对应的定点滤波目标。 */
    int64_t filter_delta_q16;           /* 保存滤波目标与当前状态之间的带符号差值。 */
    int64_t filter_next_q16;            /* 保存完成十分之一修正后的滤波状态。 */
    uint32_t raw_speed_erpm;            /* 保存完整测速窗口计算出的原始电角转速。 */
    if (elapsed_ticks == 0U)            /* 检查本次换相周期是否有效。 */
    { /* 无效换相周期处理开始。 */
        return;                         /* 不让零周期进入转速估算。 */
    } /* 无效换相周期处理结束。 */
    if (steps != 1U)                   /* 检查本次是否为跨两步失步恢复。 */
    { /* 跨两步恢复测速处理开始。 */
        g_bldc.speed_period_sum_ticks = 0U; /* 丢弃恢复前尚未完成的测速周期总和。 */
        g_bldc.speed_commutation_count = 0U; /* 丢弃恢复前尚未完成的测速换相计数。 */
        return;                         /* 保留最后一次可信滤波转速并等待六次正常换相。 */
    } /* 跨两步恢复测速处理结束。 */
    g_bldc.speed_period_sum_ticks += elapsed_ticks; /* 累加本次正常换相实际经历的周期。 */
    g_bldc.speed_commutation_count++;  /* 累加本次正常换相事件。 */
    if (g_bldc.speed_commutation_count < BLDC_SPEED_AVERAGE_COMMUTATIONS) /* 检查完整六步测速窗口是否已经收齐。 */
    { /* 六步测速窗口未完成处理开始。 */
        return;                         /* 保留已有累计并等待后续换相。 */
    } /* 六步测速窗口未完成处理结束。 */
    raw_speed_numerator = (uint64_t)BLDC_PWM_FREQUENCY_HZ * /* 使用快速控制回调频率建立时间基准。 */
                          10ULL * /* 将单次六分之一电周期换算为每分钟电角转速。 */
                          (uint64_t)g_bldc.speed_commutation_count; /* 按本窗口实际换相数量扩展计算。 */
    raw_speed_erpm = (uint32_t)(raw_speed_numerator / /* 使用六次换相的总时间计算平均转速。 */
                                (uint64_t)g_bldc.speed_period_sum_ticks); /* 除以窗口累计周期得到原始 ERPM。 */
    g_bldc.raw_electrical_rpm = raw_speed_erpm; /* 保存未滤波的完整六步原始转速供实时监视。 */
    filter_target_q16 = (uint64_t)raw_speed_erpm << BLDC_SPEED_FILTER_Q_SHIFT; /* 将原始转速转换为带十六位小数的滤波目标。 */
    if (g_bldc.speed_sample_count == 0U) /* 检查是否为本次启动的第一个完整测速样本。 */
    { /* 首个转速样本处理开始。 */
        g_bldc.speed_filter_q16 = (uint32_t)filter_target_q16; /* 首次直接装入原始转速避免启动滤波延迟。 */
    } /* 首个转速样本处理结束。 */
    else                                /* 处理已经建立滤波状态的后续样本。 */
    { /* 后续转速低通滤波开始。 */
        filter_delta_q16 = (int64_t)filter_target_q16 - (int64_t)g_bldc.speed_filter_q16; /* 计算本次原始转速与滤波状态的差值。 */
        filter_next_q16 = (int64_t)g_bldc.speed_filter_q16 + /* 从当前滤波状态开始更新。 */
                          (filter_delta_q16 / BLDC_SPEED_FILTER_DIVISOR); /* 每个六步样本只修正差值的十分之一。 */
        if (filter_next_q16 < 0L)       /* 检查带符号滤波计算是否意外低于零。 */
        { /* 滤波下溢保护开始。 */
            filter_next_q16 = 0L;      /* 正向运行的电角转速最低限制为零。 */
        } /* 滤波下溢保护结束。 */
        g_bldc.speed_filter_q16 = (uint32_t)filter_next_q16; /* 保存带小数精度的新滤波状态。 */
    } /* 后续转速低通滤波结束。 */
    g_bldc.electrical_rpm = g_bldc.speed_filter_q16 >> BLDC_SPEED_FILTER_Q_SHIFT; /* 输出整数滤波电角转速供控制器使用。 */
    if (g_motion.direction >= 0)        /* 检查当前是否为正向换相方向。 */
    { /* 正向有符号转速更新开始。 */
        g_motion.signed_erpm = (int32_t)g_bldc.electrical_rpm; /* 正向运行直接使用正数滤波 ERPM。 */
    } /* 正向有符号转速更新结束。 */
    else                                /* 预留后续反向换相方向处理。 */
    { /* 反向有符号转速更新开始。 */
        g_motion.signed_erpm = -(int32_t)g_bldc.electrical_rpm; /* 反向运行将滤波 ERPM 表示为负数。 */
    } /* 反向有符号转速更新结束。 */
    if (g_bldc.speed_sample_count < UINT32_MAX) /* 检查转速样本累计计数是否即将溢出。 */
    { /* 转速样本累计计数开始。 */
        g_bldc.speed_sample_count++;   /* 记录一个完整六步测速样本。 */
    } /* 转速样本累计计数结束。 */
    g_bldc.speed_period_sum_ticks = 0U; /* 清零周期总和以开始下一个六步测速窗口。 */
    g_bldc.speed_commutation_count = 0U; /* 清零换相计数以开始下一个六步测速窗口。 */
} /* 多换相周期转速估算结束。 */

static void BLDC_UpdateMotionTracking(BLDC_CommReason_t reason, uint8_t steps) /* 实现方向位置和启动换相诊断更新。 */
{ /* 方向位置和启动换相诊断更新开始。 */
    int32_t signed_steps;               /* 保存按照当前方向添加的有符号换相步数。 */
    signed_steps = (int32_t)steps;      /* 将已经限制到一或二的步数转换为有符号计算值。 */
    if (g_motion.direction >= 0)        /* 检查当前是否使用正向换相方向。 */
    { /* 正向里程计更新开始。 */
        if (g_motion.tachometer > (INT32_MAX - signed_steps)) /* 检查正向累计换相步数是否即将上溢。 */
        { /* 正向里程计饱和处理开始。 */
            g_motion.tachometer = INT32_MAX; /* 将有符号里程计保持在最大正数。 */
        } /* 正向里程计饱和处理结束。 */
        else                            /* 处理有符号里程计仍有正向累计空间的情况。 */
        { /* 正向里程计正常累计开始。 */
            g_motion.tachometer += signed_steps; /* 按实际换相步数增加有符号里程计。 */
        } /* 正向里程计正常累计结束。 */
    } /* 正向里程计更新结束。 */
    else                                /* 预留后续反向换相方向处理。 */
    { /* 反向里程计更新开始。 */
        if (g_motion.tachometer < (INT32_MIN + signed_steps)) /* 检查反向累计换相步数是否即将下溢。 */
        { /* 反向里程计饱和处理开始。 */
            g_motion.tachometer = INT32_MIN; /* 将有符号里程计保持在最小负数。 */
        } /* 反向里程计饱和处理结束。 */
        else                            /* 处理有符号里程计仍有反向累计空间的情况。 */
        { /* 反向里程计正常累计开始。 */
            g_motion.tachometer -= signed_steps; /* 按实际换相步数减少有符号里程计。 */
        } /* 反向里程计正常累计结束。 */
    } /* 反向里程计更新结束。 */
    if (g_motion.tachometer_abs > (UINT32_MAX - (uint32_t)steps)) /* 检查绝对里程计是否即将上溢。 */
    { /* 绝对里程计饱和处理开始。 */
        g_motion.tachometer_abs = UINT32_MAX; /* 将绝对里程计保持在无符号最大值。 */
    } /* 绝对里程计饱和处理结束。 */
    else                                /* 处理绝对里程计仍有累计空间的情况。 */
    { /* 绝对里程计正常累计开始。 */
        g_motion.tachometer_abs += (uint32_t)steps; /* 按实际换相步数增加绝对里程计。 */
    } /* 绝对里程计正常累计结束。 */
    if (reason == BLDC_COMM_REASON_OPEN_LOOP) /* 检查本次是否为开环定时强制换相。 */
    { /* 开环换相粘性统计开始。 */
        if (g_motion.open_loop_commutations < UINT32_MAX) /* 检查开环换相累计值是否仍可增加。 */
        { /* 开环换相累计开始。 */
            g_motion.open_loop_commutations++; /* 记录最近一次启动的一个开环强制换相事件。 */
        } /* 开环换相累计结束。 */
    } /* 开环换相粘性统计结束。 */
    if ((g_bldc.state == BLDC_STATE_TRANSITION) && /* 检查换相发生时是否处于闭环接管状态。 */
        (reason == BLDC_COMM_REASON_BEMF)) /* 检查本次是否由反电动势积分成功触发。 */
    { /* 接管积分换相粘性统计开始。 */
        if (g_motion.transition_bemf_commutations < UINT32_MAX) /* 检查接管积分换相累计值是否仍可增加。 */
        { /* 接管积分换相累计开始。 */
            g_motion.transition_bemf_commutations++; /* 记录最近一次启动的一次接管积分换相成功。 */
        } /* 接管积分换相累计结束。 */
    } /* 接管积分换相粘性统计结束。 */
    if ((g_bldc.state == BLDC_STATE_TRANSITION) && /* 检查换相发生时是否处于闭环接管状态。 */
        (reason == BLDC_COMM_REASON_TIMEOUT)) /* 检查本次是否为接管超时兜底换相。 */
    { /* 接管超时换相粘性统计开始。 */
        if (g_motion.transition_forced_commutations < UINT32_MAX) /* 检查接管超时换相累计值是否仍可增加。 */
        { /* 接管超时换相累计开始。 */
            g_motion.transition_forced_commutations++; /* 记录最近一次启动的一次接管超时换相。 */
        } /* 接管超时换相累计结束。 */
    } /* 接管超时换相粘性统计结束。 */
} /* 方向位置和启动换相诊断更新结束。 */

static void BLDC_CommutateSteps(BLDC_CommReason_t reason, uint8_t steps) /* 实现带触发原因的指定步数换相。 */
{ /* 指定步数换相开始。 */
    uint8_t step_before;                /* 保存换相前的步号。 */
    uint8_t next_step;                  /* 保存指定步数后的目标步号。 */
    uint32_t elapsed_ticks;             /* 保存本步运行周期。 */
    if ((steps < 1U) || (steps > 2U))   /* 检查调用者给出的换相步数是否在安全范围。 */
    { /* 非法换相步数处理开始。 */
        steps = 1U;                     /* 非法参数退回普通前进一步。 */
    } /* 非法换相步数处理结束。 */
    elapsed_ticks = g_bldc.commutation_ticks; /* 读取本步周期。 */
    step_before = g_bldc.step;          /* 读取换相前的步号。 */
    if (g_motion.direction >= 0)        /* 检查当前是否使用正转换相顺序。 */
    { /* 正转步号计算开始。 */
        next_step = (uint8_t)(g_bldc.step + steps); /* 正转按步号递增计算目标功率状态。 */
        while (next_step > BLDC_VALID_STEP_MAX) /* 检查是否超过第六步并允许两步回绕。 */
        { /* 正转步号回绕开始。 */
            next_step = (uint8_t)(next_step - BLDC_VALID_STEP_MAX); /* 按六步周期回绕到有效步号。 */
        } /* 正转步号回绕结束。 */
    } /* 正转步号计算结束。 */
    else                                /* 处理反转换相顺序。 */
    { /* 反转步号计算开始。 */
        if (g_bldc.step > steps)        /* 检查递减后是否仍在第一步以上。 */
        { /* 反转普通递减开始。 */
            next_step = (uint8_t)(g_bldc.step - steps); /* 反转按步号递减计算目标功率状态。 */
        } /* 反转普通递减结束。 */
        else                            /* 处理递减跨过第一步的情况。 */
        { /* 反转步号回绕开始。 */
            next_step = (uint8_t)(g_bldc.step + BLDC_VALID_STEP_MAX - steps); /* 从第一步向后回绕到第六步或第五步。 */
        } /* 反转步号回绕结束。 */
    } /* 反转步号计算结束。 */
    BLDC_ApplyStep(next_step);          /* 应用下一换相步。 */
    if (g_bldc.state == BLDC_STATE_FAULT) /* 检查换步故障。 */
    { /* 换步故障处理开始。 */
        return;                         /* 保持故障并返回。 */
    } /* 换步故障处理结束。 */
    BLDC_RecordCommutation(reason, step_before, next_step, elapsed_ticks); /* 保存本次成功换相的完整快照。 */
    BLDC_UpdateMotionTracking(reason, steps); /* 按实际步数同步更新方向、里程计和本次启动换相统计。 */
    if (elapsed_ticks > 0U)             /* 检查测速周期有效。 */
    { /* 速度更新开始。 */
        g_bldc.last_commutation_ticks = elapsed_ticks; /* 保存换步周期。 */
        BLDC_UpdateSpeedEstimator(elapsed_ticks, steps); /* 使用完整六步窗口更新原始和滤波电角转速。 */
        g_bldc.timeout_ticks = BLDC_GetTimeoutTicks(elapsed_ticks); /* 更新闭环超时。 */
    } /* 速度更新结束。 */
    g_bldc.commutation_ticks = 0U;      /* 清零本步计时。 */
    g_bldc.bemf_integral = 0U;          /* 清零本步积分。 */
    g_bldc.blanking_ticks = BLDC_BLANKING_TICKS; /* 设置换步消隐。 */
} /* 指定步数换相结束。 */

static void BLDC_AccumulateBemf(void)   /* 实现已经按方向归一化的反电动势积分。 */
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
                g_motion.open_loop_enter_ms = HAL_GetTick(); /* 粘性记录本次开环加速的精确进入时刻。 */
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
                    g_motion.transition_enter_ms = HAL_GetTick(); /* 粘性记录本次闭环接管的精确进入时刻。 */
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
                    g_motion.integrate_enter_ms = HAL_GetTick(); /* 粘性记录本次纯积分闭环的精确进入时刻。 */
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
                if (g_bldc.two_step_recovery_count < UINT16_MAX) /* 检查跨两步恢复累计计数是否即将溢出。 */
                { /* 跨两步恢复计数开始。 */
                    g_bldc.two_step_recovery_count++; /* 记录本次仿照 VESC 的跨两步恢复尝试。 */
                } /* 跨两步恢复计数结束。 */
                BLDC_CommutateSteps(BLDC_COMM_REASON_TIMEOUT, 2U); /* 跨过一个可能错误的扇区重新寻找有效转矩。 */
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
    BLDC_AllOutputsOff();               /* 校准结束后继续保持全部桥臂关闭。 */
    g_bldc.state_ticks = 0U;            /* 清零等待启动命令的状态计时。 */
    g_bldc.commutation_ticks = 0U;      /* 清零等待启动命令的换相计时。 */
    g_bldc.state = BLDC_STATE_STOP;     /* 校准完成后进入等待显式启动命令状态。 */
    g_run_manager.last_state = (uint8_t)BLDC_STATE_STOP; /* 同步后台管理器观察状态。 */
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
    int32_t normalized_u_ma;            /* 保存应用采样极性后的 U 相带符号电流。 */
    int32_t normalized_v_ma;            /* 保存应用采样极性后的 V 相带符号电流。 */
    int32_t normalized_w_ma;            /* 保存应用采样极性后的 W 相带符号电流。 */
    int32_t overcurrent_value_ma;       /* 保存用于快速保护的三相最大电流。 */
    int32_t step_filter_difference;     /* 保存当前扇区诊断滤波差值。 */
    int32_t step_signed_filter_difference; /* 保存当前扇区带符号诊断滤波差值。 */
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
    normalized_u_ma = g_current.current_u_ma * (int32_t)g_current_polarity.u; /* 应用 U 相采样方向归一化系数。 */
    normalized_v_ma = g_current.current_v_ma * (int32_t)g_current_polarity.v; /* 应用 V 相采样方向归一化系数。 */
    normalized_w_ma = g_current.current_w_ma * (int32_t)g_current_polarity.w; /* 应用 W 相采样方向归一化系数。 */
    absolute_u_ma = BLDC_AbsoluteCurrentMa(g_current.current_u_ma); /* 计算 U 相电流幅值。 */
    absolute_v_ma = BLDC_AbsoluteCurrentMa(g_current.current_v_ma); /* 计算 V 相电流幅值。 */
    absolute_w_ma = BLDC_AbsoluteCurrentMa(g_current.current_w_ma); /* 计算 W 相电流幅值。 */
    switch (g_bldc.step)               /* 根据非同步六步状态选择持续导通的回流相。 */
    { /* 回流相电流选择开始。 */
        case 1U:                        /* 第 1 步为 U+、V-、W 悬空。 */
            g_current.source_current_ma = normalized_u_ma; /* 保存 U 源相带符号诊断电流。 */
            g_current.signed_motor_current_ma = normalized_v_ma; /* 使用 V 回流相建立带符号电机电流。 */
            g_current.floating_current_ma = normalized_w_ma; /* 保存 W 悬空相带符号诊断电流。 */
            break;                      /* 结束第 1 步电流选择。 */
        case 2U:                        /* 第 2 步为 U+、W-、V 悬空。 */
            g_current.source_current_ma = normalized_u_ma; /* 保存 U 源相带符号诊断电流。 */
            g_current.signed_motor_current_ma = normalized_w_ma; /* 使用 W 回流相建立带符号电机电流。 */
            g_current.floating_current_ma = normalized_v_ma; /* 保存 V 悬空相带符号诊断电流。 */
            break;                      /* 结束第 2 步电流选择。 */
        case 3U:                        /* 第 3 步为 V+、W-、U 悬空。 */
            g_current.source_current_ma = normalized_v_ma; /* 保存 V 源相带符号诊断电流。 */
            g_current.signed_motor_current_ma = normalized_w_ma; /* 使用 W 回流相建立带符号电机电流。 */
            g_current.floating_current_ma = normalized_u_ma; /* 保存 U 悬空相带符号诊断电流。 */
            break;                      /* 结束第 3 步电流选择。 */
        case 4U:                        /* 第 4 步为 V+、U-、W 悬空。 */
            g_current.source_current_ma = normalized_v_ma; /* 保存 V 源相带符号诊断电流。 */
            g_current.signed_motor_current_ma = normalized_u_ma; /* 使用 U 回流相建立带符号电机电流。 */
            g_current.floating_current_ma = normalized_w_ma; /* 保存 W 悬空相带符号诊断电流。 */
            break;                      /* 结束第 4 步电流选择。 */
        case 5U:                        /* 第 5 步为 W+、U-、V 悬空。 */
            g_current.source_current_ma = normalized_w_ma; /* 保存 W 源相带符号诊断电流。 */
            g_current.signed_motor_current_ma = normalized_u_ma; /* 使用 U 回流相建立带符号电机电流。 */
            g_current.floating_current_ma = normalized_v_ma; /* 保存 V 悬空相带符号诊断电流。 */
            break;                      /* 结束第 5 步电流选择。 */
        case 6U:                        /* 第 6 步为 W+、V-、U 悬空。 */
            g_current.source_current_ma = normalized_w_ma; /* 保存 W 源相带符号诊断电流。 */
            g_current.signed_motor_current_ma = normalized_v_ma; /* 使用 V 回流相建立带符号电机电流。 */
            g_current.floating_current_ma = normalized_u_ma; /* 保存 U 悬空相带符号诊断电流。 */
            break;                      /* 结束第 6 步电流选择。 */
        default:                        /* 处理理论不可达的无效步号。 */
            g_current.motor_current_ma = 0L; /* 无效步号不输出电流值。 */
            g_current.signed_motor_current_ma = 0L; /* 无效步号不输出带符号电机电流。 */
            g_current.source_current_ma = 0L; /* 无效步号不输出源相诊断电流。 */
            g_current.floating_current_ma = 0L; /* 无效步号不输出悬空相诊断值。 */
            return;                     /* 结束本次电流保护计算。 */
    } /* 回流相电流选择结束。 */
    g_current.motor_current_ma = BLDC_AbsoluteCurrentMa(g_current.signed_motor_current_ma); /* 保持现有电流 PI 使用非负回流电流幅值。 */
    if (g_current.blanking_samples > 0U) /* 检查是否处于换相后的电流消隐窗口。 */
    { /* 电流换相消隐开始。 */
        g_current.blanking_samples--;   /* 消耗一个电流消隐样本。 */
        g_current.blanked_sample_count++; /* 累加被消隐的电流控制样本数。 */
        g_current.overcurrent_count = 0U; /* 消隐期间不保留软件过流连续计数。 */
        return;                         /* 本样本不进入电流 PI 和软件过流判断。 */
    } /* 电流换相消隐结束。 */
    if (g_current.signed_motor_current_ma > g_current.signed_positive_peak_ma) /* 检查是否出现更大的正向电流峰值。 */
    { /* 正向电流峰值更新开始。 */
        g_current.signed_positive_peak_ma = g_current.signed_motor_current_ma; /* 保存本次启动最新正峰值。 */
    } /* 正向电流峰值更新结束。 */
    if (g_current.signed_motor_current_ma < g_current.signed_negative_peak_ma) /* 检查是否出现更小的负向电流峰值。 */
    { /* 负向电流峰值更新开始。 */
        g_current.signed_negative_peak_ma = g_current.signed_motor_current_ma; /* 保存本次启动最新负峰值。 */
    } /* 负向电流峰值更新结束。 */
    if (g_current.signed_motor_current_ma >= BLDC_CURRENT_POLARITY_THRESHOLD_MA) /* 检查正常驱动电流是否明显为正。 */
    { /* 正极性有效样本累计开始。 */
        if (g_current.polarity_positive_count < UINT32_MAX) /* 检查正极性样本计数是否尚未饱和。 */
        { /* 正极性样本计数递增开始。 */
            g_current.polarity_positive_count++; /* 累加一个正常驱动显著正电流样本。 */
        } /* 正极性样本计数递增结束。 */
    } /* 正极性有效样本累计结束。 */
    else if (g_current.signed_motor_current_ma <= -BLDC_CURRENT_POLARITY_THRESHOLD_MA) /* 检查正常驱动电流是否出现显著负值。 */
    { /* 负极性异常样本累计开始。 */
        if (g_current.polarity_error_count < UINT32_MAX) /* 检查负极性样本计数是否尚未饱和。 */
        { /* 负极性样本计数递增开始。 */
            g_current.polarity_error_count++; /* 累加一个用于实机判定采样方向的显著负电流样本。 */
        } /* 负极性样本计数递增结束。 */
    } /* 负极性异常样本累计结束。 */
    else                                /* 处理接近零点且不足以判断极性的电流样本。 */
    { /* 极性死区样本处理开始。 */
        __NOP();                        /* 忽略零点噪声附近样本并保持两个计数器不变。 */
    } /* 极性死区样本处理结束。 */
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
    if (g_current.step_signed_current_ma[step_index] == 0L) /* 检查本扇区是否首次得到带符号有效电流。 */
    { /* 扇区带符号诊断电流初始化开始。 */
        g_current.step_signed_current_ma[step_index] = g_current.signed_motor_current_ma; /* 使用首个有效样本初始化带符号扇区滤波值。 */
    } /* 扇区带符号诊断电流初始化结束。 */
    else                                /* 处理本扇区后续带符号有效电流样本。 */
    { /* 扇区带符号诊断电流滤波开始。 */
        step_signed_filter_difference = g_current.signed_motor_current_ma - g_current.step_signed_current_ma[step_index]; /* 计算扇区带符号滤波差值。 */
        g_current.step_signed_current_ma[step_index] += step_signed_filter_difference >> BLDC_CURRENT_STEP_FILTER_SHIFT; /* 以三十二分之一系数更新扇区带符号电流。 */
    } /* 扇区带符号诊断电流滤波结束。 */
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
    int32_t active_target_ma;          /* 保存考虑启停命令后的实际电流目标。 */
    int32_t tentative_duty;            /* 保存限幅前的 PI 占空比。 */
    int32_t integral_minimum;          /* 保存积分项下限。 */
    int32_t integral_maximum;          /* 保存积分项上限。 */
    int32_t duty_minimum;               /* 保存统一管理器配置的最小占空比计数。 */
    int32_t duty_maximum;               /* 保存统一管理器配置的最大占空比计数。 */
    measured_current_ma = g_current.motor_current_ma; /* 读取当前回流相电流。 */
    duty_minimum = (int32_t)g_output_limit.duty_min_counts; /* 读取运行时最小占空比配置。 */
    duty_maximum = (int32_t)g_output_limit.duty_max_counts; /* 读取运行时最大占空比配置。 */
    active_target_ma = g_current_control.target_ma; /* 默认使用用户设置的运行目标电流。 */
    if (g_run_manager.run_command == 0U) /* 检查用户是否已经请求软停机。 */
    { /* 软停机电流目标处理开始。 */
        active_target_ma = 0L;          /* 将内部实际目标改为零并保留用户运行设定。 */
    } /* 软停机电流目标处理结束。 */
    else if (((g_bldc.state != BLDC_STATE_INTEGRATE) || /* 检查是否仍处于定位、开环或接管阶段。 */
              (g_bldc.state_ticks < BLDC_CLOSED_LOOP_CURRENT_HOLD_TICKS)) && /* 检查闭环接管后的三百毫秒保持时间。 */
             (active_target_ma < BLDC_STARTUP_CURRENT_MIN_MA)) /* 检查用户目标是否低于可靠启动电流。 */
    { /* 启动和接管电流补偿开始。 */
        active_target_ma = BLDC_STARTUP_CURRENT_MIN_MA; /* 启动阶段至少使用五百毫安建立可靠反电动势。 */
    } /* 启动和接管电流补偿结束。 */
    if (measured_current_ma < 0L)      /* 检查换相尖峰或零点噪声造成的负值。 */
    { /* 负电流处理开始。 */
        measured_current_ma = 0L;      /* 正向驱动阶段不让负值错误地增大占空比。 */
    } /* 负电流处理结束。 */
    if (g_current_control.filter_initialized == 0U) /* 检查滤波器是否第一次得到有效电流。 */
    { /* 滤波器初始化开始。 */
        g_current_control.filtered_current_ma = measured_current_ma; /* 直接装入首个测量值以避免启动延迟。 */
        g_current.signed_filtered_current_ma = g_current.signed_motor_current_ma; /* 同步初始化独立带符号电流低通值。 */
        g_current_control.filter_initialized = 1U; /* 标记滤波器已经初始化。 */
    } /* 滤波器初始化结束。 */
    else                                /* 处理后续连续电流采样。 */
    { /* 一阶低通滤波开始。 */
        g_current_control.filtered_current_ma += /* 累加本次滤波修正量。 */
            (measured_current_ma - g_current_control.filtered_current_ma) >> BLDC_CURRENT_FILTER_SHIFT; /* 使用十六分之一系数抑制换相噪声。 */
        g_current.signed_filtered_current_ma += /* 累加带符号电流的本次滤波修正量。 */
            (g_current.signed_motor_current_ma - g_current.signed_filtered_current_ma) >> BLDC_CURRENT_FILTER_SHIFT; /* 使用同一系数建立不参与现有 PI 的带符号低通值。 */
    } /* 一阶低通滤波结束。 */
    if (g_current_control.ramped_target_ma < active_target_ma) /* 检查目标电流是否需要向上爬升。 */
    { /* 目标电流上升斜坡开始。 */
        g_current_control.ramped_target_ma += BLDC_CURRENT_RAMP_STEP_MA; /* 每个采样周期增加固定电流。 */
        if (g_current_control.ramped_target_ma > active_target_ma) /* 检查斜坡是否越过最终目标。 */
        { /* 上升斜坡限幅开始。 */
            g_current_control.ramped_target_ma = active_target_ma; /* 精确停在当前实际目标。 */
        } /* 上升斜坡限幅结束。 */
    } /* 目标电流上升斜坡结束。 */
    else if (g_current_control.ramped_target_ma > active_target_ma) /* 检查目标电流是否需要向下变化。 */
    { /* 目标电流下降斜坡开始。 */
        g_current_control.ramped_target_ma -= BLDC_CURRENT_RAMP_STEP_MA; /* 每个采样周期降低固定电流。 */
        if (g_current_control.ramped_target_ma < active_target_ma) /* 检查下降斜坡是否越过最终目标。 */
        { /* 下降斜坡限幅开始。 */
            g_current_control.ramped_target_ma = active_target_ma; /* 精确停在当前实际目标。 */
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
    if (!(((tentative_duty >= duty_maximum) && (g_current_control.error_ma > 0L)) || /* 检查运行时上限方向积分饱和。 */
          ((tentative_duty <= duty_minimum) && (g_current_control.error_ma < 0L)))) /* 检查运行时下限方向积分饱和。 */
    { /* 条件积分开始。 */
        g_current_control.integral_accumulator += g_current_control.error_ma; /* 仅在不会加重饱和时积分。 */
    } /* 条件积分结束。 */
    integral_minimum = duty_minimum * BLDC_CURRENT_KI_DIVISOR; /* 根据运行时最小占空比计算积分器下限。 */
    integral_maximum = duty_maximum * BLDC_CURRENT_KI_DIVISOR; /* 根据运行时最大占空比计算积分器上限。 */
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
    if (tentative_duty < duty_minimum) /* 检查最终输出运行时下限。 */
    { /* 输出下限处理开始。 */
        tentative_duty = duty_minimum; /* 应用统一管理器配置的最小运行占空比。 */
    } /* 输出下限处理结束。 */
    if (tentative_duty > duty_maximum) /* 检查最终输出运行时上限。 */
    { /* 输出上限处理开始。 */
        tentative_duty = duty_maximum; /* 应用统一管理器配置的最大测试占空比。 */
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
    g_run_manager.last_fault = fault;   /* 保留最近一次故障供自动恢复后继续诊断。 */
    g_run_manager.stop_pending = 0U;    /* 故障停机不再执行电流斜坡。 */
    if ((BLDC_IsRecoverableFault(fault) != 0U) && /* 检查是否为允许自动恢复的软故障。 */
        (g_run_manager.run_command != 0U)) /* 检查用户是否仍然请求电机运行。 */
    { /* 软故障恢复调度开始。 */
        g_run_manager.stop_reason = BLDC_STOP_REASON_SOFT_FAULT_RETRY; /* 标记正在等待软故障重试。 */
        g_run_manager.restart_due_ms = HAL_GetTick() + BLDC_RESTART_DELAY_MS; /* 安排二百五十毫秒后的重启时刻。 */
    } /* 软故障恢复调度结束。 */
    else                                /* 处理硬故障或用户已经撤销运行命令。 */
    { /* 硬故障锁存处理开始。 */
        g_control_command.mode = BLDC_CONTROL_STOP; /* 硬故障时同步撤销上层控制模式。 */
        g_control_command.output_current_ma = 0L; /* 硬故障时清除上层输出电流诊断值。 */
        g_current_command.requested_ma = 0L; /* 硬故障时立即撤销内部运行电流请求。 */
        g_run_manager.run_command = 0U; /* 撤销运行命令避免后台意外重启。 */
        g_run_manager.stop_reason = BLDC_STOP_REASON_HARD_FAULT; /* 标记必须人工检查的停机原因。 */
    } /* 硬故障锁存处理结束。 */
} /* 故障停机结束。 */

void BLDC_Start(void)                  /* 实现用户运行命令接口。 */
{ /* 用户运行命令接口开始。 */
    if (g_run_manager.initialized == 0U) /* 检查底层初始化是否已经完成。 */
    { /* 尚未初始化处理开始。 */
        return;                         /* 初始化前拒绝打开功率输出。 */
    } /* 尚未初始化处理结束。 */
    if (g_control_command.mode == BLDC_CONTROL_STOP) /* 检查上层是否已经给出有效运行模式。 */
    { /* 停止模式启动拒绝开始。 */
        return;                         /* 没有电流、占空比或速度命令时不允许单独启动。 */
    } /* 停止模式启动拒绝结束。 */
    if ((g_energy_protection.restart_inhibited != 0U) || /* 检查回充或软过压保护是否仍禁止重新通电。 */
        (g_bus_voltage.measured_mv >= BLDC_VBUS_SOFT_DERATE_START_MV)) /* 检查母线是否仍处于不允许新启动的软过压区域。 */
    { /* 能量保护启动拒绝开始。 */
        g_control_command.mode = BLDC_CONTROL_STOP; /* 丢弃保护期间刚写入的上层运行模式。 */
        g_control_command.current_command_ma = 0L; /* 清除保护期间写入的电流命令。 */
        g_control_command.duty_command_permille = 0U; /* 清除保护期间写入的占空比命令。 */
        g_control_command.duty_target_counts = 0U; /* 清除保护期间写入的内部占空比目标。 */
        g_control_command.speed_command_erpm = 0U; /* 清除保护期间写入的速度命令。 */
        g_control_command.output_current_ma = 0L; /* 清除保护期间产生的外环输出诊断。 */
        g_current_command.requested_ma = 0L; /* 禁止保护解除后旧命令自行恢复。 */
        g_run_manager.run_command = 0U; /* 保持运行管理器处于停止请求状态。 */
        return;                         /* 保持桥臂关闭并要求电压恢复后重新发送运行命令。 */
    } /* 能量保护启动拒绝结束。 */
    if ((g_bldc.state == BLDC_STATE_FAULT) && /* 检查当前是否锁存故障。 */
        (BLDC_IsRecoverableFault(g_bldc.fault) == 0U)) /* 检查该故障是否属于硬故障。 */
    { /* 硬故障启动拒绝开始。 */
        return;                         /* 硬故障必须重新检查并重新初始化后才能启动。 */
    } /* 硬故障启动拒绝结束。 */
    if ((g_bldc.state == BLDC_STATE_FAULT) && /* 检查用户是否正在手动重试软故障。 */
        (BLDC_IsRecoverableFault(g_bldc.fault) != 0U)) /* 再次确认故障允许恢复。 */
    { /* 手动恢复软故障开始。 */
        g_run_manager.restart_count = 0U; /* 手动启动重新获得三次自动尝试机会。 */
        g_run_manager.restart_exhausted = 0U; /* 清除上一次重试耗尽标志。 */
        g_run_manager.restart_due_ms = HAL_GetTick(); /* 允许下一次后台处理立即启动。 */
    } /* 手动恢复软故障结束。 */
    g_run_manager.run_command = 1U;     /* 保存用户请求运行命令。 */
    g_run_manager.stop_pending = 0U;    /* 取消尚未完成的软停机等待。 */
    g_run_manager.stop_reason = BLDC_STOP_REASON_NONE; /* 清除普通启动前的停机原因。 */
} /* 用户运行命令接口结束。 */

void BLDC_Stop(void)                   /* 实现用户软停机命令接口。 */
{ /* 用户软停机命令接口开始。 */
    BLDC_CancelReversal();             /* 用户明确停机时取消尚未完成的自动换向和待恢复命令。 */
    g_control_command.mode = BLDC_CONTROL_STOP; /* 将统一上层控制器切换到停止模式。 */
    g_control_command.current_command_ma = 0L; /* 清除电流模式命令。 */
    g_control_command.duty_command_permille = 0U; /* 清除占空比模式命令。 */
    g_control_command.duty_target_counts = 0U; /* 清除内部占空比目标。 */
    g_control_command.speed_command_erpm = 0U; /* 清除速度模式命令。 */
    g_control_command.duty_error_counts = 0L; /* 清除占空比外环误差。 */
    g_control_command.speed_error_erpm = 0L; /* 清除速度外环误差。 */
    g_control_command.output_current_ma = 0L; /* 清除上层输出电流诊断值。 */
    g_current_command.requested_ma = 0L; /* 请求电流命令按下降斜率回到零。 */
    g_output_limit.requested_current_ma = 0L; /* 清除统一管理器的原始电流请求。 */
    g_output_limit.limited_current_ma = 0L; /* 清除统一管理器的限幅后电流。 */
    g_output_limit.requested_duty_permille = 0U; /* 清除统一管理器的原始占空比请求。 */
    g_output_limit.limited_duty_permille = 0U; /* 清除统一管理器的限幅后占空比。 */
    g_output_limit.requested_speed_erpm = 0U; /* 清除统一管理器的原始速度请求。 */
    g_output_limit.limited_speed_erpm = 0U; /* 清除统一管理器的限幅后速度。 */
    g_output_limit.command_reason = BLDC_LIMIT_NONE; /* 清除命令进入控制器前的限幅原因。 */
    g_output_limit.active_reason = BLDC_LIMIT_NONE; /* 清除运行过程中组合得到的限幅原因。 */
    g_output_limit.active = 0U;         /* 清除统一限幅活动标志。 */
    g_output_limit.current_saturated = 0U; /* 清除电机电流饱和标志。 */
    g_output_limit.duty_saturated = 0U; /* 清除占空比饱和标志。 */
    g_output_limit.speed_saturated = 0U; /* 清除速度饱和标志。 */
    g_run_manager.run_command = 0U;     /* 撤销运行命令并让电流目标向零下降。 */
    if (BLDC_IsPowerState() != 0U)      /* 检查电机当前是否正在输出功率。 */
    { /* 运行中软停机请求开始。 */
        g_run_manager.stop_pending = 1U; /* 标记后台等待电流斜坡到零。 */
    } /* 运行中软停机请求结束。 */
    else if ((g_bldc.state != BLDC_STATE_CALIBRATE) && /* 保留尚未完成的上电零点校准。 */
             (g_bldc.state != BLDC_STATE_FAULT)) /* 保留已经锁存的明确故障原因。 */
    { /* 空闲状态停机处理开始。 */
        BLDC_CompleteStop(BLDC_STOP_REASON_USER); /* 确保全部桥臂保持关闭。 */
    } /* 空闲状态停机处理结束。 */
} /* 用户软停机命令接口结束。 */

void BLDC_EmergencyStop(void)          /* 实现用户紧急停机接口。 */
{ /* 用户紧急停机接口开始。 */
    BLDC_CancelReversal();             /* 紧急停机优先取消尚未完成的自动换向和待恢复命令。 */
    g_control_command.mode = BLDC_CONTROL_STOP; /* 将统一上层控制器立即切换到停止模式。 */
    g_control_command.current_command_ma = 0L; /* 清除电流模式命令。 */
    g_control_command.duty_command_permille = 0U; /* 清除占空比模式命令。 */
    g_control_command.duty_target_counts = 0U; /* 清除内部占空比目标。 */
    g_control_command.speed_command_erpm = 0U; /* 清除速度模式命令。 */
    g_control_command.duty_error_counts = 0L; /* 清除占空比外环误差。 */
    g_control_command.speed_error_erpm = 0L; /* 清除速度外环误差。 */
    g_control_command.output_current_ma = 0L; /* 清除上层输出电流诊断值。 */
    g_current_command.requested_ma = 0L; /* 立即撤销内部运行电流请求。 */
    g_current_command.applied_ma = 0L; /* 紧急停机不等待命令下降斜坡。 */
    g_output_limit.requested_current_ma = 0L; /* 清除统一管理器的原始电流请求。 */
    g_output_limit.limited_current_ma = 0L; /* 清除统一管理器的限幅后电流。 */
    g_output_limit.requested_duty_permille = 0U; /* 清除统一管理器的原始占空比请求。 */
    g_output_limit.limited_duty_permille = 0U; /* 清除统一管理器的限幅后占空比。 */
    g_output_limit.requested_speed_erpm = 0U; /* 清除统一管理器的原始速度请求。 */
    g_output_limit.limited_speed_erpm = 0U; /* 清除统一管理器的限幅后速度。 */
    g_output_limit.command_reason = BLDC_LIMIT_NONE; /* 清除命令进入控制器前的限幅原因。 */
    g_output_limit.active_reason = BLDC_LIMIT_NONE; /* 清除运行过程中组合得到的限幅原因。 */
    g_output_limit.active = 0U;         /* 清除统一限幅活动标志。 */
    g_output_limit.current_saturated = 0U; /* 清除电机电流饱和标志。 */
    g_output_limit.duty_saturated = 0U; /* 清除占空比饱和标志。 */
    g_output_limit.speed_saturated = 0U; /* 清除速度饱和标志。 */
    g_run_manager.run_command = 0U;     /* 立即撤销全部运行和自动恢复请求。 */
    BLDC_AllOutputsOff();               /* 不等待电流斜坡并立即关闭全部桥臂。 */
    if ((g_bldc.state != BLDC_STATE_CALIBRATE) && /* 避免中断无功率的上电零点校准。 */
        (g_bldc.state != BLDC_STATE_FAULT)) /* 避免覆盖已经锁存的故障状态。 */
    { /* 普通运行紧急停机处理开始。 */
        BLDC_CompleteStop(BLDC_STOP_REASON_EMERGENCY); /* 清理运行数据并进入停止状态。 */
    } /* 普通运行紧急停机处理结束。 */
    else                                /* 处理校准或故障状态下的紧急停机。 */
    { /* 特殊状态紧急停机处理开始。 */
        g_run_manager.stop_pending = 0U; /* 清除可能遗留的软停机等待标志。 */
        g_run_manager.stop_reason = BLDC_STOP_REASON_EMERGENCY; /* 记录紧急停机命令。 */
    } /* 特殊状态紧急停机处理结束。 */
} /* 用户紧急停机接口结束。 */

uint8_t BLDC_IsRunning(void)           /* 实现积分闭环运行状态读取接口。 */
{ /* 积分闭环运行状态读取开始。 */
    if (g_bldc.state == BLDC_STATE_INTEGRATE) /* 检查电机是否已经完成无感闭环接管。 */
    { /* 积分闭环运行处理开始。 */
        return 1U;                      /* 返回电机已经处于积分闭环运行。 */
    } /* 积分闭环运行处理结束。 */
    return 0U;                          /* 返回电机尚未处于稳定积分闭环。 */
} /* 积分闭环运行状态读取结束。 */

uint8_t BLDC_GetRestartCount(void)     /* 实现连续自动重启次数读取接口。 */
{ /* 连续自动重启次数读取开始。 */
    return g_run_manager.restart_count; /* 返回当前连续软故障重启次数。 */
} /* 连续自动重启次数读取结束。 */

BLDC_Fault_t BLDC_GetFault(void)        /* 实现当前故障码读取接口。 */
{ /* 故障码读取开始。 */
    return g_bldc.fault;                /* 返回最近一次电机控制故障。 */
} /* 故障码读取结束。 */

void BLDC_SetRunCurrentMa(int32_t command_ma) /* 实现 VESC 风格运行电流命令接口。 */
{ /* 运行电流命令设置开始。 */
    int64_t command_magnitude;          /* 保存能够安全处理最小负整数的命令幅值。 */
    int32_t limited_command_ma;         /* 保存统一电流上限处理后的命令。 */
    int8_t direction;                   /* 保存电流命令符号对应的目标方向。 */
    direction = 1;                     /* 默认将非负电流命令解释为正转。 */
    command_magnitude = (int64_t)command_ma; /* 提升到六十四位避免最小负整数取绝对值溢出。 */
    if (command_magnitude < 0LL)        /* 检查用户是否通过负电流请求反转。 */
    { /* 负电流命令解析开始。 */
        direction = -1;                /* 将目标方向设置为反转。 */
        command_magnitude = -command_magnitude; /* 将控制幅值转换为非负毫安数。 */
    } /* 负电流命令解析结束。 */
    if (command_magnitude < BLDC_CURRENT_COMMAND_RELEASE_MA) /* 检查命令幅值是否低于电流模式释放阈值。 */
    { /* 零电流命令处理开始。 */
        BLDC_Stop();                    /* 通过原有电流降零和主输出关闭流程执行软停机。 */
        return;                         /* 停机命令处理完成后不再请求启动。 */
    } /* 零电流命令处理结束。 */
    if (command_magnitude > INT32_MAX)  /* 检查最小负整数对应的绝对值是否超过有符号范围。 */
    { /* 电流命令绝对范围处理开始。 */
        command_magnitude = INT32_MAX;  /* 限制到后续有符号电流计算能够表示的最大值。 */
    } /* 电流命令绝对范围处理结束。 */
    command_ma = (int32_t)command_magnitude; /* 将安全的非负幅值恢复为内部电流命令类型。 */
    if (BLDC_RequestSignedCommand(BLDC_CONTROL_CURRENT, direction, command_ma, 0U, 0U) != 0U) /* 检查是否需要先完成安全换向。 */
    { /* 电流命令等待换向处理开始。 */
        return;                         /* 命令已经保存在换向管理器中，暂不直接启动。 */
    } /* 电流命令等待换向处理结束。 */
    g_output_limit.command_reason = BLDC_LIMIT_NONE; /* 新电流命令首先清除上一模式的配置限幅原因。 */
    limited_command_ma = command_ma;    /* 默认不改变用户原始电流请求。 */
    if (limited_command_ma > BLDC_GetControlCurrentMaximumMa()) /* 检查考虑统一配置和过流余量后的电流上限。 */
    { /* 运行电流命令上限处理开始。 */
        limited_command_ma = BLDC_GetControlCurrentMaximumMa(); /* 限制到当前允许的最大电流目标。 */
        g_output_limit.command_reason = BLDC_LIMIT_MOTOR_CURRENT; /* 记录命令已经受到电机电流配置限制。 */
    } /* 运行电流命令上限处理结束。 */
    g_control_command.mode = BLDC_CONTROL_CURRENT; /* 将统一上层控制器切换到直接电流模式。 */
    g_control_command.current_command_ma = command_ma; /* 保存用户原始直接电流命令供统一管理器处理。 */
    g_control_command.output_current_ma = limited_command_ma; /* 立即同步限幅后的上层输出诊断值。 */
    g_output_limit.requested_current_ma = command_ma; /* 保存用户原始电流请求。 */
    g_output_limit.limited_current_ma = limited_command_ma; /* 保存配置限幅后的电流请求。 */
    g_current_command.requested_ma = limited_command_ma; /* 原子保存统一限幅后的运行电流。 */
    BLDC_Start();                       /* 有效非零电流命令自动请求启动或继续运行。 */
} /* 运行电流命令设置结束。 */

void BLDC_SetBrakeCurrentMa(uint32_t brake_current_ma) /* 实现禁止主动负转矩的制动请求接口。 */
{ /* 禁止主动负转矩的制动请求处理开始。 */
    if (brake_current_ma > 0U)          /* 检查上层是否确实请求了主动制动电流。 */
    { /* 非零主动制动请求拒绝开始。 */
        g_energy_protection.negative_torque_reject_count++; /* 记录一次因电池不能回充而拒绝的制动请求。 */
        g_energy_protection.last_event_ms = HAL_GetTick(); /* 保存最近一次负转矩请求发生时刻。 */
    } /* 非零主动制动请求拒绝结束。 */
    BLDC_Stop();                        /* 无论制动幅值大小都只执行减流后关闭桥臂的滑行停机。 */
} /* 禁止主动负转矩的制动请求处理结束。 */

void BLDC_SetSignedDutyPermille(int16_t duty_permille) /* 实现 VESC 风格有符号占空比命令接口。 */
{ /* 有符号占空比命令设置开始。 */
    int32_t duty_magnitude;             /* 保存提升到有符号三十二位后的占空比幅值。 */
    int8_t direction;                   /* 保存占空比命令符号对应的目标方向。 */
    direction = 1;                     /* 默认将非负占空比命令解释为正转。 */
    duty_magnitude = (int32_t)duty_permille; /* 提升类型以安全处理负三万二千七百六十八。 */
    if (duty_magnitude < 0L)            /* 检查用户是否通过负占空比请求反转。 */
    { /* 负占空比命令解析开始。 */
        direction = -1;                /* 将目标方向设置为反转。 */
        duty_magnitude = -duty_magnitude; /* 将控制幅值转换为非负千分比。 */
    } /* 负占空比命令解析结束。 */
    if (duty_magnitude < (int32_t)BLDC_DUTY_COMMAND_MIN_PERMILLE) /* 检查命令幅值是否低于最低有效值。 */
    { /* 零占空比命令处理开始。 */
        BLDC_Stop();                    /* 低于有效阈值时执行原有软停机流程。 */
        return;                         /* 停机命令处理完成后不再请求启动。 */
    } /* 零占空比命令处理结束。 */
    if (duty_magnitude > (int32_t)BLDC_DUTY_COMMAND_MAX_PERMILLE) /* 检查命令幅值是否超过软件绝对上限。 */
    { /* 有符号占空比绝对上限处理开始。 */
        duty_magnitude = (int32_t)BLDC_DUTY_COMMAND_MAX_PERMILLE; /* 先限制到安全范围再保存待换向命令。 */
    } /* 有符号占空比绝对上限处理结束。 */
    if (BLDC_RequestSignedCommand(BLDC_CONTROL_DUTY, direction, 0L, (uint16_t)duty_magnitude, 0U) != 0U) /* 检查是否需要先完成安全换向。 */
    { /* 占空比命令等待换向处理开始。 */
        return;                         /* 命令已经保存，等待减流和滑行完成后恢复。 */
    } /* 占空比命令等待换向处理结束。 */
    BLDC_SetDutyPermille((uint16_t)duty_magnitude); /* 方向一致时复用已经验证的幅值限幅和启动流程。 */
} /* 有符号占空比命令设置结束。 */

void BLDC_SetDutyPermille(uint16_t duty_permille) /* 实现当前方向占空比幅值模式接口。 */
{ /* 当前方向占空比幅值命令设置开始。 */
    uint32_t duty_target_counts;        /* 保存换算到内部一千二百分度的占空比目标。 */
    uint16_t requested_duty_permille;   /* 保存用户传入的原始占空比千分比。 */
    uint16_t configured_maximum_permille; /* 保存运行时配置允许的最大占空比千分比。 */
    requested_duty_permille = duty_permille; /* 在任何限幅前保存原始用户请求。 */
    if (duty_permille < BLDC_DUTY_COMMAND_MIN_PERMILLE) /* 检查占空比命令是否低于最低有效值。 */
    { /* 零占空比命令处理开始。 */
        BLDC_Stop();                    /* 将过低占空比统一解释为软停机命令。 */
        return;                         /* 停机命令处理完成后不再请求启动。 */
    } /* 零占空比命令处理结束。 */
    g_output_limit.command_reason = BLDC_LIMIT_NONE; /* 新占空比命令首先清除上一模式的配置限幅原因。 */
    configured_maximum_permille = (uint16_t)(((uint32_t)g_output_limit.duty_max_counts * BLDC_PERMILLE_SCALE) / BLDC_DUTY_SCALE); /* 将配置最大计数换算为千分比。 */
    if (configured_maximum_permille > BLDC_DUTY_COMMAND_MAX_PERMILLE) /* 检查运行时配置是否超过绝对安全上限。 */
    { /* 配置最大占空比绝对上限处理开始。 */
        configured_maximum_permille = BLDC_DUTY_COMMAND_MAX_PERMILLE; /* 限制到当前百分之五十绝对安全范围。 */
    } /* 配置最大占空比绝对上限处理结束。 */
    if (duty_permille > configured_maximum_permille) /* 检查占空比命令是否超过运行时配置上限。 */
    { /* 占空比命令上限处理开始。 */
        duty_permille = configured_maximum_permille; /* 限制到统一管理器当前允许的最大占空比。 */
        g_output_limit.command_reason = BLDC_LIMIT_DUTY; /* 记录用户占空比命令已经受到配置限制。 */
    } /* 占空比命令上限处理结束。 */
    if (g_control_command.mode != BLDC_CONTROL_DUTY) /* 检查是否正在从其他模式切换。 */
    { /* 占空比模式无扰切换准备开始。 */
        BLDC_PreloadControlIntegrators(); /* 使用当前运行电流预置占空比外环积分项。 */
        g_control_command.duty_error_counts = 0L; /* 清除旧的占空比误差诊断值。 */
    } /* 占空比模式无扰切换准备结束。 */
    duty_target_counts = ((uint32_t)duty_permille * BLDC_DUTY_SCALE) / BLDC_PERMILLE_SCALE; /* 将千分比换算为内部占空比计数。 */
    g_control_command.duty_command_permille = duty_permille; /* 保存用户占空比千分比命令。 */
    g_control_command.duty_target_counts = (uint16_t)duty_target_counts; /* 保存内部占空比目标。 */
    g_output_limit.requested_duty_permille = requested_duty_permille; /* 保存用户原始占空比请求供诊断。 */
    g_output_limit.limited_duty_permille = duty_permille; /* 保存配置限幅后的占空比目标。 */
    g_control_command.mode = BLDC_CONTROL_DUTY; /* 在全部参数有效后原子切换到占空比模式。 */
    g_current_command.requested_ma = g_control_command.output_current_ma; /* 立即保持模式切换前的电流避免命令跌落。 */
    BLDC_Start();                       /* 有效非零占空比命令自动请求启动或继续运行。 */
} /* 当前方向占空比幅值命令设置结束。 */

void BLDC_SetSignedSpeedErpm(int32_t speed_erpm) /* 实现 VESC 风格有符号电角转速命令接口。 */
{ /* 有符号电角转速命令设置开始。 */
    int64_t speed_magnitude;            /* 保存能够安全处理最小负整数的速度幅值。 */
    int8_t direction;                   /* 保存速度命令符号对应的目标方向。 */
    direction = 1;                     /* 默认将非负速度命令解释为正转。 */
    speed_magnitude = (int64_t)speed_erpm; /* 提升到六十四位避免最小负整数取绝对值溢出。 */
    if (speed_magnitude < 0LL)          /* 检查用户是否通过负速度请求反转。 */
    { /* 负速度命令解析开始。 */
        direction = -1;                /* 将目标方向设置为反转。 */
        speed_magnitude = -speed_magnitude; /* 将速度控制值转换为非负 ERPM。 */
    } /* 负速度命令解析结束。 */
    if (speed_magnitude < (int64_t)BLDC_SPEED_COMMAND_MIN_ERPM) /* 检查速度幅值是否低于无感稳定范围。 */
    { /* 零速度命令处理开始。 */
        BLDC_Stop();                    /* 低于无感速度下限时执行原有软停机流程。 */
        return;                         /* 停机命令处理完成后不再请求启动。 */
    } /* 零速度命令处理结束。 */
    if (speed_magnitude > (int64_t)BLDC_SPEED_COMMAND_MAX_ERPM) /* 检查速度幅值是否超过软件绝对上限。 */
    { /* 有符号速度绝对上限处理开始。 */
        speed_magnitude = (int64_t)BLDC_SPEED_COMMAND_MAX_ERPM; /* 先限制到安全范围再保存待换向命令。 */
    } /* 有符号速度绝对上限处理结束。 */
    if (BLDC_RequestSignedCommand(BLDC_CONTROL_SPEED, direction, 0L, 0U, (uint32_t)speed_magnitude) != 0U) /* 检查是否需要先完成安全换向。 */
    { /* 速度命令等待换向处理开始。 */
        return;                         /* 命令已经保存，等待减流和滑行完成后恢复。 */
    } /* 速度命令等待换向处理结束。 */
    BLDC_SetSpeedErpm((uint32_t)speed_magnitude); /* 方向一致时复用已经验证的速度限幅和启动流程。 */
} /* 有符号电角转速命令设置结束。 */

void BLDC_SetSpeedErpm(uint32_t speed_erpm) /* 实现当前方向电角转速幅值模式接口。 */
{ /* 当前方向电角转速幅值命令设置开始。 */
    uint32_t requested_speed_erpm;      /* 保存用户传入的原始电角转速请求。 */
    requested_speed_erpm = speed_erpm;  /* 在任何限幅前保存原始速度请求。 */
    if ((speed_erpm < BLDC_SPEED_COMMAND_MIN_ERPM) || /* 检查速度命令是否低于绝对无感稳定范围。 */
        (speed_erpm < g_output_limit.speed_min_erpm)) /* 检查速度命令是否低于运行时配置范围。 */
    { /* 零速度命令处理开始。 */
        BLDC_Stop();                    /* 将过低速度统一解释为软停机命令。 */
        return;                         /* 停机命令处理完成后不再请求启动。 */
    } /* 零速度命令处理结束。 */
    g_output_limit.command_reason = BLDC_LIMIT_NONE; /* 新速度命令首先清除上一模式的配置限幅原因。 */
    if (speed_erpm > g_output_limit.speed_max_erpm) /* 检查速度命令是否超过实机配置上限。 */
    { /* 速度命令上限处理开始。 */
        speed_erpm = g_output_limit.speed_max_erpm; /* 限制到统一管理器当前允许的最大电角转速。 */
        g_output_limit.command_reason = BLDC_LIMIT_ERPM; /* 记录用户速度命令已经受到配置限制。 */
    } /* 速度命令上限处理结束。 */
    if (g_control_command.mode != BLDC_CONTROL_SPEED) /* 检查是否正在从其他模式切换。 */
    { /* 速度模式无扰切换准备开始。 */
        BLDC_PreloadControlIntegrators(); /* 使用当前运行电流预置速度外环积分项。 */
        g_control_command.speed_error_erpm = 0L; /* 清除旧的速度误差诊断值。 */
    } /* 速度模式无扰切换准备结束。 */
    g_control_command.speed_command_erpm = speed_erpm; /* 保存用户电角转速命令。 */
    g_speed_pid.derivative_raw_ma = 0L; /* 新速度目标生效时清除旧的原始微分输出。 */
    g_speed_pid.derivative_filtered_ma = 0L; /* 新速度目标生效时清除旧的滤波微分输出。 */
    g_speed_pid.previous_error_erpm = 0L; /* 清除旧目标对应的速度误差历史。 */
    g_speed_pid.initialized = 0U;       /* 要求下个速度周期重新建立微分历史避免设定值冲击。 */
    g_output_limit.requested_speed_erpm = requested_speed_erpm; /* 保存用户原始速度请求供诊断。 */
    g_output_limit.limited_speed_erpm = speed_erpm; /* 保存配置限幅后的速度目标。 */
    g_control_command.mode = BLDC_CONTROL_SPEED; /* 在目标有效后原子切换到速度模式。 */
    g_current_command.requested_ma = g_control_command.output_current_ma; /* 立即保持模式切换前的电流避免命令跌落。 */
    BLDC_Start();                       /* 有效非零速度命令自动请求启动或继续运行。 */
} /* 当前方向电角转速幅值命令设置结束。 */

void BLDC_SetSpeedPidGains(uint32_t kp_x1000, uint32_t ki_x1000, uint32_t kd_x1000, uint16_t derivative_filter_permille) /* 实现速度 PID 参数设置接口。 */
{ /* 速度 PID 参数设置开始。 */
    if (kp_x1000 > BLDC_SPEED_PID_KP_MAX_X1000) /* 检查比例增益是否超过允许上限。 */
    { /* 比例增益上限处理开始。 */
        kp_x1000 = BLDC_SPEED_PID_KP_MAX_X1000; /* 将比例增益限制到安全配置范围。 */
    } /* 比例增益上限处理结束。 */
    if (ki_x1000 > BLDC_SPEED_PID_KI_MAX_X1000) /* 检查积分增益是否超过允许上限。 */
    { /* 积分增益上限处理开始。 */
        ki_x1000 = BLDC_SPEED_PID_KI_MAX_X1000; /* 将积分增益限制到安全配置范围。 */
    } /* 积分增益上限处理结束。 */
    if (kd_x1000 > BLDC_SPEED_PID_KD_MAX_X1000) /* 检查微分增益是否超过允许上限。 */
    { /* 微分增益上限处理开始。 */
        kd_x1000 = BLDC_SPEED_PID_KD_MAX_X1000; /* 将微分增益限制到安全配置范围。 */
    } /* 微分增益上限处理结束。 */
    if (derivative_filter_permille > BLDC_PERMILLE_SCALE) /* 检查微分低通修正比例是否超过百分之百。 */
    { /* 微分滤波参数上限处理开始。 */
        derivative_filter_permille = BLDC_PERMILLE_SCALE; /* 将滤波修正比例限制到千分之一千。 */
    } /* 微分滤波参数上限处理结束。 */
    g_speed_pid.kp_x1000 = kp_x1000;   /* 保存速度比例增益千分值。 */
    g_speed_pid.ki_x1000 = ki_x1000;   /* 保存速度积分增益千分值。 */
    g_speed_pid.kd_x1000 = kd_x1000;   /* 保存速度微分增益千分值。 */
    g_speed_pid.derivative_filter_permille = derivative_filter_permille; /* 保存微分低通每周期修正千分比。 */
    BLDC_ResetSpeedPidRuntime(g_current_command.applied_ma); /* 以当前实际命令电流无扰重建速度积分和微分状态。 */
} /* 速度 PID 参数设置结束。 */

BLDC_ControlMode_t BLDC_GetControlMode(void) /* 实现当前统一上层控制模式读取接口。 */
{ /* 当前统一上层控制模式读取开始。 */
    return g_control_command.mode;      /* 返回停止、电流、占空比或速度模式。 */
} /* 当前统一上层控制模式读取结束。 */

void BLDC_SetMotorCurrentLimitMa(int32_t limit_ma) /* 实现统一最大正向电机电流设置接口。 */
{ /* 统一最大正向电机电流设置开始。 */
    if (limit_ma < BLDC_MOTOR_CURRENT_LIMIT_MIN_MA) /* 检查统一电机电流限制下限。 */
    { /* 统一电机电流限制下限处理开始。 */
        limit_ma = BLDC_MOTOR_CURRENT_LIMIT_MIN_MA; /* 限制到一百毫安最低可配置电机电流上限。 */
    } /* 统一电机电流限制下限处理结束。 */
    if (limit_ma > BLDC_MOTOR_CURRENT_LIMIT_MAX_MA) /* 检查统一电机电流限制上限。 */
    { /* 统一电机电流限制上限处理开始。 */
        limit_ma = BLDC_MOTOR_CURRENT_LIMIT_MAX_MA; /* 限制到电流 PI 十安软件绝对上限。 */
    } /* 统一电机电流限制上限处理结束。 */
    g_output_limit.motor_current_limit_ma = limit_ma; /* 原子更新统一管理器最大电机电流。 */
} /* 统一最大正向电机电流设置结束。 */

void BLDC_SetDutyLimitsPermille(uint16_t minimum_permille, uint16_t maximum_permille) /* 实现电流 PI 占空比范围设置接口。 */
{ /* 电流 PI 占空比范围设置开始。 */
    uint32_t minimum_counts;            /* 保存最小千分比换算得到的内部计数。 */
    uint32_t maximum_counts;            /* 保存最大千分比换算得到的内部计数。 */
    if (maximum_permille > BLDC_DUTY_COMMAND_MAX_PERMILLE) /* 检查最大占空比是否超过当前安全绝对值。 */
    { /* 最大占空比绝对上限处理开始。 */
        maximum_permille = BLDC_DUTY_COMMAND_MAX_PERMILLE; /* 限制到当前安全测试百分之五十。 */
    } /* 最大占空比绝对上限处理结束。 */
    if (maximum_permille < BLDC_DUTY_COMMAND_MIN_PERMILLE) /* 检查最大占空比是否低于有效运行命令。 */
    { /* 最大占空比绝对下限处理开始。 */
        maximum_permille = BLDC_DUTY_COMMAND_MIN_PERMILLE; /* 至少允许百分之十的有效运行命令。 */
    } /* 最大占空比绝对下限处理结束。 */
    minimum_counts = ((uint32_t)minimum_permille * BLDC_DUTY_SCALE) / BLDC_PERMILLE_SCALE; /* 将最小千分比换算为内部计数。 */
    maximum_counts = ((uint32_t)maximum_permille * BLDC_DUTY_SCALE) / BLDC_PERMILLE_SCALE; /* 将最大千分比换算为内部计数。 */
    if (minimum_counts < (uint32_t)BLDC_CURRENT_DUTY_MIN) /* 检查最小占空比是否低于原安全值。 */
    { /* 最小占空比安全下限处理开始。 */
        minimum_counts = (uint32_t)BLDC_CURRENT_DUTY_MIN; /* 保持原有一百计数安全下限。 */
    } /* 最小占空比安全下限处理结束。 */
    if (minimum_counts >= maximum_counts) /* 检查用户给出的占空比范围是否有效。 */
    { /* 无效占空比范围处理开始。 */
        minimum_counts = maximum_counts - 1U; /* 至少保留一个计数的 PI 可调范围。 */
    } /* 无效占空比范围处理结束。 */
    g_output_limit.duty_min_counts = (uint16_t)minimum_counts; /* 原子更新电流 PI 最小占空比计数。 */
    g_output_limit.duty_max_counts = (uint16_t)maximum_counts; /* 原子更新电流 PI 最大占空比计数。 */
} /* 电流 PI 占空比范围设置结束。 */

void BLDC_SetSpeedLimitsErpm(uint32_t minimum_erpm, uint32_t maximum_erpm) /* 实现无感速度模式范围设置接口。 */
{ /* 无感速度模式范围设置开始。 */
    if (minimum_erpm < BLDC_SPEED_COMMAND_MIN_ERPM) /* 检查最低速度是否低于无感绝对稳定范围。 */
    { /* 最低速度绝对下限处理开始。 */
        minimum_erpm = BLDC_SPEED_COMMAND_MIN_ERPM; /* 限制到一千五百电角转每分钟。 */
    } /* 最低速度绝对下限处理结束。 */
    if (minimum_erpm > BLDC_SPEED_COMMAND_MAX_ERPM) /* 检查最低速度是否超过软件绝对上限。 */
    { /* 最低速度绝对上限处理开始。 */
        minimum_erpm = BLDC_SPEED_COMMAND_MAX_ERPM; /* 防止非法下限把整个速度范围推到软件上限之外。 */
    } /* 最低速度绝对上限处理结束。 */
    if (maximum_erpm > BLDC_SPEED_COMMAND_MAX_ERPM) /* 检查最高速度是否超过软件绝对范围。 */
    { /* 最高速度绝对上限处理开始。 */
        maximum_erpm = BLDC_SPEED_COMMAND_MAX_ERPM; /* 限制到两万电角转每分钟。 */
    } /* 最高速度绝对上限处理结束。 */
    if (maximum_erpm < minimum_erpm)    /* 检查用户给出的速度范围是否有效。 */
    { /* 无效速度范围处理开始。 */
        maximum_erpm = minimum_erpm;    /* 将最高速度提高到最低速度以形成有效范围。 */
    } /* 无效速度范围处理结束。 */
    g_output_limit.speed_min_erpm = minimum_erpm; /* 原子更新速度模式最低有效目标。 */
    g_output_limit.speed_max_erpm = maximum_erpm; /* 原子更新速度模式最高允许目标。 */
} /* 无感速度模式范围设置结束。 */

BLDC_LimitReason_t BLDC_GetLimitReason(void) /* 实现统一输出限幅原因读取接口。 */
{ /* 统一输出限幅原因读取开始。 */
    return g_output_limit.active_reason; /* 返回当前组合后的全部限制位标志。 */
} /* 统一输出限幅原因读取结束。 */

uint8_t BLDC_IsLimitActive(void)       /* 实现统一输出限幅活动状态读取接口。 */
{ /* 统一输出限幅活动状态读取开始。 */
    return g_output_limit.active;       /* 返回当前是否存在任意配置或运行限制。 */
} /* 统一输出限幅活动状态读取结束。 */

int32_t BLDC_GetUnifiedLimitedCurrentMa(void) /* 实现统一限幅后电机电流读取接口。 */
{ /* 统一限幅后电机电流读取开始。 */
    return g_output_limit.limited_current_ma; /* 返回当前统一管理器最终允许电流。 */
} /* 统一限幅后电机电流读取结束。 */

uint16_t BLDC_GetUnifiedLimitedDutyPermille(void) /* 实现统一限幅后占空比目标读取接口。 */
{ /* 统一限幅后占空比目标读取开始。 */
    return g_output_limit.limited_duty_permille; /* 返回当前配置限幅后的占空比目标。 */
} /* 统一限幅后占空比目标读取结束。 */

uint32_t BLDC_GetUnifiedLimitedSpeedErpm(void) /* 实现统一限幅后速度目标读取接口。 */
{ /* 统一限幅后速度目标读取开始。 */
    return g_output_limit.limited_speed_erpm; /* 返回当前配置限幅后的电角转速目标。 */
} /* 统一限幅后速度目标读取结束。 */

void BLDC_SetCurrentRampMaPerSecond(uint32_t rise_ma_per_second, uint32_t fall_ma_per_second) /* 实现运行电流命令斜率设置接口。 */
{ /* 运行电流命令斜率设置开始。 */
    if (rise_ma_per_second < BLDC_CURRENT_RAMP_MIN_MA_PER_SECOND) /* 检查上升斜率下限。 */
    { /* 上升斜率下限处理开始。 */
        rise_ma_per_second = BLDC_CURRENT_RAMP_MIN_MA_PER_SECOND; /* 限制到每秒一百毫安。 */
    } /* 上升斜率下限处理结束。 */
    if (rise_ma_per_second > BLDC_CURRENT_RAMP_MAX_MA_PER_SECOND) /* 检查上升斜率上限。 */
    { /* 上升斜率上限处理开始。 */
        rise_ma_per_second = BLDC_CURRENT_RAMP_MAX_MA_PER_SECOND; /* 限制到每秒二十安。 */
    } /* 上升斜率上限处理结束。 */
    if (fall_ma_per_second < BLDC_CURRENT_RAMP_MIN_MA_PER_SECOND) /* 检查下降斜率下限。 */
    { /* 下降斜率下限处理开始。 */
        fall_ma_per_second = BLDC_CURRENT_RAMP_MIN_MA_PER_SECOND; /* 限制到每秒一百毫安。 */
    } /* 下降斜率下限处理结束。 */
    if (fall_ma_per_second > BLDC_CURRENT_RAMP_MAX_MA_PER_SECOND) /* 检查下降斜率上限。 */
    { /* 下降斜率上限处理开始。 */
        fall_ma_per_second = BLDC_CURRENT_RAMP_MAX_MA_PER_SECOND; /* 限制到每秒二十安。 */
    } /* 下降斜率上限处理结束。 */
    g_current_command.rise_ma_per_second = rise_ma_per_second; /* 原子更新后续上升命令使用的斜率。 */
    g_current_command.fall_ma_per_second = fall_ma_per_second; /* 原子更新后续下降命令使用的斜率。 */
} /* 运行电流命令斜率设置结束。 */

int32_t BLDC_GetRequestedCurrentMa(void) /* 实现用户请求电流读取接口。 */
{ /* 用户请求电流读取开始。 */
    return g_current_command.requested_ma; /* 返回尚未经过外层斜坡限制的用户命令。 */
} /* 用户请求电流读取结束。 */

int32_t BLDC_GetAppliedCurrentMa(void) /* 实现斜坡后电流命令读取接口。 */
{ /* 斜坡后电流命令读取开始。 */
    return g_current_command.applied_ma; /* 返回当前已经送往电流 PI 的外层命令。 */
} /* 斜坡后电流命令读取结束。 */

void BLDC_SetCurrentTargetMa(int32_t target_ma) /* 实现旧目标电流接口的兼容转发。 */
{ /* 旧目标电流接口兼容处理开始。 */
    BLDC_SetRunCurrentMa(target_ma);    /* 统一使用带启停语义和外层斜坡的新命令接口。 */
} /* 旧目标电流接口兼容处理结束。 */

void BLDC_SetInputCurrentLimitMa(uint32_t limit_ma) /* 实现母线输入电流上限设置接口。 */
{ /* 母线输入电流上限设置开始。 */
    if (limit_ma < BLDC_INPUT_CURRENT_LIMIT_MIN_MA) /* 检查输入电流上限下限。 */
    { /* 输入电流上限下限处理开始。 */
        limit_ma = BLDC_INPUT_CURRENT_LIMIT_MIN_MA; /* 限制到最小一百毫安。 */
    } /* 输入电流上限下限处理结束。 */
    if (limit_ma > BLDC_INPUT_CURRENT_LIMIT_MAX_MA) /* 检查输入电流上限上限。 */
    { /* 输入电流上限上限处理开始。 */
        limit_ma = BLDC_INPUT_CURRENT_LIMIT_MAX_MA; /* 限制到最大二十安。 */
    } /* 输入电流上限上限处理结束。 */
    g_input_limit.current_limit_ma = limit_ma; /* 原子更新后续限幅使用的母线输入电流上限。 */
} /* 母线输入电流上限设置结束。 */

void BLDC_SetInputPowerLimitMw(uint32_t limit_mw) /* 实现母线输入功率上限设置接口。 */
{ /* 母线输入功率上限设置开始。 */
    if (limit_mw < BLDC_INPUT_POWER_LIMIT_MIN_MW) /* 检查输入功率上限下限。 */
    { /* 输入功率上限下限处理开始。 */
        limit_mw = BLDC_INPUT_POWER_LIMIT_MIN_MW; /* 限制到最小一瓦。 */
    } /* 输入功率上限下限处理结束。 */
    if (limit_mw > BLDC_INPUT_POWER_LIMIT_MAX_MW) /* 检查输入功率上限上限。 */
    { /* 输入功率上限上限处理开始。 */
        limit_mw = BLDC_INPUT_POWER_LIMIT_MAX_MW; /* 限制到最大一千瓦。 */
    } /* 输入功率上限上限处理结束。 */
    g_input_limit.power_limit_mw = limit_mw; /* 原子更新后续限幅使用的母线输入功率上限。 */
} /* 母线输入功率上限设置结束。 */

uint32_t BLDC_GetEstimatedInputCurrentMa(void) /* 实现估算母线输入电流读取接口。 */
{ /* 估算母线输入电流读取开始。 */
    return g_input_limit.estimated_current_ma; /* 返回 VESC BLDC 公式得到的输入电流。 */
} /* 估算母线输入电流读取结束。 */

uint32_t BLDC_GetEstimatedInputPowerMw(void) /* 实现估算母线输入功率读取接口。 */
{ /* 估算母线输入功率读取开始。 */
    return g_input_limit.estimated_power_mw; /* 返回输入电流乘母线电压得到的功率。 */
} /* 估算母线输入功率读取结束。 */

int32_t BLDC_GetEstimatedSignedInputCurrentMa(void) /* 实现带符号估算母线电流读取接口。 */
{ /* 带符号估算母线电流读取开始。 */
    return g_input_limit.estimated_signed_current_ma; /* 返回正值耗电、负值潜在回充的估算母线电流。 */
} /* 带符号估算母线电流读取结束。 */

int32_t BLDC_GetEstimatedSignedInputPowerMw(void) /* 实现带符号估算母线功率读取接口。 */
{ /* 带符号估算母线功率读取开始。 */
    return g_input_limit.estimated_signed_power_mw; /* 返回正值耗电、负值潜在回充的估算母线功率。 */
} /* 带符号估算母线功率读取结束。 */

uint8_t BLDC_IsRegenerationProtectionActive(void) /* 实现回充保护活动状态读取接口。 */
{ /* 回充保护活动状态读取开始。 */
    return g_energy_protection.regen_protection_active; /* 返回最近一次运行是否确认了持续回充。 */
} /* 回充保护活动状态读取结束。 */

uint8_t BLDC_IsSoftOvervoltageActive(void) /* 实现软过压保护活动状态读取接口。 */
{ /* 软过压保护活动状态读取开始。 */
    return g_energy_protection.soft_overvoltage_active; /* 返回软过压降额或释放回差是否正在生效。 */
} /* 软过压保护活动状态读取结束。 */

uint16_t BLDC_GetSoftOvervoltageDeratePermille(void) /* 实现软过压允许功率比例读取接口。 */
{ /* 软过压允许功率比例读取开始。 */
    return g_energy_protection.soft_derate_permille; /* 返回零到一千之间的当前允许功率比例。 */
} /* 软过压允许功率比例读取结束。 */

uint32_t BLDC_GetRegenerationEventCount(void) /* 实现持续回充事件次数读取接口。 */
{ /* 持续回充事件次数读取开始。 */
    return g_energy_protection.regen_event_count; /* 返回上电以来确认的持续回充次数。 */
} /* 持续回充事件次数读取结束。 */

uint32_t BLDC_GetSoftOvervoltageEventCount(void) /* 实现软过压事件次数读取接口。 */
{ /* 软过压事件次数读取开始。 */
    return g_energy_protection.soft_overvoltage_event_count; /* 返回上电以来进入软过压区域的次数。 */
} /* 软过压事件次数读取结束。 */

uint32_t BLDC_GetEnergyCoastStopCount(void) /* 实现能量保护滑行停机次数读取接口。 */
{ /* 能量保护滑行停机次数读取开始。 */
    return g_energy_protection.coast_stop_count; /* 返回回充或软过压触发的立即关桥次数。 */
} /* 能量保护滑行停机次数读取结束。 */

uint32_t BLDC_GetNegativeTorqueRejectCount(void) /* 实现主动负转矩拒绝次数读取接口。 */
{ /* 主动负转矩拒绝次数读取开始。 */
    return g_energy_protection.negative_torque_reject_count; /* 返回非零制动电流请求被拒绝的累计次数。 */
} /* 主动负转矩拒绝次数读取结束。 */

int32_t BLDC_GetPeakRegenerationCurrentMa(void) /* 实现回充负母线电流峰值读取接口。 */
{ /* 回充负母线电流峰值读取开始。 */
    return g_energy_protection.peak_regen_current_ma; /* 返回上电以来最小的带符号估算母线电流。 */
} /* 回充负母线电流峰值读取结束。 */

int32_t BLDC_GetPeakRegenerationPowerMw(void) /* 实现回充负母线功率峰值读取接口。 */
{ /* 回充负母线功率峰值读取开始。 */
    return g_energy_protection.peak_regen_power_mw; /* 返回上电以来最小的带符号估算母线功率。 */
} /* 回充负母线功率峰值读取结束。 */

int32_t BLDC_GetLimitedCurrentTargetMa(void) /* 实现限幅后电机电流目标读取接口。 */
{ /* 限幅后电机电流目标读取开始。 */
    return g_input_limit.limited_target_ma; /* 返回送往快速电流 PI 的最终外层目标。 */
} /* 限幅后电机电流目标读取结束。 */

BLDC_InputLimitReason_t BLDC_GetInputLimitReason(void) /* 实现输入限幅原因读取接口。 */
{ /* 输入限幅原因读取开始。 */
    return g_input_limit.reason;        /* 返回无、电流、功率或同时限幅状态。 */
} /* 输入限幅原因读取结束。 */

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

int32_t BLDC_GetSignedMotorCurrentMa(void) /* 实现带符号电机电流瞬时值读取接口。 */
{ /* 带符号电机电流瞬时值读取开始。 */
    return g_current.signed_motor_current_ma; /* 返回当前回流相经过配置极性归一化后的瞬时电流。 */
} /* 带符号电机电流瞬时值读取结束。 */

int32_t BLDC_GetSignedFilteredCurrentMa(void) /* 实现带符号滤波电机电流读取接口。 */
{ /* 带符号滤波电机电流读取开始。 */
    return g_current.signed_filtered_current_ma; /* 返回不参与当前正向 PI 的独立带符号低通值。 */
} /* 带符号滤波电机电流读取结束。 */

int32_t BLDC_GetSourcePhaseCurrentMa(void) /* 实现当前源相带符号电流读取接口。 */
{ /* 当前源相带符号电流读取开始。 */
    return g_current.source_current_ma; /* 返回六步表当前源相经过配置极性归一化后的电流。 */
} /* 当前源相带符号电流读取结束。 */

int32_t BLDC_GetFloatingPhaseCurrentMa(void) /* 实现当前悬空相带符号电流读取接口。 */
{ /* 当前悬空相带符号电流读取开始。 */
    return g_current.floating_current_ma; /* 返回六步表当前悬空相经过配置极性归一化后的电流。 */
} /* 当前悬空相带符号电流读取结束。 */

uint32_t BLDC_GetCurrentPolarityErrorCount(void) /* 实现正常驱动负极性样本计数读取接口。 */
{ /* 正常驱动负极性样本计数读取开始。 */
    return g_current.polarity_error_count; /* 返回本次启动消隐后显著负电流样本累计值。 */
} /* 正常驱动负极性样本计数读取结束。 */

uint8_t BLDC_SetCurrentPolarity(int8_t u_polarity, int8_t v_polarity, int8_t w_polarity) /* 实现停机状态三相电流采样极性设置接口。 */
{ /* 三相电流采样极性设置开始。 */
    uint32_t interrupt_mask;            /* 保存修改三相配置前的全局中断屏蔽状态。 */
    uint8_t step_index;                 /* 保存正在清零的六扇区带符号诊断下标。 */
    if (BLDC_IsPowerState() != 0U)      /* 检查功率级是否仍处于定位、开环、接管或闭环状态。 */
    { /* 运行中极性修改拒绝开始。 */
        return 0U;                      /* 拒绝运行中改变采样方向，避免后续制动控制极性突变。 */
    } /* 运行中极性修改拒绝结束。 */
    interrupt_mask = __get_PRIMASK();   /* 读取调用接口前的全局中断屏蔽状态。 */
    __disable_irq();                    /* 暂停 ADC 快速回调，保证三相极性和诊断同时更新。 */
    g_current_polarity.u = (u_polarity < 0) ? -1 : 1; /* 将 U 相任意输入规范为负一或正一。 */
    g_current_polarity.v = (v_polarity < 0) ? -1 : 1; /* 将 V 相任意输入规范为负一或正一。 */
    g_current_polarity.w = (w_polarity < 0) ? -1 : 1; /* 将 W 相任意输入规范为负一或正一。 */
    g_current.signed_motor_current_ma = 0L; /* 清零旧极性下的带符号瞬时电流。 */
    g_current.signed_filtered_current_ma = 0L; /* 清零旧极性下的带符号滤波电流。 */
    g_current.source_current_ma = 0L;   /* 清零旧极性下的源相诊断电流。 */
    g_current.floating_current_ma = 0L; /* 清零旧极性下的悬空相诊断电流。 */
    g_current.signed_positive_peak_ma = 0L; /* 清零旧极性下的正电流峰值。 */
    g_current.signed_negative_peak_ma = 0L; /* 清零旧极性下的负电流峰值。 */
    g_current.polarity_positive_count = 0U; /* 清零旧极性下的显著正样本计数。 */
    g_current.polarity_error_count = 0U; /* 清零旧极性下的显著负样本计数。 */
    for (step_index = 0U; step_index < 6U; step_index++) /* 遍历全部六个换相扇区。 */
    { /* 六扇区带符号诊断清零开始。 */
        g_current.step_signed_current_ma[step_index] = 0L; /* 清除旧极性对应的扇区带符号滤波值。 */
    } /* 六扇区带符号诊断清零结束。 */
    __DMB();                            /* 保证全部极性配置和诊断清零在恢复中断前完成。 */
    if (interrupt_mask == 0U)           /* 检查调用接口前全局中断是否处于开启状态。 */
    { /* 恢复全局中断处理开始。 */
        __enable_irq();                 /* 仅在调用前允许中断时重新开启全局中断。 */
    } /* 恢复全局中断处理结束。 */
    return 1U;                          /* 返回本次停机极性配置已经接受。 */
} /* 三相电流采样极性设置结束。 */

uint16_t BLDC_GetDuty(void)             /* 实现当前占空比命令读取接口。 */
{ /* 占空比读取开始。 */
    return g_bldc.duty;                 /* 返回满量程为 1200 的占空比命令。 */
} /* 占空比读取结束。 */

uint32_t BLDC_GetBusVoltageMv(void)     /* 实现母线电压读取接口。 */
{ /* 母线电压读取开始。 */
    return g_bus_voltage.filtered_mv;   /* 返回低通滤波后的母线电压毫伏值。 */
} /* 母线电压读取结束。 */

uint8_t BLDC_SetDirection(int8_t direction) /* 实现仅在功率关闭时允许调用的方向设置接口。 */
{ /* 方向设置接口开始。 */
    if (g_reversal.state != BLDC_REVERSAL_IDLE) /* 检查自动换向状态机是否拥有方向控制权。 */
    { /* 自动换向期间手动方向设置拒绝开始。 */
        return 0U;                      /* 拒绝手动接口打断减流或滑行等待流程。 */
    } /* 自动换向期间手动方向设置拒绝结束。 */
    if (BLDC_IsPowerState() != 0U)      /* 检查定位、开环、接管或闭环是否仍在输出功率。 */
    { /* 运行中方向修改拒绝处理开始。 */
        return 0U;                      /* 拒绝带电反向以避免电流冲击和功率级损坏。 */
    } /* 运行中方向修改拒绝处理结束。 */
    if (direction < 0)                  /* 检查用户是否请求反转换相。 */
    { /* 反转方向设置开始。 */
        g_motion.direction = -1;        /* 保存反转方向符号。 */
    } /* 反转方向设置结束。 */
    else                                /* 处理正转和零方向请求。 */
    { /* 正转方向设置开始。 */
        g_motion.direction = 1;         /* 保存正转方向符号并把零请求解释为正转。 */
    } /* 正转方向设置结束。 */
    g_reversal.requested_direction = g_motion.direction; /* 同步手动设置后的目标方向诊断。 */
    g_motion.signed_erpm = 0L;          /* 停机切换方向后保持有符号转速为零。 */
    return 1U;                          /* 返回本次停机方向设置已经接受。 */
} /* 方向设置接口结束。 */

int8_t BLDC_GetDirection(void)          /* 实现当前运动方向状态读取接口。 */
{ /* 当前运动方向状态读取开始。 */
    return g_motion.direction;          /* 返回正一表示正转，负一表示反转。 */
} /* 当前运动方向状态读取结束。 */

int8_t BLDC_GetRequestedDirection(void) /* 实现最近一次有符号命令目标方向读取接口。 */
{ /* 目标方向读取开始。 */
    return g_reversal.requested_direction; /* 返回正一表示目标正转，负一表示目标反转。 */
} /* 目标方向读取结束。 */

BLDC_ReversalState_t BLDC_GetReversalState(void) /* 实现当前安全换向状态读取接口。 */
{ /* 安全换向状态读取开始。 */
    return g_reversal.state;            /* 返回空闲、减流或滑行等待状态。 */
} /* 安全换向状态读取结束。 */

uint32_t BLDC_GetReversalCount(void)    /* 实现已经完成的安全换向次数读取接口。 */
{ /* 安全换向完成次数读取开始。 */
    return g_reversal.completed_count;  /* 返回完成关断、方向切换和命令恢复的累计次数。 */
} /* 安全换向完成次数读取结束。 */

int32_t BLDC_GetSignedErpm(void)        /* 实现有符号滤波电角转速读取接口。 */
{ /* 有符号滤波电角转速读取开始。 */
    return g_motion.signed_erpm;        /* 返回已经乘以当前方向符号的滤波 ERPM。 */
} /* 有符号滤波电角转速读取结束。 */

int32_t BLDC_GetTachometer(void)        /* 实现有符号累计换相步数读取接口。 */
{ /* 有符号累计换相步数读取开始。 */
    return g_motion.tachometer;         /* 返回跨停机和自动重启保留的有符号换相步数。 */
} /* 有符号累计换相步数读取结束。 */

uint32_t BLDC_GetTachometerAbs(void)    /* 实现绝对累计换相步数读取接口。 */
{ /* 绝对累计换相步数读取开始。 */
    return g_motion.tachometer_abs;     /* 返回不区分方向且跨停机保留的累计换相步数。 */
} /* 绝对累计换相步数读取结束。 */

void BLDC_ResetTachometer(void)         /* 实现有符号和绝对换相计数器原子清零接口。 */
{ /* 换相计数器原子清零开始。 */
    uint32_t interrupt_mask;            /* 保存调用接口前的全局中断屏蔽状态。 */
    interrupt_mask = __get_PRIMASK();   /* 读取当前是否已经处于中断屏蔽环境。 */
    __disable_irq();                    /* 暂停快速换相中断避免两个计数器只清零一半。 */
    g_motion.tachometer = 0L;           /* 清零带方向符号的累计换相步数。 */
    g_motion.tachometer_abs = 0U;       /* 清零不区分方向的累计换相步数。 */
    __DMB();                            /* 保证两个计数器写入在恢复中断前完成。 */
    if (interrupt_mask == 0U)           /* 检查调用接口前全局中断是否处于开启状态。 */
    { /* 恢复全局中断处理开始。 */
        __enable_irq();                 /* 仅在调用前允许中断时重新开启全局中断。 */
    } /* 恢复全局中断处理结束。 */
} /* 换相计数器原子清零结束。 */

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

static int32_t BLDC_GetBemfDiff(void)   /* 实现按照当前方向归一化的反电动势差值计算。 */
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
    if (g_motion.direction < 0)         /* 检查当前是否按照反转顺序运行。 */
    { /* 反转反电动势极性处理开始。 */
        v_diff = -v_diff;               /* 反转时将预期过零斜率整体取反并重新归一为正值。 */
    } /* 反转反电动势极性处理结束。 */
    return v_diff;                      /* 返回归一化差值。 */
} /* 差值计算结束。 */
