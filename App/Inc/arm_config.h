#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H

/*
 * 机械臂工程的集中配置文件。
 *
 * 修改舵机零位、方向、限位、串口波特率或安全策略时，优先修改本文件。
 * 除非更换了 STM32 引脚，否则不需要改 BSP 驱动源码。
 */

#include "stm32f4xx_hal.h"

/* -------------------------- 单舵机调零模式 ---------------------------- */
/*
 * 当前按“暂不接视觉、逐个机械调零”的需求设为 1。
 *
 * 调零模式下：
 *   1. 只给 ARM_ZERO_CALIBRATION_SERVO 指定的舵机输出固定脉宽；
 *   2. 其余四路 PWM 为 0，不创建通信/运动/安全任务；
 *   3. K230 和电脑 USB-TTL 串口均不需要连接。
 *
 * 五个舵机全部调好后，务必把 ARM_ZERO_CALIBRATION_MODE 改回 0U，
 * 然后重新编译，恢复正常机械臂控制。
 */
#define ARM_ZERO_CALIBRATION_MODE          0U

/* 1～5，首次从舵机 1 开始；每调完一路再修改编号并重新下载。 */
#define ARM_ZERO_CALIBRATION_SERVO         1U

/*
 * 标准舵机电气中位为 1500 us。通常保持不变，通过重新安装舵盘/调整
 * 连杆完成机械零位校准，不建议用大幅修改脉宽来补偿装配误差。
 */
#define ARM_ZERO_CALIBRATION_PULSE_US      1500U

#if ARM_ZERO_CALIBRATION_MODE
#if ((ARM_ZERO_CALIBRATION_SERVO < 1U) || \
     (ARM_ZERO_CALIBRATION_SERVO > 5U))
#error "ARM_ZERO_CALIBRATION_SERVO must be in range 1..5"
#endif
#if ((ARM_ZERO_CALIBRATION_PULSE_US < 500U) || \
     (ARM_ZERO_CALIBRATION_PULSE_US > 2500U))
#error "ARM_ZERO_CALIBRATION_PULSE_US must be in range 500..2500 us"
#endif
#endif

/* ---------------------------- 舵机公共参数 ---------------------------- */
#define ARM_SERVO_COUNT                    5U
#define ARM_SERVO_PWM_FREQUENCY_HZ         50U
#define ARM_SERVO_PWM_TIMER_TICK_HZ        1000000U
#define ARM_SERVO_DEFAULT_MIN_PULSE_US     500U
#define ARM_SERVO_DEFAULT_MAX_PULSE_US     2500U

/*
 * 五个关节的逻辑角度单位均为 0.1 度。
 *
 * 删除原夹爪舵机后，编号按原机械臂顺序整体前移：
 *   舵机1：末端电磁铁旋转（原2号）
 *   舵机2：腕部/末端俯仰（原3号）
 *   舵机3：肘部（原4号）
 *   舵机4：肩部（原5号）
 *   舵机5：底座旋转（原6号）
 *
 * 首次装机时请先拆下舵盘进行空载测试，再根据实际机械限位缩小范围。
 * reverse=1 表示逻辑角度增大时，PWM 脉宽反向变化。
 */
#define ARM_SERVO1_MIN_ANGLE_X10           0
#define ARM_SERVO1_MAX_ANGLE_X10           1800
#define ARM_SERVO1_HOME_ANGLE_X10          900
#define ARM_SERVO1_MIN_PULSE_US            ARM_SERVO_DEFAULT_MIN_PULSE_US
#define ARM_SERVO1_MAX_PULSE_US            ARM_SERVO_DEFAULT_MAX_PULSE_US
#define ARM_SERVO1_REVERSED                0U

#define ARM_SERVO2_MIN_ANGLE_X10           0
#define ARM_SERVO2_MAX_ANGLE_X10           1800
#define ARM_SERVO2_HOME_ANGLE_X10          1730
#define ARM_SERVO2_MIN_PULSE_US            ARM_SERVO_DEFAULT_MIN_PULSE_US
#define ARM_SERVO2_MAX_PULSE_US            ARM_SERVO_DEFAULT_MAX_PULSE_US
#define ARM_SERVO2_REVERSED                0U

/*
 * J3肘部硬防撞限位：
 *   最新实测J3=80°时大臂与小臂成直角，J3增大时继续向A4方向弯曲。
 *   机械端点约为180°，软件只允许到175°，预留5°余量，禁止串口
 *   手动命令和自动逆解撞到机械端点。
 */
#define ARM_SERVO3_MIN_ANGLE_X10           0
#define ARM_SERVO3_MAX_ANGLE_X10           1750
#define ARM_SERVO3_HOME_ANGLE_X10          900
#define ARM_SERVO3_MIN_PULSE_US            ARM_SERVO_DEFAULT_MIN_PULSE_US
#define ARM_SERVO3_MAX_PULSE_US            ARM_SERVO_DEFAULT_MAX_PULSE_US
#define ARM_SERVO3_REVERSED                0U

#define ARM_SERVO4_MIN_ANGLE_X10           0
#define ARM_SERVO4_MAX_ANGLE_X10           1800
#define ARM_SERVO4_HOME_ANGLE_X10          900
#define ARM_SERVO4_MIN_PULSE_US            ARM_SERVO_DEFAULT_MIN_PULSE_US
#define ARM_SERVO4_MAX_PULSE_US            ARM_SERVO_DEFAULT_MAX_PULSE_US
#define ARM_SERVO4_REVERSED                0U

#define ARM_SERVO5_MIN_ANGLE_X10           0
#define ARM_SERVO5_MAX_ANGLE_X10           1800
#define ARM_SERVO5_HOME_ANGLE_X10          0
#define ARM_SERVO5_MIN_PULSE_US            ARM_SERVO_DEFAULT_MIN_PULSE_US
#define ARM_SERVO5_MAX_PULSE_US            ARM_SERVO_DEFAULT_MAX_PULSE_US
#define ARM_SERVO5_REVERSED                0U

/* ----------------------------- 双路串口 ------------------------------- */
/* USART1 视觉口与 USART2 调试口默认使用相同波特率。 */
#define ARM_UART_BAUDRATE                  115200U
#define ARM_DEBUG_UART_BAUDRATE            115200U
#define ARM_UART_RX_DMA_BUFFER_SIZE        128U
#define ARM_UART_RX_QUEUE_DEPTH            8U
#define ARM_EVENT_QUEUE_DEPTH              8U
#define ARM_DEBUG_CLI_LINE_SIZE            128U

/*
 * USART2 调试诊断标记：
 *   UART2_HW_READY  - USART2 硬件初始化完成；
 *   UART2_DMA_READY - FreeRTOS 通信任务和 DMA 接收已经启动；
 *   UART2_RX        - PA3 确实收到了一批数据。
 * USART2 已改成纯文本命令行，正常情况下不需要额外诊断标记。
 * 如需排查最底层接线，可临时设为1；调通后保持0，避免干扰CLI回复。
 */
#define ARM_DEBUG_UART_DIAGNOSTIC_ENABLED  0U

/* VREAD调试命令等待K230返回完整视觉帧的最长时间。 */
#define ARM_VISION_DEBUG_TIMEOUT_MS         5000U

/*
 * 当前 FreeRTOSConfig.h 中 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5。
 * 会调用 xQueueSendFromISR 的 USART/DMA 中断，其数值优先级必须不小于 5。
 */
#define ARM_UART_IRQ_PRIORITY              6U

/* ----------------------------- 运动控制 ------------------------------- */
#define ARM_MOTION_PERIOD_MS               10U
#define ARM_MOTION_QUEUE_DEPTH             4U
#define ARM_MOTION_DEFAULT_DURATION_MS     1000U
#define ARM_MOTION_MAX_DURATION_MS         30000U

/* 原生 FreeRTOS 优先级；本工程 configMAX_PRIORITIES=56。 */
#define ARM_COMM_TASK_PRIORITY             32U
#define ARM_PUZZLE_TASK_PRIORITY           30U
#define ARM_CALIBRATION_TASK_PRIORITY      30U
#define ARM_MOTION_TASK_PRIORITY           40U
#define ARM_SAFETY_TASK_PRIORITY           48U
#define ARM_BUTTON_TASK_PRIORITY           28U

/* 栈大小单位为 StackType_t，即 STM32F4 上的 4 字节字。 */
#define ARM_COMM_TASK_STACK_WORDS          512U
#define ARM_PUZZLE_TASK_STACK_WORDS        768U
#define ARM_CALIBRATION_TASK_STACK_WORDS   640U
#define ARM_MOTION_TASK_STACK_WORDS        512U
#define ARM_SAFETY_TASK_STACK_WORDS        256U
#define ARM_BUTTON_TASK_STACK_WORDS        256U

/* ------------------------------ 按键输入 ------------------------------- */
/*
 * 10个独立按键，默认接法：按键一端接GPIO，另一端接GND；
 * GPIO内部上拉，按下为低电平。若实际按键模块为高电平触发，
 * 将ARM_BUTTON_ACTIVE_LEVEL改为GPIO_PIN_SET，并把Pull改为GPIO_PULLDOWN。
 *
 * 默认使用PE0~PE8、PE10，避开PE9完成指示和PE11/PE13/PE14舵机PWM。
 * 若你实际接线不同，只修改下面的端口/引脚宏即可。
 */
#define ARM_BUTTON_ENABLED                 1U
#define ARM_BUTTON_SCAN_PERIOD_MS          10U
#define ARM_BUTTON_DEBOUNCE_MS             40U
#define ARM_BUTTON_ACTIVE_LEVEL            GPIO_PIN_RESET
#define ARM_BUTTON_GPIO_PULL               GPIO_PULLUP

#define ARM_BUTTON1_GPIO_PORT              GPIOE
#define ARM_BUTTON1_GPIO_PIN               GPIO_PIN_0
#define ARM_BUTTON2_GPIO_PORT              GPIOE
#define ARM_BUTTON2_GPIO_PIN               GPIO_PIN_1
#define ARM_BUTTON3_GPIO_PORT              GPIOE
#define ARM_BUTTON3_GPIO_PIN               GPIO_PIN_2
#define ARM_BUTTON4_GPIO_PORT              GPIOE
#define ARM_BUTTON4_GPIO_PIN               GPIO_PIN_3
#define ARM_BUTTON5_GPIO_PORT              GPIOE
#define ARM_BUTTON5_GPIO_PIN               GPIO_PIN_4
#define ARM_BUTTON6_GPIO_PORT              GPIOE
#define ARM_BUTTON6_GPIO_PIN               GPIO_PIN_5
#define ARM_BUTTON7_GPIO_PORT              GPIOE
#define ARM_BUTTON7_GPIO_PIN               GPIO_PIN_6
#define ARM_BUTTON8_GPIO_PORT              GPIOE
#define ARM_BUTTON8_GPIO_PIN               GPIO_PIN_7
#define ARM_BUTTON9_GPIO_PORT              GPIOE
#define ARM_BUTTON9_GPIO_PIN               GPIO_PIN_8
#define ARM_BUTTON10_GPIO_PORT             GPIOE
#define ARM_BUTTON10_GPIO_PIN              GPIO_PIN_10

/* -------------------------- 机械臂几何模型 ---------------------------- */
/*
 * 所有长度均按旋转轴中心测量，单位mm：
 *   肩轴 -> 肘轴             165.0（原仿真105.0，加长60.0）
 *   肘轴 -> 腕轴             153.0（原仿真90.0，加长63.0）
 *   腕轴 -> 电磁铁吸附面中心  80.0
 *
 * 题面坐标原点为A4左边线与中间分界线交点，X向右、Y向下。
 * 重新建系基准：
 *   H直接表示“肩轴中心到A4纸面”的垂直距离，不再与垫板高度二次
 *   相加减。A4原点、Z=10mm处实测平行姿态为
 *   J1=90, J2=110, J3=150, J4=87, J5=90；底座旋转中心投影
 *   为(-75,0)，机械臂正前方为题面+X。
 */
#define ARM_KIN_LINK_SHOULDER_ELBOW_MM     165.0F
#define ARM_KIN_LINK_ELBOW_WRIST_MM        153.0F
#define ARM_KIN_LINK_WRIST_TOOL_MM           80.0F
#define ARM_KIN_SHOULDER_HEIGHT_MM           63.0F
#define ARM_KIN_BASE_TO_SHOULDER_MM          10.0F
#define ARM_KIN_BASE_X_MM                   (-75.0F)
#define ARM_KIN_BASE_Y_MM                     0.0F

/*
 * X轴实机反标定表（单位mm）。
 *
 * COMMAND是曾经发送给GOTO/IK的X，MEASURED是电磁铁实际到达的X。
 * 逆解接收的是“期望实际坐标”，固件按MEASURED->COMMAND做分段线性
 * 反插值，再送入原有坐标补偿和IK。这样无需为每个点写死舵机角，
 * GOTO、视觉抓放和六点校准共用同一条连续修正曲线。
 *
 * 0->0为原点补充锚点。用户最新反馈：X从40mm开始实际比目标
 * 多约10mm，因此在原实测表基础上把40mm及之后的MEASURED整体
 * 加10mm，让同样目标反算出的命令X变小。
 */
#define ARM_KIN_X_AXIS_CALIBRATION_ENABLED       0U
#define ARM_KIN_X_AXIS_CALIBRATION_POINT_COUNT  11U
#define ARM_KIN_X_AXIS_COMMAND_MM \
    {0.0F, 20.0F, 40.0F, 60.0F, 80.0F, 100.0F, 120.0F, \
     140.0F, 160.0F, 180.0F, 210.0F}
#define ARM_KIN_X_AXIS_MEASURED_MM \
    {0.0F, 15.0F, 45.0F, 65.0F, 85.0F, 105.0F, 125.0F, \
     140.0F, 155.0F, 175.0F, 200.0F}

/*
 * Y轴关于题面中线对称的实机反标定表（单位mm）。
 *
 * 用户提供的数据为厘米，已换算成毫米。|Y|小于50mm按无明显误差
 * 处理；从命令60mm实测65mm开始逐点反插值。负Y先对绝对值反算，
 * 再恢复负号，因此上下半区使用同一条连续曲线。
 */
#define ARM_KIN_Y_AXIS_CALIBRATION_ENABLED       0U
#define ARM_KIN_Y_AXIS_CALIBRATION_POINT_COUNT  11U
#define ARM_KIN_Y_AXIS_COMMAND_ABS_MM \
    {0.0F, 50.0F, 60.0F, 70.0F, 80.0F, 90.0F, 100.0F, \
     110.0F, 120.0F, 130.0F, 140.0F}
#define ARM_KIN_Y_AXIS_MEASURED_ABS_MM \
    {0.0F, 50.0F, 65.0F, 75.0F, 85.0F, 95.0F, 110.0F, \
     115.0F, 130.0F, 140.0F, 148.0F}

/*
 * 5x3二维XY实机反标定网格（单位mm）。
 *
 * COMMAND是送入未补偿IK的坐标，MEASURED是电磁铁圆面中心粗测实际
 * 坐标。固件先对“实际-命令”误差做分段双线性插值，再进行4次固定点
 * 反算，求出为了到达用户/视觉目标应送入IK的坐标。这样可以同时处理
 * X/Y耦合，避免把正Y、负Y区域错误地当成完全对称。
 *
 * 表格按Y=-135、0、+135逐行，每行X=10、55、105、155、200。
 * 标定范围外使用最近边界的误差，不根据人工粗测数据继续外推。
 */
#define ARM_KIN_XY_RESIDUAL_ENABLED              1U
#define ARM_KIN_XY_GRID_X_COUNT                   5U
#define ARM_KIN_XY_GRID_Y_COUNT                   3U
#define ARM_KIN_XY_GRID_INVERSE_ITERATIONS        4U
#define ARM_KIN_XY_GRID_COMMAND_X_MM \
    {10.0F, 55.0F, 105.0F, 155.0F, 200.0F}
#define ARM_KIN_XY_GRID_COMMAND_Y_MM \
    {-135.0F, 0.0F, 135.0F}
#define ARM_KIN_XY_GRID_MEASURED_X_MM \
    {{10.0F, 55.0F, 100.0F, 150.0F, 200.0F}, \
     { 9.0F, 54.0F, 102.0F, 151.0F, 194.0F}, \
     {17.0F, 58.0F, 108.0F, 153.0F, 201.0F}}
#define ARM_KIN_XY_GRID_MEASURED_Y_MM \
    {{-140.0F, -137.0F, -133.0F, -133.0F, -130.0F}, \
     {   0.0F,    0.0F,    0.0F,    0.0F,    0.0F}, \
     { 142.0F,  135.0F,  136.0F,  131.0F,  133.0F}}

/*
 * 实机测得所有目标点的X方向偏小；为保证A4六点在20mm搬运高度仍全部
 * 可达，只对运动目标增加统一的+10mm补偿。A4坐标定义和底座坐标保持
 * 不变，视觉无需改数据。
 */
#define ARM_KIN_TARGET_X_OFFSET_MM             0.0F

/*
 * P3（A4右边中点）实测仍比右边线短20mm。该误差不能继续叠加到全局
 * X偏移，否则右上、右下角P2/P4会因斜向距离过大而不可达。
 *
 * 因此仅在A4右侧区域增加平滑X补偿：X从150mm开始介入，P3达到
 * +20mm；补偿随|Y|增大而衰减，在P2/P4处为0。视觉给出的中间坐标
 * 也会连续插值，不会在P3附近产生角度跳变。
 */
#define ARM_KIN_P3_X_COMP_ENABLED               0U
#define ARM_KIN_P3_X_COMP_START_X_MM          150.0F
#define ARM_KIN_P3_X_COMP_FULL_X_MM           210.0F
#define ARM_KIN_P3_X_COMP_MAX_MM                20.0F
#define ARM_KIN_P3_X_COMP_Y_FADE_MM            148.5F

/*
 * X轴中段补偿：
 *   实测GOTO(105,0)只到原点右侧约87mm，少约18mm；
 *   但GOTO(210,0)到A4右边线基本准确。
 *
 * 因此额外增加一条“中间大、两端小”的补偿曲线：在X=105,Y=0时
 * 补偿最大18mm，在X=0和X=210处衰减到0。Y方向同样向上下边缘
 * 衰减，避免明显改变四个角点。
 */
#define ARM_KIN_X_MID_COMP_ENABLED             0U
#define ARM_KIN_X_MID_COMP_CENTER_MM         105.0F
#define ARM_KIN_X_MID_COMP_HALF_WIDTH_MM     105.0F
#define ARM_KIN_X_MID_COMP_MAX_MM             18.0F
#define ARM_KIN_X_MID_COMP_Y_FADE_MM         148.5F

/*
 * A4实测坐标补偿表：
 *   输入为用户/视觉期望的A4坐标，补偿量为“目标坐标 - 实际落点”。
 *   固件用3x3双线性插值把目标坐标先修正，再进入逆运动学。
 *
 * 当前实测数据：
 *   目标(0,-120)   实际(0,-112)    -> (  0, -8)
 *   目标(105,-120) 实际(95,-105)   -> (+10,-15)
 *   目标(210,-120) 实际(200,-105)  -> (+10,-15)
 *   目标(0,0)      实际(10,0)      -> (-10,  0)
 *   目标(105,0)    实际(105,0)     -> (  0,  0)
 *   目标(210,0)    实际(210,0)     -> (  0,  0)
 *   目标(0,120)    实际(0,105)     -> (  0,+15)
 *   目标(105,120)  实际(95,100)    -> (+10,+20)
 *   目标(210,120)  实际(200,105)   -> (+10,+15)
 */
#define ARM_KIN_COORD_CORRECTION_ENABLED       0U
#define ARM_KIN_COORD_CORRECTION_X0_MM         0.0F
#define ARM_KIN_COORD_CORRECTION_X1_MM       105.0F
#define ARM_KIN_COORD_CORRECTION_X2_MM       210.0F
#define ARM_KIN_COORD_CORRECTION_Y0_MM      (-120.0F)
#define ARM_KIN_COORD_CORRECTION_Y1_MM         0.0F
#define ARM_KIN_COORD_CORRECTION_Y2_MM       120.0F
#define ARM_KIN_COORD_CORRECTION_DX00_MM       0.0F
#define ARM_KIN_COORD_CORRECTION_DY00_MM      (-8.0F)
#define ARM_KIN_COORD_CORRECTION_DX10_MM      10.0F
#define ARM_KIN_COORD_CORRECTION_DY10_MM     (-15.0F)
#define ARM_KIN_COORD_CORRECTION_DX20_MM      10.0F
#define ARM_KIN_COORD_CORRECTION_DY20_MM     (-15.0F)
#define ARM_KIN_COORD_CORRECTION_DX01_MM     (-10.0F)
#define ARM_KIN_COORD_CORRECTION_DY01_MM       0.0F
#define ARM_KIN_COORD_CORRECTION_DX11_MM       0.0F
#define ARM_KIN_COORD_CORRECTION_DY11_MM       0.0F
#define ARM_KIN_COORD_CORRECTION_DX21_MM       0.0F
#define ARM_KIN_COORD_CORRECTION_DY21_MM       0.0F
#define ARM_KIN_COORD_CORRECTION_DX02_MM       0.0F
#define ARM_KIN_COORD_CORRECTION_DY02_MM      15.0F
#define ARM_KIN_COORD_CORRECTION_DX12_MM      10.0F
#define ARM_KIN_COORD_CORRECTION_DY12_MM      20.0F
#define ARM_KIN_COORD_CORRECTION_DX22_MM      10.0F
#define ARM_KIN_COORD_CORRECTION_DY22_MM      15.0F
/*
 * 实测表的全局比例。1.0表示完全按实测误差补偿；右下/右上远角全量
 * 补偿会把虚拟目标推到臂展外，因此先用0.6，保证GOTO 210 120可达。
 */
#define ARM_KIN_COORD_CORRECTION_SCALE         1.0F
/*
 * 坐标补偿自动降档步进。远角/高Z点如果按默认补偿比例不可达，逆解会
 * 自动降低补偿比例重试，最低降到0，优先保证巡边和调试动作能执行。
 */
#define ARM_KIN_COORD_CORRECTION_RETRY_STEP    0.2F

/*
 * A4右半区残差补偿：
 *   最新实测目标(160,-60)，实际落点(150,-50)，因此还需
 *   X方向+10mm、Y方向-10mm。
 *
 * 为避免X=105中线处产生坐标跳变，补偿从105mm开始线性介入，
 * 到160mm完全生效，之后保持(+10,-10)mm。该补偿不参与上面的
 * “不可达自动降档”，确保GOTO、视觉抓放和底座偏航始终使用相同
 * 的右半区修正量。
 */
#define ARM_KIN_RIGHT_RESIDUAL_COMP_ENABLED      0U
#define ARM_KIN_RIGHT_RESIDUAL_START_X_MM      105.0F
#define ARM_KIN_RIGHT_RESIDUAL_FULL_X_MM       160.0F
#define ARM_KIN_RIGHT_RESIDUAL_DX_MM             10.0F
#define ARM_KIN_RIGHT_RESIDUAL_DY_MM            (-10.0F)
/*
 * 对外的board_z_mm表示“电磁铁吸附面中心距A4纸面的高度”。
 * H已经直接使用肩轴到纸面的高度，因此纸面相对偏移必须为0。
 * 对外Z始终表示电磁铁吸附面中心到A4纸面的法向间隙。
 */
#define ARM_KIN_BOARD_PLANE_HEIGHT_MM           0.0F

/*
 * X-Z耦合高度补偿：
 *   不改变上位机/GOTO的Z定义，仍然表示离A4纸面高度；仅在逆解内部
 *   随X增大把目标Z预先压低。
 *
 * 旧表曾在X=160处下压90mm，已经远超合理的残差补偿量，并直接
 * 导致中远点贴纸。重新标定期间关闭；只有完成三高度全局打点后，
 * 才允许用新的二维Z增益/偏置表替代它。
 */
#define ARM_KIN_X_Z_COMP_ENABLED               0U
#define ARM_KIN_X_Z_COMP_POINT_COUNT           5U
#define ARM_KIN_X_Z_COMP_X_MM \
    {0.0F, 50.0F, 100.0F, 160.0F, 210.0F}
#define ARM_KIN_X_Z_COMP_Z_MM \
    {0.0F, 40.0F, 65.0F, 90.0F, 90.0F}

/*
 * 局部Z补偿：
 *   实测GOTO(30,-105,5)区域电磁铁偏高，额外下压5mm。
 *   采用矩形线性衰减，避免只在单点产生姿态突变。
 */
#define ARM_KIN_LOCAL_Z_COMP_ENABLED           0U
#define ARM_KIN_LOCAL_Z_COMP_CENTER_X_MM      30.0F
#define ARM_KIN_LOCAL_Z_COMP_CENTER_Y_MM    (-105.0F)
#define ARM_KIN_LOCAL_Z_COMP_HALF_X_MM        35.0F
#define ARM_KIN_LOCAL_Z_COMP_HALF_Y_MM        35.0F
#define ARM_KIN_LOCAL_Z_COMP_Z_MM              5.0F

/*
 * 负Y远端Z补偿：
 *   实测沿Y负轴接近-140mm时，末端会被拉高。该误差主要和底座旋转
 *   后的斜向距离/受力有关，不应写进X轴补偿。这里从Y=-100mm开始
 *   逐渐下压，到A4上边缘附近完全生效。
 */
#define ARM_KIN_NEG_Y_Z_COMP_ENABLED           0U
#define ARM_KIN_NEG_Y_Z_COMP_START_Y_MM     (-100.0F)
#define ARM_KIN_NEG_Y_Z_COMP_FULL_Y_MM      (-148.5F)
#define ARM_KIN_NEG_Y_Z_COMP_Z_MM              7.0F

/*
 * 实物确认：全部舵机90°时，腕轴到电磁铁中心的80mm向量竖直向上；
 * J2增大时向A4方向转动。抓放时需要电磁铁中心位于腕轴正下方，因此
 * 该向量的绝对侧视俯仰设为-90°（+X为0°、向上为+90°）。
 */
#define ARM_KIN_TOOL_PITCH_DEG              (-90.0F)

/*
 * 远点末端俯仰补偿：
 *   理论IK会让电磁铁吸附面保持与A4平行，但远点受重力、舵机虚位和
 *   连杆弹性影响，实物末端会出现俯仰偏差。这里按底座到目标点的
 *   水平距离线性修正目标工具俯仰角，重新进入IK联动计算J2/J3/J4。
 *
 * 正值会让J2/J3/J4组合把电磁铁向“更抬头”的方向补偿；若实物远点
 * 变成前端翘起，则把最大补偿改成负值。
 */
#define ARM_KIN_FAR_POINT_PITCH_COMP_ENABLED   0U
#define ARM_KIN_FAR_POINT_PITCH_START_MM     180.0F
#define ARM_KIN_FAR_POINT_PITCH_FULL_MM      290.0F
#define ARM_KIN_FAR_POINT_PITCH_MAX_DEG        7.0F

/*
 * 数学关节角到当前0~180度逻辑舵机角的映射：
 *   logical = zero + direction * joint_angle
 *
 * 这些值与舵盘安装齿位直接相关。首次带连杆运行前必须参考说明文档，
 * 通过USART2逐关节低速验证；若方向或零位不同，只修改本组参数。
 */
#define ARM_KIN_MAGNET_ZERO_ANGLE_X10        900
#define ARM_KIN_MAGNET_DIRECTION               1
/*
 * 三个关节各自按数学关节角独立映射。零位以A4原点Z=10mm处的
 * 2026-08-01使用GOTO(80,0,20)实机标定：原逆解输出J2=144.9°，
 * 手动调到J2=129.0°时电磁铁面与A4纸面平行，因此腕部逻辑零位
 * 由106.4°减少15.9°，修正为90.5°。
 * 这里只修正腕部安装零偏；IK仍会随J3/J4联动改变J2，不能把J2
 * 在所有坐标强制锁死为129°。
 */
#define ARM_KIN_WRIST_ZERO_ANGLE_X10         905
/*
 * 实机运动方向及最新两点标定：
 *   腕部零偏由GOTO(80,0,20)处J2=129°水平姿态重新标定。
 *   以新测H=63mm和GOTO(0,0,10)的几何逆解为锚点，重新计算零位。
 *   J3比例继续使用“J3约80°时两臂近似直角”的两点标定结果。
 * 因此当前近似映射为：
 *   J2 = 90.5° - q_wrist
 *   J3 = -16.4° - 1.074719*q_elbow
 *   J4 = 177.4° - q_shoulder
 * PWM的0~180°定义不反转，SERVO命令仍直接表示物理舵机角。
 */
#define ARM_KIN_WRIST_DIRECTION               (-1)
#define ARM_KIN_ELBOW_ZERO_ANGLE_X10         (-164)
#define ARM_KIN_ELBOW_DIRECTION                (-1)
#define ARM_KIN_ELBOW_GAIN                1.074719F
#define ARM_KIN_SHOULDER_ZERO_ANGLE_X10     1774
#define ARM_KIN_SHOULDER_DIRECTION             (-1)
/*
 * 底座0°转向操作者所述的原点右侧，180°转向原点左侧；按90°正对
 * A4原点建立初始映射。题面偏航角向+Y（照片下方）为正，因此舵机角
 * 随偏航角增大而减小。首次巡边前必须先用 SERVO 5 90 2000 验证。
 */
#define ARM_KIN_BASE_FORWARD_ANGLE_X10        900
#define ARM_KIN_BASE_DIRECTION                  (-1)

/*
 * A4六点实机角度拟合补偿，单位0.1°，每行顺序为J1、J2、J3、J4、J5。
 *
 * 补偿量 = 距A4纸面5mm高度实机调准角 - 几何逆解角。
 * P1、P2为完整实测；P3、P4、P5使用已实测关节和当前拟合缺失关节；
 * P6暂用P1/P2径向拟合值。取得更多实测值后只修改本表即可。
 *
 * J2/J3/J4每行补偿量之和严格为0，保证补偿前后均满足
 * J2+J3+J4=353°，电磁铁吸附面保持与A4平行。
 */
#define ARM_KIN_BOARD_CORRECTION_ENABLED       0U
#define ARM_KIN_CORRECTION_P1_X10              { 0, 22, -51,  29, -78}
#define ARM_KIN_CORRECTION_P2_X10              { 0, 84, -63, -21, -39}
#define ARM_KIN_CORRECTION_P3_X10              { 0, 87, -68, -19,   0}
#define ARM_KIN_CORRECTION_P4_X10              { 0,114, -93, -21, -21}
#define ARM_KIN_CORRECTION_P5_X10              { 0, 52, -31, -21,  57}
#define ARM_KIN_CORRECTION_P6_X10              { 0,-37, -68, 105, -10}

/* ------------------------- A4六点坐标校准 ----------------------------- */
/*
 * 六点顺序：
 *   P1左上 -> P2右上 -> P3右中 -> P4右下 -> P5左下 -> P6左中。
 * 高度是电磁铁中心距纸面的理论高度；P2/P4在高Z时接近极限，
 * 因此点检先保持30mm安全高度，确认无碰撞后再逐步降低。
 * 校准过程不吸合电磁铁。
 */
#define ARM_CALIBRATION_POINT_COUNT             6U
#define ARM_CALIBRATION_ALL_POINTS_MASK       0x3FU
#define ARM_CALIBRATION_A4_WIDTH_MM           210.0F
#define ARM_CALIBRATION_A4_HALF_HEIGHT_MM     148.5F
#define ARM_CALIBRATION_HEIGHT_MM              30.0F
#define ARM_CALIBRATION_MAGNET_ANGLE_DEG        0.0F
#define ARM_CALIBRATION_MOVE_DURATION_MS       2500U
#define ARM_CALIBRATION_APPROACH_DURATION_MS   3000U
#define ARM_CALIBRATION_VERTICAL_DURATION_MS   2000U
#define ARM_CALIBRATION_HOME_DURATION_MS       3000U
#define ARM_CALIBRATION_DWELL_MS               1500U
#define ARM_CALIBRATION_MOTION_TIMEOUT_MARGIN_MS 1000U
#define ARM_CALIBRATION_QUEUE_DEPTH               1U
/*
 * 旧版连续巡边参数保留备用。当前CALIB START执行逐点点检：
 * (0,0)->HOME->P1->HOME->P2...，不再沿边框连续插补。
 */
#define ARM_CALIBRATION_EDGE_STEP_MM            10.0F
#define ARM_CALIBRATION_EDGE_SPEED_MM_S         15.0F
#define ARM_CALIBRATION_EDGE_MIN_DURATION_MS     200U

/* ------------------------- 通用抓取/放置动作参数 ------------------------ */
#define ARM_PUZZLE_PLAN_QUEUE_DEPTH            1U       // 只允许一个计划在执行
/*
 * 视觉坐标到机械臂执行坐标的额外偏移。
 *
 * 当前以USART2手动GOTO为坐标标定基准：GOTO准，视觉自动抓取就按
 * 同一套坐标进入二维补偿表和逆运动学。因此这里保持0，避免视觉坐标
 * 相比GOTO多加一次偏移。
 */
#define ARM_PUZZLE_VISION_TO_ARM_X_OFFSET_MM   0.0F
#define ARM_PUZZLE_VISION_TO_ARM_Y_OFFSET_MM   0.0F

#define ARM_PUZZLE_TRAVEL_HEIGHT_MM           60.0F     // 运动中安全高度，避免碰撞题面
#define ARM_PUZZLE_PICK_HEIGHT_MM             0.0F     // 抓取高度，电磁铁吸附面距A4纸面
#define ARM_PUZZLE_PLACE_HEIGHT_MM            1.0F     // 放置高度，电磁铁吸附面距A4纸面
/*
 * 视觉抓取专用Z微调。当前目标0mm仍明显偏高、无法吸片，因此将
 * pick_surface内部目标下压5mm。仅作用于视觉抓取，不改变GOTO、
 * 校准、搬运和放置高度；若出现明显压纸应减小绝对值。
 */
#define ARM_PUZZLE_PICK_Z_TRIM_MM             (-9.0F)
/*
 * 视觉抓取X向高度补偿：实机近X=0区域的电磁铁实际高度高于远点，
 * 同一抓取Z会导致近点吸不起。这里只修正pick_surface，不影响GOTO、
 * 放置高度和安全搬运高度。X=0额外下压5mm，至X=210线性衰减为0。
 * 若近点仍偏高，增大NEAR值的负数绝对值；若开始压纸则减小绝对值。
 */
#define ARM_PUZZLE_PICK_X_Z_COMP_ENABLED        1U
#define ARM_PUZZLE_PICK_X_Z_COMP_NEAR_X_MM      0.0F
#define ARM_PUZZLE_PICK_X_Z_COMP_FAR_X_MM     210.0F
#define ARM_PUZZLE_PICK_X_Z_COMP_NEAR_MM       (-5.0F)
#define ARM_PUZZLE_PICK_X_Z_COMP_FAR_MM          0.0F
/*
 * 视觉放置专用Z微调。设置5mm时实测仍超过10mm，放置阶段还包含
 * J1目标旋转和铁片负载，不能与抓取微调共用。先额外下压5.5mm；
 * 只影响place_surface，不改变GOTO、抓取和安全搬运高度。
 */
#define ARM_PUZZLE_PLACE_Z_TRIM_MM            (-8.5F)
/*
 * 自动抓放表面高度表：
 *   当前关闭，视觉抓取/放置使用固定高度：
 *   ARM_PUZZLE_PICK_HEIGHT_MM / ARM_PUZZLE_PLACE_HEIGHT_MM。
 *   后续若确实需要按区域修正抓取高度，再打开本表。
 */
#define ARM_PUZZLE_SURFACE_Z_TABLE_ENABLED      0U
#define ARM_PUZZLE_SURFACE_Z_X0_MM              0.0F
#define ARM_PUZZLE_SURFACE_Z_X1_MM            105.0F
#define ARM_PUZZLE_SURFACE_Z_X2_MM            210.0F
#define ARM_PUZZLE_SURFACE_Z_Y0_MM           (-120.0F)
#define ARM_PUZZLE_SURFACE_Z_Y1_MM              0.0F
#define ARM_PUZZLE_SURFACE_Z_Y2_MM            120.0F
#define ARM_PUZZLE_SURFACE_Z_00_MM              5.0F
#define ARM_PUZZLE_SURFACE_Z_10_MM             15.0F
#define ARM_PUZZLE_SURFACE_Z_20_MM             25.0F
#define ARM_PUZZLE_SURFACE_Z_01_MM              5.0F
#define ARM_PUZZLE_SURFACE_Z_11_MM             15.0F
#define ARM_PUZZLE_SURFACE_Z_21_MM             25.0F
#define ARM_PUZZLE_SURFACE_Z_02_MM              5.0F
#define ARM_PUZZLE_SURFACE_Z_12_MM             15.0F
#define ARM_PUZZLE_SURFACE_Z_22_MM             25.0F

#define ARM_PUZZLE_APPROACH_DURATION_MS       1500U      // 移动到目标点正上方安全高度
#define ARM_PUZZLE_VERTICAL_DURATION_MS       1200U      // 安全高度与抓/放表面之间的低速升降时间
#define ARM_PUZZLE_TRANSFER_DURATION_MS       1500U      // 末端在安全高度水平移动的时间
#define ARM_PUZZLE_PICK_LIFT_SEGMENTED          0U       // 关闭中间高度，吸合后一次直接抬到安全高度
#define ARM_PUZZLE_LOCK_WRIST_AFTER_PICK        1U       // 吸合后抬升阶段锁住J2，到达安全高度后再参与搬运联动
#define ARM_PUZZLE_PICK_LIFT_DIRECT_DURATION_MS 2000U    // 抓取高度直接抬到安全高度的时间
#define ARM_PUZZLE_PICK_LIFT_MID_DURATION_MS  1000U       // 抓取高度到中间高度的时间
#define ARM_PUZZLE_PICK_LIFT_SAFE_DURATION_MS 1000U      // 中间高度到安全高度的时间
#define ARM_PUZZLE_HOME_DURATION_MS           1000U      // 末端从安全高度上升到初始位置的时间

/*
 * 任务3独立运动速度参数。运动控制采用“指定完成时间”，数值越大越慢。
 * 只要视觉任务来源为3（按键3、TASK3或VFRAME 3），便使用本组参数；
 * 任务1和任务2仍使用上面的通用参数。每项不得超过30000ms。
 */
#define ARM_PUZZLE_TASK3_APPROACH_DURATION_MS       1000U // 前往抓取点安全高度
#define ARM_PUZZLE_TASK3_VERTICAL_DURATION_MS       1500U // 抓取/放置垂直升降
#define ARM_PUZZLE_TASK3_TRANSFER_DURATION_MS       1500U // 安全高度边移动边旋转
#define ARM_PUZZLE_TASK3_PICK_LIFT_DIRECT_MS        1000U // 吸取后直接抬到安全高度
#define ARM_PUZZLE_TASK3_PICK_LIFT_MID_MS           1250U // 分段抬升第一段（启用分段时）
#define ARM_PUZZLE_TASK3_PICK_LIFT_SAFE_MS          1250U // 分段++抬升第二段（启用分段时）
#define ARM_PUZZLE_TASK3_HOME_DURATION_MS           1500U // 完成后回HOME

#define ARM_PUZZLE_PRE_PICK_SETTLE_MS         800U      // 下降到抓取高度后先稳定，再吸合电磁铁
#define ARM_PUZZLE_MAGNET_ON_SETTLE_MS        900U      // 电磁铁吸合后等待时间，确保吸附稳定
#define ARM_PUZZLE_PRE_PLACE_SETTLE_MS        1500U      // 下降到放置高度后等待，待贴片稳定再断开电磁铁
#define ARM_PUZZLE_MAGNET_OFF_SETTLE_MS       500U      // 电磁铁断电后等待时间，确保释放稳定
#define ARM_PUZZLE_DONE_DELAY_MS               300U      // 最后一块断开电磁铁后延时，再输出完成高电平
#define ARM_PUZZLE_DONE_PULSE_MS               300U      // 完成蜂鸣器高电平脉冲宽度，结束后自动拉低
#define ARM_PUZZLE_MOTION_TIMEOUT_MARGIN_MS  1000U      // 运动超时保护，避免机械卡死
#define ARM_PUZZLE_PLAN_TIMEOUT_MS         130000U      // 单个拼图计划最长执行时间，避免死循环
#define ARM_PUZZLE_RETURN_HOME                 1U       // 拼图完成后是否返回初始位置

/*
 * 原夹爪舵机PE9已停用，复用为拼图完成/有源蜂鸣器触发输出。
 * 最后一块断开电磁铁并等待ARM_PUZZLE_DONE_DELAY_MS后输出高电平，
 * 持续ARM_PUZZLE_DONE_PULSE_MS后自动拉低。只能接高电平触发蜂鸣器
 * 模块或MOSFET驱动板输入，不得由GPIO直接驱动蜂鸣器线圈。
 */
#define ARM_PUZZLE_DONE_GPIO_PORT             GPIOE
#define ARM_PUZZLE_DONE_GPIO_PIN              GPIO_PIN_9
#define ARM_PUZZLE_DONE_ACTIVE_LEVEL          GPIO_PIN_SET

/* ------------------------------ 电磁铁 -------------------------------- */
#define ARM_MAGNET_GPIO_PORT               GPIOD
#define ARM_MAGNET_GPIO_PIN                GPIO_PIN_12
#define ARM_MAGNET_ACTIVE_LEVEL            GPIO_PIN_SET

/* 设为 0 可关闭电磁铁最长吸合时间保护。 */
#define ARM_MAGNET_MAX_ON_TIME_MS           18000U

/* ------------------------------ 急停输入 ------------------------------- */
/*
 * 默认关闭急停输入，避免未接线时误触发。
 * 如需启用：改为 1，并将常闭急停开关接在 PD13 与 GND 之间。
 */
#define ARM_ESTOP_ENABLED                  0U
#define ARM_ESTOP_GPIO_PORT                GPIOD
#define ARM_ESTOP_GPIO_PIN                 GPIO_PIN_13
#define ARM_ESTOP_ACTIVE_LEVEL             GPIO_PIN_RESET

/* ------------------------------ 通信看门狗 ----------------------------- */
/*
 * 默认关闭。若比赛策略要求控制通信掉线后自动停机，可设为 1。
 * 超时只停止新动作并关闭电磁铁，不会复位 MCU。
 */
#define ARM_COMM_WATCHDOG_ENABLED          0U
#define ARM_COMM_WATCHDOG_TIMEOUT_MS       2000U

#endif /* ARM_CONFIG_H */
