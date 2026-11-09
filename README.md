# 2026 电赛 E 题 · 拼图装置（五轴机械臂 + 电磁铁）

基于 **STM32F407VET6 + FreeRTOS** 的五自由度舵机机械臂固件。上位视觉模块（K230）识别拼图碎片的抓取点、放置点与旋转角，通过 USART1 下发定长计划帧；本固件完成**坐标变换 → 逆运动学 → 五路舵机目标角 → 平滑插值 → 电磁铁吸取 / 搬运 / 旋转 / 放置 → 完成提示**的全流程控制。

> 仓库内容为 2026 年电赛 E 题备赛期的实机工程整理：包含可编译的完整 MDK/EIDE 工程、实机标定参数、串口协议实现、二分调试命令集与配套调试脚本。

---

## 目录

- [1. 功能概览](#1-功能概览)
- [2. 硬件平台](#2-硬件平台)
- [3. 软件架构](#3-软件架构)
- [4. 数据流与执行时序](#4-数据流与执行时序)
- [5. 坐标系与逆运动学](#5-坐标系与逆运动学)
- [6. 通信协议](#6-通信协议)
- [7. 拼图动作流程](#7-拼图动作流程)
- [8. 标定与调试](#8-标定与调试)
- [9. 构建与烧录](#9-构建与烧录)
- [10. 目录结构](#10-目录结构)
- [11. 关键配置项](#11-关键配置项)
- [12. 已知限制](#12-已知限制)
- [13. 文档索引](#13-文档索引)
- [14. 第三方组件](#14-第三方组件)

---

## 1. 功能概览

| 能力 | 说明 |
|---|---|
| 视觉联动 | USART1 接收 52 字节定长计划帧（2～4 片），校验通过后回 `0xAC` |
| 自动拼图 | 逐片执行「安全高度 → 抓取面 → 吸合 → 抬升 → 搬运 + 旋转 → 放置面 → 释放 → 回安全高度」 |
| 解析逆解 | 底座偏航 + 平面三连杆（肩 / 肘 / 腕），始终保持电磁铁吸附面平行 A4 |
| 整单预解算 | 一套计划的全部抓放姿态先全部求解，任一点不可达则整单拒绝，不会拼到一半停在纸面 |
| 端点旋转 | 视觉给出的世界旋转角扣除底座自身转动后，由 J1 补足；超程时抓取端与放置端分摊 |
| 平滑运动 | 10 ms 周期三次 S 曲线插值，起点 / 终点速度为 0；每段按「指定完成时间」调速 |
| 重复帧免疫 | 52 字节帧做 FNV-1a 签名（混入任务号），重发只回 ACK 不重复动作 |
| A4 校准备 | 六点 `CALIB` 点检 / 巡边、`IK` 只解算不动作、`GOTO` 直接驱动机器人 |
| 按键面板 | 10 个独立按键：任务 1/2/3、三个标定点、HOME、STOP、视觉校准 |
| 安全保护 | 舵机软限位、运动超时、电磁铁最长吸合、可选急停与通信看门狗 |
| 完成提示 | 最后一片释放后 PE9 输出 300 ms 高电平（蜂鸣器 / MOSFET 模块） |

---

## 2. 硬件平台

### 2.1 系统组成

```text
        ┌──────────────┐   52B 计划帧 / 0xAC      ┌────────────────────┐
        │  K230 视觉   │ ◄──────────────────────► │  STM32F407VET6     │
        │  (OpenMV 系) │   0x3A/0x3B/0x3C 触发    │  FreeRTOS @168MHz  │
        └──────────────┘                          └─────────┬──────────┘
                                                            │
                       USART2 文本 CLI                      │  50Hz PWM ×5 + GPIO
                  ┌─────────────────────┐                   │
                  │ 电脑 USB-TTL / 串口助手│                  │
                  └─────────────────────┘                   │
                                        ┌───────────────────┴───────────────────┐
                                        │  5× 舵机（J1 末端旋转 / J2 腕 / J3 肘  │
                                        │  J4 肩 / J5 底座） + 电磁铁 + 蜂鸣器    │
                                        └───────────────────────────────────────┘
```

- **MCU**：STM32F407VET6，HSE 8 MHz → PLL（M=4 / N=168 / P=2）→ **168 MHz**，APB2 定时器时钟 168 MHz。
- **执行机构**：5 路 50 Hz 标准舵机（500～2500 μs 对应 0～180°）+ 电磁铁（继电器 / MOSFET 驱动）。
- **末端**：电磁铁吸附面中心到腕轴 80 mm，抓取时该向量竖直向下（绝对俯仰 −90°）。
- **电源**：舵机与电磁铁必须独立大电流供电，逻辑地与功率地共地。

### 2.2 关键引脚

| 功能 | 外设 | 引脚 | 说明 |
|---|---|---|---|
| J1 电磁铁末端旋转 | TIM1_CH2 | PE11 | 50 Hz PWM |
| J2 腕部俯仰 | TIM1_CH3 | PE13 | 50 Hz PWM |
| J3 肘部 | TIM1_CH4 | PE14 | 50 Hz PWM |
| J4 肩部 | TIM8_CH1 | PC6 | 50 Hz PWM |
| J5 底座旋转 | TIM8_CH2 | PC7 | 50 Hz PWM |
| 视觉 TX / RX | USART1 | PB6 / PB7 | 115200 8N1，DMA + 空闲中断 |
| 调试 TX / RX | USART2 | PA2 / PA3 | 115200 8N1，DMA + 空闲中断 |
| 电磁铁控制 | GPIO 输出 | PD12 | 高电平吸合 |
| 完成指示 / 蜂鸣器 | GPIO 输出 | PE9 | 高电平有效，脉冲 300 ms |
| 急停输入（预留） | GPIO 输入 | PD13 | 低电平有效，默认关闭 |
| 按键 1～10 | GPIO 输入 | PE0～PE8、PE10 | 内部上拉，按下为低 |
| SWD 下载 | SWDIO / SWCLK | PA13 / PA14 | Keil / EIDE 烧录 |

完整接线、继电器接法、供电与安全事项见 [`硬件接线表.md`](硬件接线表.md)。

---

## 3. 软件架构

### 3.1 分层

```text
┌──────────────────────────────────────────────────────────────┐
│ App/            业务层：协议、逆解、动作状态机、CLI、安全      │
│  arm_app  arm_comm  arm_puzzle  arm_puzzle_protocol           │
│  arm_kinematics  arm_calibration  arm_motion  arm_servo       │
│  arm_debug_cli  arm_buttons  arm_safety  arm_protocol         │
├──────────────────────────────────────────────────────────────┤
│ BSP/            板级驱动：PWM、双路 UART+DMA、磁铁、指示灯     │
├──────────────────────────────────────────────────────────────┤
│ Core/           CubeMX 生成：时钟、GPIO、TIM1/8、USART1/2、中断│
│ Middlewares/    FreeRTOS 内核 + CMSIS-RTOS V2                  │
│ Drivers/        STM32F4xx HAL + CMSIS                          │
└──────────────────────────────────────────────────────────────┘
```

所有可调参数集中在 [`App/Inc/arm_config.h`](App/Inc/arm_config.h)，修改零位、方向、限位、波特率、时序、补偿表都只改这一处，不需要改驱动源码。

### 3.2 FreeRTOS 任务

| 任务 | 优先级 | 栈（word） | 职责 |
|---|---:|---:|---|
| `ArmSafety` | 48 | 256 | 急停、电磁铁超时、通信看门狗 |
| `ArmMotion` | 40 | 512 | 关节插值、完成计数、STOP 处理 |
| `ArmComm` | 32 | 512 | USART1 计划帧、USART2 文本命令、ACK |
| `ArmPuzzle` | 30 | 768 | 计划预解算与抓放状态机 |
| `ArmCalibration` | 30 | 640 | 六点巡边 / 点检 |
| `ArmButtons` | 28 | 256 | 10 键扫描与防抖 |
| `defaultTask` | 24 | 128 | CubeMX 保留 |

任务间只通过队列与完成计数交互：

- `s_plan_queue`（深度 1）：拼图计划；
- `s_motion_queue`（深度 4）：逐段舵机目标；
- `s_event_queue`（深度 8）：动作 / 安全事件 → USART2 文本输出；
- `ArmMotion_GetCompletionCount()`：每个运动段的完成序号，拼图任务据此阻塞等待，不忙等。

### 3.3 模块职责

| 模块 | 关键内容 |
|---|---|
| `arm_puzzle_protocol` | 52 字节帧的状态机解析、帧头 / 版本 / 数量 / 帧尾 / 累加和校验、大端解码、范围检查、FNV-1a 签名 |
| `arm_kinematics` | A4 坐标 → 底座角 + 三连杆解析逆解；补偿链；关节角 → 舵机角映射 |
| `arm_puzzle` | 计划去重、整单预解算、分段动作执行、完成提示、状态发布 |
| `arm_motion` | 10 ms 插值、三次平滑、`stop_epoch` 作废机制、队列重置 |
| `arm_servo` | 逻辑角 ↔ 脉宽映射、限位校验、HOME 角度 |
| `arm_comm` | 双串口轮询、视觉触发、ACK、注入帧 |
| `arm_debug_cli` | 文本命令行、`IK` / `GOTO` / `CALIB` / `VREAD` / `STATUS` 等 |
| `arm_calibration` | A4 六点定义、可达性检查、点检与巡边 |
| `arm_buttons` | 按键动作表、忙状态保护 |
| `arm_safety` | 安全轮询与停机事件 |

---

## 4. 数据流与执行时序

```text
按键 / 串口 TASK1          K230 视觉
      │                       │
      ├── USART1: 0x3A ──────►│  识别、生成计划
      │                       │
      │◄──── 52B 计划帧 ──────┤
      │                       │
 ArmComm: 校验 → 签名 → 判重 → 提交队列
      │
      ├── USART1: 0xAC ──────►│  K230 停止重发
      │
 ArmPuzzle: 4 片 × 6 姿态全部逆解
      │         └─ 任一点不可达 → 整单拒绝（ERR ... IK_UNREACHABLE）
      │
      ├── 逐段下发 ArmMotionCommand（5 路目标角 + 时长）
      │        └─ ArmMotion: 10ms × N 三次平滑插值 → PWM
      │
      ├── 电磁铁 PD12 通断 + 稳定延时
      │
      ├── 最后一片：PE9 高电平 300ms
      │
      └── 回 HOME → USART2: DONE PUZZLE OK (0)
```

---

## 5. 坐标系与逆运动学

### 5.1 题面坐标系

```text
原点：A4 左边线 与 中间分界线 的交点
  X：向右为正，范围 0 ~ 210 mm
  Y：向下为正，范围 −148.5 ~ +148.5 mm
  Z：从纸面向上为正（对外表示电磁铁吸附面中心到纸面的法向间隙）
底座旋转中心在纸面的投影：(−75, 0) mm
机械臂位于 A4 左侧，正前方为 +X
```

### 5.2 几何参数

| 参数 | 值 | 含义 |
|---|---:|---|
| `L1` | 165.0 mm | 肩轴 → 肘轴 |
| `L2` | 153.0 mm | 肘轴 → 腕轴 |
| `L3` | 80.0 mm | 腕轴 → 电磁铁吸附面中心 |
| `H` | 63.0 mm | 肩轴中心到 A4 纸面 |
| `D0` | 10.0 mm | 底座旋转轴 → 肩轴径向偏置 |
| `Zboard` | 0.0 mm | 纸面相对基准的附加高度 |
| `alpha` | −90.0° | 工具绝对俯仰（吸附面平行 A4） |

### 5.3 求解过程

```text
dx = X_eff − base_x,  dy = Y − base_y
q_base = atan2(dy, dx)
r      = hypot(dx, dy) − D0
px     = r
pz     = Z + Zboard − H

wx = px − L3·cos(alpha)
wz = pz − L3·sin(alpha)

c2 = (wx² + wz² − L1² − L2²) / (2·L1·L2)
q_elbow    = atan2(±√(1−c2²), c2)          # 先试负肘分支
q_shoulder = atan2(wz, wx) − atan2(L2·sin(q_elbow), L1 + L2·cos(q_elbow))
q_wrist    = alpha − q_shoulder − q_elbow
```

两个肘部分支都要换算成五路舵机角并检查限位；只有全部合法的分支才会被采用，**不会把越界角裁剪后当作可达姿态**。

### 5.4 关节角 → 舵机角

以 0.1° 为单位，四舍五入后输出：

```text
J1 =  900 + 10 × magnet_relative_deg          # 末端旋转
J2 =  905 − 10 × q_wrist                      # 腕部俯仰
J3 = −164 − 10 × 1.074719 × q_elbow           # 肘部（含非 1:1 连杆比例）
J4 = 1774 − 10 × q_shoulder                   # 肩部
J5 =  900 − 10 × q_base                       # 底座（90° 正对 A4 原点）
```

上电 HOME：`J1=90°、J2=173°、J3=90°、J4=90°、J5=0°`。

### 5.5 实机补偿链

`ArmKinematics_SolveBoardPose()` 在逆解前依次执行：

1. **Z 向补偿**：X-Z 耦合表、局部点补偿、负 Y 远端补偿（当前均为关闭）；
2. **5×3 实测残差网格**（`ARM_KIN_XY_RESIDUAL_ENABLED = 1`）：按 X = 10/55/105/155/200、Y = −135/0/+135 的实测落点误差做双线性插值，再以 4 次定点迭代反算「为到达目标应送入 IK 的坐标」；
3. **单轴反标定表**、3×3 坐标补偿表、右半区残差（当前关闭，保留数据结构）；
4. **A4 六点角度补偿**（当前关闭）：左 / 右边界、上 / 下半区分段双线性插值，并由肘、肩补偿反算腕部补偿以保持末端俯仰。

若某个补偿比例下不可达，会按 0.2 的步长自动降档重试，优先保证动作可执行。

---

## 6. 通信协议

### 6.1 USART1：视觉计划帧（115200 8N1）

```text
偏移  长度  内容
0     2     AA 55            帧头
2     1     01               协议版本
3     1     SEQ              计划序号
4     1     COUNT            碎片数量，2 ~ 4
5     44    4 × 11 字节记录
49    1     CHECKSUM         offset 2..48 的 8 位累加和
50    2     55 CC            帧尾
```

单条记录（11 字节，多字节均大端）：

| 偏移 | 长度 | 字段 | 单位 / 范围 |
|---|---:|---|---|
| 0 | 1 | `id` | 碎片编号 |
| 1 | 2 | `pick_x` | 0.1 mm，0 ~ 2100 |
| 3 | 2 | `pick_y` | 0.1 mm，−1485 ~ 1485 |
| 5 | 2 | `place_x` | 0.1 mm，0 ~ 2100 |
| 7 | 2 | `place_y` | 0.1 mm，−1485 ~ 1485 |
| 9 | 2 | `rotate` | 0.01°，(−18000, 18000] |

**触发与应答**

| 方向 | 字节 | 含义 |
|---|---|---|
| STM32 → K230 | `0x3A` | 启动任务 1（按键 1 / `TASK1`） |
| STM32 → K230 | `0x3B` | 启动任务 2 |
| STM32 → K230 | `0x3C` | 启动任务 3（独立速度参数） |
| STM32 → K230 | `0x3F` | 视觉校准（按键 9） |
| STM32 → K230 | `0xAD` | 视觉辅助校准（按键 10） |
| STM32 → K230 | `0xAC` | 计划已接受 / 重复包已确认 |

**安全语义**

- 未收到触发命令时，任何残留 52 字节帧都会被忽略（`s_active_vision_task == 0`）；
- 解析器支持半包、粘包、前导噪声，接收满 52 字节即重新搜索帧头；
- 同一计划重发只回 `0xAC`，不重复执行；正在执行另一套计划时不回 ACK，由视觉端重试；
- 签名混入任务号，任务 1 与任务 2 即使坐标相同也能各执行一次；
- `VREAD` 模式下只解析并打印坐标，**绝不**驱动舵机或电磁铁。

### 6.2 USART2：调试命令行（115200 8N1）

ASCII 文本命令，大小写不敏感，`\r\n` 或单个 `\r`/`\n` 结束。发送 `HELP` 可打印完整列表。

| 命令 | 作用 |
|---|---|
| `PING` | 返回 `OK PONG` |
| `SERVO <1-5> <ANGLE> [TIME_MS]` | 单舵机低速点动（标定方向 / 零位） |
| `BASE <ANGLE> [TIME_MS]` | 底座单轴 |
| `MOVE <J1> <J2> <J3> <J4> <J5> [TIME_MS]` | 五轴目标角 |
| `HOME [TIME_MS]` | 回 HOME |
| `MAGNET <ON\|OFF>` | 电磁铁通断 |
| `TASK1` / `TASK2` / `TASK3` | 向视觉发送触发并等待计划帧 |
| `VREAD <1\|2\|3>` | 只读取并打印视觉帧，不动作 |
| `VSIM <1\|2\|3>` | 注入内置两片测试帧，完整走通「解析→ACK→执行」 |
| `VFRAME <1\|2\|3> <104 个 HEX 字符>` | 注入自定义 52 字节帧（逐字节校验） |
| `IK <X> <Y> <Z> <MAG_DEG>` | 仅解算并返回五路角度，**不驱动舵机** |
| `GOTO <X> <Y> <Z> <MAG_DEG> [TIME_MS]` | 解算并移动，不自动吸放 |
| `CALIB CHECK` | 检查六点可达性并打印各点角度 |
| `CALIB START` | 30 mm 高度逐点点检：`(0,0) → HOME → P1 → HOME → … → P6 → HOME` |
| `CALIB POINT <1-6>` | 移动到单个标定点 |
| `CALIB STATUS` / `CALIB STOP` | 校准状态 / 中止 |
| `STATUS` | 舵机、运动、拼图、视觉统计 |
| `STOP` | 停止拼图、校准、运动并关闭电磁铁 |

示例：

```text
> IK 105 0 20 0
< IK J1=90.0 J2=... J3=... J4=... J5=... BASE=... SHOULDER=... ELBOW=... WRIST=...

> GOTO 0 0 5 0 2000
< OK GOTO X=0.0 Y=0.0 Z=5.0 M=0.0 J1=... J2=... J3=... J4=... J5=... TIME=2000

> TASK1
< OK TASK1 TX=3A WAIT_VISION_FRAME
```

`IK` / `GOTO` 的角度字段格式为 `整数.十分位`（例如 `J2=129.0`）。

命令行也提供 Python 封装：

```powershell
python tools/arm_pc_serial.py --port COM5 goto 105 0 20 0 --duration 2000
python tools/arm_pc_serial.py --port COM5 status
python tools/arm_pc_serial.py --port COM5 magnet on
```

### 6.3 USART2：事件输出

| 事件 | 输出 |
|---|---|
| 拼图完成 | `DONE PUZZLE OK (0)` |
| 逆解失败 | `ERR PUZZLE IK_UNREACHABLE (14)` |
| 被 STOP 中止 | `ERR PUZZLE STOPPED (9)` |
| 校准单点完成 | `DONE CALIB POINT=<n> OK (0)` |
| 安全事件 | `ALARM ESTOP (10)` / `ALARM MAGNET_TIMEOUT (11)` / `ALARM COMM_TIMEOUT (12)` |
| 只读视觉帧 | `OK VREAD ACK=AC NO_MOTION` |

---

## 7. 拼图动作流程

### 7.1 单片刻度

| 阶段 | 目标姿态 | 默认时长（任务 1/2） | 任务 3 |
|---|---|---:|---:|
| 移动到抓取点上方 | `pick_safe`（Z = 60 mm） | 1500 ms | 1000 ms |
| 下降到抓取面 | `pick_surface`（Z = 0 + 微调） | 1200 ms | 1500 ms |
| 稳定后吸合 | 保持姿态 → PD12 高 | 800 ms → 900 ms | 同 |
| 抬升 | `pick_lift_safe`（J2 可锁定抓取角） | 2000 ms | 1000 ms |
| 搬运 + 旋转 | `place_safe`（Z = 60 mm） | 1500 ms | 1500 ms |
| 下降到放置面 | `place_surface`（Z = 1 + 微调） | 1200 ms | 1500 ms |
| 稳定后释放 | 保持 → PD12 低 | 1500 ms → 500 ms | 同 |
| 抬回安全高度 | `place_safe` | 1200 ms | 1500 ms |
| 全部完成回 HOME | HOME 姿态 | 1000 ms | 1500 ms |

### 7.2 抓取 / 放置 Z 微调

```c
ARM_PUZZLE_TRAVEL_HEIGHT_MM     60.0F   // 搬运安全高度
ARM_PUZZLE_PICK_HEIGHT_MM        0.0F   // 抓取面基准
ARM_PUZZLE_PLACE_HEIGHT_MM       1.0F   // 放置面基准
ARM_PUZZLE_PICK_Z_TRIM_MM       -9.0F   // 视觉抓取专用下压
ARM_PUZZLE_PLACE_Z_TRIM_MM      -8.5F   // 视觉放置专用下压
ARM_PUZZLE_PICK_X_Z_COMP_*              // X = 0 处额外 −5 mm，至 X=210 衰减为 0
```

这些偏移只作用于视觉抓放，不影响 `GOTO`、`CALIB`。若出现压纸或吸不起，优先调这三组量。

### 7.3 旋转分配

```text
delta = normalize(rotate_world − (place_base_yaw − pick_base_yaw))
```

- `|delta| ≤ 90°`：抓取端 J1 保持 0°，放置端承担 `delta`；
- `|delta| > 90°`：放置端固定 ±90°，抓取端承担剩余的最小角度。

这样 J1 不会越界，且搬运过程中底座与末端同时转动（`place_safe` 一段完成）。

### 7.4 完成提示

最后一片释放后等待 300 ms → PE9 拉高 300 ms → 拉低，然后抬升、回 HOME。`STATUS` 中状态依次为 `QUEUED → RUNNING → COMPLETE`。

---

## 8. 标定与调试

### 8.1 首次上电顺序

1. **断开舵机与电磁铁动力电源**，只给 STM32 与串口供电；
2. `PING`、`STATUS` 确认任务运行；
3. 逐路 `SERVO <n> <angle> 2000` 低速验证方向、零位与机械限位；
4. 修正 `ARM_KIN_*_ZERO_ANGLE_X10`、`ARM_KIN_*_DIRECTION`、`ARM_KIN_ELBOW_GAIN`；
5. `IK` 检查目标点可达性（不动作）；
6. `CALIB CHECK` 检查 A4 六点；
7. 通电后先悬空 `GOTO`，再接电磁铁做单点吸放；
8. 最后用 `VSIM 1` 走完整套两片流程。

### 8.2 实机标定表的位置

| 标定内容 | 配置项 |
|---|---|
| 舵机零位 / 方向 / 限位 | `ARM_SERVO*_MIN/MAX/HOME_ANGLE_X10`、`ARM_SERVO*_REVERSED` |
| 连杆尺寸与安装位置 | `ARM_KIN_LINK_*`、`ARM_KIN_BASE_*`、`ARM_KIN_SHOULDER_HEIGHT_MM` |
| 数学关节 → 舵机映射 | `ARM_KIN_*_ZERO_ANGLE_X10`、`ARM_KIN_*_DIRECTION`、`ARM_KIN_ELBOW_GAIN` |
| XY 落点残差 | `ARM_KIN_XY_GRID_*`（5×3 实测网格，当前启用） |
| 单轴 / 3×3 / 右半区 / Z 向补偿 | `ARM_KIN_*_COMP_*`、`ARM_KIN_COORD_CORRECTION_*`（当前关闭，留作备用） |
| 抓放高度与时序 | `ARM_PUZZLE_*` |

> 更换机械臂、重装舵盘或改变连杆长度后，上述参数必须重新测量；逆解正确不代表未标定的舵机会朝正确方向运动。

### 8.3 串口工具

| 脚本 | 用途 |
|---|---|
| [`tools/arm_pc_serial.py`](tools/arm_pc_serial.py) | USART2 文本命令封装（ping/status/servo/goto/magnet/calib/stop） |
| [`tools/puzzle1_uart_test.py`](tools/puzzle1_uart_test.py) | 生成 / 校验 / 发送 52 字节示例计划帧，检查 `0xAC` |
| [`tools/make_arm_report_docx.py`](tools/make_arm_report_docx.py) | 由源码生成算法与控制流程报告（docx） |
| [`tools/generate_wiring_docx.py`](tools/generate_wiring_docx.py) | 生成接线表文档 |
| [`tools/make_contact_sheet.py`](tools/make_contact_sheet.py) / [`render_pdf_to_png.py`](tools/render_pdf_to_png.py) | 文档渲染辅助 |

```powershell
python -m pip install pyserial
python tools/puzzle1_uart_test.py                 # 仅在 PC 端解析校验，不发送
python tools/puzzle1_uart_test.py --port COM7 --send   # 真正发送，会启动机械动作
```

---

## 9. 构建与烧录

### 9.1 Keil MDK（推荐，实机使用）

1. 安装 Keil MDK-ARM 5.x 与 `Keil.STM32F4xx_DFP` 器件包；
2. 打开 [`MDK-ARM/26.uvprojx`](MDK-ARM/26.uvprojx)，目标名 `26`，器件 `STM32F407VETx`；
3. 编译（F7），SWD 下载（PA13/PA14，可选 NRST）；
4. 需要 C99 与硬件浮点（工程已配置 `__FPU_PRESENT`、`ARM_MATH_CM4` 相关的编译选项）。

### 9.2 EIDE / VS Code

1. 安装 EIDE 扩展与 ARM 工具链（`arm-none-eabi-gcc`）；
2. 打开 [`26-eide.code-workspace`](26-eide.code-workspace) 或直接导入 `.eide/eide.yml`；
3. 器件包使用 `STM32F4xx_DFP 3.1.1`，安装到 `MDK-ARM/.pack`（该目录为本地缓存，未随仓库提供，需要自行安装或修改 `packDir`）；
4. 编译输出在 `EIDE-build/`（已加入 `.gitignore`）。

### 9.3 工程配置要点

- 头文件路径：`Core/Inc`、`App/Inc`、`BSP/Inc`、HAL、CMSIS、FreeRTOS；
- 中断优先级：USART/DMA 使用数值优先级 6（不小于 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`），因为中断中调用了 `xQueueSendFromISR`；
- 时基：`HAL` 使用 TIM4 作为 1 ms 时基（`stm32f4xx_hal_timebase_tim.c`）。

---

## 10. 目录结构

```text
26/
├─ App/                     业务层
│  ├─ Inc/                  arm_config.h（全部可调参数）与各模块头文件
│  └─ Src/                  arm_app / comm / puzzle / kinematics / motion / servo /
│                           calibration / debug_cli / buttons / safety / protocol
├─ BSP/                     板级驱动（PWM、双路 UART+DMA、磁铁、完成指示、急停输入）
├─ Core/                    CubeMX：main、时钟、GPIO、TIM1/8、USART1/2、中断、FreeRTOS 初始化
├─ Drivers/                 STM32F4xx HAL + CMSIS
├─ Middlewares/             FreeRTOS 内核与 CMSIS-RTOS V2
├─ MDK-ARM/                 Keil 工程（26.uvprojx、startup、RTE）
├─ .eide/                   EIDE 工程配置
├─ docs/                    设计说明、调试手册、接线文档、生成报告
├─ tools/                   PC 端串口与文档生成脚本
├─ 26.ioc                   CubeMX 工程
├─ 硬件接线表.md             引脚 / 接线 / 供电说明
└─ README.md
```

---

## 11. 关键配置项

| 配置 | 当前值 | 说明 |
|---|---|---|
| `ARM_ZERO_CALIBRATION_MODE` | `0` | 置 1 时只输出单路中位脉宽，用于逐个机械调零 |
| `ARM_SERVO_COUNT` | 5 | 舵机数量 |
| `ARM_SERVO_PWM_FREQUENCY_HZ` | 50 | 舵机 PWM 频率 |
| `ARM_UART_BAUDRATE` / `ARM_DEBUG_UART_BAUDRATE` | 115200 | 视觉口 / 调试口 |
| `ARM_BUTTON_ENABLED` | 1 | 10 键面板 |
| `ARM_ESTOP_ENABLED` | 0 | 急停输入未启用（PD13） |
| `ARM_COMM_WATCHDOG_ENABLED` | 0 | 通信看门狗未启用 |
| `ARM_MAGNET_MAX_ON_TIME_MS` | 18000 | 电磁铁最长连续吸合 |
| `ARM_PUZZLE_PLAN_TIMEOUT_MS` | 130000 | 单套计划超时 |
| `ARM_KIN_XY_RESIDUAL_ENABLED` | 1 | 5×3 实测残差网格 |
| `ARM_PUZZLE_RETURN_HOME` | 1 | 完成后回 HOME |

---

## 12. 已知限制

- **开环舵机**：无位置反馈，精度依赖机械装配与标定表；负载、电压跌落会造成落点偏差。
- **参数强耦合实物**：`arm_config.h` 中的连杆尺寸、零位、补偿表均来自本机实测，换机构必须重标。
- **视觉端不在本仓库**：K230 侧的识别与拼图求解程序需另行配套；本仓库只定义协议与执行端。
- **补偿表需持续维护**：P3/P4/P5/P6 的六点角度补偿中部分为拟合值（当前总开关关闭）。
- **文档存在历史版本**：`docs/` 下部分说明为早期四片固定方案（H=90 mm、Zboard=7 mm、搬运 20 mm 等），**以 `App/Inc/arm_config.h` 和源码为准**；本 README 已按当前代码校对。
- **未做 CI 编译验证**：仓库为实机工程整理，构建依赖本地 Keil / EIDE 与器件包。
- **安全边界**：软件 `STOP` 不能替代物理断电；调试大角度动作时必须先断开舵机动力。

---

## 13. 文档索引

| 文档 | 内容 |
|---|---|
| [`docs/软件架构与数据流.md`](docs/软件架构与数据流.md) | 启动顺序、任务模型、调用链、STOP 语义、扩展点（建议先读） |
| [`硬件接线表.md`](硬件接线表.md) | 全引脚表、舵机 / 视觉 / 调试 / 按键 / 继电器接线、供电与安全 |
| [`docs/机械臂调试指令手册.md`](docs/机械臂调试指令手册.md) | USART2 命令逐条说明与返回格式 |
| [`docs/第一问逆解与自动拼图说明.md`](docs/第一问逆解与自动拼图说明.md) | 逆解推导、动作顺序、协议示例（部分参数为历史版本） |
| [`docs/A4六点巡边校准说明.md`](docs/A4六点巡边校准说明.md) | 六点定义、巡边流程与判定 |
| [`docs/机械臂工程接线引脚说明.md`](docs/机械臂工程接线引脚说明.md) | 引脚与外设配置的代码对应关系 |
| [`docs/机械臂驱动移植与修改指南.md`](docs/机械臂驱动移植与修改指南.md) | 换板 / 换舵机 / 改参数时的移植步骤 |
| [`docs/机械臂按键控制说明.md`](docs/机械臂按键控制说明.md) | 10 键功能与改键方法（GOTO 坐标以源码为准） |
| [`docs/机械臂当前代码算法与控制流程分析.docx`](docs/机械臂当前代码算法与控制流程分析.docx) | 自动生成的算法与控制流程报告 |
| [`docs/_render_arm_report/`](docs/_render_arm_report) | 上述报告的 PDF 与逐页 PNG |

---

## 14. 第三方组件

| 组件 | 版权 / 许可 | 位置 |
|---|---|---|
| STM32F4xx HAL Driver | BSD-3-Clause，© STMicroelectronics | `Drivers/STM32F4xx_HAL_Driver` |
| CMSIS | Apache-2.0，© ARM | `Drivers/CMSIS` |
| FreeRTOS Kernel | MIT，© Amazon.com / Real Time Engineers | `Middlewares/Third_Party/FreeRTOS` |
| STM32CubeMX 生成代码 | 随 ST 许可 | `Core/`、`26.ioc` |

业务层代码（`App/`、`BSP/`、`tools/`）为本项目原创，随仓库一并公开用于学习与复现。

---

**硬件平台**：STM32F407VET6 · FreeRTOS · 5× 舵机 · 电磁铁 · K230 视觉
**赛题**：2026 年全国大学生电子设计竞赛 E 题 · 拼图装置
