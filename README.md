# DJI RoboMaster Motor Control

基于 STM32F407 的 DJI RoboMaster 电机控制固件，通过 CAN 总线驱动 M3508、M2006、GM6020 系列电机，支持多种闭环控制模式。

## 硬件平台

| 项目 | 详情 |
|------|------|
| MCU | STM32F407VET6 (Cortex-M4) |
| 主频 | 168 MHz (HSI + PLL) |
| CAN | CAN1 (PD0-RX, PD1-TX)，支持 CAN2 扩展 |
| 定时器 | TIM8 (PID 计算周期 + CAN 发送) |
| 开发环境 | CLion + CMake + GCC (arm-none-eabi) |
| HAL 库 | STM32F4xx HAL Driver |
| 代码生成 | STM32CubeMX |

## 支持的电机

| 电机型号 | 配套电调 | CAN ID 范围 | 编码器分辨率 |
|----------|----------|-------------|-------------|
| **M3508** | C620 | 0x201 ~ 0x204 | 8192 P/R |
| **M2006** | C610 | 0x205 ~ 0x208 | — |
| **GM6020** | 内置 | 0x209 ~ 0x20B | — |

> 所有电机均支持 **双 CAN 总线** (CAN1 / CAN2)，每个 CAN 总线最多挂载 4×M3508 + 4×M2006 + 3×GM6020。

## 控制模式

| 模式 | 说明 |
|------|------|
| `Control_Method_OPENLOOP` | 开环控制 |
| `Control_Method_VOLTAGE` | 电压控制 |
| `Control_Method_CURRENT` | 电流控制 |
| `Control_Method_TORQUE` | **力矩闭环** — 直接控制输出扭矩 |
| `Control_Method_OMEGA` | **速度闭环** — PID 控制角速度 |
| `Control_Method_ANGLE` | **位置闭环** — 串级 PID (角度环 → 速度环) |

## 项目结构

```
DJI_RoboMaster_Motor/
├── Core/                        # STM32CubeMX 生成的外设初始化代码
│   ├── Inc/                     # 头文件 (main.h, can.h, tim.h, gpio.h...)
│   └── Src/                     # 源文件 (main.c, can.c, tim.c, gpio.c...)
├── HLY/                         # 用户电机控制库 (核心代码)
│   ├── Inc/
│   │   ├── drv_can.h            # CAN 总线驱动 — 收发/滤波/缓冲区管理
│   │   ├── drv_math.h           # 数学工具 — 限幅/绝对值/大小端/sinc
│   │   ├── alg_PID.h            # PID 控制器 — 串级/变速积分/积分分离/微分先行
│   │   ├── dvc_DJI_M3508.h      # M3508 电机驱动 (C620 电调)
│   │   ├── dvc_DJI_M2006.h      # M2006 电机驱动 (C610 电调)
│   │   └── dvc_DJI_GM6020.h     # GM6020 电机驱动
│   └── Src/
│       ├── drv_can.c
│       ├── drv_math.c
│       ├── alg_PID.c
│       ├── dvc_DJI_M3508.c
│       ├── dvc_DJI_M2006.c
│       └── dvc_DJI_GM6020.c
├── Drivers/                     # STM32 标准驱动库
│   ├── STM32F4xx_HAL_Driver/    # HAL 库
│   └── CMSIS/                   # CMSIS (Core + DSP + NN)
├── cmake/                       # CMake 子模块 (CubeMX 生成)
├── CMakeLists.txt               # 顶层 CMake 构建文件
├── DJI_RoboMaster_Motor.ioc     # STM32CubeMX 项目配置
└── startup_stm32f407xx.s        # 启动文件
```

## PID 控制器特性

`alg_PID` 模块实现了一个功能完整的 PID 控制器，支持：

- **P / I / D / 前馈** 四项独立调节
- **变速积分** — 误差大时减弱积分，避免超调
- **积分分离** — 误差过大时清零积分，防止积分饱和
- **微分先行** — 微分作用于测量值而非误差，减少设定值突变冲击
- **死区** — 小误差范围内不调节
- **积分限幅 & 输出限幅**
- **串级 PID** — 位置环输出作为速度环输入 (用于 `ANGLE` 模式)

### PID 初始化示例

```c
PID_Init(&motor.PID_Omega,
    500.0f,     // Kp
    0.0f,       // Ki
    0.0f,       // Kd
    0.0f,       // Kf (前馈)
    0.0f,       // 积分限幅 (0=不限)
    16384.0f,   // 输出限幅
    0.001f,     // 控制周期 (秒)
    0.0f,       // 死区
    0.0f,       // 变速积分 A
    0.0f,       // 变速积分 B
    0.0f,       // 积分分离阈值 (0=不启用)
    1           // 微分先行使能
);
```

## 快速开始

### 1. 克隆仓库

```bash
git clone https://github.com/YLING-O1/DJI_RoboMaster_Motor.git
```

### 2. 用 CLion 打开

使用 CLion 打开项目目录，CMake 会自动配置工具链。需要提前安装 `arm-none-eabi-gcc` 工具链。

### 3. 编译

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)
```

### 4. 烧录

使用 ST-Link / J-Link / DAP-Link 等调试器烧录 `build/DJI_RoboMaster_Motor.elf` 到 STM32F407。

### 5. 硬件连接

```
STM32F407          DJI C620/C610/GM6020
  PD0 (CAN1_RX)  <---->  电调 CAN 总线
  PD1 (CAN1_TX)  <---->  电调 CAN 总线
  GND            <---->  共地
```

> ⚠️ CAN 总线两端需要并联 120Ω 终端电阻。

## 使用示例

### 初始化四个 M3508 电机 (速度闭环)

```c
// main.c

Struct_RM_Motor_M3508 LF_M3508, LB_M3508, RB_M3508, RF_M3508;

// 初始化 CAN 并注册回调
CAN_Init(&hcan1, DJI_RM_CAN_Callback);

// 初始化电机 (CAN, 电机结构体, ID, 控制模式, 减速比, 最大扭矩)
DJI_M3508_Init(&hcan1, &LF_M3508, RM_Motor_ID_0x201, Control_Method_OMEGA, 3591.0f/187.0f, 2.5f);
DJI_M3508_Init(&hcan1, &LB_M3508, RM_Motor_ID_0x202, Control_Method_OMEGA, 3591.0f/187.0f, 2.5f);
DJI_M3508_Init(&hcan1, &RB_M3508, RM_Motor_ID_0x203, Control_Method_OMEGA, 3591.0f/187.0f, 2.5f);
DJI_M3508_Init(&hcan1, &RF_M3508, RM_Motor_ID_0x204, Control_Method_OMEGA, 3591.0f/187.0f, 2.5f);

// 初始化 PID (每个电机独立 PID)
PID_Init(&LF_M3508.PID_Omega, 500.0f, 0.0f, 0.0f, 0.0f, 0.0f, 16384.0f, 0.001f, 0.0f, 0.0f, 0.0f, 1);

// 启动定时器中断
HAL_TIM_Base_Start_IT(&htim8);

// 设置目标速度并运行
while (1)
{
    DJI_M3508_Set_Target_Omega(&LF_M3508, 10.0f);   // 10 rad/s
    DJI_M3508_Set_Target_Omega(&LB_M3508, 10.0f);
    DJI_M3508_Set_Target_Omega(&RB_M3508, 10.0f);
    DJI_M3508_Set_Target_Omega(&RF_M3508, 10.0f);
    HAL_Delay(5000);

    DJI_M3508_Set_Target_Omega(&LF_M3508, 30.0f);   // 30 rad/s
    DJI_M3508_Set_Target_Omega(&LB_M3508, 30.0f);
    DJI_M3508_Set_Target_Omega(&RB_M3508, 30.0f);
    DJI_M3508_Set_Target_Omega(&RF_M3508, 30.0f);
    HAL_Delay(5000);
}
```

### CAN 回调函数

```c
void DJI_RM_CAN_Callback(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.StdId)
    {
        case 0x201:
            DJI_M3508_CAN_RxCpltCallback(&LF_M3508, Rx_Buffer->Data);
            break;
        case 0x202:
            DJI_M3508_CAN_RxCpltCallback(&LB_M3508, Rx_Buffer->Data);
            break;
        // ... 更多电机 ID
    }
}
```

### 定时器回调 (在 TIM8 中断中调用)

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8)
    {
        // 存活检测 (判断电机是否在线)
        DJI_M3508_TIM_Alive_PeriodElapsedCallback(&LF_M3508);
        // PID 计算 + CAN 数据更新
        DJI_M3508_TIM_PID_PeriodElapsedCallback(&LF_M3508);
        // 发送 CAN 数据帧到电调
        TIM_CAN_PeriodElapsedCallback();
    }
}
```

## CAN 通信协议

DJI RoboMaster 电机使用标准 CAN 2.0 协议：

- **反馈帧 (电机 → MCU)**：每个电机以 1kHz 频率发送，包含编码器位置、转速、扭矩、温度
- **控制帧 (MCU → 电调)**：MCU 通过 `0x200`、`0x1FF`、`0x1FE` 等 ID 发送控制指令，每帧可携带 4 个电机的电流值
- **波特率**：1 Mbps
- **帧格式**：标准帧 (11-bit ID)

## 许可证

本项目采用 MIT License 开源。HAL 库和 CMSIS 部分版权归 STMicroelectronics 所有，详见 `Drivers/` 目录下的 LICENSE 文件。

## 作者

- GitHub: [YLING-O1](https://github.com/YLING-O1)
