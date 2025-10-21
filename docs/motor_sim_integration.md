# 电机仿真集成指南

## 概述
本文档描述了集成到 STM32H723 项目中的电机仿真模块。该仿真模拟直流电机动力学并输出正交霍尔传感器信号。

## 添加的文件
- `Core/Inc/motor_sim.h` - 电机仿真头文件，包含 API 和结构体
- `Core/Src/motor_sim.c` - 电机仿真实现

## 架构

### 电机模型
仿真实现了标准的直流电机微分方程:

**电气方程:**
```
L * di/dt = V - R*i - Ke*ω
```

**机械方程:**
```
J * dω/dt = Kt*i - b*ω
```

**运动学方程:**
```
dθ/dt = ω
```

其中:
- `V` = 施加电压（来自 ADC）
- `i` = 电枢电流
- `ω` = 角速度
- `θ` = 位置（角度）
- `R` = 电枢电阻
- `L` = 电枢电感
- `Kt` = 转矩常数
- `Ke` = 反电动势常数
- `J` = 转动惯量
- `b` = 粘性摩擦系数

### 霍尔传感器编码
模块生成正交霍尔信号（2位格雷码）:
- 正向: `00 → 01 → 11 → 10 → 00`
- 反向: `00 → 10 → 11 → 01 → 00`

输出引脚:
- `HALL_A_Pin` (PB0)
- `HALL_B_Pin` (PB1)

### 与主循环的集成

**定时器驱动执行:**
- TIM2 以固定间隔触发 `HAL_TIM_PeriodElapsedCallback()`
- 回调设置 `g_motorStepFlag` 以调度仿真步进
- 主循环在标志置位时调用 `MotorSim_Step()`

**ADC 输入:**
- ADC1 通过 DMA 采样电压
- 最新采样值转换为 0-3.3V 范围
- 作为电机输入电压应用

**流程:**
```
TIM2 中断 → 设置 g_motorStepFlag
          ↓
主循环 → MotorSim_Step(来自ADC的电压)
          ↓
     计算新状态
          ↓
     更新霍尔GPIO
```

## 配置

### 默认参数（超高速直流电机）
```c
dt = 1e-4f;           // 100 μs 时间步长（10 kHz 更新速率）
J = 2e-7f;            // 0.2 g·cm² 转动惯量（超轻）
b = 1e-5f;            // 0.01 mN·m·s 摩擦（极小）
Kt = 0.05f;           // 50 mN·m/A 转矩常数（高）
Ke = 0.003f;          // 3 mV·s/rad 反电动势（低，适合高速）
R = 1.0f;             // 1 Ω 电阻
L = 0.0001f;          // 0.1 mH 电感
hall_cpr = 4U;        // 每转 4 个脉冲
```

**注意:** 时间步长 `dt` 必须匹配定时器中断周期。TIM2 配置为:
- 预分频器 = 274 (275-1)
- 周期 = 100
- 定时器时钟 = 275 MHz (APB1)
- 中断频率 = 275 MHz / 275 / 100 = 10 kHz → **dt = 100 μs**

**性能:** 在 3V 驱动下，预期霍尔频率约为 300-600 Hz（约 9000-18000 RPM）

### 位置限制
设置 `pos_min` 和 `pos_max`（以弧度为单位）以约束运动:
```c
g_motorParam.pos_min = 0.0f;         // 下限
g_motorParam.pos_max = 2.0f * M_PI;  // 上限（一圈）
```
使用 `-1.0f` 禁用限制。

## 使用示例

```c
/* 在 main.c 中 */
MotorSim_StateTypeDef motor_state;
MotorSim_ParamTypeDef motor_param;

/* 使用默认值初始化 */
MotorSim_GetDefaultParams(&motor_param);
MotorSim_Init(&motor_state, &motor_param);

/* 启动定时器以进行周期性调用 */
HAL_TIM_Base_Start_IT(&htim2);

/* 在定时器回调中 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        g_motorStepFlag = 1U;
    }
}

/* 在主循环中 */
while (1)
{
    if (g_motorStepFlag)
    {
        g_motorStepFlag = 0U;
        float voltage = convert_adc_to_voltage(g_adcLatestSample);
        MotorSim_Step(&motor_state, &motor_param, voltage);
    }
}
```

## API 参考

### 初始化
```c
void MotorSim_GetDefaultParams(MotorSim_ParamTypeDef *param);
void MotorSim_Init(MotorSim_StateTypeDef *state, const MotorSim_ParamTypeDef *param);
```

### 运行时
```c
void MotorSim_Step(MotorSim_StateTypeDef *state, 
                   const MotorSim_ParamTypeDef *param, 
                   float voltage_input);
```

### 状态访问
```c
state->position;    // 当前角度（弧度）
state->velocity;    // 角速度（rad/s）
state->current;     // 电机电流（A）
state->hall_state;  // 霍尔传感器状态（0x00, 0x01, 0x11, 0x10）
state->direction;   // MOTOR_DIR_FORWARD/REVERSE/STOPPED
```

## 调整指南

1. **调整时间步长**（`dt`）以匹配定时器周期
2. **缩放电气参数**（`R`, `L`, `Kt`, `Ke`）以适应电机尺寸
3. **缩放机械参数**（`J`, `b`）以适应负载特性
4. **设置 hall_cpr** 以匹配编码器分辨率（典型值: 4, 12, 100）
5. **启用位置限制** 如果仿真受约束的执行器

## 注意事项
- 状态变量使用国际单位制（rad, rad/s, A, V）
- 霍尔输出在 `MotorSim_Step()` 中自动更新
- 对于自由旋转（禁用限制时），位置在 2π 处回绕
- 方向检测的速度阈值: 0.01 rad/s

