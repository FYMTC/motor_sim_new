/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor_sim.c
  * @brief          : 电机仿真模块，实现霍尔传感器输出
  *                   仿真直流电机动力学并生成正交霍尔编码信号
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "motor_sim.h"
#include "gpio.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

static void MotorSim_ComputeHallState(MotorSim_StateTypeDef *state, const MotorSim_ParamTypeDef *param);
static void MotorSim_EnforcePositionLimits(MotorSim_StateTypeDef *state, const MotorSim_ParamTypeDef *param);
static void MotorSim_UpdateDirection(MotorSim_StateTypeDef *state);

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化电机仿真状态
  * @param  state 指向电机状态结构体的指针
  * @param  param 指向电机参数结构体的指针
  * @retval 无
  */
void MotorSim_Init(MotorSim_StateTypeDef *state, const MotorSim_ParamTypeDef *param)
{
    /* USER CODE BEGIN MotorSim_Init */
    
    /* 复位所有状态变量 */
    state->position = 0.0f;
    state->velocity = 0.0f;
    state->current = 0.0f;
    state->voltage = 0.0f;
    state->hall_state = MOTOR_HALL_STATE_00;
    state->direction = MOTOR_DIR_STOPPED;
    
    /* 计算初始霍尔状态 */
    MotorSim_ComputeHallState(state, param);
    
    /* 设置初始GPIO输出 */
    MotorSim_UpdateHallOutputs(state);
    
    /* USER CODE END MotorSim_Init */
}

/**
  * @brief  执行单步电机仿真
  * @param  state 指向电机状态结构体的指针
  * @param  param 指向电机参数结构体的指针
  * @param  voltage_input 施加的电压(单位:V)
  * @retval 无
  * @note   该函数实现直流电机微分方程:
  *         di/dt = (V - R*i - Ke*w) / L
  *         dw/dt = (Kt*i - b*w) / J
  *         dθ/dt = w
  */
void MotorSim_Step(MotorSim_StateTypeDef *state, const MotorSim_ParamTypeDef *param, float voltage_input)
{
    /* USER CODE BEGIN MotorSim_Step */
    
    float di_dt;
    float dw_dt;
    float back_emf;
    float torque;
    
    /* Store input voltage */
    state->voltage = voltage_input;
    
    /* Compute back-EMF: e = Ke * ω */
    back_emf = param->Ke * state->velocity;
    
    /* Electrical equation: L*di/dt = V - R*i - e */
    di_dt = (voltage_input - param->R * state->current - back_emf) / param->L;
    
    /* Mechanical torque: τ = Kt * i */
    torque = param->Kt * state->current;
    
    /* Mechanical equation: J*dω/dt = τ - b*ω */
    dw_dt = (torque - param->b * state->velocity) / param->J;
    
    /* Euler integration */
    state->current += di_dt * param->dt;
    state->velocity += dw_dt * param->dt;
    state->position += state->velocity * param->dt;
    
    /* Enforce position limits if configured */
    MotorSim_EnforcePositionLimits(state, param);
    
    /* Update direction indicator */
    MotorSim_UpdateDirection(state);
    
    /* Compute Hall sensor state based on position */
    MotorSim_ComputeHallState(state, param);
    
    /* Update GPIO outputs */
    MotorSim_UpdateHallOutputs(state);
    
    /* USER CODE END MotorSim_Step */
}

/**
  * @brief  更新霍尔传感器GPIO输出
  * @param  state 指向电机状态结构体的指针
  * @retval 无
  */
void MotorSim_UpdateHallOutputs(const MotorSim_StateTypeDef *state)
{
    /* USER CODE BEGIN MotorSim_UpdateHallOutputs */
    
    GPIO_PinState hall_a;
    GPIO_PinState hall_b;
    
    /* 将霍尔状态解码为GPIO电平 */
    hall_a = (state->hall_state & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    hall_b = (state->hall_state & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    
    /* 写入GPIO端口 */
    HAL_GPIO_WritePin(HALL_A_GPIO_Port, HALL_A_Pin, hall_a);
    HAL_GPIO_WritePin(HALL_B_GPIO_Port, HALL_B_Pin, hall_b);
    
    /* USER CODE END MotorSim_UpdateHallOutputs */
}

/**
  * @brief  获取默认电机参数(小型直流电机示例)
  * @param  param 指向待填充的电机参数结构体的指针
  * @retval 无
  */
void MotorSim_GetDefaultParams(MotorSim_ParamTypeDef *param)
{
    /* USER CODE BEGIN MotorSim_GetDefaultParams */
    
    /* 时间步长: 100 μs (10 kHz中断速率) */
    param->dt = 1e-4f;
    
    /* 机械参数(超高速电机) */
    param->J = 2e-7f;        /* 转动惯量: 0.2 g·cm² (极低) */
    param->b = 1e-5f;        /* 粘性摩擦: 0.01 mN·m·s (极低) */
    
    /* 电气参数(高速、低反电动势电机) */
    param->Kt = 0.05f;       /* 转矩常数: 50 mN·m/A (高) */
    param->Ke = 0.003f;      /* 反电动势常数: 3 mV·s/rad (低,适合高速) */
    param->R = 1.0f;         /* 电枢电阻: 1 Ω (低损耗) */
    param->L = 0.0001f;      /* 电枢电感: 0.1 mH (快速电流响应) */
    
    /* 位置限制(默认禁用) */
    param->pos_min = -1.0f;  /* 无最小限制 */
    param->pos_max = -1.0f;  /* 无最大限制 */
    
    /* 霍尔传感器: 每转4个脉冲(正交,2位) */
    param->hall_cpr = 4U;
    
    /* USER CODE END MotorSim_GetDefaultParams */
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  根据位置计算霍尔传感器状态
  * @param  state 指向电机状态结构体的指针
  * @param  param 指向电机参数结构体的指针
  * @retval 无
  * @note   生成正交编码: 00 -> 01 -> 11 -> 10 -> 00 (正向)
  *         或反向运动时的反序列
  */
static void MotorSim_ComputeHallState(MotorSim_StateTypeDef *state, const MotorSim_ParamTypeDef *param)
{
    /* USER CODE BEGIN MotorSim_ComputeHallState */
    
    float position_normalized;
    float cycles;
    uint8_t sector;
    
    /* 将位置归一化到 [0, 2π) 范围 */
    position_normalized = fmodf(state->position, 2.0f * M_PI);
    if (position_normalized < 0.0f)
    {
        position_normalized += 2.0f * M_PI;
    }
    
    /* 转换为霍尔计数 */
    cycles = position_normalized / (2.0f * M_PI);
    sector = (uint8_t)(cycles * (float)param->hall_cpr) % param->hall_cpr;
    
    /* 将扇区映射到霍尔状态(正交编码的格雷码) */
    switch (sector)
    {
        case 0:
            state->hall_state = MOTOR_HALL_STATE_00;
            break;
        case 1:
            state->hall_state = MOTOR_HALL_STATE_01;
            break;
        case 2:
            state->hall_state = MOTOR_HALL_STATE_11;
            break;
        case 3:
            state->hall_state = MOTOR_HALL_STATE_10;
            break;
        default:
            state->hall_state = MOTOR_HALL_STATE_00;
            break;
    }
    
    /* USER CODE END MotorSim_ComputeHallState */
}

/**
  * @brief  强制执行位置限制并在边界处停止电机
  * @param  state 指向电机状态结构体的指针
  * @param  param 指向电机参数结构体的指针
  * @retval 无
  */
static void MotorSim_EnforcePositionLimits(MotorSim_StateTypeDef *state, const MotorSim_ParamTypeDef *param)
{
    /* USER CODE BEGIN MotorSim_EnforcePositionLimits */
    
    /* 检查最小限制 */
    if ((param->pos_min >= 0.0f) && (state->position < param->pos_min))
    {
        state->position = param->pos_min;
        state->velocity = 0.0f;
        state->current = 0.0f;
    }
    
    /* 检查最大限制 */
    if ((param->pos_max >= 0.0f) && (state->position > param->pos_max))
    {
        state->position = param->pos_max;
        state->velocity = 0.0f;
        state->current = 0.0f;
    }
    
    /* USER CODE END MotorSim_EnforcePositionLimits */
}

/**
  * @brief  根据速度更新方向指示器
  * @param  state 指向电机状态结构体的指针
  * @retval 无
  */
static void MotorSim_UpdateDirection(MotorSim_StateTypeDef *state)
{
    /* USER CODE BEGIN MotorSim_UpdateDirection */
    
    const float velocity_threshold = 0.01f;  /* rad/s (速度阈值) */
    
    if (fabsf(state->velocity) < velocity_threshold)
    {
        state->direction = MOTOR_DIR_STOPPED;
    }
    else if (state->velocity > 0.0f)
    {
        state->direction = MOTOR_DIR_FORWARD;
    }
    else
    {
        state->direction = MOTOR_DIR_REVERSE;
    }
    
    /* USER CODE END MotorSim_UpdateDirection */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
