/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor_sim.h
  * @brief          : 电机仿真模块头文件
  *                   实现直流电机动力学仿真并输出霍尔传感器信号
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_SIM_H
#define __MOTOR_SIM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  电机仿真状态结构体
  */
typedef struct {
    float position;           /* 累计角度（弧度） */
    float velocity;           /* 角速度（rad/s） */
    float current;            /* 电机电流（A） */
    float voltage;            /* 施加电压（V） */
    uint8_t hall_state;       /* 霍尔传感器状态：0b00, 0b01, 0b11, 0b10 */
    uint8_t direction;        /* 0: 停止, 1: 正转, 2: 反转 */
} MotorSim_StateTypeDef;

/**
  * @brief  电机仿真参数结构体
  */
typedef struct {
    float dt;                 /* 仿真步长（秒） */
    float J;                  /* 转动惯量（kg·m²） */
    float b;                  /* 粘性摩擦系数（N·m·s） */
    float Kt;                 /* 转矩常数（N·m/A） */
    float Ke;                 /* 反电动势常数（V·s/rad） */
    float R;                  /* 电枢电阻（Ω） */
    float L;                  /* 电枢电感（H） */
    float pos_min;            /* 最小位置限制（弧度，-1 = 无限制） */
    float pos_max;            /* 最大位置限制（弧度，-1 = 无限制） */
    uint16_t hall_cpr;        /* 霍尔每转计数（如正交编码为4） */
} MotorSim_ParamTypeDef;

/* Exported constants --------------------------------------------------------*/

/* 霍尔传感器状态定义 */
#define MOTOR_HALL_STATE_00    0x00U
#define MOTOR_HALL_STATE_01    0x01U
#define MOTOR_HALL_STATE_11    0x03U
#define MOTOR_HALL_STATE_10    0x02U

/* 方向定义 */
#define MOTOR_DIR_STOPPED      0x00U  /* 停止 */
#define MOTOR_DIR_FORWARD      0x01U  /* 正转 */
#define MOTOR_DIR_REVERSE      0x02U  /* 反转 */

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  初始化电机仿真
  * @param  state 指向电机状态结构体的指针
  * @param  param 指向电机参数结构体的指针
  * @retval 无
  */
void MotorSim_Init(MotorSim_StateTypeDef *state, const MotorSim_ParamTypeDef *param);

/**
  * @brief  执行一次仿真步进
  * @param  state 指向电机状态结构体的指针
  * @param  param 指向电机参数结构体的指针
  * @param  voltage_input 施加电压（V）
  * @retval 无
  */
void MotorSim_Step(MotorSim_StateTypeDef *state, const MotorSim_ParamTypeDef *param, float voltage_input);

/**
  * @brief  更新霍尔传感器GPIO输出
  * @param  state 指向电机状态结构体的指针
  * @retval 无
  */
void MotorSim_UpdateHallOutputs(const MotorSim_StateTypeDef *state);

/**
  * @brief  获取默认电机参数
  * @param  param 指向待填充的电机参数结构体的指针
  * @retval 无
  */
void MotorSim_GetDefaultParams(MotorSim_ParamTypeDef *param);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_SIM_H */
