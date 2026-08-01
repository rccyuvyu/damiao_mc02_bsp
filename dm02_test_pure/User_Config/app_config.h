#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <cstdint>

#include "stm32h7xx_hal.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"

namespace App_Config
{
// Chassis motor mapping.
// 四个底盘电机的 DJI 电调 ID，需和 C610/C620 拨码或配置保持一致。
constexpr Enum_Motor_DJI_ID MOTOR_LEFT_FRONT_ID = Motor_DJI_ID_0x203;
constexpr Enum_Motor_DJI_ID MOTOR_RIGHT_FRONT_ID = Motor_DJI_ID_0x204;
constexpr Enum_Motor_DJI_ID MOTOR_LEFT_REAR_ID = Motor_DJI_ID_0x202;
constexpr Enum_Motor_DJI_ID MOTOR_RIGHT_REAR_ID = Motor_DJI_ID_0x201;

// 四个底盘电机的实际 CAN 反馈 ID，用于接收回调分发。
constexpr uint32_t MOTOR_LEFT_FRONT_CAN_ID = 0x203u;
constexpr uint32_t MOTOR_RIGHT_FRONT_CAN_ID = 0x204u;
constexpr uint32_t MOTOR_LEFT_REAR_CAN_ID = 0x202u;
constexpr uint32_t MOTOR_RIGHT_REAR_CAN_ID = 0x201u;

// DJI 2006 减速箱减速比，用于把电机端速度换算到输出轴。
constexpr float MOTOR_GEARBOX_RATE = 36.0f;

// Motor speed-loop PID: output is normalized by the C610 driver.
// 底盘速度环 P；越大响应越快，过大会振荡。
constexpr float MOTOR_SPEED_KP = 0.5f;
// 底盘速度环 I；用于消除稳态误差，当前关闭。
constexpr float MOTOR_SPEED_KI = 0.0f;
// 底盘速度环 D；用于抑制速度变化，当前基本关闭。
constexpr float MOTOR_SPEED_KD = 0.00f;
// 底盘速度前馈，当前关闭。
constexpr float MOTOR_SPEED_KF = 0.0f;
// 底盘速度环积分输出限幅，防止积分累积过大。
constexpr float MOTOR_SPEED_I_OUT_MAX = 2.0f;
// 底盘速度环总输出限幅，输出会被 C610 驱动归一化。
constexpr float MOTOR_SPEED_OUT_MAX = 8.0f;
// 底盘速度 PID 计算周期，单位 s。
constexpr float MOTOR_SPEED_D_T = 0.001f;

// Change these independently if a motor is mounted in the opposite direction.
// 底盘电机安装方向修正，某个轮子转向反了就把对应符号取反。
constexpr float MOTOR_LEFT_FRONT_SIGN = 1.0f;
constexpr float MOTOR_RIGHT_FRONT_SIGN = -1.0f;
constexpr float MOTOR_LEFT_REAR_SIGN = 1.0f;
constexpr float MOTOR_RIGHT_REAR_SIGN = -1.0f;

// Four-channel line tracking, ordered from right to left (0 -> 3).
// 循迹传感器数量。
constexpr uint8_t LINE_SENSOR_COUNT = 4u;
// 黑线对应的 GPIO 电平；当前为低电平表示检测到黑线。
constexpr GPIO_PinState LINE_BLACK_STATE = GPIO_PIN_RESET;
// 循迹基础速度，单位使用电机速度环的目标单位。
constexpr float LINE_BASE_SPEED = 5.0f;
// 循迹横向误差 P；越大转向越积极。
constexpr float LINE_KP = 2.0f;
// 循迹横向误差 D；抑制快速偏差变化。
constexpr float LINE_KD = 0.8f;
// 四路传感器权重，从右到左排列；正值表示线偏右，负值表示线偏左。
constexpr float LINE_WEIGHT[4] = {1.5f, 1.0f, -1.0f, -1.5f};
// 认为是大偏差/急弯的误差阈值。
constexpr float LINE_SHARP_ERROR = 1.2f;
// 急弯或标记线处基础速度缩放系数。
constexpr float LINE_CORNER_SPEED_SCALE = 0.55f;
// 丢线时用 IMU 航向保持的 P。
constexpr float LINE_YAW_KP = 50.0f;
// 丢线航向修正最大值，防止左右轮差速过大。
constexpr float LINE_YAW_CORRECTION_MAX = 2.0f;
// 同时检测到多少个黑线传感器时认为遇到起终点/横向标记。
constexpr uint8_t LINE_MARKER_MIN_BLACK_COUNT = 3u;
// 起步后需要连续离开标记线的时间，单位 ms。
constexpr uint32_t LINE_LEAVE_MARKER_MS = 150u;
// 运行达到该时间后才允许识别终点，单位 us。
constexpr uint32_t LINE_MIN_RUN_TIME_US = 500000u;
// 终点标记需要稳定检测的连续次数，1 次约 1 ms。
constexpr uint8_t LINE_MARKER_STABLE_MS = 5u;

// Task 3: DM4310 external position-loop control.
// 水管达妙电机反馈 CAN ID，需等于电机 Master_ID。
constexpr uint8_t WATER_PIPE_CAN_RX_ID = 0x10u;
// 水管达妙电机控制 CAN ID，MIT 模式下通常为电机 CAN_ID。
constexpr uint8_t WATER_PIPE_CAN_TX_ID = 0x01u;

// 电机软件最小角度，单位 rad；超过边界时禁止继续向外输出力矩。
constexpr float WATER_PIPE_ANGLE_MIN = -2.76f;
// 电机软件最大角度，单位 rad；超过边界时禁止继续向外输出力矩。
constexpr float WATER_PIPE_ANGLE_MAX = -2.20f;
// 当电机位于 WATER_PIPE_ANGLE_MIN 时，对应的水管物理倾角，单位 deg。
constexpr float WATER_PIPE_PIPE_ANGLE_AT_MIN_DEG = 3.15f;
// 当电机位于 WATER_PIPE_ANGLE_MAX 时，对应的水管物理倾角，单位 deg。
constexpr float WATER_PIPE_PIPE_ANGLE_AT_MAX_DEG = -3.23f;

// 准滑模内环线性增益，单位 Nm/rad；只使用目标角与当前角的误差。
constexpr float WATER_PIPE_SMC_LINEAR_GAIN = 2.0f;
// 准滑模内环切换增益，单位 Nm；越大越能克服摩擦和外部扰动。
constexpr float WATER_PIPE_SMC_SWITCHING_GAIN = 0.35f;
// 准滑模边界层厚度，单位 rad；0.5 deg 可降低角度测量噪声引起的抖振。
constexpr float WATER_PIPE_SMC_BOUNDARY_LAYER = 0.0872665f;
// 发给达妙电机的最大力矩，单位 Nm。
constexpr float WATER_PIPE_TORQUE_MAX = 2.0f;
// 启动位置基准的偏移量，单位和视觉 distance 一致；0 表示锁存任务开始时的当前位置。
constexpr float WATER_PIPE_BALL_TARGET_POSITION = 0.0f;
// 外环位置增益，输入为 target_position - distance，输出为虚拟水管角度 deg。
constexpr float WATER_PIPE_OUTER_POSITION_KP = -0.9f;
// 外环速度阻尼增益，输入为视觉 velocity，输出为虚拟水管角度 deg。
constexpr float WATER_PIPE_OUTER_VELOCITY_KP = -0.03f;
// 外环输出限幅，也就是水管允许的最大虚拟目标倾角，单位 deg。
constexpr float WATER_PIPE_PIPE_ANGLE_LIMIT_DEG = 3.0f;
// 上位机速度输入限幅，防止异常视觉速度把速度环打满。
constexpr float WATER_PIPE_BALL_VELOCITY_MAX = 3000.0f;
// 小球质量，单位 kg；当前控制逻辑暂未使用，保留给模型前馈。
constexpr float WATER_PIPE_BALL_MASS_KG = 0.004f;
// 电机外部角度环死区，单位 rad；小于该误差时不加静摩擦补偿。
constexpr float WATER_PIPE_POSITION_DEADBAND = 0.005f;
// 静摩擦补偿力矩，单位 Nm；误差超过死区后按误差方向叠加。
constexpr float WATER_PIPE_STATIC_FRICTION_TORQUE = 0.0f;

// LCD key thresholds, set midway between the measured key voltages.
// LCD 五向按键 ADC 阈值，按键识别使用从小到大的区间判断。
constexpr uint16_t LCD_KEY_CENTER_MAX = 600u;
constexpr uint16_t LCD_KEY_LEFT_MAX = 1450u;
constexpr uint16_t LCD_KEY_RIGHT_MAX = 2250u;
constexpr uint16_t LCD_KEY_UP_MAX = 3050u;
constexpr uint16_t LCD_KEY_DOWN_MAX = 3700u;

// LCD 刷新周期，单位 ms。
constexpr uint32_t LCD_REFRESH_PERIOD_MS = 50u;
}

#endif
