/**
 * @file tsk_config_and_callback.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-01-17 1.1 调试到机器人层
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

#ifndef TSK_CONFIG_AND_CALLBACK_H
#define TSK_CONFIG_AND_CALLBACK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

typedef struct __attribute__((packed)) VisionToGimbalPacket
{
    uint8_t head[2];
    float distance;
    float velocity;
    uint16_t crc16;
} VisionToGimbalPacket;

typedef struct __attribute__((packed)) GimbalToVision
{
    uint8_t head[2];
    // 当前选择或正在运行的 Test 编号。
    uint8_t task;
    // 0: 菜单空闲, 1: 任务运行中, 2: 任务完成。
    uint8_t status;
    // 预留 CRC16 字段，当前发送端置 0。
    uint16_t crc16;
} GimbalToVision;

typedef struct VisionToGimbal
{
    uint8_t head[2];
    float distance;
    float target_angle;
    float velocity;
    uint16_t crc16;
    uint32_t rx_count;
    uint32_t rx_period_us;
    uint64_t last_rx_timestamp_us;
} VisionToGimbal;

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

extern volatile VisionToGimbal vision_to_gimbal;
extern volatile float water_pipe_motor_target_angle;
// 任务开始时锁存的钢球平衡位置，单位与视觉 distance 一致。
extern volatile float water_pipe_ball_target_position;

/* Exported function declarations --------------------------------------------*/

void Task_Init();

void Task_Loop();

#ifdef __cplusplus
};
#endif

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
