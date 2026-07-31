#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <cstdint>

#include "stm32h7xx_hal.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"

namespace App_Config
{
    // Chassis motor mapping.
    constexpr Enum_Motor_DJI_ID MOTOR_LEFT_FRONT_ID = Motor_DJI_ID_0x203;
    constexpr Enum_Motor_DJI_ID MOTOR_RIGHT_FRONT_ID = Motor_DJI_ID_0x204;
    constexpr Enum_Motor_DJI_ID MOTOR_LEFT_REAR_ID = Motor_DJI_ID_0x202;
    constexpr Enum_Motor_DJI_ID MOTOR_RIGHT_REAR_ID = Motor_DJI_ID_0x201;
    constexpr uint32_t MOTOR_LEFT_FRONT_CAN_ID = 0x203u;
    constexpr uint32_t MOTOR_RIGHT_FRONT_CAN_ID = 0x204u;
    constexpr uint32_t MOTOR_LEFT_REAR_CAN_ID = 0x202u;
    constexpr uint32_t MOTOR_RIGHT_REAR_CAN_ID = 0x201u;
    constexpr float MOTOR_GEARBOX_RATE = 36.0f;

    // Motor speed-loop PID: output is normalized by the C610 driver.
    constexpr float MOTOR_SPEED_KP = 1.0f;
    constexpr float MOTOR_SPEED_KI = 0.0f;
    constexpr float MOTOR_SPEED_KD = 0.00f;
    constexpr float MOTOR_SPEED_KF = 0.0f;
    constexpr float MOTOR_SPEED_I_OUT_MAX = 2.0f;
    constexpr float MOTOR_SPEED_OUT_MAX = 8.0f;
    constexpr float MOTOR_SPEED_D_T = 0.001f;

    // Change these independently if a motor is mounted in the opposite direction.
    constexpr float MOTOR_LEFT_FRONT_SIGN = 1.0f;
    constexpr float MOTOR_RIGHT_FRONT_SIGN = -1.0f;
    constexpr float MOTOR_LEFT_REAR_SIGN = 1.0f;
    constexpr float MOTOR_RIGHT_REAR_SIGN = -1.0f;

    // Four-channel line tracking, ordered from right to left (0 -> 3).
    constexpr uint8_t LINE_SENSOR_COUNT = 4u;
    constexpr GPIO_PinState LINE_BLACK_STATE = GPIO_PIN_RESET;
    constexpr float LINE_BASE_SPEED = 5.0f;
    constexpr float LINE_KP = 2.0f;
    constexpr float LINE_KD = 0.8f;
    constexpr float LINE_WEIGHT[4] = {2.5f, 1.0f, -1.0f, -2.5f};
    constexpr float LINE_SHARP_ERROR = 1.2f;
    constexpr float LINE_CORNER_SPEED_SCALE = 0.55f;
    constexpr uint8_t LINE_MARKER_MIN_BLACK_COUNT = 3u;
    constexpr uint32_t LINE_LEAVE_MARKER_MS = 150u;
    constexpr uint32_t LINE_MIN_RUN_TIME_US = 500000u;
    constexpr uint8_t LINE_MARKER_STABLE_MS = 5u;

    // Single-channel analog line-sensor test on PA2 / ADC1 channel 14.
    // ADC DMA index 0 remains reserved for the LCD key.
    constexpr uint16_t LINE_SENSOR_ADC_BUFFER_INDEX = 1u;
    constexpr float ADC_REFERENCE_VOLTAGE = 3.3f;

    // Measured LCD-key ADC values, with midpoint thresholds between adjacent keys.
    constexpr uint16_t LCD_KEY_NONE_MAX = 515u;
    constexpr uint16_t LCD_KEY_DOWN_MAX = 990u;
    constexpr uint16_t LCD_KEY_UP_MAX = 1457u;
    constexpr uint16_t LCD_KEY_RIGHT_MAX = 1935u;
    constexpr uint16_t LCD_KEY_LEFT_MAX = 2457u;

    constexpr uint32_t LCD_REFRESH_PERIOD_MS = 50u;
}

#endif
