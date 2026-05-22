/**
 * @file bsp_lcd_key.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief LCD板载五向按键驱动
 * @version 0.1
 * @date 2026-05-22 0.1 
 *
 * @copyright USTC-RoboWalker (c) 2026
 *
 */

#ifndef BSP_LCD_KEY_H
#define BSP_LCD_KEY_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

#include "1_Middleware/Driver/ADC/drv_adc.h"
#include "stm32h7xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief LCD按键键值
 *
 */
enum Enum_BSP_LCD_Key
{
    BSP_LCD_Key_NONE = 0,
    BSP_LCD_Key_CENTER,
    BSP_LCD_Key_LEFT,
    BSP_LCD_Key_RIGHT,
    BSP_LCD_Key_DOWN,
    BSP_LCD_Key_UP,
};

/**
 * @brief LCD按键状态
 *
 */
enum Enum_BSP_LCD_Key_Status
{
    BSP_LCD_Key_Status_FREE = 0,
    BSP_LCD_Key_Status_TRIG_FREE_PRESSED,
    BSP_LCD_Key_Status_TRIG_PRESSED_FREE,
    BSP_LCD_Key_Status_PRESSED,
};

/**
 * @brief Specialized, LCD板载五向按键
 *
 */
class Class_LCD_Key
{
public:
    void Init(Struct_ADC_Manage_Object *__ADC_Manage_Object, const uint16_t &__ADC_Buffer_Offset = 0, const uint16_t &__ADC_Full_Scale = 4095);

    inline Enum_BSP_LCD_Key Get_Key() const;

    inline Enum_BSP_LCD_Key Get_Last_Key() const;

    inline Enum_BSP_LCD_Key_Status Get_Key_Status() const;

    inline uint16_t Get_ADC_Value() const;

    inline uint16_t Get_ADC_Value_12Bit() const;

    inline bool Is_Key_Pressed(const Enum_BSP_LCD_Key &__Key) const;

    void TIM_1ms_Process_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 绑定的ADC
    Struct_ADC_Manage_Object *ADC_Manage_Object = nullptr;
    // 绑定的ADC缓冲区偏移量
    uint16_t ADC_Buffer_Offset = 0;
    // ADC满量程
    uint16_t ADC_Full_Scale = 4095;

    // 常量

    static constexpr uint8_t Debounce_Count_Threshold = 8;

    // 内部变量

    uint8_t Stable_Count = 0;

    Enum_BSP_LCD_Key Sample_Key = BSP_LCD_Key_NONE;
    Enum_BSP_LCD_Key Now_Key = BSP_LCD_Key_NONE;
    Enum_BSP_LCD_Key Last_Key = BSP_LCD_Key_NONE;

    // 读变量

    Enum_BSP_LCD_Key_Status Key_Status = BSP_LCD_Key_Status_FREE;
    uint16_t ADC_Value = 0;
    uint16_t ADC_Value_12Bit = 0;

    // 写变量

    // 读写变量

    // 内部函数

    Enum_BSP_LCD_Key Decode_Key(const uint16_t &__ADC_Value_12Bit) const;
};

/* Exported variables --------------------------------------------------------*/

extern Class_LCD_Key BSP_LCD_Key;

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取当前稳定键值
 *
 * @return Enum_BSP_LCD_Key 键值
 */
inline Enum_BSP_LCD_Key Class_LCD_Key::Get_Key() const
{
    return (Now_Key);
}

/**
 * @brief 获取最近一次有效键值
 *
 * @return Enum_BSP_LCD_Key 最近一次按下的键值
 */
inline Enum_BSP_LCD_Key Class_LCD_Key::Get_Last_Key() const
{
    return (Last_Key);
}

/**
 * @brief 获取按键状态
 *
 * @return Enum_BSP_LCD_Key_Status 按键状态
 */
inline Enum_BSP_LCD_Key_Status Class_LCD_Key::Get_Key_Status() const
{
    return (Key_Status);
}

/**
 * @brief 获取原始ADC值
 *
 * @return uint16_t 原始ADC值
 */
inline uint16_t Class_LCD_Key::Get_ADC_Value() const
{
    return (ADC_Value);
}

/**
 * @brief 获取归一化后的12位ADC值
 *
 * @return uint16_t 12位ADC值
 */
inline uint16_t Class_LCD_Key::Get_ADC_Value_12Bit() const
{
    return (ADC_Value_12Bit);
}

/**
 * @brief 判断某个按键是否按下
 *
 * @param __Key 目标键值
 * @return true 当前稳定键值与目标一致
 * @return false 当前稳定键值与目标不一致
 */
inline bool Class_LCD_Key::Is_Key_Pressed(const Enum_BSP_LCD_Key &__Key) const
{
    return (Now_Key == __Key);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
