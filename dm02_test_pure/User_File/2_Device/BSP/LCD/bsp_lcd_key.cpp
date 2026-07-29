/**
 * @file bsp_lcd_key.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief LCD板载五向按键驱动
 * @version 0.1
 * @date 2026-05-22 0.1 
 *
 * @copyright USTC-RoboWalker (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_lcd_key.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Class_LCD_Key BSP_LCD_Key;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化LCD按键
 *
 * @param __ADC_Manage_Object ADC管理对象
 * @param __ADC_Buffer_Offset ADC缓冲区偏移量
 * @param __ADC_Full_Scale ADC满量程
 */
void Class_LCD_Key::Init(Struct_ADC_Manage_Object *__ADC_Manage_Object, const uint16_t &__ADC_Buffer_Offset, const uint16_t &__ADC_Full_Scale)
{
    ADC_Manage_Object = __ADC_Manage_Object;
    ADC_Buffer_Offset = __ADC_Buffer_Offset;
    ADC_Full_Scale = (__ADC_Full_Scale == 0) ? 4095 : __ADC_Full_Scale;

    Stable_Count = 0;
    Sample_Key = BSP_LCD_Key_NONE;
    Now_Key = BSP_LCD_Key_NONE;
    Last_Key = BSP_LCD_Key_NONE;
    Key_Status = BSP_LCD_Key_Status_FREE;
    ADC_Value = 0;
    ADC_Value_12Bit = 0;
}

/**
 * @brief 1ms周期按键处理
 *
 */
void Class_LCD_Key::TIM_1ms_Process_PeriodElapsedCallback()
{
    Enum_BSP_LCD_Key candidate_key = BSP_LCD_Key_NONE;

    if (ADC_Manage_Object == nullptr)
    {
        Key_Status = BSP_LCD_Key_Status_FREE;
        return;
    }

    ADC_Value = ADC_Manage_Object->ADC_Data[ADC_Buffer_Offset];
    ADC_Value_12Bit = (uint16_t)(((uint32_t)(ADC_Value) * 4095u + (uint32_t)(ADC_Full_Scale / 2u)) / (uint32_t)(ADC_Full_Scale));
    candidate_key = Decode_Key(ADC_Value_12Bit);

    if (candidate_key != Sample_Key)
    {
        Sample_Key = candidate_key;
        Stable_Count = 0;
    }
    else if (Stable_Count < Debounce_Count_Threshold)
    {
        Stable_Count++;
    }

    if ((Stable_Count >= Debounce_Count_Threshold) && (Now_Key != Sample_Key))
    {
        const Enum_BSP_LCD_Key pre_key = Now_Key;

        Now_Key = Sample_Key;

        if (Now_Key != BSP_LCD_Key_NONE)
        {
            Last_Key = Now_Key;
        }

        if ((pre_key == BSP_LCD_Key_NONE) && (Now_Key != BSP_LCD_Key_NONE))
        {
            Key_Status = BSP_LCD_Key_Status_TRIG_FREE_PRESSED;
        }
        else if ((pre_key != BSP_LCD_Key_NONE) && (Now_Key == BSP_LCD_Key_NONE))
        {
            Key_Status = BSP_LCD_Key_Status_TRIG_PRESSED_FREE;
        }
        else if (Now_Key == BSP_LCD_Key_NONE)
        {
            Key_Status = BSP_LCD_Key_Status_FREE;
        }
        else
        {
            Key_Status = BSP_LCD_Key_Status_PRESSED;
        }

        return;
    }

    if (Now_Key == BSP_LCD_Key_NONE)
    {
        Key_Status = BSP_LCD_Key_Status_FREE;
    }
    else
    {
        Key_Status = BSP_LCD_Key_Status_PRESSED;
    }
}

/**
 * @brief 根据原理图中的ADC电压表解码按键
 *
 * @param __ADC_Value_12Bit 12位ADC值
 * @return Enum_BSP_LCD_Key 键值
 */
Enum_BSP_LCD_Key Class_LCD_Key::Decode_Key(const uint16_t &__ADC_Value_12Bit) const
{
    // Measured on this board (12-bit ADC): none=270, down=760, up=1220,
    // right=1695, left=2175, center=2740. Use midpoint thresholds.
    const uint16_t key_value = __ADC_Value_12Bit;

    if (key_value < 515)
    {
        return (BSP_LCD_Key_NONE);
    }
    else if (key_value < 990)
    {
        return (BSP_LCD_Key_DOWN);
    }
    else if (key_value < 1457)
    {
        return (BSP_LCD_Key_UP);
    }
    else if (key_value < 1935)
    {
        return (BSP_LCD_Key_RIGHT);
    }
    else if (key_value < 2457)
    {
        return (BSP_LCD_Key_LEFT);
    }
    else
    {
        return (BSP_LCD_Key_CENTER);
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
