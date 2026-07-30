/**
 * @file bsp_lcd.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief ST7789V2 SPI LCD显示屏驱动
 * @version 0.1
 * @date 2026-05-22 0.1 参考1文件夹中的PDF新建文档
 *
 * @copyright USTC-RoboWalker (c) 2026
 *
 */

#ifndef BSP_LCD_H
#define BSP_LCD_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

#include "gpio.h"
#include "spi.h"
#include "stm32h7xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

constexpr uint16_t BSP_LCD_COLOR_BLACK = 0x0000;
constexpr uint16_t BSP_LCD_COLOR_WHITE = 0xffff;
constexpr uint16_t BSP_LCD_COLOR_RED = 0xf800;
constexpr uint16_t BSP_LCD_COLOR_GREEN = 0x07e0;
constexpr uint16_t BSP_LCD_COLOR_BLUE = 0x001f;
constexpr uint16_t BSP_LCD_COLOR_YELLOW = 0xffe0;
constexpr uint16_t BSP_LCD_COLOR_CYAN = 0x07ff;
constexpr uint16_t BSP_LCD_COLOR_MAGENTA = 0xf81f;
constexpr uint16_t BSP_LCD_COLOR_GRAY = 0x8410;
constexpr uint16_t BSP_LCD_COLOR_ORANGE = 0xfd20;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief LCD旋转方向
 *
 */
enum Enum_BSP_LCD_Rotation
{
    BSP_LCD_Rotation_0 = 0,
    BSP_LCD_Rotation_90,
    BSP_LCD_Rotation_180,
    BSP_LCD_Rotation_270,
};

/**
 * @brief LCD硬件配置
 *
 */
struct Struct_BSP_LCD_Config
{
    SPI_HandleTypeDef *SPI_Handler = nullptr;

    GPIO_TypeDef *CS_GPIOx = nullptr;
    uint16_t CS_GPIO_Pin = 0;
    GPIO_PinState CS_Active_Level = GPIO_PIN_RESET;

    GPIO_TypeDef *DC_GPIOx = nullptr;
    uint16_t DC_GPIO_Pin = 0;

    GPIO_TypeDef *RST_GPIOx = nullptr;
    uint16_t RST_GPIO_Pin = 0;
    GPIO_PinState RST_Active_Level = GPIO_PIN_RESET;

    GPIO_TypeDef *BL_GPIOx = nullptr;
    uint16_t BL_GPIO_Pin = 0;
    GPIO_PinState BL_Active_Level = GPIO_PIN_SET;

    uint16_t Width = 240;
    uint16_t Height = 240;
    uint16_t X_Offset = 0;
    uint16_t Y_Offset = 0;

    Enum_BSP_LCD_Rotation Rotation = BSP_LCD_Rotation_0;
};

/**
 * @brief Specialized, ST7789V2 SPI LCD显示屏
 *
 */
class Class_LCD
{
public:
    void Init(const Struct_BSP_LCD_Config &__LCD_Config);

    inline uint16_t Get_Width() const;

    inline uint16_t Get_Height() const;

    inline bool Is_Inited() const;

    void Reset() const;

    void Set_Rotation(const Enum_BSP_LCD_Rotation &__Rotation);

    void Set_Backlight(const bool &__Status) const;

    void Display_On() const;

    void Display_Off() const;

    void Invert_Color(const bool &__Status) const;

    void Clear(const uint16_t &__Color);

    void Draw_Point(const uint16_t &__X, const uint16_t &__Y, const uint16_t &__Color);

    void Fill_Rectangle(const uint16_t &__X, const uint16_t &__Y, const uint16_t &__Width, const uint16_t &__Height, const uint16_t &__Color);

    void Draw_Line(const uint16_t &__X0, const uint16_t &__Y0, const uint16_t &__X1, const uint16_t &__Y1, const uint16_t &__Color);

    void Draw_Rectangle(const uint16_t &__X, const uint16_t &__Y, const uint16_t &__Width, const uint16_t &__Height, const uint16_t &__Color);

    void Draw_RGB565_Picture(const uint16_t &__X, const uint16_t &__Y, const uint16_t &__Width, const uint16_t &__Height, const uint16_t *__Picture);

    void Draw_Char(const uint16_t &__X, const uint16_t &__Y, const char &__Char, const uint16_t &__Font_Color, const uint16_t &__Background_Color = BSP_LCD_COLOR_BLACK, const uint8_t &__Scale = 2);

    void Draw_String(const uint16_t &__X, const uint16_t &__Y, const char *__String, const uint16_t &__Font_Color, const uint16_t &__Background_Color = BSP_LCD_COLOR_BLACK, const uint8_t &__Scale = 2, const bool &__Auto_Wrap = true);

protected:
    // 初始化相关常量

    // 常量

    static constexpr uint32_t SPI_TIMEOUT = 100;
    static constexpr uint16_t STREAM_BUFFER_PIXEL_NUM = 64;
    static constexpr uint8_t ASCII_FONT_WIDTH = 6;
    static constexpr uint8_t ASCII_FONT_HEIGHT = 8;

    // 内部变量

    Struct_BSP_LCD_Config LCD_Config;

    Enum_BSP_LCD_Rotation Rotation = BSP_LCD_Rotation_0;

    uint16_t Now_Width = 0;
    uint16_t Now_Height = 0;

    bool LCD_Inited = false;

    uint8_t Stream_Buffer[STREAM_BUFFER_PIXEL_NUM * 2] = {0};

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数

    inline void Activate_Chip_Select() const;

    inline void Deactivate_Chip_Select() const;

    inline void Set_Data_Command_Pin(const GPIO_PinState &__GPIO_State) const;

    inline void Set_Reset_Pin(const GPIO_PinState &__GPIO_State) const;

    void Write_Command(const uint8_t &__Command) const;

    void Write_Data(const uint8_t *__Data, const uint32_t &__Length) const;

    void Write_Color_Burst(const uint16_t &__Color, const uint32_t &__Pixel_Number);

    void Write_Picture_Burst(const uint16_t *__Picture, const uint32_t &__Pixel_Number);

    void Set_Address_Window(const uint16_t &__X_Start, const uint16_t &__Y_Start, const uint16_t &__X_End, const uint16_t &__Y_End) const;

    uint8_t Get_MADCTL_Value(const Enum_BSP_LCD_Rotation &__Rotation) const;

    void Get_ASCII_5x7_Dot_Matrix(const char &__Char, uint8_t *__Dot_Matrix) const;

    void Clear_Controller_GRAM(const uint16_t &__Color);
};

/* Exported variables --------------------------------------------------------*/

extern Class_LCD BSP_LCD;

/* Exported function declarations --------------------------------------------*/

/**
 * @brief RGB888转RGB565
 *
 * @param __Red 红色分量
 * @param __Green 绿色分量
 * @param __Blue 蓝色分量
 * @return uint16_t RGB565颜色
 */
constexpr uint16_t BSP_LCD_Make_RGB565(const uint8_t &__Red, const uint8_t &__Green, const uint8_t &__Blue)
{
    return (((uint16_t)(__Red & 0xf8) << 8) | ((uint16_t)(__Green & 0xfc) << 3) | ((uint16_t)(__Blue) >> 3));
}

/**
 * @brief 获取当前显示宽度
 *
 * @return uint16_t 宽度
 */
inline uint16_t Class_LCD::Get_Width() const
{
    return (Now_Width);
}

/**
 * @brief 获取当前显示高度
 *
 * @return uint16_t 高度
 */
inline uint16_t Class_LCD::Get_Height() const
{
    return (Now_Height);
}

/**
 * @brief 获取LCD是否已初始化
 *
 * @return true 已初始化
 * @return false 未初始化
 */
inline bool Class_LCD::Is_Inited() const
{
    return (LCD_Inited);
}

/**
 * @brief 片选有效
 *
 */
inline void Class_LCD::Activate_Chip_Select() const
{
    if (LCD_Config.CS_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(LCD_Config.CS_GPIOx, LCD_Config.CS_GPIO_Pin, LCD_Config.CS_Active_Level);
    }
}

/**
 * @brief 片选无效
 *
 */
inline void Class_LCD::Deactivate_Chip_Select() const
{
    if (LCD_Config.CS_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(LCD_Config.CS_GPIOx, LCD_Config.CS_GPIO_Pin, LCD_Config.CS_Active_Level == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

/**
 * @brief 设置DC引脚电平
 *
 * @param __GPIO_State 引脚电平
 */
inline void Class_LCD::Set_Data_Command_Pin(const GPIO_PinState &__GPIO_State) const
{
    if (LCD_Config.DC_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(LCD_Config.DC_GPIOx, LCD_Config.DC_GPIO_Pin, __GPIO_State);
    }
}

/**
 * @brief 设置复位引脚电平
 *
 * @param __GPIO_State 引脚电平
 */
inline void Class_LCD::Set_Reset_Pin(const GPIO_PinState &__GPIO_State) const
{
    if (LCD_Config.RST_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(LCD_Config.RST_GPIOx, LCD_Config.RST_GPIO_Pin, __GPIO_State);
    }
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
