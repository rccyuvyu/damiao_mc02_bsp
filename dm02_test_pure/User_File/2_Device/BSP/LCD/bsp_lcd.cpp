/**
 * @file bsp_lcd.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief ST7789V2 SPI LCD显示屏驱动
 * @version 0.1
 * @date 2026-05-22 0.1 参考1文件夹中的PDF新建文档
 *
 * @copyright USTC-RoboWalker (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_lcd.h"

/* Private macros ------------------------------------------------------------*/

// ST7789V2命令
#define BSP_LCD_CMD_SWRESET 0x01
#define BSP_LCD_CMD_SLPOUT 0x11
#define BSP_LCD_CMD_NORON 0x13
#define BSP_LCD_CMD_INVOFF 0x20
#define BSP_LCD_CMD_INVON 0x21
#define BSP_LCD_CMD_DISPOFF 0x28
#define BSP_LCD_CMD_DISPON 0x29
#define BSP_LCD_CMD_CASET 0x2a
#define BSP_LCD_CMD_RASET 0x2b
#define BSP_LCD_CMD_RAMWR 0x2c
#define BSP_LCD_CMD_MADCTL 0x36
#define BSP_LCD_CMD_COLMOD 0x3a

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Class_LCD BSP_LCD;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化LCD
 *
 * @param __LCD_Config LCD硬件配置
 */
void Class_LCD::Init(const Struct_BSP_LCD_Config &__LCD_Config)
{
    static const uint8_t cmd_b2[] = {0x0c, 0x0c, 0x00, 0x33, 0x33};
    static const uint8_t cmd_bb[] = {0x19};
    static const uint8_t cmd_c0[] = {0x2c};
    static const uint8_t cmd_c2[] = {0x01};
    static const uint8_t cmd_c3[] = {0x12};
    static const uint8_t cmd_c4[] = {0x20};
    static const uint8_t cmd_c6[] = {0x0f};
    static const uint8_t cmd_d0[] = {0xa4, 0xa1};
    static const uint8_t cmd_e0[] = {0xd0, 0x04, 0x0d, 0x11, 0x13, 0x2b, 0x3f, 0x54, 0x4c, 0x18, 0x0d, 0x0b, 0x1f, 0x23};
    static const uint8_t cmd_e1[] = {0xd0, 0x04, 0x0c, 0x11, 0x13, 0x2c, 0x3f, 0x44, 0x51, 0x2f, 0x1f, 0x1f, 0x20, 0x23};
    static const uint8_t color_mode = 0x55;
    static const uint8_t porch_control = 0x35;

    LCD_Config = __LCD_Config;
    Rotation = LCD_Config.Rotation;

    Now_Width = LCD_Config.Width;
    Now_Height = LCD_Config.Height;

    if (LCD_Config.BL_GPIOx != nullptr)
    {
        Set_Backlight(false);
    }

    Reset();

    Write_Command(BSP_LCD_CMD_SWRESET);
    ::HAL_Delay(150u);

    Write_Command(0xb2);
    Write_Data(cmd_b2, sizeof(cmd_b2));

    Write_Command(0xb7);
    Write_Data(&porch_control, 1);

    Write_Command(0xbb);
    Write_Data(cmd_bb, sizeof(cmd_bb));

    Write_Command(0xc0);
    Write_Data(cmd_c0, sizeof(cmd_c0));

    Write_Command(0xc2);
    Write_Data(cmd_c2, sizeof(cmd_c2));

    Write_Command(0xc3);
    Write_Data(cmd_c3, sizeof(cmd_c3));

    Write_Command(0xc4);
    Write_Data(cmd_c4, sizeof(cmd_c4));

    Write_Command(0xc6);
    Write_Data(cmd_c6, sizeof(cmd_c6));

    Write_Command(0xd0);
    Write_Data(cmd_d0, sizeof(cmd_d0));

    Write_Command(BSP_LCD_CMD_COLMOD);
    Write_Data(&color_mode, 1);

    Write_Command(0xe0);
    Write_Data(cmd_e0, sizeof(cmd_e0));

    Write_Command(0xe1);
    Write_Data(cmd_e1, sizeof(cmd_e1));

    Write_Command(BSP_LCD_CMD_SLPOUT);
    ::HAL_Delay(120u);

    Set_Rotation(Rotation);
    Invert_Color(true);

    Write_Command(BSP_LCD_CMD_NORON);
    ::HAL_Delay(10u);

    Write_Command(BSP_LCD_CMD_DISPON);
    ::HAL_Delay(20u);

    LCD_Inited = true;

    // 先清除整个ST7789控制器GRAM, 避免修改Offset后残留上一版显示内容
    Clear_Controller_GRAM(BSP_LCD_COLOR_BLACK);

    Set_Backlight(true);
    Clear(BSP_LCD_COLOR_BLACK);
}

/**
 * @brief 硬件复位LCD
 *
 */
void Class_LCD::Reset() const
{
    if (LCD_Config.RST_GPIOx == nullptr)
    {
        return;
    }

    Set_Reset_Pin(LCD_Config.RST_Active_Level);
    ::HAL_Delay(20u);
    Set_Reset_Pin(LCD_Config.RST_Active_Level == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    ::HAL_Delay(150u);
}

/**
 * @brief 设置LCD旋转方向
 *
 * @param __Rotation 旋转方向
 */
void Class_LCD::Set_Rotation(const Enum_BSP_LCD_Rotation &__Rotation)
{
    const uint8_t madctl = Get_MADCTL_Value(__Rotation);

    Rotation = __Rotation;

    if ((__Rotation == BSP_LCD_Rotation_0) || (__Rotation == BSP_LCD_Rotation_180))
    {
        Now_Width = LCD_Config.Width;
        Now_Height = LCD_Config.Height;
    }
    else
    {
        Now_Width = LCD_Config.Height;
        Now_Height = LCD_Config.Width;
    }

    Write_Command(BSP_LCD_CMD_MADCTL);
    Write_Data(&madctl, 1);
}

/**
 * @brief 设置背光开关
 *
 * @param __Status true为打开背光, false为关闭背光
 */
void Class_LCD::Set_Backlight(const bool &__Status) const
{
    if (LCD_Config.BL_GPIOx == nullptr)
    {
        return;
    }

    HAL_GPIO_WritePin(LCD_Config.BL_GPIOx, LCD_Config.BL_GPIO_Pin, __Status ? LCD_Config.BL_Active_Level : (LCD_Config.BL_Active_Level == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET));
}

/**
 * @brief 打开显示
 *
 */
void Class_LCD::Display_On() const
{
    Write_Command(BSP_LCD_CMD_DISPON);
}

/**
 * @brief 关闭显示
 *
 */
void Class_LCD::Display_Off() const
{
    Write_Command(BSP_LCD_CMD_DISPOFF);
}

/**
 * @brief 设置是否颜色反显
 *
 * @param __Status true为反显, false为正常
 */
void Class_LCD::Invert_Color(const bool &__Status) const
{
    Write_Command(__Status ? BSP_LCD_CMD_INVON : BSP_LCD_CMD_INVOFF);
}

/**
 * @brief 清屏
 *
 * @param __Color 颜色
 */
void Class_LCD::Clear(const uint16_t &__Color)
{
    Fill_Rectangle(0, 0, Now_Width, Now_Height, __Color);
}

/**
 * @brief 画点
 *
 * @param __X x坐标
 * @param __Y y坐标
 * @param __Color 颜色
 */
void Class_LCD::Draw_Point(const uint16_t &__X, const uint16_t &__Y, const uint16_t &__Color)
{
    if ((__X >= Now_Width) || (__Y >= Now_Height))
    {
        return;
    }

    Set_Address_Window(__X, __Y, __X, __Y);
    Write_Color_Burst(__Color, 1);
}

/**
 * @brief 填充矩形
 *
 * @param __X 起始x
 * @param __Y 起始y
 * @param __Width 宽度
 * @param __Height 高度
 * @param __Color 颜色
 */
void Class_LCD::Fill_Rectangle(const uint16_t &__X, const uint16_t &__Y, const uint16_t &__Width, const uint16_t &__Height, const uint16_t &__Color)
{
    uint32_t pixel_num = 0;
    uint16_t x_end = 0;
    uint16_t y_end = 0;

    if ((__X >= Now_Width) || (__Y >= Now_Height) || (__Width == 0) || (__Height == 0))
    {
        return;
    }

    x_end = __X + __Width - 1;
    y_end = __Y + __Height - 1;

    if (x_end >= Now_Width)
    {
        x_end = Now_Width - 1;
    }

    if (y_end >= Now_Height)
    {
        y_end = Now_Height - 1;
    }

    pixel_num = (uint32_t)(x_end - __X + 1) * (uint32_t)(y_end - __Y + 1);

    Set_Address_Window(__X, __Y, x_end, y_end);
    Write_Color_Burst(__Color, pixel_num);
}

/**
 * @brief Bresenham画线
 *
 * @param __X0 起始x
 * @param __Y0 起始y
 * @param __X1 终止x
 * @param __Y1 终止y
 * @param __Color 颜色
 */
void Class_LCD::Draw_Line(const uint16_t &__X0, const uint16_t &__Y0, const uint16_t &__X1, const uint16_t &__Y1, const uint16_t &__Color)
{
    int32_t x0 = __X0;
    int32_t y0 = __Y0;
    const int32_t x1 = __X1;
    const int32_t y1 = __Y1;
    const int32_t dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    const int32_t sx = (x0 < x1) ? 1 : -1;
    const int32_t dy = (y1 > y0) ? -(y1 - y0) : -(y0 - y1);
    const int32_t sy = (y0 < y1) ? 1 : -1;
    int32_t err = dx + dy;

    while (1)
    {
        if ((x0 >= 0) && (x0 < Now_Width) && (y0 >= 0) && (y0 < Now_Height))
        {
            Draw_Point((uint16_t)(x0), (uint16_t)(y0), __Color);
        }

        if ((x0 == x1) && (y0 == y1))
        {
            break;
        }

        const int32_t err_2 = err << 1;

        if (err_2 >= dy)
        {
            err += dy;
            x0 += sx;
        }

        if (err_2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

/**
 * @brief 画矩形边框
 *
 * @param __X 起始x
 * @param __Y 起始y
 * @param __Width 宽度
 * @param __Height 高度
 * @param __Color 颜色
 */
void Class_LCD::Draw_Rectangle(const uint16_t &__X, const uint16_t &__Y, const uint16_t &__Width, const uint16_t &__Height, const uint16_t &__Color)
{
    if ((__Width == 0) || (__Height == 0))
    {
        return;
    }

    Draw_Line(__X, __Y, __X + __Width - 1, __Y, __Color);
    Draw_Line(__X, __Y, __X, __Y + __Height - 1, __Color);
    Draw_Line(__X + __Width - 1, __Y, __X + __Width - 1, __Y + __Height - 1, __Color);
    Draw_Line(__X, __Y + __Height - 1, __X + __Width - 1, __Y + __Height - 1, __Color);
}

/**
 * @brief 绘制RGB565图片
 *
 * @param __X 起始x
 * @param __Y 起始y
 * @param __Width 图片宽度
 * @param __Height 图片高度
 * @param __Picture 图片数据指针
 */
void Class_LCD::Draw_RGB565_Picture(const uint16_t &__X, const uint16_t &__Y, const uint16_t &__Width, const uint16_t &__Height, const uint16_t *__Picture)
{
    uint32_t pixel_num = 0;
    uint16_t x_end = 0;
    uint16_t y_end = 0;

    if ((__Picture == nullptr) || (__X >= Now_Width) || (__Y >= Now_Height) || (__Width == 0) || (__Height == 0))
    {
        return;
    }

    x_end = __X + __Width - 1;
    y_end = __Y + __Height - 1;

    if ((x_end >= Now_Width) || (y_end >= Now_Height))
    {
        return;
    }

    pixel_num = (uint32_t)(__Width) * (uint32_t)(__Height);

    Set_Address_Window(__X, __Y, x_end, y_end);
    Write_Picture_Burst(__Picture, pixel_num);
}

/**
 * @brief 绘制ASCII字符
 *
 * @param __X 左上角x坐标
 * @param __Y 左上角y坐标
 * @param __Char ASCII字符
 * @param __Font_Color 字体颜色
 * @param __Background_Color 背景颜色
 * @param __Scale 字体缩放倍数, 建议1或2
 */
void Class_LCD::Draw_Char(const uint16_t &__X, const uint16_t &__Y, const char &__Char, const uint16_t &__Font_Color, const uint16_t &__Background_Color, const uint8_t &__Scale)
{
    uint8_t dot_matrix[7] = {0};
    const uint8_t scale = (__Scale == 0) ? 1 : __Scale;
    const uint16_t char_width = ASCII_FONT_WIDTH * scale;
    const uint16_t char_height = ASCII_FONT_HEIGHT * scale;

    if ((__X >= Now_Width) || (__Y >= Now_Height))
    {
        return;
    }

    if ((__X + char_width > Now_Width) || (__Y + char_height > Now_Height))
    {
        return;
    }

    Get_ASCII_5x7_Dot_Matrix(__Char, dot_matrix);

    for (uint8_t row = 0; row < ASCII_FONT_HEIGHT; row++)
    {
        for (uint8_t column = 0; column < ASCII_FONT_WIDTH; column++)
        {
            uint16_t pixel_color = __Background_Color;

            if ((row < 7) && (column < 5))
            {
                pixel_color = (dot_matrix[row] & (1u << (4u - column))) ? __Font_Color : __Background_Color;
            }

            Fill_Rectangle(__X + column * scale, __Y + row * scale, scale, scale, pixel_color);
        }
    }
}

/**
 * @brief 绘制ASCII字符串
 *
 * @param __X 左上角x坐标
 * @param __Y 左上角y坐标
 * @param __String 字符串指针
 * @param __Font_Color 字体颜色
 * @param __Background_Color 背景颜色
 * @param __Scale 字体缩放倍数, 建议1或2
 * @param __Auto_Wrap 是否自动换行
 */
void Class_LCD::Draw_String(const uint16_t &__X, const uint16_t &__Y, const char *__String, const uint16_t &__Font_Color, const uint16_t &__Background_Color, const uint8_t &__Scale, const bool &__Auto_Wrap)
{
    uint16_t now_x = __X;
    uint16_t now_y = __Y;
    const uint8_t scale = (__Scale == 0) ? 1 : __Scale;
    const uint16_t char_width = ASCII_FONT_WIDTH * scale;
    const uint16_t char_height = ASCII_FONT_HEIGHT * scale;

    if (__String == nullptr)
    {
        return;
    }

    while (*__String != '\0')
    {
        if (*__String == '\n')
        {
            now_x = __X;
            now_y += char_height;
            __String++;
            continue;
        }

        if (*__String == '\t')
        {
            now_x += char_width * 4;
            __String++;
            continue;
        }

        if (__Auto_Wrap && (now_x + char_width > Now_Width))
        {
            now_x = __X;
            now_y += char_height;
        }

        if (now_y + char_height > Now_Height)
        {
            break;
        }

        Draw_Char(now_x, now_y, *__String, __Font_Color, __Background_Color, scale);
        now_x += char_width;
        __String++;
    }
}

/**
 * @brief 写LCD命令
 *
 * @param __Command 命令
 */
void Class_LCD::Write_Command(const uint8_t &__Command) const
{
    if (LCD_Config.SPI_Handler == nullptr)
    {
        return;
    }

    Activate_Chip_Select();
    Set_Data_Command_Pin(GPIO_PIN_RESET);
    HAL_SPI_Transmit(LCD_Config.SPI_Handler, (uint8_t *)(&__Command), 1, SPI_TIMEOUT);
    Deactivate_Chip_Select();
}

/**
 * @brief 写LCD数据
 *
 * @param __Data 数据地址
 * @param __Length 数据长度
 */
void Class_LCD::Write_Data(const uint8_t *__Data, const uint32_t &__Length) const
{
    if ((LCD_Config.SPI_Handler == nullptr) || (__Data == nullptr) || (__Length == 0))
    {
        return;
    }

    Activate_Chip_Select();
    Set_Data_Command_Pin(GPIO_PIN_SET);
    HAL_SPI_Transmit(LCD_Config.SPI_Handler, (uint8_t *)(__Data), __Length, SPI_TIMEOUT);
    Deactivate_Chip_Select();
}

/**
 * @brief 连续写入同一种颜色
 *
 * @param __Color 颜色
 * @param __Pixel_Number 像素数量
 */
void Class_LCD::Write_Color_Burst(const uint16_t &__Color, const uint32_t &__Pixel_Number)
{
    const uint8_t color_high = (uint8_t)(__Color >> 8);
    const uint8_t color_low = (uint8_t)(__Color & 0xff);
    uint32_t remain_pixel_num = __Pixel_Number;

    for (uint16_t i = 0; i < STREAM_BUFFER_PIXEL_NUM; i++)
    {
        Stream_Buffer[i * 2] = color_high;
        Stream_Buffer[i * 2 + 1] = color_low;
    }

    Activate_Chip_Select();
    Set_Data_Command_Pin(GPIO_PIN_SET);

    while (remain_pixel_num > 0)
    {
        const uint16_t send_pixel_num = (remain_pixel_num > STREAM_BUFFER_PIXEL_NUM) ? STREAM_BUFFER_PIXEL_NUM : remain_pixel_num;
        HAL_SPI_Transmit(LCD_Config.SPI_Handler, Stream_Buffer, send_pixel_num * 2, SPI_TIMEOUT);
        remain_pixel_num -= send_pixel_num;
    }

    Deactivate_Chip_Select();
}

/**
 * @brief 连续写入图片数据
 *
 * @param __Picture 图片数据地址
 * @param __Pixel_Number 像素数量
 */
void Class_LCD::Write_Picture_Burst(const uint16_t *__Picture, const uint32_t &__Pixel_Number)
{
    uint32_t offset = 0;

    if ((__Picture == nullptr) || (__Pixel_Number == 0))
    {
        return;
    }

    Activate_Chip_Select();
    Set_Data_Command_Pin(GPIO_PIN_SET);

    while (offset < __Pixel_Number)
    {
        const uint16_t send_pixel_num = ((__Pixel_Number - offset) > STREAM_BUFFER_PIXEL_NUM) ? STREAM_BUFFER_PIXEL_NUM : (__Pixel_Number - offset);

        for (uint16_t i = 0; i < send_pixel_num; i++)
        {
            const uint16_t color = __Picture[offset + i];
            Stream_Buffer[i * 2] = (uint8_t)(color >> 8);
            Stream_Buffer[i * 2 + 1] = (uint8_t)(color & 0xff);
        }

        HAL_SPI_Transmit(LCD_Config.SPI_Handler, Stream_Buffer, send_pixel_num * 2, SPI_TIMEOUT);
        offset += send_pixel_num;
    }

    Deactivate_Chip_Select();
}

/**
 * @brief 设置GRAM地址窗口
 *
 * @param __X_Start 起始x
 * @param __Y_Start 起始y
 * @param __X_End 终止x
 * @param __Y_End 终止y
 */
void Class_LCD::Set_Address_Window(const uint16_t &__X_Start, const uint16_t &__Y_Start, const uint16_t &__X_End, const uint16_t &__Y_End) const
{
    uint8_t data_buffer[4] = {0};
    const uint16_t x_start = __X_Start + LCD_Config.X_Offset;
    const uint16_t x_end = __X_End + LCD_Config.X_Offset;
    const uint16_t y_start = __Y_Start + LCD_Config.Y_Offset;
    const uint16_t y_end = __Y_End + LCD_Config.Y_Offset;

    Write_Command(BSP_LCD_CMD_CASET);
    data_buffer[0] = (uint8_t)(x_start >> 8);
    data_buffer[1] = (uint8_t)(x_start & 0xff);
    data_buffer[2] = (uint8_t)(x_end >> 8);
    data_buffer[3] = (uint8_t)(x_end & 0xff);
    Write_Data(data_buffer, sizeof(data_buffer));

    Write_Command(BSP_LCD_CMD_RASET);
    data_buffer[0] = (uint8_t)(y_start >> 8);
    data_buffer[1] = (uint8_t)(y_start & 0xff);
    data_buffer[2] = (uint8_t)(y_end >> 8);
    data_buffer[3] = (uint8_t)(y_end & 0xff);
    Write_Data(data_buffer, sizeof(data_buffer));

    Write_Command(BSP_LCD_CMD_RAMWR);
}

/**
 * @brief 获取MADCTL寄存器配置值
 *
 * @param __Rotation 旋转方向
 * @return uint8_t MADCTL寄存器值
 */
uint8_t Class_LCD::Get_MADCTL_Value(const Enum_BSP_LCD_Rotation &__Rotation) const
{
    constexpr uint8_t MADCTL_MY = 0x80;
    constexpr uint8_t MADCTL_MX = 0x40;
    constexpr uint8_t MADCTL_MV = 0x20;
    constexpr uint8_t MADCTL_BGR = 0x08;

    switch (__Rotation)
    {
    case BSP_LCD_Rotation_0:
    {
        return (MADCTL_BGR);
    }
    case BSP_LCD_Rotation_90:
    {
        return (MADCTL_MX | MADCTL_MV | MADCTL_BGR);
    }
    case BSP_LCD_Rotation_180:
    {
        return (MADCTL_MX | MADCTL_MY | MADCTL_BGR);
    }
    case BSP_LCD_Rotation_270:
    {
        return (MADCTL_MY | MADCTL_MV | MADCTL_BGR);
    }
    default:
    {
        return (MADCTL_BGR);
    }
    }
}

/**
 * @brief 获取ASCII 5x7点阵
 *
 * @param __Char ASCII字符
 * @param __Dot_Matrix 7字节点阵输出缓冲区
 */
void Class_LCD::Get_ASCII_5x7_Dot_Matrix(const char &__Char, uint8_t *__Dot_Matrix) const
{
    char tmp_char = __Char;

    if (__Dot_Matrix == nullptr)
    {
        return;
    }

    for (uint8_t i = 0; i < 7; i++)
    {
        __Dot_Matrix[i] = 0x00;
    }

    if ((tmp_char >= 'a') && (tmp_char <= 'z'))
    {
        switch (tmp_char)
        {
        case 'a': __Dot_Matrix[2] = 0x0e; __Dot_Matrix[3] = 0x01; __Dot_Matrix[4] = 0x0f; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x0f; return;
        case 'b': __Dot_Matrix[0] = 0x10; __Dot_Matrix[1] = 0x10; __Dot_Matrix[2] = 0x16; __Dot_Matrix[3] = 0x19; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x1e; return;
        case 'c': __Dot_Matrix[2] = 0x0e; __Dot_Matrix[3] = 0x10; __Dot_Matrix[4] = 0x10; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x0e; return;
        case 'd': __Dot_Matrix[0] = 0x01; __Dot_Matrix[1] = 0x01; __Dot_Matrix[2] = 0x0d; __Dot_Matrix[3] = 0x13; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x0f; return;
        case 'e': __Dot_Matrix[2] = 0x0e; __Dot_Matrix[3] = 0x11; __Dot_Matrix[4] = 0x1f; __Dot_Matrix[5] = 0x10; __Dot_Matrix[6] = 0x0e; return;
        case 'f': __Dot_Matrix[0] = 0x06; __Dot_Matrix[1] = 0x08; __Dot_Matrix[2] = 0x1c; __Dot_Matrix[3] = 0x08; __Dot_Matrix[4] = 0x08; __Dot_Matrix[5] = 0x08; __Dot_Matrix[6] = 0x08; return;
        case 'g': __Dot_Matrix[1] = 0x0f; __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x11; __Dot_Matrix[4] = 0x0f; __Dot_Matrix[5] = 0x01; __Dot_Matrix[6] = 0x0e; return;
        case 'h': __Dot_Matrix[0] = 0x10; __Dot_Matrix[1] = 0x10; __Dot_Matrix[2] = 0x16; __Dot_Matrix[3] = 0x19; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x11; return;
        case 'i': __Dot_Matrix[0] = 0x04; __Dot_Matrix[2] = 0x0c; __Dot_Matrix[3] = 0x04; __Dot_Matrix[4] = 0x04; __Dot_Matrix[5] = 0x04; __Dot_Matrix[6] = 0x0e; return;
        case 'j': __Dot_Matrix[0] = 0x02; __Dot_Matrix[2] = 0x06; __Dot_Matrix[3] = 0x02; __Dot_Matrix[4] = 0x02; __Dot_Matrix[5] = 0x12; __Dot_Matrix[6] = 0x0c; return;
        case 'k': __Dot_Matrix[0] = 0x10; __Dot_Matrix[1] = 0x10; __Dot_Matrix[2] = 0x12; __Dot_Matrix[3] = 0x14; __Dot_Matrix[4] = 0x18; __Dot_Matrix[5] = 0x14; __Dot_Matrix[6] = 0x12; return;
        case 'l': __Dot_Matrix[0] = 0x0c; __Dot_Matrix[1] = 0x04; __Dot_Matrix[2] = 0x04; __Dot_Matrix[3] = 0x04; __Dot_Matrix[4] = 0x04; __Dot_Matrix[5] = 0x04; __Dot_Matrix[6] = 0x0e; return;
        case 'm': __Dot_Matrix[2] = 0x1a; __Dot_Matrix[3] = 0x15; __Dot_Matrix[4] = 0x15; __Dot_Matrix[5] = 0x15; __Dot_Matrix[6] = 0x15; return;
        case 'n': __Dot_Matrix[2] = 0x16; __Dot_Matrix[3] = 0x19; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x11; return;
        case 'o': __Dot_Matrix[2] = 0x0e; __Dot_Matrix[3] = 0x11; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x0e; return;
        case 'p': __Dot_Matrix[2] = 0x1e; __Dot_Matrix[3] = 0x11; __Dot_Matrix[4] = 0x1e; __Dot_Matrix[5] = 0x10; __Dot_Matrix[6] = 0x10; return;
        case 'q': __Dot_Matrix[2] = 0x0d; __Dot_Matrix[3] = 0x13; __Dot_Matrix[4] = 0x0f; __Dot_Matrix[5] = 0x01; __Dot_Matrix[6] = 0x01; return;
        case 'r': __Dot_Matrix[2] = 0x16; __Dot_Matrix[3] = 0x19; __Dot_Matrix[4] = 0x10; __Dot_Matrix[5] = 0x10; __Dot_Matrix[6] = 0x10; return;
        case 's': __Dot_Matrix[2] = 0x0f; __Dot_Matrix[3] = 0x10; __Dot_Matrix[4] = 0x0e; __Dot_Matrix[5] = 0x01; __Dot_Matrix[6] = 0x1e; return;
        case 't': __Dot_Matrix[0] = 0x08; __Dot_Matrix[1] = 0x08; __Dot_Matrix[2] = 0x1c; __Dot_Matrix[3] = 0x08; __Dot_Matrix[4] = 0x08; __Dot_Matrix[5] = 0x08; __Dot_Matrix[6] = 0x06; return;
        case 'u': __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x11; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x13; __Dot_Matrix[6] = 0x0d; return;
        case 'v': __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x11; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x0a; __Dot_Matrix[6] = 0x04; return;
        case 'w': __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x11; __Dot_Matrix[4] = 0x15; __Dot_Matrix[5] = 0x15; __Dot_Matrix[6] = 0x0a; return;
        case 'x': __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x0a; __Dot_Matrix[4] = 0x04; __Dot_Matrix[5] = 0x0a; __Dot_Matrix[6] = 0x11; return;
        case 'y': __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x11; __Dot_Matrix[4] = 0x0f; __Dot_Matrix[5] = 0x01; __Dot_Matrix[6] = 0x0e; return;
        case 'z': __Dot_Matrix[2] = 0x1f; __Dot_Matrix[3] = 0x02; __Dot_Matrix[4] = 0x04; __Dot_Matrix[5] = 0x08; __Dot_Matrix[6] = 0x1f; return;
        default: break;
        }
    }

    switch (tmp_char)
    {
    case ' ': return;
    case '!': __Dot_Matrix[0] = 0x04; __Dot_Matrix[1] = 0x04; __Dot_Matrix[2] = 0x04; __Dot_Matrix[3] = 0x04; __Dot_Matrix[4] = 0x04; __Dot_Matrix[6] = 0x04; break;
    case '"': __Dot_Matrix[0] = 0x0a; __Dot_Matrix[1] = 0x0a; __Dot_Matrix[2] = 0x0a; break;
    case '#': __Dot_Matrix[0] = 0x0a; __Dot_Matrix[1] = 0x1f; __Dot_Matrix[2] = 0x0a; __Dot_Matrix[3] = 0x0a; __Dot_Matrix[4] = 0x1f; __Dot_Matrix[5] = 0x0a; break;
    case '$': __Dot_Matrix[0] = 0x04; __Dot_Matrix[1] = 0x0f; __Dot_Matrix[2] = 0x14; __Dot_Matrix[3] = 0x0e; __Dot_Matrix[4] = 0x05; __Dot_Matrix[5] = 0x1e; __Dot_Matrix[6] = 0x04; break;
    case '%': __Dot_Matrix[0] = 0x19; __Dot_Matrix[1] = 0x19; __Dot_Matrix[2] = 0x02; __Dot_Matrix[3] = 0x04; __Dot_Matrix[4] = 0x08; __Dot_Matrix[5] = 0x13; __Dot_Matrix[6] = 0x13; break;
    case '&': __Dot_Matrix[0] = 0x0c; __Dot_Matrix[1] = 0x12; __Dot_Matrix[2] = 0x14; __Dot_Matrix[3] = 0x08; __Dot_Matrix[4] = 0x15; __Dot_Matrix[5] = 0x12; __Dot_Matrix[6] = 0x0d; break;
    case '\'': __Dot_Matrix[0] = 0x04; __Dot_Matrix[1] = 0x04; __Dot_Matrix[2] = 0x08; break;
    case '(': __Dot_Matrix[0] = 0x02; __Dot_Matrix[1] = 0x04; __Dot_Matrix[2] = 0x08; __Dot_Matrix[3] = 0x08; __Dot_Matrix[4] = 0x08; __Dot_Matrix[5] = 0x04; __Dot_Matrix[6] = 0x02; break;
    case ')': __Dot_Matrix[0] = 0x08; __Dot_Matrix[1] = 0x04; __Dot_Matrix[2] = 0x02; __Dot_Matrix[3] = 0x02; __Dot_Matrix[4] = 0x02; __Dot_Matrix[5] = 0x04; __Dot_Matrix[6] = 0x08; break;
    case '*': __Dot_Matrix[1] = 0x04; __Dot_Matrix[2] = 0x15; __Dot_Matrix[3] = 0x0e; __Dot_Matrix[4] = 0x15; __Dot_Matrix[5] = 0x04; break;
    case '+': __Dot_Matrix[1] = 0x04; __Dot_Matrix[2] = 0x04; __Dot_Matrix[3] = 0x1f; __Dot_Matrix[4] = 0x04; __Dot_Matrix[5] = 0x04; break;
    case ',': __Dot_Matrix[4] = 0x04; __Dot_Matrix[5] = 0x04; __Dot_Matrix[6] = 0x08; break;
    case '-': __Dot_Matrix[3] = 0x1f; break;
    case '.': __Dot_Matrix[5] = 0x0c; __Dot_Matrix[6] = 0x0c; break;
    case '/': __Dot_Matrix[0] = 0x01; __Dot_Matrix[1] = 0x02; __Dot_Matrix[2] = 0x04; __Dot_Matrix[3] = 0x08; __Dot_Matrix[4] = 0x10; break;
    case '0': __Dot_Matrix[0] = 0x0e; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x13; __Dot_Matrix[3] = 0x15; __Dot_Matrix[4] = 0x19; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x0e; break;
    case '1': __Dot_Matrix[0] = 0x04; __Dot_Matrix[1] = 0x0c; __Dot_Matrix[2] = 0x04; __Dot_Matrix[3] = 0x04; __Dot_Matrix[4] = 0x04; __Dot_Matrix[5] = 0x04; __Dot_Matrix[6] = 0x0e; break;
    case '2': __Dot_Matrix[0] = 0x0e; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x01; __Dot_Matrix[3] = 0x02; __Dot_Matrix[4] = 0x04; __Dot_Matrix[5] = 0x08; __Dot_Matrix[6] = 0x1f; break;
    case '3': __Dot_Matrix[0] = 0x1e; __Dot_Matrix[1] = 0x01; __Dot_Matrix[2] = 0x01; __Dot_Matrix[3] = 0x0e; __Dot_Matrix[4] = 0x01; __Dot_Matrix[5] = 0x01; __Dot_Matrix[6] = 0x1e; break;
    case '4': __Dot_Matrix[0] = 0x02; __Dot_Matrix[1] = 0x06; __Dot_Matrix[2] = 0x0a; __Dot_Matrix[3] = 0x12; __Dot_Matrix[4] = 0x1f; __Dot_Matrix[5] = 0x02; __Dot_Matrix[6] = 0x02; break;
    case '5': __Dot_Matrix[0] = 0x1f; __Dot_Matrix[1] = 0x10; __Dot_Matrix[2] = 0x10; __Dot_Matrix[3] = 0x1e; __Dot_Matrix[4] = 0x01; __Dot_Matrix[5] = 0x01; __Dot_Matrix[6] = 0x1e; break;
    case '6': __Dot_Matrix[0] = 0x06; __Dot_Matrix[1] = 0x08; __Dot_Matrix[2] = 0x10; __Dot_Matrix[3] = 0x1e; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x0e; break;
    case '7': __Dot_Matrix[0] = 0x1f; __Dot_Matrix[1] = 0x01; __Dot_Matrix[2] = 0x02; __Dot_Matrix[3] = 0x04; __Dot_Matrix[4] = 0x08; __Dot_Matrix[5] = 0x08; __Dot_Matrix[6] = 0x08; break;
    case '8': __Dot_Matrix[0] = 0x0e; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x0e; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x0e; break;
    case '9': __Dot_Matrix[0] = 0x0e; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x0f; __Dot_Matrix[4] = 0x01; __Dot_Matrix[5] = 0x02; __Dot_Matrix[6] = 0x0c; break;
    case ':': __Dot_Matrix[1] = 0x0c; __Dot_Matrix[2] = 0x0c; __Dot_Matrix[4] = 0x0c; __Dot_Matrix[5] = 0x0c; break;
    case ';': __Dot_Matrix[1] = 0x0c; __Dot_Matrix[2] = 0x0c; __Dot_Matrix[4] = 0x0c; __Dot_Matrix[5] = 0x0c; __Dot_Matrix[6] = 0x08; break;
    case '<': __Dot_Matrix[0] = 0x02; __Dot_Matrix[1] = 0x04; __Dot_Matrix[2] = 0x08; __Dot_Matrix[3] = 0x10; __Dot_Matrix[4] = 0x08; __Dot_Matrix[5] = 0x04; __Dot_Matrix[6] = 0x02; break;
    case '=': __Dot_Matrix[2] = 0x1f; __Dot_Matrix[4] = 0x1f; break;
    case '>': __Dot_Matrix[0] = 0x08; __Dot_Matrix[1] = 0x04; __Dot_Matrix[2] = 0x02; __Dot_Matrix[3] = 0x01; __Dot_Matrix[4] = 0x02; __Dot_Matrix[5] = 0x04; __Dot_Matrix[6] = 0x08; break;
    case '?': __Dot_Matrix[0] = 0x0e; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x01; __Dot_Matrix[3] = 0x02; __Dot_Matrix[4] = 0x04; __Dot_Matrix[6] = 0x04; break;
    case '@': __Dot_Matrix[0] = 0x0e; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x17; __Dot_Matrix[3] = 0x15; __Dot_Matrix[4] = 0x17; __Dot_Matrix[5] = 0x10; __Dot_Matrix[6] = 0x0e; break;
    case 'A': __Dot_Matrix[0] = 0x0e; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x1f; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x11; break;
    case 'B': __Dot_Matrix[0] = 0x1e; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x1e; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x1e; break;
    case 'C': __Dot_Matrix[0] = 0x0e; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x10; __Dot_Matrix[3] = 0x10; __Dot_Matrix[4] = 0x10; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x0e; break;
    case 'D': __Dot_Matrix[0] = 0x1c; __Dot_Matrix[1] = 0x12; __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x11; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x12; __Dot_Matrix[6] = 0x1c; break;
    case 'E': __Dot_Matrix[0] = 0x1f; __Dot_Matrix[1] = 0x10; __Dot_Matrix[2] = 0x10; __Dot_Matrix[3] = 0x1e; __Dot_Matrix[4] = 0x10; __Dot_Matrix[5] = 0x10; __Dot_Matrix[6] = 0x1f; break;
    case 'F': __Dot_Matrix[0] = 0x1f; __Dot_Matrix[1] = 0x10; __Dot_Matrix[2] = 0x10; __Dot_Matrix[3] = 0x1e; __Dot_Matrix[4] = 0x10; __Dot_Matrix[5] = 0x10; __Dot_Matrix[6] = 0x10; break;
    case 'G': __Dot_Matrix[0] = 0x0e; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x10; __Dot_Matrix[3] = 0x17; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x0f; break;
    case 'H': __Dot_Matrix[0] = 0x11; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x1f; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x11; break;
    case 'I': __Dot_Matrix[0] = 0x0e; __Dot_Matrix[1] = 0x04; __Dot_Matrix[2] = 0x04; __Dot_Matrix[3] = 0x04; __Dot_Matrix[4] = 0x04; __Dot_Matrix[5] = 0x04; __Dot_Matrix[6] = 0x0e; break;
    case 'J': __Dot_Matrix[0] = 0x01; __Dot_Matrix[1] = 0x01; __Dot_Matrix[2] = 0x01; __Dot_Matrix[3] = 0x01; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x0e; break;
    case 'K': __Dot_Matrix[0] = 0x11; __Dot_Matrix[1] = 0x12; __Dot_Matrix[2] = 0x14; __Dot_Matrix[3] = 0x18; __Dot_Matrix[4] = 0x14; __Dot_Matrix[5] = 0x12; __Dot_Matrix[6] = 0x11; break;
    case 'L': __Dot_Matrix[0] = 0x10; __Dot_Matrix[1] = 0x10; __Dot_Matrix[2] = 0x10; __Dot_Matrix[3] = 0x10; __Dot_Matrix[4] = 0x10; __Dot_Matrix[5] = 0x10; __Dot_Matrix[6] = 0x1f; break;
    case 'M': __Dot_Matrix[0] = 0x11; __Dot_Matrix[1] = 0x1b; __Dot_Matrix[2] = 0x15; __Dot_Matrix[3] = 0x15; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x11; break;
    case 'N': __Dot_Matrix[0] = 0x11; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x19; __Dot_Matrix[3] = 0x15; __Dot_Matrix[4] = 0x13; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x11; break;
    case 'O': __Dot_Matrix[0] = 0x0e; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x11; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x0e; break;
    case 'P': __Dot_Matrix[0] = 0x1e; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x1e; __Dot_Matrix[4] = 0x10; __Dot_Matrix[5] = 0x10; __Dot_Matrix[6] = 0x10; break;
    case 'Q': __Dot_Matrix[0] = 0x0e; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x11; __Dot_Matrix[4] = 0x15; __Dot_Matrix[5] = 0x12; __Dot_Matrix[6] = 0x0d; break;
    case 'R': __Dot_Matrix[0] = 0x1e; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x1e; __Dot_Matrix[4] = 0x14; __Dot_Matrix[5] = 0x12; __Dot_Matrix[6] = 0x11; break;
    case 'S': __Dot_Matrix[0] = 0x0f; __Dot_Matrix[1] = 0x10; __Dot_Matrix[2] = 0x10; __Dot_Matrix[3] = 0x0e; __Dot_Matrix[4] = 0x01; __Dot_Matrix[5] = 0x01; __Dot_Matrix[6] = 0x1e; break;
    case 'T': __Dot_Matrix[0] = 0x1f; __Dot_Matrix[1] = 0x04; __Dot_Matrix[2] = 0x04; __Dot_Matrix[3] = 0x04; __Dot_Matrix[4] = 0x04; __Dot_Matrix[5] = 0x04; __Dot_Matrix[6] = 0x04; break;
    case 'U': __Dot_Matrix[0] = 0x11; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x11; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x0e; break;
    case 'V': __Dot_Matrix[0] = 0x11; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x11; __Dot_Matrix[4] = 0x11; __Dot_Matrix[5] = 0x0a; __Dot_Matrix[6] = 0x04; break;
    case 'W': __Dot_Matrix[0] = 0x11; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x11; __Dot_Matrix[3] = 0x15; __Dot_Matrix[4] = 0x15; __Dot_Matrix[5] = 0x15; __Dot_Matrix[6] = 0x0a; break;
    case 'X': __Dot_Matrix[0] = 0x11; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x0a; __Dot_Matrix[3] = 0x04; __Dot_Matrix[4] = 0x0a; __Dot_Matrix[5] = 0x11; __Dot_Matrix[6] = 0x11; break;
    case 'Y': __Dot_Matrix[0] = 0x11; __Dot_Matrix[1] = 0x11; __Dot_Matrix[2] = 0x0a; __Dot_Matrix[3] = 0x04; __Dot_Matrix[4] = 0x04; __Dot_Matrix[5] = 0x04; __Dot_Matrix[6] = 0x04; break;
    case 'Z': __Dot_Matrix[0] = 0x1f; __Dot_Matrix[1] = 0x01; __Dot_Matrix[2] = 0x02; __Dot_Matrix[3] = 0x04; __Dot_Matrix[4] = 0x08; __Dot_Matrix[5] = 0x10; __Dot_Matrix[6] = 0x1f; break;
    case '[': __Dot_Matrix[0] = 0x0e; __Dot_Matrix[1] = 0x08; __Dot_Matrix[2] = 0x08; __Dot_Matrix[3] = 0x08; __Dot_Matrix[4] = 0x08; __Dot_Matrix[5] = 0x08; __Dot_Matrix[6] = 0x0e; break;
    case '\\': __Dot_Matrix[0] = 0x10; __Dot_Matrix[1] = 0x08; __Dot_Matrix[2] = 0x04; __Dot_Matrix[3] = 0x02; __Dot_Matrix[4] = 0x01; break;
    case ']': __Dot_Matrix[0] = 0x0e; __Dot_Matrix[1] = 0x02; __Dot_Matrix[2] = 0x02; __Dot_Matrix[3] = 0x02; __Dot_Matrix[4] = 0x02; __Dot_Matrix[5] = 0x02; __Dot_Matrix[6] = 0x0e; break;
    case '^': __Dot_Matrix[0] = 0x04; __Dot_Matrix[1] = 0x0a; __Dot_Matrix[2] = 0x11; break;
    case '_': __Dot_Matrix[6] = 0x1f; break;
    case '`': __Dot_Matrix[0] = 0x08; __Dot_Matrix[1] = 0x04; __Dot_Matrix[2] = 0x02; break;
    case '{': __Dot_Matrix[0] = 0x02; __Dot_Matrix[1] = 0x04; __Dot_Matrix[2] = 0x04; __Dot_Matrix[3] = 0x08; __Dot_Matrix[4] = 0x04; __Dot_Matrix[5] = 0x04; __Dot_Matrix[6] = 0x02; break;
    case '|': __Dot_Matrix[0] = 0x04; __Dot_Matrix[1] = 0x04; __Dot_Matrix[2] = 0x04; __Dot_Matrix[3] = 0x04; __Dot_Matrix[4] = 0x04; __Dot_Matrix[5] = 0x04; __Dot_Matrix[6] = 0x04; break;
    case '}': __Dot_Matrix[0] = 0x08; __Dot_Matrix[1] = 0x04; __Dot_Matrix[2] = 0x04; __Dot_Matrix[3] = 0x02; __Dot_Matrix[4] = 0x04; __Dot_Matrix[5] = 0x04; __Dot_Matrix[6] = 0x08; break;
    case '~': __Dot_Matrix[2] = 0x09; __Dot_Matrix[3] = 0x16; break;
    default: Get_ASCII_5x7_Dot_Matrix('?', __Dot_Matrix); break;
    }
}

/**
 * @brief 清除整个ST7789控制器GRAM
 *
 * @param __Color 填充颜色
 */
void Class_LCD::Clear_Controller_GRAM(const uint16_t &__Color)
{
    const uint16_t tmp_x_offset = LCD_Config.X_Offset;
    const uint16_t tmp_y_offset = LCD_Config.Y_Offset;

    LCD_Config.X_Offset = 0;
    LCD_Config.Y_Offset = 0;
    Set_Address_Window(0, 0, 239, 319);
    Write_Color_Burst(__Color, 240u * 320u);
    LCD_Config.X_Offset = tmp_x_offset;
    LCD_Config.Y_Offset = tmp_y_offset;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
