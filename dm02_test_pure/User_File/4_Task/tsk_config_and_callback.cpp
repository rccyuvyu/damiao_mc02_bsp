/**
 * @file tsk_config_and_callback.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-01-17 1.1 调试到机器人层
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"

#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "2_Device/BSP/BMI088/bsp_bmi088.h"
#include "2_Device/Plotter/Vofa/dvc_vofa.h"
#include "2_Device/BSP/W25Q64JV/bsp_w25q64jv.h"
#include "2_Device/BSP/WS2812/bsp_ws2812.h"
#include "2_Device/BSP/Buzzer/bsp_buzzer.h"
#include "2_Device/BSP/Power/bsp_power.h"
#include "2_Device/BSP/Key/bsp_key.h"
#include "2_Device/BSP/LCD/bsp_lcd.h"
#include "1_Middleware/Algorithm/Filter/Kalman/alg_filter_kalman.h"
#include "1_Middleware/Algorithm/Matrix/alg_matrix.h"
#include "1_Middleware/Driver/WDG/drv_wdg.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 全局初始化完成标志位
bool init_finished = false;

/* Private function declarations ---------------------------------------------*/

extern SPI_HandleTypeDef hspi1;

static void LCD_Demo_Init()
{
    Struct_BSP_LCD_Config lcd_config;

    // 当前CubeMX未设置用户标签, 这里直接按实际引脚绑定:
    // CS = PE15, BL = PB10, RST = PB11, DC = PD10, SPI = SPI1
    lcd_config.SPI_Handler = &hspi1;
    lcd_config.CS_GPIOx = GPIOE;
    lcd_config.CS_GPIO_Pin = GPIO_PIN_15;
    lcd_config.DC_GPIOx = GPIOD;
    lcd_config.DC_GPIO_Pin = GPIO_PIN_10;
    lcd_config.RST_GPIOx = GPIOB;
    lcd_config.RST_GPIO_Pin = GPIO_PIN_11;
    lcd_config.BL_GPIOx = GPIOB;
    lcd_config.BL_GPIO_Pin = GPIO_PIN_10;
    lcd_config.Width = 240;
    lcd_config.Height = 240;
    lcd_config.X_Offset = 0;
    lcd_config.Y_Offset = 80;
    lcd_config.Rotation = BSP_LCD_Rotation_0;

    BSP_LCD.Init(lcd_config);
}

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 每3600s调用一次
 *
 */
void Task3600s_Callback()
{
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
}

/**
 * @brief 每1s调用一次
 *
 */
void Task1s_Callback()
{   

    static uint32_t count = 0;
    char lcd_buf[32];
    count++;
    snprintf(lcd_buf, sizeof(lcd_buf), "Count: %lu", count);
    BSP_LCD.Draw_String(8, 20, lcd_buf, BSP_LCD_COLOR_GREEN, BSP_LCD_COLOR_BLACK, 2);
}

/**
 * @brief 每1ms调用一次
 *
 */
void Task1ms_Callback()
{
    TIM_1ms_IWDG_PeriodElapsedCallback();
}

/**
 * @brief 每125us调用一次
 *
 */
void Task125us_Callback()
{

}

/**
 * @brief 每10us调用一次
 *
 */
void Task10us_Callback()
{

}

/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
    SYS_Timestamp.Init(&htim5);

    LCD_Demo_Init();

    // 定时器中断初始化
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim8);

    // 标记初始化完成
    init_finished = true;
}

/**
 * @brief 前台循环任务
 *
 */
void Task_Loop()
{

}

/**
 * @brief GPIO中断回调函数
 *
 * @param GPIO_Pin 中断引脚
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (!init_finished)
    {
        return;
    }
}

/**
 * @brief 定时器中断回调函数
 *
 * @param htim
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (!init_finished)
    {
        return;
    }

    // 选择回调函数
    if (htim->Instance == TIM4)
    {
        Task10us_Callback();
    }
    else if (htim->Instance == TIM5)
    {
        Task3600s_Callback();
    }
    else if (htim->Instance == TIM6)
    {
        Task1s_Callback();
    }
    else if (htim->Instance == TIM7)
    {
        Task1ms_Callback();
    }
    else if (htim->Instance == TIM8)
    {
        Task125us_Callback();
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
