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
#include "2_Device/BSP/LCD/bsp_lcd_key.h"
#include "1_Middleware/Algorithm/Filter/Kalman/alg_filter_kalman.h"
#include "1_Middleware/Algorithm/Matrix/alg_matrix.h"
#include "1_Middleware/Driver/WDG/drv_wdg.h"
#include "1_Middleware/Driver/ADC/drv_adc.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "app_config.h"
#include <cstdio>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 全局初始化完成标志位
bool init_finished = false;

// Chassis motors are global so other control modules can access their state.
Class_Motor_DJI_C610 motor_left_front;
Class_Motor_DJI_C610 motor_right_front;
Class_Motor_DJI_C610 motor_left_rear;
Class_Motor_DJI_C610 motor_right_rear;

// Four line sensors, ordered from right to left (index 0 -> 3).
uint16_t Line_Sensor_Pin[App_Config::LINE_SENSOR_COUNT] = {
    LINE_SENSOR_0_Pin, LINE_SENSOR_1_Pin, LINE_SENSOR_2_Pin,
    LINE_SENSOR_3_Pin};
GPIO_TypeDef *Line_Sensor_Port[App_Config::LINE_SENSOR_COUNT] = {
    LINE_SENSOR_0_GPIO_Port, LINE_SENSOR_1_GPIO_Port, LINE_SENSOR_2_GPIO_Port,
    LINE_SENSOR_3_GPIO_Port};
GPIO_PinState Line_Sensor_Data[App_Config::LINE_SENSOR_COUNT] = {
    GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET};

namespace
{
// 2006 + C610, IDs are the default DJI IDs. Change only these constants if
// the two motor controllers are configured differently.
enum class LineRunState : uint8_t { Idle, LeavingStart, Running, Finished };
LineRunState line_state = LineRunState::Idle;
float line_last_error = 0.0f;
float line_previous_error = 0.0f;
uint32_t line_clear_count = 0;
uint64_t line_start_time = 0;
uint64_t line_finish_time = 0;
uint8_t line_marker_stable = 0;
uint32_t line_display_tick = 0;

uint8_t menu_selected = 0;
uint8_t menu_active_test = 0;
bool menu_running = false;
volatile bool menu_dirty = true;
uint64_t menu_start_time = 0;

bool LineSensorBlack(uint8_t index)
{
    return Line_Sensor_Data[index] == App_Config::LINE_BLACK_STATE;
}

void LineSensorRead()
{
    for (uint8_t i = 0; i < App_Config::LINE_SENSOR_COUNT; ++i)
    {
        Line_Sensor_Data[i] = HAL_GPIO_ReadPin(Line_Sensor_Port[i], Line_Sensor_Pin[i]);
    }
}

uint8_t LineBlackCount()
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < App_Config::LINE_SENSOR_COUNT; ++i)
    {
        count += LineSensorBlack(i) ? 1u : 0u;
    }
    return count;
}

float LineError()
{
    float sum = 0.0f;
    uint8_t count = 0;
    for (uint8_t i = 0; i < App_Config::LINE_SENSOR_COUNT; ++i)
    {
        if (LineSensorBlack(i))
        {
            sum += App_Config::LINE_WEIGHT[i];
            ++count;
        }
    }
    if (count != 0u)
    {
        line_last_error = sum / static_cast<float>(count);
    }
    return line_last_error;
}

void LineSetSpeed(float left, float right)
{
    motor_left_front.Set_Target_Omega(App_Config::MOTOR_LEFT_FRONT_SIGN * left);
    motor_left_rear.Set_Target_Omega(App_Config::MOTOR_LEFT_REAR_SIGN * left);
    motor_right_front.Set_Target_Omega(App_Config::MOTOR_RIGHT_FRONT_SIGN * right);
    motor_right_rear.Set_Target_Omega(App_Config::MOTOR_RIGHT_REAR_SIGN * right);
}

void LineStop()
{
    LineSetSpeed(0.0f, 0.0f);
}

void LineSensorInit()
{
    GPIO_InitTypeDef config = {};
    config.Mode = GPIO_MODE_INPUT;
    config.Pull = GPIO_NOPULL;
    config.Speed = GPIO_SPEED_FREQ_LOW;
    for (uint8_t i = 0; i < App_Config::LINE_SENSOR_COUNT; ++i)
    {
        config.Pin = Line_Sensor_Pin[i];
        HAL_GPIO_Init(Line_Sensor_Port[i], &config);
    }
}

void Motor_CAN_Callback(FDCAN_RxHeaderTypeDef &header, uint8_t *)
{
    if (header.Identifier == App_Config::MOTOR_LEFT_FRONT_CAN_ID)
    {
        motor_left_front.CAN_RxCpltCallback();
    }
    else if (header.Identifier == App_Config::MOTOR_RIGHT_FRONT_CAN_ID)
    {
        motor_right_front.CAN_RxCpltCallback();
    }
    else if (header.Identifier == App_Config::MOTOR_LEFT_REAR_CAN_ID)
    {
        motor_left_rear.CAN_RxCpltCallback();
    }
    else if (header.Identifier == App_Config::MOTOR_RIGHT_REAR_CAN_ID)
    {
        motor_right_rear.CAN_RxCpltCallback();
    }
}

void LineFollowerStart()
{
    line_state = LineRunState::LeavingStart;
    line_start_time = SYS_Timestamp.Get_Current_Timestamp();
    line_finish_time = 0;
    line_clear_count = 0;
    line_marker_stable = 0;
    line_last_error = 0.0f;
    line_previous_error = 0.0f;
}

void LineFollowerProcess1ms()
{
    LineSensorRead();

    if (line_state == LineRunState::Idle || line_state == LineRunState::Finished)
    {
        LineStop();
        return;
    }

    const uint8_t black_count = LineBlackCount();
    // A perpendicular start or finish line covers at least three sensors.
    const bool marker = black_count >= App_Config::LINE_MARKER_MIN_BLACK_COUNT;
    const float error = LineError();
    const float correction = App_Config::LINE_KP * error + App_Config::LINE_KD * (error - line_previous_error);
    line_previous_error = error;

    // Slow down on a wide marker and during a sharp correction.
    float base_speed = App_Config::LINE_BASE_SPEED;
    if (marker || error > App_Config::LINE_SHARP_ERROR || error < -App_Config::LINE_SHARP_ERROR)
    {
        base_speed *= App_Config::LINE_CORNER_SPEED_SCALE;
    }
    LineSetSpeed(base_speed + correction, base_speed - correction);

    if (line_state == LineRunState::LeavingStart)
    {
        if (!marker)
        {
            ++line_clear_count;
        }
        else
        {
            line_clear_count = 0;
        }
        if (line_clear_count >= App_Config::LINE_LEAVE_MARKER_MS)
        {
            line_state = LineRunState::Running;
        }
    }
    else if (line_state == LineRunState::Running)
    {
        if (marker && (SYS_Timestamp.Get_Current_Timestamp() - line_start_time) > App_Config::LINE_MIN_RUN_TIME_US)
        {
            if (++line_marker_stable >= App_Config::LINE_MARKER_STABLE_MS)
            {
                line_state = LineRunState::Finished;
                line_finish_time = SYS_Timestamp.Get_Current_Timestamp();
                LineStop();
            }
        }
        else
        {
            line_marker_stable = 0;
        }
    }
}

void MenuHandleKey1ms()
{
    if (BSP_LCD_Key.Get_Key_Status() != BSP_LCD_Key_Status_TRIG_FREE_PRESSED)
    {
        return;
    }

    const Enum_BSP_LCD_Key key = BSP_LCD_Key.Get_Key();
    if (!menu_running && key == BSP_LCD_Key_UP && menu_selected > 0u)
    {
        --menu_selected;
        menu_dirty = true;
    }
    else if (!menu_running && key == BSP_LCD_Key_DOWN && menu_selected < 4u)
    {
        ++menu_selected;
        menu_dirty = true;
    }
    else if (!menu_running && key == BSP_LCD_Key_CENTER)
    {
        menu_active_test = static_cast<uint8_t>(menu_selected + 2u);
        menu_start_time = SYS_Timestamp.Get_Current_Timestamp();
        menu_running = true;
        menu_dirty = true;

        if (menu_active_test == 2u)
        {
            LineFollowerStart();
        }
    }
    else if (menu_running && key == BSP_LCD_Key_LEFT)
    {
        menu_running = false;
        line_state = LineRunState::Idle;
        LineStop();
        menu_dirty = true;
    }
}

void LineFollowerDisplay()
{
    const uint32_t now = HAL_GetTick();
    static bool display_initialized = false;
    static bool display_was_running = false;
    static uint8_t drawn_menu_selected = 0;
    const bool first_display = !display_initialized;
    const bool mode_changed = display_was_running != menu_running;
    const bool redraw_menu = first_display || menu_dirty || mode_changed;
    if (!redraw_menu && now - line_display_tick < App_Config::LCD_REFRESH_PERIOD_MS)
    {
        return;
    }
    line_display_tick = now;
    menu_dirty = false;
    display_initialized = true;

    uint64_t elapsed = 0;
    if (menu_running && menu_active_test == 2u &&
        (line_state == LineRunState::LeavingStart || line_state == LineRunState::Running))
    {
        elapsed = SYS_Timestamp.Get_Current_Timestamp() - line_start_time;
    }
    else if (menu_running && menu_active_test == 2u && line_state == LineRunState::Finished)
    {
        elapsed = line_finish_time - line_start_time;
    }
    else if (menu_running)
    {
        elapsed = SYS_Timestamp.Get_Current_Timestamp() - menu_start_time;
    }

    char timer_text[32];
    const char *state = !menu_running ? "READY" :
                        (menu_active_test == 2u && line_state == LineRunState::Finished) ? "DONE" : "RUN";
    std::snprintf(timer_text, sizeof(timer_text), "T%d %-5s %02lu.%03lus", menu_running ? menu_active_test : 0,
                  state,
                  static_cast<unsigned long>(elapsed / 1000000ULL),
                  static_cast<unsigned long>((elapsed / 1000ULL) % 1000ULL));

    // Draw over the fixed-width text directly. Avoid clearing the whole
    // header first, which produces a visible black flash on the LCD.
    BSP_LCD.Draw_String(8, 4, timer_text, BSP_LCD_COLOR_GREEN, BSP_LCD_COLOR_BLACK, 2);

    const uint16_t line_sensor_adc = ADC1_Manage_Object.ADC_Data[App_Config::LINE_SENSOR_ADC_BUFFER_INDEX];
    const uint32_t line_sensor_mv =
        (static_cast<uint32_t>(line_sensor_adc) * 3300u + 2047u) / 4095u;
    char adc_text[32];
    std::snprintf(adc_text, sizeof(adc_text), "ADC %4u %lu.%03luV",
                  static_cast<unsigned>(line_sensor_adc),
                  static_cast<unsigned long>(line_sensor_mv / 1000u),
                  static_cast<unsigned long>(line_sensor_mv % 1000u));
    BSP_LCD.Draw_String(8, 218, adc_text, BSP_LCD_COLOR_YELLOW, BSP_LCD_COLOR_BLACK, 2);

    auto draw_menu_item = [](const uint8_t index)
    {
        const uint16_t y = static_cast<uint16_t>(42u + index * 35u);
        const bool selected = index == menu_selected;
        BSP_LCD.Fill_Rectangle(0, y - 2u, 240, 30,
                               selected ? BSP_LCD_COLOR_BLUE : BSP_LCD_COLOR_BLACK);
        char item_text[16];
        std::snprintf(item_text, sizeof(item_text), "Test %u", static_cast<unsigned>(index + 2u));
        BSP_LCD.Draw_String(14, y + 4u, item_text,
                            selected ? BSP_LCD_COLOR_WHITE : BSP_LCD_COLOR_GREEN,
                            selected ? BSP_LCD_COLOR_BLUE : BSP_LCD_COLOR_BLACK, 2);
    };

    if (!menu_running && redraw_menu)
    {
        if (first_display || mode_changed)
        {
            BSP_LCD.Fill_Rectangle(0, 38, 240, 202, BSP_LCD_COLOR_BLACK);
            for (uint8_t i = 0; i < 5u; ++i)
            {
                draw_menu_item(i);
            }
        }
        else if (drawn_menu_selected != menu_selected)
        {
            draw_menu_item(drawn_menu_selected);
            draw_menu_item(menu_selected);
        }
        drawn_menu_selected = menu_selected;
    }
    else if (menu_running && redraw_menu)
    {
        BSP_LCD.Fill_Rectangle(0, 38, 240, 202, BSP_LCD_COLOR_BLACK);
        char item_text[24];
        std::snprintf(item_text, sizeof(item_text), "Test %u running", static_cast<unsigned>(menu_active_test));
        BSP_LCD.Draw_String(14, 85, item_text, BSP_LCD_COLOR_YELLOW, BSP_LCD_COLOR_BLACK, 2);
        BSP_LCD.Draw_String(14, 125, "LEFT: menu", BSP_LCD_COLOR_GRAY, BSP_LCD_COLOR_BLACK, 2);
    }
    display_was_running = menu_running;
}
}

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
}

/**
 * @brief 每1ms调用一次
 *
 */
void Task1ms_Callback()
{
    TIM_1ms_IWDG_PeriodElapsedCallback();
    TIM_1ms_CAN_PeriodElapsedCallback();
    BSP_LCD_Key.TIM_1ms_Process_PeriodElapsedCallback();
    MenuHandleKey1ms();
    motor_left_front.TIM_Calculate_PeriodElapsedCallback();
    motor_right_front.TIM_Calculate_PeriodElapsedCallback();
    motor_left_rear.TIM_Calculate_PeriodElapsedCallback();
    motor_right_rear.TIM_Calculate_PeriodElapsedCallback();
    LineFollowerProcess1ms();
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

    // Enable the board-controlled 5 V rail; keep both 24 V rails disabled.
    BSP_Power.Init(false, false, true);
    LCD_Demo_Init();

    // TIM2 initially assigns PA2 to the Servo 2 PWM output. Restore PA2 to ADC mode after all peripheral initialisation.
    GPIO_InitTypeDef line_sensor_adc_config = {};
    line_sensor_adc_config.Pin = GPIO_PIN_2;
    line_sensor_adc_config.Mode = GPIO_MODE_ANALOG;
    line_sensor_adc_config.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &line_sensor_adc_config);

    ADC_Init(&hadc1, 2);
    BSP_LCD_Key.Init(&ADC1_Manage_Object, 0, 4095);
    LineSensorInit();

    // Chassis motor mapping: LF=0x203, RF=0x204, LR=0x202, RR=0x201.
    motor_left_front.Init(&hfdcan1, App_Config::MOTOR_LEFT_FRONT_ID, Motor_DJI_Control_Method_OMEGA, App_Config::MOTOR_GEARBOX_RATE);
    motor_right_front.Init(&hfdcan1, App_Config::MOTOR_RIGHT_FRONT_ID, Motor_DJI_Control_Method_OMEGA, App_Config::MOTOR_GEARBOX_RATE);
    motor_left_rear.Init(&hfdcan1, App_Config::MOTOR_LEFT_REAR_ID, Motor_DJI_Control_Method_OMEGA, App_Config::MOTOR_GEARBOX_RATE);
    motor_right_rear.Init(&hfdcan1, App_Config::MOTOR_RIGHT_REAR_ID, Motor_DJI_Control_Method_OMEGA, App_Config::MOTOR_GEARBOX_RATE);
    motor_left_front.PID_Omega.Init(App_Config::MOTOR_SPEED_KP, App_Config::MOTOR_SPEED_KI, App_Config::MOTOR_SPEED_KD, App_Config::MOTOR_SPEED_KF, App_Config::MOTOR_SPEED_I_OUT_MAX, App_Config::MOTOR_SPEED_OUT_MAX, App_Config::MOTOR_SPEED_D_T);
    motor_right_front.PID_Omega.Init(App_Config::MOTOR_SPEED_KP, App_Config::MOTOR_SPEED_KI, App_Config::MOTOR_SPEED_KD, App_Config::MOTOR_SPEED_KF, App_Config::MOTOR_SPEED_I_OUT_MAX, App_Config::MOTOR_SPEED_OUT_MAX, App_Config::MOTOR_SPEED_D_T);
    motor_left_rear.PID_Omega.Init(App_Config::MOTOR_SPEED_KP, App_Config::MOTOR_SPEED_KI, App_Config::MOTOR_SPEED_KD, App_Config::MOTOR_SPEED_KF, App_Config::MOTOR_SPEED_I_OUT_MAX, App_Config::MOTOR_SPEED_OUT_MAX, App_Config::MOTOR_SPEED_D_T);
    motor_right_rear.PID_Omega.Init(App_Config::MOTOR_SPEED_KP, App_Config::MOTOR_SPEED_KI, App_Config::MOTOR_SPEED_KD, App_Config::MOTOR_SPEED_KF, App_Config::MOTOR_SPEED_I_OUT_MAX, App_Config::MOTOR_SPEED_OUT_MAX, App_Config::MOTOR_SPEED_D_T);
    // All three FDCAN peripherals use Classic CAN frames. The chassis motors
    // are connected to CAN1; CAN2 and CAN3 remain available as normal CAN buses.
    CAN_Init(&hfdcan1, Motor_CAN_Callback);
    CAN_Init(&hfdcan2, nullptr);
    CAN_Init(&hfdcan3, nullptr);
    LineStop();

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
    LineFollowerDisplay();
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
