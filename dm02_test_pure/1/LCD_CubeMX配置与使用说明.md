# LCD CubeMX配置与使用说明

这套库是按 `1/LCD原理图.pdf` 和 `1/原理.pdf` 里的两层原理图一起整理的：

- `LCD原理图.pdf` 是屏子小板自身的连接
- `原理.pdf` 里的 `外接LCD` 是主控板真正接出去的接口

最终在这块 `dm02_test_pure` 主板上，建议按下面的功能理解引脚：

| LCD功能 | 主板信号 | 说明 |
| --- | --- | --- |
| `LCD_CS` | `PE15` | SPI片选 |
| `LCD_SCK` | `PB3` | SPI时钟 |
| `LCD_MOSI` | `PD7` | SPI发送数据 |
| `LCD_MISO` | `PB4` | 这块屏基本用不到, 可不接 |
| `LCD_BL` | `PB10` | 背光控制, 高电平点亮 |
| `LCD_RST` | `PB11` | 液晶复位 |
| `LCD_DC` | `PD10` | 命令/数据选择 |
| `LCD_KEY_ADC` | `PA5 / ADC1_INP18` | 屏上五向按键分压采样 |

## 1. CubeMX配置

### SPI1

建议这样配：

- Mode: `Full-Duplex Master` 或 `Transmit Only Master`
- Data Size: `8 Bits`
- First Bit: `MSB First`
- Clock Polarity: `Low`
- Clock Phase: `1 Edge`
- NSS: `Software`
- Baud Rate Prescaler: 先用 `16` 或 `32` 起步, 点亮后再提高
- CRC: `Disable`

推荐引脚：

- `PB3 -> SPI1_SCK`
- `PD7 -> SPI1_MOSI`
- `PE15 -> GPIO_Output` 作为 `LCD_CS`
- `PB4 -> SPI1_MISO` 可选, 不读屏时可以不配

说明：

- `ST7789V2` 4线串口在手册里说明 `SDA` 在 `SCL` 上升沿采样, 所以这里按常用 `SPI Mode 0` 配置。

### GPIO

把下面 3 个脚配置成普通推挽输出：

- `PB10 -> LCD_BL`
- `PB11 -> LCD_RST`
- `PD10 -> LCD_DC`

建议：

- Pull: `No pull`
- Speed: `Very High`
- 输出初值：
  - `LCD_BL` 先 `Low`
  - `LCD_RST` 先 `High`
  - `LCD_DC` 无所谓

`PE15` 的 `LCD_CS` 如果不走硬件 NSS，也同样配置成普通推挽输出。

### ADC1

如果你只想读屏上按键：

- `PA5 -> ADC1_INP18`
- Resolution: `12 Bits`
- Scan Conversion Mode: `Disable`
- Continuous Conversion Mode: `Enable`
- Conversion Data Management: `DMA Circular`
- Nbr Of Conversion: `1`

如果你还想保留当前工程里的电池电压采样 `PC4/ADC1_INP4`，建议这样配：

- 打开 `Scan Conversion Mode`
- `Rank 1 = ADC1_INP4 (PC4)` 给 `BSP_Power`
- `Rank 2 = ADC1_INP18 (PA5)` 给 `BSP_LCD_Key`
- `Nbr Of Conversion = 2`
- DMA 继续用 `Circular`

这样：

- `ADC1_Manage_Object.ADC_Data[0]` 对应电池电压
- `ADC1_Manage_Object.ADC_Data[1]` 对应LCD按键

### 时钟与DMA

- `SPI1` 如果后面想提速，可以顺手给 TX DMA
- 当前这版库内部使用阻塞式 `HAL_SPI_Transmit()`，所以 DMA 不是必须
- `ADC1` 建议继续用 DMA 循环采样，这样按键读取最省事

## 2. 按键电压表

`LCD原理图.pdf` 右下角已经给了 12bit ADC 理论值，这版库按它做了解码：

| 键位 | 12bit ADC典型值 |
| --- | --- |
| 中键 | `0` |
| 左 | `816` |
| 右 | `1635` |
| 下 | `2457` |
| 上 | `3279` |
| 松开 | `4095` |

库内部做了归一化处理，所以即使你把 ADC 配成 `16bit`，也仍然能正常解码。

## 3. 代码接法

新增文件：

- `User_File/2_Device/BSP/LCD/bsp_lcd.h`
- `User_File/2_Device/BSP/LCD/bsp_lcd.cpp`
- `User_File/2_Device/BSP/LCD/bsp_lcd_key.h`
- `User_File/2_Device/BSP/LCD/bsp_lcd_key.cpp`

这两个类都是“外部传配置”的写法，所以不会强依赖你现在还没配好的 `ioc`。

示例初始化代码可以放进 `Task_Init()`：

```cpp
#include "2_Device/BSP/LCD/bsp_lcd.h"
#include "2_Device/BSP/LCD/bsp_lcd_key.h"
#include "1_Middleware/Driver/ADC/drv_adc.h"

void Task_Init()
{
    Struct_BSP_LCD_Config lcd_config;

    ADC_Init(&hadc1, 2);

    lcd_config.SPI_Handler = &hspi1;
    lcd_config.CS_GPIOx = LCD_CS_GPIO_Port;
    lcd_config.CS_GPIO_Pin = LCD_CS_Pin;
    lcd_config.DC_GPIOx = LCD_DC_GPIO_Port;
    lcd_config.DC_GPIO_Pin = LCD_DC_Pin;
    lcd_config.RST_GPIOx = LCD_RST_GPIO_Port;
    lcd_config.RST_GPIO_Pin = LCD_RST_Pin;
    lcd_config.BL_GPIOx = LCD_BL_GPIO_Port;
    lcd_config.BL_GPIO_Pin = LCD_BL_Pin;
    lcd_config.Width = 240;
    lcd_config.Height = 240;
    lcd_config.Rotation = BSP_LCD_Rotation_0;

    BSP_LCD.Init(lcd_config);
    BSP_LCD_Key.Init(&ADC1_Manage_Object, 1, 4095);

    BSP_LCD.Clear(BSP_LCD_COLOR_BLACK);
    BSP_LCD.Fill_Rectangle(20, 20, 80, 40, BSP_LCD_COLOR_GREEN);
}
```

如果你不保留 `PC4` 电池电压采样，那么：

```cpp
ADC_Init(&hadc1, 1);
BSP_LCD_Key.Init(&ADC1_Manage_Object, 0, 4095);
```

## 4. 建议的任务调用方式

按你现在工程的风格，建议在 `1ms` 定时任务里轮询按键：

```cpp
void Task1ms_Callback()
{
    BSP_LCD_Key.TIM_1ms_Process_PeriodElapsedCallback();
}
```

前台循环里处理业务：

```cpp
void Task_Loop()
{
    if (BSP_LCD_Key.Get_Key_Status() == BSP_LCD_Key_Status_TRIG_FREE_PRESSED)
    {
        if (BSP_LCD_Key.Get_Key() == BSP_LCD_Key_UP)
        {
            BSP_LCD.Clear(BSP_LCD_COLOR_BLUE);
        }
    }
}
```

## 5. 现阶段注意点

- 这版默认按常见 `240x240 ST7789` 初始化
- 如果你后面接的不是 `240x240` 模组，而是 `135x240` 之类的变种，需要改 `Width/Height/X_Offset/Y_Offset`
- 现在库里先做的是稳定的底层显示和按键读取，没塞字库；如果你要，我下一步可以继续帮你补 `ASCII/数字/菜单UI` 那层
