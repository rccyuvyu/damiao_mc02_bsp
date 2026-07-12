/**
 * @file dvc_motor_jc.h
 * @author Codex
 * @brief JC2804/JC4010/JC4310 CAN驱动
 * @version 0.1
 * @date 2026-07-12
 *
 */

#ifndef DVC_MOTOR_JC_H
#define DVC_MOTOR_JC_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/Algorithm/Basic/alg_basic.h"
#include "1_Middleware/Driver/CAN/drv_can.h"

/* Exported types ------------------------------------------------------------*/

enum Enum_Motor_JC_Status
{
    Motor_JC_Status_DISABLE = 0,
    Motor_JC_Status_ENABLE,
};

enum Enum_Motor_JC_Mode : uint16_t
{
    Motor_JC_Mode_TORQUE = 0x0000,
    Motor_JC_Mode_SPEED = 0x0001,
    Motor_JC_Mode_POSITION_TRAPEZOID = 0x0002,
    Motor_JC_Mode_POSITION_SMOOTH = 0x0003,
    Motor_JC_Mode_POSITION_DIRECT = 0x0004,
    Motor_JC_Mode_LOW_SPEED_TORQUE = 0x0005,
};

struct Struct_Motor_JC_Feedback
{
    float Voltage_V = 0.0f;
    float Position_Deg = 0.0f;
    float Position_Rad = 0.0f;
    float Speed_Rpm = 0.0f;
    float Speed_Radps = 0.0f;
    float Torque_Nm = 0.0f;
    uint8_t Last_Raw_Data[8] = {0};
};

class Class_Motor_JC
{
public:
    void Init(FDCAN_HandleTypeDef *hcan, uint8_t __Device_ID = 1);

    inline Enum_Motor_JC_Status Get_Status() const;
    inline uint8_t Get_Device_ID() const;
    inline uint16_t Get_CAN_Tx_ID() const;
    inline uint16_t Get_CAN_Rx_ID() const;
    inline Enum_Motor_JC_Mode Get_Mode() const;
    inline const Struct_Motor_JC_Feedback &Get_Feedback() const;

    void Set_Mode(Enum_Motor_JC_Mode __Mode);
    void Enter_Close_Loop();
    void Enter_Idle();
    void Reboot();

    void Read_Voltage();
    void Read_Position();
    void Read_Speed();

    void Set_Absolute_Position_Deg(float __Position_Deg);
    void Set_Relative_Position_Deg(float __Position_Deg);
    void Set_Speed_Rpm(float __Speed_Rpm);
    void Set_Torque_Nm(float __Torque_Nm);
    void Set_PVT(float __Position_Deg, float __Speed_Rpm, float __Torque_Percent);

    void TIM_100ms_Alive_PeriodElapsedCallback();

protected:
    Struct_CAN_Manage_Object *CAN_Manage_Object = nullptr;
    uint8_t Device_ID = 1;
    uint16_t CAN_Tx_ID = 0x601;
    uint16_t CAN_Rx_ID = 0x581;

    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;

    Enum_Motor_JC_Status Motor_JC_Status = Motor_JC_Status_DISABLE;
    Enum_Motor_JC_Mode Motor_JC_Mode = Motor_JC_Mode_POSITION_TRAPEZOID;
    Struct_Motor_JC_Feedback Feedback;

    static void CAN_Rx_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer, void *User_Data);

    void Data_Process(const uint8_t *Rx_Data);
    void Write_SDO_U16(uint16_t Index, uint16_t Value) const;
    void Write_SDO_I16(uint16_t Index, int16_t Value) const;
    void Write_SDO_I32(uint16_t Index, int32_t Value) const;
    void Read_SDO_U16(uint16_t Index) const;
    void Read_SDO_I32(uint16_t Index) const;
    void Send_Frame(const uint8_t *Tx_Data) const;
};

inline Enum_Motor_JC_Status Class_Motor_JC::Get_Status() const
{
    return (Motor_JC_Status);
}

inline uint8_t Class_Motor_JC::Get_Device_ID() const
{
    return (Device_ID);
}

inline uint16_t Class_Motor_JC::Get_CAN_Tx_ID() const
{
    return (CAN_Tx_ID);
}

inline uint16_t Class_Motor_JC::Get_CAN_Rx_ID() const
{
    return (CAN_Rx_ID);
}

inline Enum_Motor_JC_Mode Class_Motor_JC::Get_Mode() const
{
    return (Motor_JC_Mode);
}

inline const Struct_Motor_JC_Feedback &Class_Motor_JC::Get_Feedback() const
{
    return (Feedback);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
