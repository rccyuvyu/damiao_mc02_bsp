/**
 * @file dvc_motor_jc.cpp
 * @author Codex
 * @brief JC2804/JC4010/JC4310 CAN驱动
 * @version 0.1
 * @date 2026-07-12
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_jc.h"

#include <string.h>

/* Private function declarations ---------------------------------------------*/

static inline uint16_t jc_get_u16_be(const uint8_t *data)
{
    return ((uint16_t) data[0] << 8) | (uint16_t) data[1];
}

static inline int32_t jc_get_i32_be(const uint8_t *data)
{
    return ((int32_t) data[0] << 24) | ((int32_t) data[1] << 16) | ((int32_t) data[2] << 8) | ((int32_t) data[3]);
}

static inline int32_t jc_get_i24_be(const uint8_t *data)
{
    int32_t value = ((int32_t) data[0] << 16) | ((int32_t) data[1] << 8) | ((int32_t) data[2]);

    if ((value & 0x00800000) != 0)
    {
        value |= 0xff000000;
    }

    return (value);
}

static inline void jc_set_u16_be(uint8_t *data, uint16_t value)
{
    data[0] = (value >> 8) & 0xff;
    data[1] = value & 0xff;
}

static inline void jc_set_i32_be(uint8_t *data, int32_t value)
{
    data[0] = (value >> 24) & 0xff;
    data[1] = (value >> 16) & 0xff;
    data[2] = (value >> 8) & 0xff;
    data[3] = value & 0xff;
}

/* Function prototypes -------------------------------------------------------*/

void Class_Motor_JC::Init(FDCAN_HandleTypeDef *hcan, uint8_t __Device_ID)
{
    Device_ID = __Device_ID;
    CAN_Tx_ID = 0x600 + Device_ID;
    CAN_Rx_ID = 0x580 + Device_ID;

    if (hcan->Instance == FDCAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    else if (hcan->Instance == FDCAN3)
    {
        CAN_Manage_Object = &CAN3_Manage_Object;
    }

    if (CAN_Manage_Object != nullptr)
    {
        CAN_Register_StdID_Callback(hcan, CAN_Rx_ID, CAN_Rx_Callback, this);
    }
}

void Class_Motor_JC::Set_Mode(Enum_Motor_JC_Mode __Mode)
{
    Motor_JC_Mode = __Mode;
    Write_SDO_U16(0x0060, static_cast<uint16_t>(__Mode));
}

void Class_Motor_JC::Enter_Close_Loop()
{
    Write_SDO_U16(0x00A2, 0x0001);
}

void Class_Motor_JC::Enter_Idle()
{
    Write_SDO_U16(0x00A0, 0x0001);
}

void Class_Motor_JC::Reboot()
{
    Write_SDO_U16(0x00A5, 0x0001);
}

void Class_Motor_JC::Read_Voltage()
{
    Read_SDO_U16(0x0004);
}

void Class_Motor_JC::Read_Position()
{
    Read_SDO_I32(0x0008);
}

void Class_Motor_JC::Read_Speed()
{
    Read_SDO_I32(0x0006);
}

void Class_Motor_JC::Set_Absolute_Position_Deg(float __Position_Deg)
{
    Write_SDO_I32(0x0023, (int32_t)(__Position_Deg * 100.0f));
}

void Class_Motor_JC::Set_Relative_Position_Deg(float __Position_Deg)
{
    Write_SDO_I32(0x0025, (int32_t)(__Position_Deg * 100.0f));
}

void Class_Motor_JC::Set_Speed_Rpm(float __Speed_Rpm)
{
    Write_SDO_I32(0x0021, (int32_t)(__Speed_Rpm * 100.0f));
}

void Class_Motor_JC::Set_Torque_Nm(float __Torque_Nm)
{
    Write_SDO_I16(0x0020, (int16_t)(__Torque_Nm * 100.0f));
}

void Class_Motor_JC::Set_PVT(float __Position_Deg, float __Speed_Rpm, float __Torque_Percent)
{
    uint8_t tx_data[8] = {0};
    int32_t position_cmd = (int32_t)(__Position_Deg * 100.0f);
    uint16_t speed_cmd = (uint16_t)(__Speed_Rpm);
    uint8_t torque_cmd;

    Basic_Math_Constrain(&__Torque_Percent, 0.0f, 100.0f);
    torque_cmd = (uint8_t)(__Torque_Percent);

    tx_data[0] = 0x25;
    jc_set_i32_be(&tx_data[1], position_cmd);
    jc_set_u16_be(&tx_data[5], speed_cmd);
    tx_data[7] = torque_cmd;

    Send_Frame(tx_data);
}

void Class_Motor_JC::TIM_100ms_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        Motor_JC_Status = Motor_JC_Status_DISABLE;
    }
    else
    {
        Motor_JC_Status = Motor_JC_Status_ENABLE;
    }

    Pre_Flag = Flag;
}

void Class_Motor_JC::CAN_Rx_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer, void *User_Data)
{
    Class_Motor_JC *motor = reinterpret_cast<Class_Motor_JC *>(User_Data);

    if (motor == nullptr || Header.Identifier != motor->CAN_Rx_ID)
    {
        return;
    }

    motor->Flag++;
    motor->Data_Process(Buffer);
}

void Class_Motor_JC::Data_Process(const uint8_t *Rx_Data)
{
    uint16_t index;

    memcpy(Feedback.Last_Raw_Data, Rx_Data, sizeof(Feedback.Last_Raw_Data));

    if (Rx_Data[0] == 0x2a)
    {
        Feedback.Position_Deg = (float) jc_get_i24_be(&Rx_Data[1]) / 100.0f;
        Feedback.Position_Rad = Feedback.Position_Deg / 180.0f * PI;
        Feedback.Speed_Rpm = (float) (int16_t) jc_get_u16_be(&Rx_Data[4]);
        Feedback.Speed_Radps = Feedback.Speed_Rpm * BASIC_MATH_RPM_TO_RADPS;
        Feedback.Torque_Nm = (float) jc_get_u16_be(&Rx_Data[6]) / 100.0f;
    }
    else if (Rx_Data[0] == 0x4b)
    {
        index = jc_get_u16_be(&Rx_Data[1]);
        if (index == 0x0004)
        {
            Feedback.Voltage_V = (float) jc_get_u16_be(&Rx_Data[4]) / 10.0f;
        }
    }
    else if (Rx_Data[0] == 0x43)
    {
        index = jc_get_u16_be(&Rx_Data[1]);
        if (index == 0x0006)
        {
            Feedback.Speed_Rpm = (float) jc_get_i32_be(&Rx_Data[4]) / 100.0f;
            Feedback.Speed_Radps = Feedback.Speed_Rpm * BASIC_MATH_RPM_TO_RADPS;
        }
        else if (index == 0x0008)
        {
            Feedback.Position_Deg = (float) jc_get_i32_be(&Rx_Data[4]) / 100.0f;
            Feedback.Position_Rad = Feedback.Position_Deg / 180.0f * PI;
        }
    }
}

void Class_Motor_JC::Write_SDO_U16(uint16_t Index, uint16_t Value) const
{
    uint8_t tx_data[8] = {0};

    tx_data[0] = 0x2b;
    jc_set_u16_be(&tx_data[1], Index);
    jc_set_u16_be(&tx_data[4], Value);

    Send_Frame(tx_data);
}

void Class_Motor_JC::Write_SDO_I16(uint16_t Index, int16_t Value) const
{
    uint8_t tx_data[8] = {0};

    tx_data[0] = 0x2b;
    jc_set_u16_be(&tx_data[1], Index);
    jc_set_u16_be(&tx_data[4], (uint16_t) Value);

    Send_Frame(tx_data);
}

void Class_Motor_JC::Write_SDO_I32(uint16_t Index, int32_t Value) const
{
    uint8_t tx_data[8] = {0};

    tx_data[0] = 0x23;
    jc_set_u16_be(&tx_data[1], Index);
    jc_set_i32_be(&tx_data[4], Value);

    Send_Frame(tx_data);
}

void Class_Motor_JC::Read_SDO_U16(uint16_t Index) const
{
    uint8_t tx_data[8] = {0};

    tx_data[0] = 0x4b;
    jc_set_u16_be(&tx_data[1], Index);

    Send_Frame(tx_data);
}

void Class_Motor_JC::Read_SDO_I32(uint16_t Index) const
{
    uint8_t tx_data[8] = {0};

    tx_data[0] = 0x43;
    jc_set_u16_be(&tx_data[1], Index);

    Send_Frame(tx_data);
}

void Class_Motor_JC::Send_Frame(const uint8_t *Tx_Data) const
{
    if (CAN_Manage_Object == nullptr || CAN_Manage_Object->CAN_Handler == nullptr)
    {
        return;
    }

    CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, const_cast<uint8_t *>(Tx_Data), 8);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
