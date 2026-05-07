#ifndef __FORCE_CONTROL_H
#define __FORCE_CONTROL_H

#include "main.h"
#include "loadcell_i2c.h"

// 제어 모드 열거형
typedef enum {
    CTRL_MODE_TEMPERATURE = 0,   // 기존 온도제어 모드
    CTRL_MODE_FORCE       = 1,   // 힘 제어 모드
} ControlMode_TypeDef;

typedef struct {
    float kp;
    float ki;
    float kd;
    float target_force;         // 목표 힘 (g 또는 N)
    float integral;
    float last_error;
    float output_min;
    float output_max;
    float max_force;            // 안전 상한
    uint8_t enabled;
} Force_PID_TypeDef;

typedef struct {
    ControlMode_TypeDef mode;           // 현재 제어 모드
    Force_PID_TypeDef force_pid;        // 힘 PID 파라미터
    LoadCell_Data_TypeDef loadcell;     // 로드셀 데이터
    uint8_t active_channel;             // 힘 제어가 적용되는 PWM 채널
} ForceControl_TypeDef;

extern ForceControl_TypeDef force_ctrl;

void ForceControl_Init(void);
float ForceControl_Calculate(float current_force);
void ForceControl_SetTarget(float target_force);
void ForceControl_Enable(uint8_t channel);
void ForceControl_Disable(void);

#endif