/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-06-18     DREAM       the first version
 */
#ifndef APPLICATIONS_PID_H_
#define APPLICATIONS_PID_H_

typedef struct {
    float current_temp;      // 当前温度
    float current_temp2;
    float target_temp;       // 目标温度
    float target_temp2;
    float pwm_duty;          // PWM占空比
    float pwm_duty2;
    rt_bool_t is_running;    // PID是否运行
    rt_bool_t adjust_mode;   // 是否处于调整模式
} TempControlData;


typedef struct {
    float Kp;         // 比例系数
    float Ki;         // 积分系数
    float Kd;         // 微分系数
    float integral;   // 积分项累积值
    float prev_error; // 上一次误差
    rt_tick_t prev_tick; // 上一次计算时间
} PID_Controller;

// 两个独立的PID控制器
static PID_Controller pid_controller;


void pid_init(PID_Controller* pid, float Kp, float Ki, float Kd);
float pid_calculate(PID_Controller* pid, float setpoint, float input);
void set_pwm_duty(float duty_cycle, float duty_cycle2);
float read_temperature(void);
float read_temperature2(void);
float read_humi(void);
float read_humi2(void);
void temp_control_thread_entry(void *parameter);
int pwm_init(void);
void update_display(void);
void encoder_isr(void *args);
void encoder_btn_isr(void *args);
void encoder_init(void);
void display_thread_entry(void *parameter);
int pid_pwm_init(void);
#endif /* APPLICATIONS_PID_H_ */
