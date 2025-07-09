#include <rtthread.h>
#include <rtdevice.h>
#include <drv_pwm.h>
#include <stdlib.h>
#include <math.h>
#include "drv_gpio.h"
#include "aht10.h"
#include "ssd1306.h"
#include "cyhal.h"
// PWM配置
#define PWM_DEV_NAME        "pwm0"      // PWM设备名称
#define PWM_CHANNEL         3           // PWM通道号
#define PWM_CHANNEL2        1
#define SAMPLE_PERIOD       200         // PID采样周期(ms)
#define MIN_SAFE_TEMP       -10.0f      // 最低安全温度
#define MAX_SAFE_TEMP       40.0f       // 最高安全温度
#define PWM_PERIOD_NS       1000000     // PWM周期(1ms = 1000000ns)
#define TEMP_STEP           0.5f        // 编码器每步温度变化量

// 旋转编码器引脚配置
#define ENCODER_A_PIN       GET_PIN(5, 0)  // 编码器A相引脚
#define ENCODER_B_PIN       GET_PIN(5, 1)  // 编码器B相引脚
#define ENCODER_BTN_PIN     GET_PIN(6, 2)  // 编码器按钮引脚
static cyhal_gpio_callback_data_t encoder_a_cb_data;

#define RELAY_HUMIDITY_1_PIN   GET_PIN(11, 3)  // 湿度控制继电器1引脚
#define RELAY_HUMIDITY_2_PIN   GET_PIN(11, 2)  // 湿度控制继电器2引脚

// 全局温度控制结构体
typedef struct {
    float current_temp;      // 当前温度
    float current_temp2;
    float target_temp;       // 目标温度
    float target_temp2;
    float current_humidity;
    float pwm_duty;          // PWM占空比
    float pwm_duty2;
    rt_bool_t is_running;    // PID是否运行
    rt_bool_t adjust_mode;   // 是否处于调整模式
    rt_bool_t relay_humidity_1_state;   // 湿度继电器1状态
    rt_bool_t relay_humidity_2_state;   // 湿度继电器2状态
} TempControlData;


// 全局温度控制实例
TempControlData temp_control = {
    .current_temp = 25.0f,
    .current_temp2 = 26.0f,
    .target_temp = 5.0f,   // 初始目标温度
    .target_temp2 = 20.0f,
    .current_humidity = 0.0f,
    .pwm_duty = 0.0f,
    .pwm_duty2 = 0.0f,
    .is_running = RT_TRUE,
    .adjust_mode = RT_FALSE,
    .relay_humidity_1_state = RT_FALSE,
    .relay_humidity_2_state = RT_FALSE
};

#define I2C_BUS_NAME_1 "i2c1"  // 根据实际I2C设备名修改
#define I2C_BUS_NAME_2 "i2c3"

aht10_device_t dev1, dev2;
static volatile rt_bool_t display_update_required = RT_TRUE;
// PWM设备句柄
static struct rt_device_pwm *pwm_dev;
static struct rt_device_pwm *pwm_dev1;

// PID控制器结构体
typedef struct {
    float Kp;         // 比例系数
    float Ki;         // 积分系数
    float Kd;         // 微分系数
    float integral;   // 积分项累积值
    float prev_error; // 上一次误差
    rt_tick_t prev_tick; // 上一次计算时间
} PID_Controller;

// PID控制器实例
static PID_Controller pid_controller;

// 初始化PID控制器
void pid_init(PID_Controller* pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_tick = rt_tick_get();
}

// 计算PID输出 (0~100%百分比) - 制冷模式
float pid_calculate(PID_Controller* pid, float setpoint, float input) {
    // 获取当前时间并计算时间差(秒)
    rt_tick_t now = rt_tick_get();
    float dt = (pid->prev_tick == 0) ? 0.01f :
               (float)(now - pid->prev_tick) / RT_TICK_PER_SECOND;
    pid->prev_tick = now;

    // 制冷模式误差计算：只考虑需要制冷的正误差
    float error = (input > setpoint) ? (input - setpoint) : 0.0f;

    // 当不需要制冷时，重置积分项
    if (input <= setpoint) {
        pid->integral = 0;
    }

    // 比例项
    float P = pid->Kp * error;

    // 积分项 (带抗饱和) - 只在需要制冷时累积
    if (error > 0) {
        pid->integral += error * dt;
    }

    // 积分限幅防止超调
    float max_integral = 100.0f / fabsf(pid->Ki);
    float min_integral = -max_integral;
    if (pid->integral > max_integral) pid->integral = max_integral;
    if (pid->integral < min_integral) pid->integral = min_integral;
    float I = pid->Ki * pid->integral;

    // 微分项 - 只在需要制冷时计算
    float derivative = 0;
    if (error > 0 && pid->prev_error != 0 && dt > 0.001f) {
        derivative = (error - pid->prev_error) / dt;
    }
    float D = pid->Kd * derivative;
    pid->prev_error = error;

    // 计算总输出
    float output = P + I + D;

    // 输出限幅 (0~100%)
    if (output > 100.0f) output = 100.0f;
    if (output < 0.0f) output = 0.0f;

    // 确保当不需要制冷时输出为0
    if (input <= setpoint) {
        output = 0.0f;
    }

    // 调试输出PID各项
    //rt_kprintf("PID: P=%.2f, I=%.2f, D=%.2f, Out=%.2f%%, Error=%.2f\n", P, I, D, output, error);

    return output;
}

// 设置PWM占空比 (0~100%)
void set_pwm_duty(float duty_cycle, float duty_cycle2) {
    // 确保百分比在合理范围
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;
    if (duty_cycle > 100.0f) duty_cycle = 100.0f;
    if (duty_cycle2 < 0.0f) duty_cycle2 = 0.0f;
    if (duty_cycle2 > 100.0f) duty_cycle2 = 100.0f;

    // 保存到全局结构
    temp_control.pwm_duty = duty_cycle;
    temp_control.pwm_duty2 = duty_cycle2;

    // 计算脉冲宽度 (单位:ns)
    rt_uint32_t pulse = (rt_uint32_t)(PWM_PERIOD_NS * duty_cycle / 100.0f);
    rt_uint32_t pulse2 = (rt_uint32_t)(PWM_PERIOD_NS * duty_cycle2 / 100.0f);

    // 确保脉冲宽度在有效范围内
    if (pulse > PWM_PERIOD_NS && pulse2 > PWM_PERIOD_NS) pulse = pulse2 = PWM_PERIOD_NS;


    // 设置PWM参数并启用
    if (rt_pwm_set(pwm_dev, PWM_CHANNEL, PWM_PERIOD_NS, pulse) != RT_EOK) {
        rt_kprintf("PWM set failed! Channel: %d\n", PWM_CHANNEL);
    }
    if (rt_pwm_set(pwm_dev, PWM_CHANNEL2, PWM_PERIOD_NS, pulse2) != RT_EOK) {
        rt_kprintf("PWM set failed! Channel: %d\n", PWM_CHANNEL2);
    }
    rt_pwm_enable(pwm_dev, PWM_CHANNEL);
    rt_pwm_enable(pwm_dev, PWM_CHANNEL2);
}

void relay_init(void) {
   // 初始化湿度继电器1
   rt_pin_mode(RELAY_HUMIDITY_1_PIN, PIN_MODE_OUTPUT);
   rt_pin_write(RELAY_HUMIDITY_1_PIN, PIN_LOW);

   // 初始化湿度继电器2
   rt_pin_mode(RELAY_HUMIDITY_2_PIN, PIN_MODE_OUTPUT);
   rt_pin_write(RELAY_HUMIDITY_2_PIN, PIN_LOW);

   rt_kprintf("Humidity relays initialized (both OFF)\n");
}


void set_humidity_relays(rt_bool_t state) {
    // 控制继电器1
    if (state) {
        rt_pin_write(RELAY_HUMIDITY_1_PIN, PIN_HIGH);
        temp_control.relay_humidity_1_state = RT_TRUE;
    } else {
        rt_pin_write(RELAY_HUMIDITY_1_PIN, PIN_LOW);
        temp_control.relay_humidity_1_state = RT_FALSE;
    }

    // 控制继电器2
    if (state) {
        rt_pin_write(RELAY_HUMIDITY_2_PIN, PIN_HIGH);
        temp_control.relay_humidity_2_state = RT_TRUE;
    } else {
        rt_pin_write(RELAY_HUMIDITY_2_PIN, PIN_LOW);
        temp_control.relay_humidity_2_state = RT_FALSE;
    }

    rt_kprintf("Humidity relays turned %s\n", state ? "ON" : "OFF");
}


// 读取温度函数
float read_temperature(void) {
    // 从传感器1读取温度

    temp_control.current_temp = aht10_read_temperature(dev1);
    temp_control.current_humidity = aht10_read_humidity(dev1);
    return temp_control.current_temp;
}
float read_humi(void){
    float humidity = aht10_read_humidity(dev1);

    return humidity;
}

float read_humi2(void){
    float humidity2 = aht10_read_humidity(dev2);

    return humidity2;
}

float read_temperature2(void) {

    // 从传感器1读取温度

    // 保存到全局结构
    temp_control.current_temp2 = aht10_read_temperature(dev2);

    return temp_control.current_temp2;
}

// PID温度控制线程
void temp_control_thread_entry(void *parameter) {
    // 初始化PID参数 (制冷系统参数)
    pid_init(&pid_controller, 30.0f, 0.01f, 0.0f); // Kp, Ki, Kd

    // 初始温度读取
    float current_temp = read_temperature();
    float current_temp2 = read_temperature2();
/*
    rt_kprintf("Cooling control started. Target: %.1fC, Initial temp: %.1fC\n",
              temp_control.target_temp, current_temp);
*/
    while (temp_control.is_running) {

        rt_tick_t last_relay_check = rt_tick_get();
        const rt_tick_t RELAY_CHECK_INTERVAL = 2000;
        // 1. 读取当前温度
        current_temp = read_temperature();
        current_temp2 = read_temperature2();

        if (rt_tick_get() - last_relay_check > RELAY_CHECK_INTERVAL) {
                    // 当湿度 > 50% 时开启两个继电器
                    if (temp_control.current_humidity > 50.0f) {
                        if (!temp_control.relay_humidity_1_state || !temp_control.relay_humidity_2_state) {
                            set_humidity_relays(RT_TRUE); // 同时开启两个继电器
                            display_update_required = RT_TRUE;
                        }
                    }
                    // 当湿度 ≤ 50% 时关闭两个继电器
                    else {
                        if (temp_control.relay_humidity_1_state || temp_control.relay_humidity_2_state) {
                            set_humidity_relays(RT_FALSE); // 同时关闭两个继电器
                            display_update_required = RT_TRUE;
                        }
                    }
                    last_relay_check = rt_tick_get();
                }
        // 2. 温度安全保护
        if (current_temp < MIN_SAFE_TEMP) {
            rt_kprintf("WARNING! Temperature too low: %.1fC < %.1fC\n",
                      current_temp, MIN_SAFE_TEMP);
            set_pwm_duty(0,0); // 关闭制冷
            rt_thread_mdelay(5000); // 等待5秒
            continue;
        }

        if (current_temp > MAX_SAFE_TEMP) {
            rt_kprintf("WARNING! Temperature too high: %.1fC > %.1fC\n",
                      current_temp, MAX_SAFE_TEMP);
            set_pwm_duty(0,0); // 关闭制冷
            rt_thread_mdelay(5000); // 等待5秒
            continue;
        }

        // 3. 计算PID输出 (0~100%)
        float pwm_duty = pid_calculate(&pid_controller, temp_control.target_temp, temp_control.current_temp);
        float pwm_duty2 = pid_calculate(&pid_controller, temp_control.target_temp2, temp_control.current_temp2);
        // 4. 设置PWM输出
        set_pwm_duty(pwm_duty, pwm_duty2);

        // 5. 打印调试信息
        //rt_kprintf(" PWM1: %.1f%%, PWM2: %.1f%%\n", pwm_duty, pwm_duty2);

        // 6. 等待下一个控制周期
        rt_thread_mdelay(SAMPLE_PERIOD);
    }
}

// PWM初始化
int pwm_init(void) {
    // 查找PWM设备
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (pwm_dev == RT_NULL) {
        rt_kprintf("PWM init failed! Can't find %s device!\n", PWM_DEV_NAME);
        return -RT_ERROR;
    }

    // 初始设置0%占空比(制冷系统初始应关闭)
    set_pwm_duty(0.0f, 0.0f);

    // 验证PWM设备配置
    rt_uint32_t period, pulse;
   // rt_pwm_get(pwm_dev, PWM_CHANNEL, &period, &pulse);
    rt_kprintf("PWM[%s] Channel[%d] initialized. Period: %uns, Pulse: %uns\n",
              PWM_DEV_NAME, PWM_CHANNEL, period, pulse);
    rt_kprintf("PWM[%s] Channel[%d] initialized. Period: %uns, Pulse: %uns\n",
                  PWM_DEV_NAME, PWM_CHANNEL2, period, pulse);

    return RT_EOK;
}

// OLED显示更新函数
void update_display(void) {
    char temp_str[40];

    ssd1306_Fill(Black);

    // 第一行：显示状态
    ssd1306_SetCursor(0, 0);
    if (temp_control.is_running) {
        ssd1306_WriteString("PID: RUNNING", Font_7x10, White);
    } else {
        ssd1306_WriteString("PID: STOPPED", Font_7x10, White);
    }

    // 第二行：显示目标温度
    ssd1306_SetCursor(0, 10);
    snprintf(temp_str, sizeof(temp_str), "Target: %.1f C", temp_control.target_temp);
    ssd1306_WriteString(temp_str, Font_7x10, White);

    // 第三行：显示当前温度
    ssd1306_SetCursor(0, 20);
    snprintf(temp_str, sizeof(temp_str), "Current: %.1f C", temp_control.current_temp);
    ssd1306_WriteString(temp_str, Font_7x10, White);

    ssd1306_SetCursor(0, 30);
    snprintf(temp_str, sizeof(temp_str), "Humidity: %.1f%%", temp_control.current_humidity);
    ssd1306_WriteString(temp_str, Font_7x10, White);
    // 第四行：显示PWM占空比
    ssd1306_SetCursor(0, 40);
    snprintf(temp_str, sizeof(temp_str), "PWM: %.1f%%", temp_control.pwm_duty);
    ssd1306_WriteString(temp_str, Font_7x10, White);

    // 第五行：显示调整模式状态
    ssd1306_SetCursor(0, 50);
    if (temp_control.adjust_mode) {
        ssd1306_WriteString("ADJUST MODE", Font_7x10, White);
    } else {
        ssd1306_WriteString("NORMAL MODE", Font_7x10, White);
    }

    ssd1306_UpdateScreen();
}

// 修改后的编码器中断服务函数（完整状态机）
void encoder_isr(void *args) {
    rt_interrupt_enter();


    // 读取引脚状态
    bool a = rt_pin_read(ENCODER_A_PIN);
    bool b = rt_pin_read(ENCODER_B_PIN);

    // 四步法状态机


    // 检测顺时针旋转: 00->10->11->01->00
    if(a ==  b){

        if (temp_control.adjust_mode) {

            temp_control.target_temp += TEMP_STEP;
            if (temp_control.target_temp > MAX_SAFE_TEMP) {
                            temp_control.target_temp = MAX_SAFE_TEMP;
                        }

            display_update_required = RT_TRUE;
        }

    }
    // 检测逆时针旋转: 00->01->11->10->00

    else {
        if(temp_control.adjust_mode) {

            temp_control.target_temp -= TEMP_STEP;
            if (temp_control.target_temp < MIN_SAFE_TEMP) {
                            temp_control.target_temp = MIN_SAFE_TEMP;
                        }

            display_update_required = RT_TRUE;
        }

    }
    rt_interrupt_leave();
}

// 编码器按钮中断服务函数（简化版）
void encoder_btn_isr(void *args) {
    static rt_tick_t last_time = 0;

    // 防抖动处理
    rt_tick_t now = rt_tick_get();
    if (now - last_time < 20) {
        return;
    }
    last_time = now;

    // 切换调整模式
    temp_control.adjust_mode = !temp_control.adjust_mode;
    display_update_required = RT_TRUE; // 设置显示更新标志

    rt_kprintf("Adjust mode %s\n", temp_control.adjust_mode ? "ON" : "OFF");
}

// 初始化编码器
void encoder_init(void) {
    cyhal_gpio_init(ENCODER_A_PIN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    cyhal_gpio_init(ENCODER_B_PIN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);

    // 设置中断触发（上升沿）
    encoder_a_cb_data.callback = encoder_isr;
    encoder_a_cb_data.callback_arg = NULL;

    cyhal_gpio_register_callback(ENCODER_A_PIN, &encoder_a_cb_data);
    cyhal_gpio_enable_event(ENCODER_A_PIN, CYHAL_GPIO_IRQ_RISE, 1, true);

        // 配置编码器按钮引脚
        rt_pin_mode(ENCODER_BTN_PIN, PIN_MODE_INPUT_PULLUP);
        rt_pin_attach_irq(ENCODER_BTN_PIN, PIN_IRQ_MODE_FALLING, encoder_btn_isr, RT_NULL);
        rt_pin_irq_enable(ENCODER_BTN_PIN, PIN_IRQ_ENABLE);

        rt_kprintf("Rotary encoder initialized\n");
}

// 显示更新线程
void display_thread_entry(void *parameter) {
    ssd1306_Init();
        rt_tick_t last_update = rt_tick_get();

        while (1) {
            // 1. 检查是否需要更新显示
            if (display_update_required) {
                update_display();
                display_update_required = RT_FALSE; // 清除标志
                last_update = rt_tick_get();
            }
            // 2. 定期强制更新（即使没有标志变化）
            else if (rt_tick_get() - last_update > 1000) { // 至少每秒更新一次
                update_display();
                last_update = rt_tick_get();
            }

            rt_thread_mdelay(50); // 缩短检查间隔到50ms
        }
}

// 应用初始化
int pid_pwm_init(void) {
    // 初始化AHT10传感器
    dev1 = aht10_init(I2C_BUS_NAME_1);
    if (dev1 == RT_NULL) {
        rt_kprintf("AHT10 #1 init failed!\n");
        return -1;
    }

    dev2 = aht10_init(I2C_BUS_NAME_2);
    if (dev2 == RT_NULL) {
        rt_kprintf("AHT10 #2 init failed!\n");
    }

    // 初始化PWM
    if (pwm_init() != RT_EOK) {
        return -RT_ERROR;
    }

    // 初始化编码器
    encoder_init();

    // 创建温度控制线程
    rt_thread_t pid_tid = rt_thread_create("cooling_ctrl",
                                     temp_control_thread_entry,
                                     RT_NULL,
                                     2048,
                                     RT_THREAD_PRIORITY_MAX / 2,
                                     20);
    if (pid_tid != RT_NULL) {
        rt_thread_startup(pid_tid);
        rt_kprintf("Cooling control thread started\n");
    } else {
        rt_kprintf("Failed to create cooling control thread\n");
        return -RT_ERROR;
    }

    // 创建显示更新线程
    rt_thread_t display_tid = rt_thread_create("display_ctrl",
                                     display_thread_entry,
                                     RT_NULL,
                                     1024,
                                     RT_THREAD_PRIORITY_MAX / 4,
                                     10);
    if (display_tid != RT_NULL) {
        rt_thread_startup(display_tid);
        rt_kprintf("Display thread started\n");
    } else {
        rt_kprintf("Failed to create display thread\n");
    }

    return RT_EOK;
}

// 注册初始化函数 (自动启动)
//INIT_APP_EXPORT(pid_pwm_init);

// MSH命令：设置目标温度
static void set_target_humi(int argc, char **argv) {
    if (argc != 2) {
        rt_kprintf("Usage: set_temp <humidity>\n");
        rt_kprintf("Example: set_humi 5.5\n");
        return;
    }

    float new_humi = atof(argv[1]);


    set_humidity_relays(RT_TRUE);
    rt_kprintf("Target temperature set to %.1fC\n", new_humi);
}
MSH_CMD_EXPORT(set_target_humi, Set humi);
