/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-05-10     DREAM       the first version
 */
/*
#include <rtthread.h>
#include <rtdevice.h>

#define PWM_DEV_NAME "pwm0"
#define PWM_DEV_CHANNEL 3
#define PWM1_DEV_NAME "pwm0"
#define PWM1_DEV_CHANNEL 1

struct rt_device_pwm *pwm_dev;
struct rt_device_pwm *pwm1_dev;

int pwm_init(void)
{
    rt_uint32_t period, pulse, dir;

    period = 1 * 1000 * 1000;
    dir = 1;
    pulse = 0;

    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    pwm1_dev = (struct rt_device_pwm *)rt_device_find(PWM1_DEV_NAME);

    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("pwm sample run failed! can't find %s device!\n", PWM_DEV_NAME);
        return -RT_ERROR;
    }

    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, pulse);
    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);
    rt_pwm_set(pwm1_dev, PWM1_DEV_CHANNEL, period, pulse);
    rt_pwm_enable(pwm1_dev, PWM1_DEV_CHANNEL);

    rt_kprintf("Now PWM[%s] Channel[%d] Period[%d] Pulse[%d]\n", PWM_DEV_NAME, PWM_DEV_CHANNEL, period, pulse);

    while (1)
    {
        rt_thread_mdelay(50);

        if (dir)
        {
            pulse += 100000;
        }
        else
        {
            pulse -= 100000;
        }

        if (pulse >= period)
        {
            dir = 0;
        }

        if (0 == pulse)
        {
            dir = 1;
        }

        rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, pulse);
        rt_pwm_set(pwm1_dev, PWM1_DEV_CHANNEL, period, pulse);
    }
}
*/
