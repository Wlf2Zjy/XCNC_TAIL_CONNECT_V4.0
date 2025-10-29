#ifndef __SPINDLE_CONTROL_H
#define __SPINDLE_CONTROL_H

#include "main.h"
#include "tim.h"

#include "main.h"
#include "tim.h"

// PWM参数定义
#define LASER_PWM_MAX_VALUE     1000    // 最大PWM值 (100%功率)
#define LASER_PWM_MIN_VALUE     0       // 最小PWM值 (0%功率)
#define LASER_PWM_OFF_VALUE     0       // 关闭激光

// 激光状态定义
#define LASER_DISABLE           0
#define LASER_ENABLE            1

// 函数声明
void laser_init(void);
void laser_stop(void);
void laser_set_power(uint16_t pwm_value);
void laser_set_power_percent(uint8_t percent);
void laser_set_state(uint8_t state, uint16_t power_level);
void laser_set_power_from_byte(uint8_t power_byte);

#endif
