#include "spindle_control.h"

// 激光器初始化
void laser_init(void)
{
    // 启动PWM定时器
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    
    // 初始状态为关闭
    laser_stop();
}

// 立即关闭激光器
void laser_stop(void)
{
    // 设置PWM为0，完全关闭激光输出
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, LASER_PWM_OFF_VALUE);
}

// 直接设置PWM功率值 (0-1000)
void laser_set_power(uint16_t pwm_value)
{
    // 边界检查
    if (pwm_value > LASER_PWM_MAX_VALUE) {
        pwm_value = LASER_PWM_MAX_VALUE;
    }
    
    // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_value);
}

// 按百分比设置功率 (0-100%)
void laser_set_power_percent(uint8_t percent)
{
    uint16_t pwm_value;
    
    // 边界检查
    if (percent > 100) {
        percent = 100;
    }
    
    // 转换为0-1000范围的PWM值
    pwm_value = (percent * LASER_PWM_MAX_VALUE) / 100;
    
    // 设置PWM
    laser_set_power(pwm_value);
}

void laser_set_power_from_byte(uint8_t power_byte)
{
    // 将0-255映射到0-1000
    uint16_t pwm_value = (power_byte * 1000 + 127) / 255;  // 四舍五入
    
    // 边界检查
    if (pwm_value > LASER_PWM_MAX_VALUE) {
        pwm_value = LASER_PWM_MAX_VALUE;
    }
    
    // 设置PWM
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_value);
}

// 设置激光器状态和功率
void laser_set_state(uint8_t state, uint16_t power_level)
{
    if (state == LASER_DISABLE) {
        // 关闭激光
        laser_stop();
    } else {
        // 启用激光，设置指定功率
        laser_set_power(power_level);
    }

}
/*激光器测试
//		        laser_set_power_percent(75);
//        HAL_Delay(2000);
//		        laser_set_state(LASER_DISABLE, 0);
//        HAL_Delay(1000);
*/

