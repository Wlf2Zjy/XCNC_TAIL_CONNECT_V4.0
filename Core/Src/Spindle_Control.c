#include "spindle_control.h"

// ��������ʼ��
void laser_init(void)
{
    // ����PWM��ʱ��
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    
    // ��ʼ״̬Ϊ�ر�
    laser_stop();
}

// �����رռ�����
void laser_stop(void)
{
    // ����PWMΪ0����ȫ�رռ������
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, LASER_PWM_OFF_VALUE);
}

// ֱ������PWM����ֵ (0-1000)
void laser_set_power(uint16_t pwm_value)
{
    // �߽���
    if (pwm_value > LASER_PWM_MAX_VALUE) {
        pwm_value = LASER_PWM_MAX_VALUE;
    }
    
    // ����PWMռ�ձ�
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_value);
}

// ���ٷֱ����ù��� (0-100%)
void laser_set_power_percent(uint8_t percent)
{
    uint16_t pwm_value;
    
    // �߽���
    if (percent > 100) {
        percent = 100;
    }
    
    // ת��Ϊ0-1000��Χ��PWMֵ
    pwm_value = (percent * LASER_PWM_MAX_VALUE) / 100;
    
    // ����PWM
    laser_set_power(pwm_value);
}

void laser_set_power_from_byte(uint8_t power_byte)
{
    // ��0-255ӳ�䵽0-1000
    uint16_t pwm_value = (power_byte * 1000 + 127) / 255;  // ��������
    
    // �߽���
    if (pwm_value > LASER_PWM_MAX_VALUE) {
        pwm_value = LASER_PWM_MAX_VALUE;
    }
    
    // ����PWM
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_value);
}

// ���ü�����״̬�͹���
void laser_set_state(uint8_t state, uint16_t power_level)
{
    if (state == LASER_DISABLE) {
        // �رռ���
        laser_stop();
    } else {
        // ���ü��⣬����ָ������
        laser_set_power(power_level);
    }

}
/*����������
//		        laser_set_power_percent(75);
//        HAL_Delay(2000);
//		        laser_set_state(LASER_DISABLE, 0);
//        HAL_Delay(1000);
*/

