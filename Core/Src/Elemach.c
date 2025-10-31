#include "Elemach.h"
#include "main.h"
#include "tim.h"
/*���Ե���
//    Motor_SetSpeedFromInput(2500);  // ��Ӧ250 RPM  
//		Motor_SetDirection(0);          // ����ת
//		HAL_Delay(50000);
*/

Motor_TypeDef motor = {8, 0, 0};  // Ĭ��8ϸ�֣�60RPM����ת

// ���״̬����
volatile uint8_t motor_enabled = 0;
volatile uint16_t motor_current_speed = 0;
volatile uint8_t motor_current_direction = 0;

// �������˶�����
volatile uint32_t target_pulse_count = 0;
volatile uint32_t current_pulse_count = 0;
volatile uint8_t motor_distance_mode = 0;

// ����ֵ��Χ����
#define INPUT_MIN 0
#define INPUT_MAX 3000
#define RPM_MIN 0
#define RPM_MAX 300

/**
 * @brief ��������ֵ(0-3000)����ӳ�䵽RPM(0-300)
 * @param input_value ����ֵ (0-3000)
 * @return ��Ӧ��RPMֵ (0-300)
 */
uint16_t MapInputToRPM(uint16_t input_value)
{
    // �������뷶Χ
    //if (input_value < INPUT_MIN) input_value = INPUT_MIN;
    if (input_value > INPUT_MAX) input_value = INPUT_MAX;
    
    // ����ӳ��: RPM = (input * RPM_MAX) / INPUT_MAX
    uint32_t rpm = ((uint32_t)input_value * RPM_MAX) / INPUT_MAX;
    
    return (uint16_t)rpm;
}

/**
 * @brief ��������ֵ���õ���ٶ�
 * @param input_value ����ֵ (0-3000)
 */
void Motor_SetSpeedFromInput(uint16_t input_value)
{
    // ӳ�䵽RPM
    uint16_t rpm = MapInputToRPM(input_value);
    
    // �����ٶ�
    Motor_SetSpeed(rpm);
}

/**
 * @brief ���õ�����ж������˶�
 */
void Motor_DistanceMove(uint8_t direction, uint32_t pulse_count, uint16_t speed_value)
{
    target_pulse_count = pulse_count;
    current_pulse_count = 0;
    motor_distance_mode = 1;
    
    Motor_SetDirection(direction);
    Motor_SetSpeedFromInput(speed_value);
    Motor_Enable();
    
    motor_enabled = 1;
    motor_current_speed = speed_value;
    motor_current_direction = direction;
}


/**
 * @brief �����ʼ��
 */
void Motor_Init(void)
{
    // ���ó�ʼϸ��
    SubdivisionSet(motor.current_subdivision);
    
    // ���ó�ʼ����
    Motor_SetDirection(motor.direction);
    
    // ʹ�ܵ��
    Motor_Enable();
    
    
    // ��ȷ���ó�ʼ�ٶ�Ϊ0����ֹͣPWM
    motor.target_rpm = 0;
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);  // ȷ��PWMֹͣ
}

/**
 * @brief ����ϸ��ģʽ
 * @param subdivision ϸ��ֵ (8, 16, 32, 64)
 */
void SubdivisionSet(uint8_t subdivision)
{
    switch(subdivision)
    {
        case 8:   // 8ϸ��
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
            break;
        case 16:  // 16ϸ��
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
            break;
        case 32:  // 32ϸ��
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
            break;
        case 64:  // 64ϸ��
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
            break;
        default:  // Ĭ��8ϸ��
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
            subdivision = 8;
            break;
    }
    motor.current_subdivision = subdivision;
}

/**
 * @brief ���õ����ת����
 * @param dir 1 = ̽������ת, 0 =̽������ת
 */
void Motor_SetDirection(uint8_t dir)
{
    if(dir == 1)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); 
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);   
    }
    motor.direction = dir;
}

/**
 * @brief ���õ��ת��
 * @param rpm ת��ֵ (RPM)
 */
void Motor_SetSpeed(uint16_t rpm)
{
    motor.target_rpm = rpm;
    
    // ����ٶ�Ϊ0������ֹͣPWM
    if (rpm == 0) {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    } else {
        // ֻ���ٶȲ�Ϊ0ʱ�Ÿ���Ƶ��
        Motor_UpdateFrequency(motor.current_subdivision);
    }
}

/**
 * @brief ʹ�ܵ��
 */
void Motor_Enable(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);  // EN�͵�ƽʹ��
}

/**
 * @brief ���õ��
 */
void Motor_Disable(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);    // EN�ߵ�ƽ����
}

/**
 * @brief ����PWMƵ���Ա����趨ת��
 * @param subdivision ϸ��ֵ
 */
void Motor_UpdateFrequency(uint8_t subdivision)
{
    uint32_t steps_per_rev;
    uint32_t required_freq;
    uint32_t arr_value;
    
    // ����ϸ�ּ���ÿת����������
    switch(subdivision)
    {
        case 8:
            steps_per_rev = 1600;  // 200 * 8
            break;
        case 16:
            steps_per_rev = 3200;  // 200 * 16
            break;
        case 32:
            steps_per_rev = 6400;  // 200 * 32
            break;
        case 64:
            steps_per_rev = 12800; // 200 * 64
            break;
        default:
            steps_per_rev = 1600;  // Ĭ��8ϸ��
            break;
    }
    
    // ��������Ƶ�ʣ�Ƶ�� = (RPM �� ÿת������) �� 60
    required_freq = (motor.target_rpm * steps_per_rev) / 60;
    
    // ����ٶ�Ϊ0��ֹͣPWM���
    if (motor.target_rpm == 0) {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
        return;
    }
    
    // ����ARRֵ��ARR = (��ʱ��ʱ�� �� Ƶ��) - 1
    // ��ʱ��ʱ�� = 72MHz / (PSC + 1) = 72MHz / 72 = 1MHz
    arr_value = (1000000 / required_freq) - 1;
    
    // ����ARRֵ��Χ
    if(arr_value < 10) arr_value = 10;
    if(arr_value > 0xFFFF) arr_value = 0xFFFF;
    
    // ֹͣPWM���
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    
    // ����ARRֵ
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr_value);
    
    // ��������PWM���
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

/**
 * @brief �л�ϸ�ֲ�����ת��
 * @param new_subdivision �µ�ϸ��ֵ
 */
void Motor_ChangeSubdivision(uint8_t new_subdivision)
{
    if(new_subdivision == motor.current_subdivision) {
        return;
    }
    
    // ֹͣPWM���
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    
    // �����µ�ϸ��
    SubdivisionSet(new_subdivision);
    
    // ����Ƶ���Ա���ת��
    Motor_UpdateFrequency(new_subdivision);
}
