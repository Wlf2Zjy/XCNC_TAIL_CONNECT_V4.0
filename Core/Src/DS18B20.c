#include "ds18b20.h"

/* -------------------- Ӳ�����Ŷ��� -------------------- */
// ���¶ȴ����� - PC13
#define DS18B20_MAIN_IO_IN()  {GPIOC->CRH &= ~(0x0F << ((13 - 8) * 4)); GPIOC->CRH |= (0x08 << ((13 - 8) * 4));}
#define DS18B20_MAIN_IO_OUT() {GPIOC->CRH &= ~(0x0F << ((13 - 8) * 4)); GPIOC->CRH |= (0x03 << ((13 - 8) * 4));}
#define DS18B20_MAIN_DQ_OUT(n) (n ? HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET))
#define DS18B20_MAIN_DQ_IN()   HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)

// ���¶ȴ����� - PC14
#define DS18B20_LEFT_IO_IN()  {GPIOC->CRH &= ~(0x0F << ((14 - 8) * 4)); GPIOC->CRH |= (0x08 << ((14 - 8) * 4));}
#define DS18B20_LEFT_IO_OUT() {GPIOC->CRH &= ~(0x0F << ((14 - 8) * 4)); GPIOC->CRH |= (0x03 << ((14 - 8) * 4));}
#define DS18B20_LEFT_DQ_OUT(n) (n ? HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET))
#define DS18B20_LEFT_DQ_IN()   HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)

// ���¶ȴ����� - PC15
#define DS18B20_RIGHT_IO_IN()  {GPIOC->CRH &= ~(0x0F << ((15 - 8) * 4)); GPIOC->CRH |= (0x08 << ((15 - 8) * 4));}
#define DS18B20_RIGHT_IO_OUT() {GPIOC->CRH &= ~(0x0F << ((15 - 8) * 4)); GPIOC->CRH |= (0x03 << ((15 - 8) * 4));}
#define DS18B20_RIGHT_DQ_OUT(n) (n ? HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET))
#define DS18B20_RIGHT_DQ_IN()   HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)

/* -------- ΢����ʱ���� (TIM4 ʵ��) -------- */
static void DS18B20_DelayUs(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    HAL_TIM_Base_Start(&htim4);
    while (__HAL_TIM_GET_COUNTER(&htim4) < us);
    HAL_TIM_Base_Stop(&htim4);
}

/* -------- ���ݴ�����ѡ��IO���� -------- */
static void DS18B20_SelectSensor(DS18B20_Sensor_t sensor)
{
    switch(sensor) {
        case DS18B20_MAIN:
            DS18B20_MAIN_IO_OUT();
            DS18B20_MAIN_DQ_OUT(1);
            break;
        case DS18B20_LEFT:
            DS18B20_LEFT_IO_OUT();
            DS18B20_LEFT_DQ_OUT(1);
            break;
        case DS18B20_RIGHT:
            DS18B20_RIGHT_IO_OUT();
            DS18B20_RIGHT_DQ_OUT(1);
            break;
    }
}

/* -------- ��ʼ��ָ�������� -------- */
void DS18B20_Init(DS18B20_Sensor_t sensor)
{
    DS18B20_SelectSensor(sensor);
}

/* -------- ���߸�λ������豸���� -------- */
uint8_t DS18B20_Start(DS18B20_Sensor_t sensor)
{
    uint8_t ack = 0;
    
    // �������ģʽ
    switch(sensor) {
        case DS18B20_MAIN:
            DS18B20_MAIN_IO_OUT();
            DS18B20_MAIN_DQ_OUT(0);
            break;
        case DS18B20_LEFT:
            DS18B20_LEFT_IO_OUT();
            DS18B20_LEFT_DQ_OUT(0);
            break;
        case DS18B20_RIGHT:
            DS18B20_RIGHT_IO_OUT();
            DS18B20_RIGHT_DQ_OUT(0);
            break;
    }
    
    DS18B20_DelayUs(600);   // 480us����
    
    // �ͷ�����
    switch(sensor) {
        case DS18B20_MAIN:
            DS18B20_MAIN_DQ_OUT(1);
            break;
        case DS18B20_LEFT:
            DS18B20_LEFT_DQ_OUT(1);
            break;
        case DS18B20_RIGHT:
            DS18B20_RIGHT_DQ_OUT(1);
            break;
    }
    
    DS18B20_DelayUs(30);    // 15-60us
    
    // ����Ϊ����ģʽ���ȴ�DS18B20��Ӧ
    switch(sensor) {
        case DS18B20_MAIN:
            DS18B20_MAIN_IO_IN();
            break;
        case DS18B20_LEFT:
            DS18B20_LEFT_IO_IN();
            break;
        case DS18B20_RIGHT:
            DS18B20_RIGHT_IO_IN();
            break;
    }
    
    DS18B20_DelayUs(10);
    
    // ���Ӧ���ź�
    switch(sensor) {
        case DS18B20_MAIN:
            if (DS18B20_MAIN_DQ_IN() == 0) ack = 1;
            break;
        case DS18B20_LEFT:
            if (DS18B20_LEFT_DQ_IN() == 0) ack = 1;
            break;
        case DS18B20_RIGHT:
            if (DS18B20_RIGHT_DQ_IN() == 0) ack = 1;
            break;
    }
    
    DS18B20_DelayUs(480);
    
    // �ָ����ģʽ
    DS18B20_SelectSensor(sensor);
    
    return ack;
}

/* -------- ��ָ��������д��һ���ֽ� -------- */
void DS18B20_WriteByte(DS18B20_Sensor_t sensor, uint8_t dat)
{
    // �������ģʽ
    switch(sensor) {
        case DS18B20_MAIN:
            DS18B20_MAIN_IO_OUT();
            break;
        case DS18B20_LEFT:
            DS18B20_LEFT_IO_OUT();
            break;
        case DS18B20_RIGHT:
            DS18B20_RIGHT_IO_OUT();
            break;
    }
    
    for (uint8_t i = 0; i < 8; i++)
    {
        // �������߿�ʼдʱ��
        switch(sensor) {
            case DS18B20_MAIN:
                DS18B20_MAIN_DQ_OUT(0);
                break;
            case DS18B20_LEFT:
                DS18B20_LEFT_DQ_OUT(0);
                break;
            case DS18B20_RIGHT:
                DS18B20_RIGHT_DQ_OUT(0);
                break;
        }
        
        DS18B20_DelayUs(2);
        
        // д������λ
        switch(sensor) {
            case DS18B20_MAIN:
                if (dat & 0x01) DS18B20_MAIN_DQ_OUT(1);
                else DS18B20_MAIN_DQ_OUT(0);
                break;
            case DS18B20_LEFT:
                if (dat & 0x01) DS18B20_LEFT_DQ_OUT(1);
                else DS18B20_LEFT_DQ_OUT(0);
                break;
            case DS18B20_RIGHT:
                if (dat & 0x01) DS18B20_RIGHT_DQ_OUT(1);
                else DS18B20_RIGHT_DQ_OUT(0);
                break;
        }
        
        DS18B20_DelayUs(60);
        
        // �ͷ�����
        switch(sensor) {
            case DS18B20_MAIN:
                DS18B20_MAIN_DQ_OUT(1);
                break;
            case DS18B20_LEFT:
                DS18B20_LEFT_DQ_OUT(1);
                break;
            case DS18B20_RIGHT:
                DS18B20_RIGHT_DQ_OUT(1);
                break;
        }
        
        dat >>= 1;
        DS18B20_DelayUs(2);
    }
}

/* -------- ��ָ����������ȡһ���ֽ� -------- */
uint8_t DS18B20_ReadByte(DS18B20_Sensor_t sensor)
{
    uint8_t dat = 0;
    
    for (uint8_t i = 0; i < 8; i++)
    {
        dat >>= 1;
        
        // ����Ϊ���ģʽ���������߿�ʼ��ʱ��
        switch(sensor) {
            case DS18B20_MAIN:
                DS18B20_MAIN_IO_OUT();
                DS18B20_MAIN_DQ_OUT(0);
                break;
            case DS18B20_LEFT:
                DS18B20_LEFT_IO_OUT();
                DS18B20_LEFT_DQ_OUT(0);
                break;
            case DS18B20_RIGHT:
                DS18B20_RIGHT_IO_OUT();
                DS18B20_RIGHT_DQ_OUT(0);
                break;
        }
        
        DS18B20_DelayUs(2);
        
        // �ͷ����߲�����Ϊ����ģʽ
        switch(sensor) {
            case DS18B20_MAIN:
                DS18B20_MAIN_DQ_OUT(1);
                DS18B20_MAIN_IO_IN();
                break;
            case DS18B20_LEFT:
                DS18B20_LEFT_DQ_OUT(1);
                DS18B20_LEFT_IO_IN();
                break;
            case DS18B20_RIGHT:
                DS18B20_RIGHT_DQ_OUT(1);
                DS18B20_RIGHT_IO_IN();
                break;
        }
        
        DS18B20_DelayUs(8);
        
        // ��ȡ����λ
        switch(sensor) {
            case DS18B20_MAIN:
                if (DS18B20_MAIN_DQ_IN()) dat |= 0x80;
                break;
            case DS18B20_LEFT:
                if (DS18B20_LEFT_DQ_IN()) dat |= 0x80;
                break;
            case DS18B20_RIGHT:
                if (DS18B20_RIGHT_DQ_IN()) dat |= 0x80;
                break;
        }
        
        DS18B20_DelayUs(50);
        
        // �ָ����ģʽ
        switch(sensor) {
            case DS18B20_MAIN:
                DS18B20_MAIN_IO_OUT();
                DS18B20_MAIN_DQ_OUT(1);
                break;
            case DS18B20_LEFT:
                DS18B20_LEFT_IO_OUT();
                DS18B20_LEFT_DQ_OUT(1);
                break;
            case DS18B20_RIGHT:
                DS18B20_RIGHT_IO_OUT();
                DS18B20_RIGHT_DQ_OUT(1);
                break;
        }
    }
    return dat;
}

/* -------- ��ȡָ�����������¶� -------- */
float DS18B20_ReadTemp(DS18B20_Sensor_t sensor)
{
    uint8_t tempL, tempH;
    int16_t temp;
    float temperature;
    
    if (DS18B20_Start(sensor)) 
    {
        DS18B20_WriteByte(sensor, 0xCC);  // ����ROM
        DS18B20_WriteByte(sensor, 0x44);  // �����¶�ת��
        HAL_Delay(800);                   // �ȴ�ת�����
        
        DS18B20_Start(sensor);
        DS18B20_WriteByte(sensor, 0xCC);  // ����ROM
        DS18B20_WriteByte(sensor, 0xBE);  // ��ȡ�ݴ���
        
        tempL = DS18B20_ReadByte(sensor); // ��ȡ�¶ȵ��ֽ�
        tempH = DS18B20_ReadByte(sensor); // ��ȡ�¶ȸ��ֽ�
        
        temp = (tempH << 8) | tempL;
        temperature = temp * 0.0625f;
        return temperature;
    }
    return -1000.0f;  // ���󷵻�ֵ
}

/* -------- ͬʱ��ȡ�����������������¶� -------- */
void DS18B20_ReadAllTemps(float *main_temp, float *left_temp, float *right_temp)
{
    if (main_temp) *main_temp = DS18B20_ReadTemp(DS18B20_MAIN);
    if (left_temp) *left_temp = DS18B20_ReadTemp(DS18B20_LEFT);
    if (right_temp) *right_temp = DS18B20_ReadTemp(DS18B20_RIGHT);
}

/* -------- ��ȡ���������� -------- */
const char* DS18B20_GetSensorName(DS18B20_Sensor_t sensor)
{
    switch(sensor) {
        case DS18B20_MAIN: return "MAIN";
        case DS18B20_LEFT: return "LEFT";
        case DS18B20_RIGHT: return "RIGHT";
        default: return "UNKNOWN";
    }
}
