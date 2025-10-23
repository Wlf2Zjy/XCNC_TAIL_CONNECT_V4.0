#include "ds18b20.h"

/* -------------------- 硬件引脚定义 -------------------- */
// 主温度传感器 - PC13
#define DS18B20_MAIN_IO_IN()  {GPIOC->CRH &= ~(0x0F << ((13 - 8) * 4)); GPIOC->CRH |= (0x08 << ((13 - 8) * 4));}
#define DS18B20_MAIN_IO_OUT() {GPIOC->CRH &= ~(0x0F << ((13 - 8) * 4)); GPIOC->CRH |= (0x03 << ((13 - 8) * 4));}
#define DS18B20_MAIN_DQ_OUT(n) (n ? HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET))
#define DS18B20_MAIN_DQ_IN()   HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)

// 左温度传感器 - PC14
#define DS18B20_LEFT_IO_IN()  {GPIOC->CRH &= ~(0x0F << ((14 - 8) * 4)); GPIOC->CRH |= (0x08 << ((14 - 8) * 4));}
#define DS18B20_LEFT_IO_OUT() {GPIOC->CRH &= ~(0x0F << ((14 - 8) * 4)); GPIOC->CRH |= (0x03 << ((14 - 8) * 4));}
#define DS18B20_LEFT_DQ_OUT(n) (n ? HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET))
#define DS18B20_LEFT_DQ_IN()   HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)

// 右温度传感器 - PC15
#define DS18B20_RIGHT_IO_IN()  {GPIOC->CRH &= ~(0x0F << ((15 - 8) * 4)); GPIOC->CRH |= (0x08 << ((15 - 8) * 4));}
#define DS18B20_RIGHT_IO_OUT() {GPIOC->CRH &= ~(0x0F << ((15 - 8) * 4)); GPIOC->CRH |= (0x03 << ((15 - 8) * 4));}
#define DS18B20_RIGHT_DQ_OUT(n) (n ? HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET))
#define DS18B20_RIGHT_DQ_IN()   HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)

/* -------- 微秒延时函数 (TIM4 实现) -------- */
static void DS18B20_DelayUs(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    HAL_TIM_Base_Start(&htim4);
    while (__HAL_TIM_GET_COUNTER(&htim4) < us);
    HAL_TIM_Base_Stop(&htim4);
}

/* -------- 根据传感器选择IO操作 -------- */
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

/* -------- 初始化指定传感器 -------- */
void DS18B20_Init(DS18B20_Sensor_t sensor)
{
    DS18B20_SelectSensor(sensor);
}

/* -------- 总线复位并检测设备存在 -------- */
uint8_t DS18B20_Start(DS18B20_Sensor_t sensor)
{
    uint8_t ack = 0;
    
    // 设置输出模式
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
    
    DS18B20_DelayUs(600);   // 480us以上
    
    // 释放总线
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
    
    // 设置为输入模式，等待DS18B20回应
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
    
    // 检测应答信号
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
    
    // 恢复输出模式
    DS18B20_SelectSensor(sensor);
    
    return ack;
}

/* -------- 向指定传感器写入一个字节 -------- */
void DS18B20_WriteByte(DS18B20_Sensor_t sensor, uint8_t dat)
{
    // 设置输出模式
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
        // 拉低总线开始写时序
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
        
        // 写入数据位
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
        
        // 释放总线
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

/* -------- 从指定传感器读取一个字节 -------- */
uint8_t DS18B20_ReadByte(DS18B20_Sensor_t sensor)
{
    uint8_t dat = 0;
    
    for (uint8_t i = 0; i < 8; i++)
    {
        dat >>= 1;
        
        // 设置为输出模式，拉低总线开始读时序
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
        
        // 释放总线并设置为输入模式
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
        
        // 读取数据位
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
        
        // 恢复输出模式
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

/* -------- 读取指定传感器的温度 -------- */
float DS18B20_ReadTemp(DS18B20_Sensor_t sensor)
{
    uint8_t tempL, tempH;
    int16_t temp;
    float temperature;
    
    if (DS18B20_Start(sensor)) 
    {
        DS18B20_WriteByte(sensor, 0xCC);  // 跳过ROM
        DS18B20_WriteByte(sensor, 0x44);  // 启动温度转换
        HAL_Delay(800);                   // 等待转换完成
        
        DS18B20_Start(sensor);
        DS18B20_WriteByte(sensor, 0xCC);  // 跳过ROM
        DS18B20_WriteByte(sensor, 0xBE);  // 读取暂存器
        
        tempL = DS18B20_ReadByte(sensor); // 读取温度低字节
        tempH = DS18B20_ReadByte(sensor); // 读取温度高字节
        
        temp = (tempH << 8) | tempL;
        temperature = temp * 0.0625f;
        return temperature;
    }
    return -1000.0f;  // 错误返回值
}

/* -------- 同时读取所有三个传感器的温度 -------- */
void DS18B20_ReadAllTemps(float *main_temp, float *left_temp, float *right_temp)
{
    if (main_temp) *main_temp = DS18B20_ReadTemp(DS18B20_MAIN);
    if (left_temp) *left_temp = DS18B20_ReadTemp(DS18B20_LEFT);
    if (right_temp) *right_temp = DS18B20_ReadTemp(DS18B20_RIGHT);
}

/* -------- 获取传感器名称 -------- */
const char* DS18B20_GetSensorName(DS18B20_Sensor_t sensor)
{
    switch(sensor) {
        case DS18B20_MAIN: return "MAIN";
        case DS18B20_LEFT: return "LEFT";
        case DS18B20_RIGHT: return "RIGHT";
        default: return "UNKNOWN";
    }
}
