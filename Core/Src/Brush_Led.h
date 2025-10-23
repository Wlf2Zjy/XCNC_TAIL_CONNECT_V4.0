#include "main.h"

#define BRUSH_PIN     GPIO_PIN_4
#define BRUSH_PORT    GPIOA


// 返回 1 = 毛刷正常，0 = 毛刷脱落
//uint8_t Brush_IsNormal(void)
//{
//    if (HAL_GPIO_ReadPin(BRUSH_PORT, BRUSH_PIN) == GPIO_PIN_SET)
//        return 1;   // 高电平 -> 毛刷未脱落
//    else
//        return 0;   // 低电平 -> 毛刷脱落
//}

uint8_t Brush_GetStatus(void)
{
    // 读取 PA4 引脚状态
    GPIO_PinState state = HAL_GPIO_ReadPin(BRUSH_GPIO_Port, BRUSH_Pin);

    // 根据逻辑返回：高电平为未脱落，低电平为脱落
    if (state == GPIO_PIN_SET)
    {
        return 1; // 未脱落 (OK)
    }
    else
    {
        return 0; // 已脱落 (ALERT)
    }
}


void CheckBrush(void)
{
    if (Brush_GetStatus())
    {
        printf("1\r\n");
    }
    else
    {
        printf("0\r\n");

    }
}
