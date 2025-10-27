#include "main.h"

#define BRUSH_PIN     GPIO_PIN_4
#define BRUSH_PORT    GPIOA


uint8_t Brush_GetStatus(void)
{
    // ��ȡ PA4 ����״̬
    GPIO_PinState state = HAL_GPIO_ReadPin(BRUSH_GPIO_Port, BRUSH_Pin);

    // �����߼����أ��ߵ�ƽΪδ���䣬�͵�ƽΪ����
    if (state == GPIO_PIN_SET)
    {
        return 1; // δ���� (OK)
    }
    else
    {
        return 0; // ������ (ALERT)
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
/*
//		CheckBrush();
//    HAL_Delay(500); // ������һ��
*/
