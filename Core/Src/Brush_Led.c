#include "Brush_Led.h"

#define BRUSH1_PIN     GPIO_PIN_4    //ëˢ���
#define BRUSH1_PORT    GPIOA

#define SW_PROBEU_Pin      GPIO_PIN_5  // ̽������λ
#define SW_PROBEU_GPIO_Port GPIOA

#define SW_PROBED_Pin      GPIO_PIN_6  // ̽������λ
#define SW_PROBEU_GPIO_Port GPIOA


uint8_t Brush1_GetStatus(void)
{
    // ��ȡ PA4 ����״̬
    GPIO_PinState state = HAL_GPIO_ReadPin(BRUSH_GPIO_Port, BRUSH_Pin);

    // �����߼����أ��ߵ�ƽΪδ���䣬�͵�ƽΪ����
    if (state == GPIO_PIN_SET)
    {
        return 1; // δ���� 
    }
    else
    {
        return 0; // ������ 
    }
}

//δ������λ����1��������0
uint8_t Brush2_GetStatus(void)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(SW_PROBEU_GPIO_Port, SW_PROBEU_Pin);

    if (state == GPIO_PIN_SET)
        return 1;  //
    else
        return 0;  //��������λ
}
uint8_t Brush3_GetStatus(void)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(SW_PROBED_GPIO_Port, SW_PROBED_Pin);

    if (state == GPIO_PIN_SET)
        return 1;  
    else
        return 0;  //��������λ
}

//void CheckBrush1(void)
//{
//    if (Brush1_GetStatus())
//    {
//        printf("1\r\n");
//    }
//    else
//    {
//        printf("0\r\n");

//    }
//}

//void CheckBrush2(void)
//{
//    if (Brush2_GetStatus())
//    {
//        printf("1\r\n");
//    }
//    else
//    {
//        printf("0\r\n");

//    }
//}

//void CheckBrush3(void)
//{
//    if (Brush3_GetStatus())
//    {
//        printf("1\r\n");
//    }
//    else
//    {
//        printf("0\r\n");

//    }
//}
