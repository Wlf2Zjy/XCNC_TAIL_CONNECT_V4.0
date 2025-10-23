/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DS18B20.h"
#include "Brush_Led.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart2;  //串口2的485通信
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);  //打印输出到串口2
    return ch;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRAME_HEADER_1 0xFE     // 帧头第一个字节
#define FRAME_HEADER_2 0xFE     // 帧头第二个字节
#define FRAME_END 0xFA          // 帧结束字节

// 接收状态枚举
typedef enum {
    RX_STATE_WAIT_HEADER1,      // 等待帧头第一个字节
    RX_STATE_WAIT_HEADER2,      // 等待帧头第二个字节
    RX_STATE_WAIT_LENGTH,       // 等待长度字节
	  RX_STATE_WAIT_ID,           // 等待ID字节
    RX_STATE_WAIT_CMD,          // 等待指令字节
    RX_STATE_WAIT_CONTENT,      // 等待内容字节
    RX_STATE_WAIT_END           // 等待结束字节
} RxState;


volatile RxState rxState = RX_STATE_WAIT_HEADER1;  // 接收状态
uint8_t uart_rxByte;  // 串口接收的单个字节
// 协议帧变量
uint8_t rx_id;
uint8_t rx_cmd;                 // 接收到的指令
uint8_t rx_len;                 // 接收到的长度
uint8_t rx_content[64];         // 接收到的内容
uint8_t rx_content_index;       // 内容索引
volatile uint8_t frameReceived = 0; // 帧接收完成标志
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// 发送数据函数（无需控制方向引脚）
void RS485_SendData(uint8_t *pData, uint16_t Size)
{
    HAL_UART_Transmit(&huart2, pData, Size, 1000);
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 发送响应帧
void send_response_frame(uint8_t cmd, uint8_t return_len, uint8_t *return_content) {
    uint8_t frame[64];
    uint8_t index = 0;
    
    // 帧头
    frame[index++] = FRAME_HEADER_1;
    frame[index++] = FRAME_HEADER_2;
    
    // 返回长度 (ID1字节 + 指令1字节 + 内容n字节 + 结束符1字节)
    frame[index++] = return_len + 3;
    
    // 舵机ID
    frame[index++] = rx_id;
    
    // 返回指令
    frame[index++] = cmd;
    
    // 返回内容
    if (return_content != NULL && return_len > 0) {
        memcpy(&frame[index], return_content, return_len);
        index += return_len;
    }
    
    // 结束位
    frame[index++] = FRAME_END;
    
    // 发送响应
    RS485_SendData(frame, index);
}

// 处理接收到的协议帧 - LED控制专用
void process_protocol_frame(void) {
    uint8_t response_content[1] = {0};
    uint8_t response_len = 1;
    
    // LED控制指令 (0x03)
    if (rx_cmd == 0x04 && rx_content_index >= 1) {
        // 根据内容控制LED
        if (rx_content[0] == 0x01) {
            // 点亮LED
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
            response_content[0] = 0x01; // 成功响应
        } 
        else if (rx_content[0] == 0x00) {
            // 熄灭LED  
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            response_content[0] = 0x01; // 成功响应
        }
        else {
            response_content[0] = 0x00; // 失败响应
        }
        
        // 发送响应帧 (指令码0x04)
        send_response_frame(0x04, response_len, response_content);
    }
    
    // 重置接收状态
    rx_content_index = 0;
}

// 串口接收完成回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        switch (rxState) {
            case RX_STATE_WAIT_HEADER1:
                if (uart_rxByte == FRAME_HEADER_1) {
                    rxState = RX_STATE_WAIT_HEADER2;
                }
                break;
                
            case RX_STATE_WAIT_HEADER2:
                if (uart_rxByte == FRAME_HEADER_2) {
                    rxState = RX_STATE_WAIT_LENGTH;
                } else {
                    rxState = RX_STATE_WAIT_HEADER1;
                }
                break;
                
            case RX_STATE_WAIT_LENGTH:
                rx_len = uart_rxByte;
                rx_content_index = 0;
                
                if (rx_len >= 3) {  // 至少包含ID、指令和结束符
                    rxState = RX_STATE_WAIT_ID;  // 新增状态
                } else {
                    rxState = RX_STATE_WAIT_HEADER1;  // 无效长度
                }
                break;
                
            case RX_STATE_WAIT_ID: //等待ID状态
                rx_id = uart_rxByte;  // 存储接收到的ID
                rxState = RX_STATE_WAIT_CMD;
                break;
                
            case RX_STATE_WAIT_CMD:
                rx_cmd = uart_rxByte;
                if (rx_len > 3) {  // 接收 (长度-ID-指令-结束符)
                    rxState = RX_STATE_WAIT_CONTENT;
                } else {
                    rxState = RX_STATE_WAIT_END;
                    frameReceived = 1;
                }
                break;
                
            case RX_STATE_WAIT_CONTENT:
                if (rx_content_index < sizeof(rx_content)) {
                    rx_content[rx_content_index++] = uart_rxByte;
                } else {
                    rxState = RX_STATE_WAIT_HEADER1;
                }
								
                // 检查是否接收完所有内容 (长度 = ID1字节 + 指令1字节 + 内容n字节 + 结束符1字节)
                if (rx_content_index >= (rx_len - 3)) {  
                    rxState = RX_STATE_WAIT_END;
                }
                break;
                
            case RX_STATE_WAIT_END:
                if (uart_rxByte == FRAME_END) {  // 完整帧接收完成
                    frameReceived = 1;
                }
                rxState = RX_STATE_WAIT_HEADER1;
                break;
        }
        
        // 重新启动接收
        HAL_UART_Receive_IT(&huart2, &uart_rxByte, 1);
    }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  DS18B20_Init(DS18B20_MAIN);
  DS18B20_Init(DS18B20_LEFT);
  DS18B20_Init(DS18B20_RIGHT);
  HAL_TIM_Base_Start(&htim4);
	
	    
  HAL_UART_Receive_IT(&huart2, &uart_rxByte, 1);  // 启动串口接收中断
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  float main_temp, left_temp, right_temp;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		        if(frameReceived) {  // 检查帧接收标志
            process_protocol_frame();  // 处理协议帧
            frameReceived = 0;  // 重置标志
        }
        HAL_Delay(1);
//		CheckBrush();
//    HAL_Delay(500); // 半秒检测一次
//        DS18B20_ReadAllTemps(&main_temp, &left_temp, &right_temp);
//        
//        printf("Main: %.2f C, Left: %.2f C, Right: %.2f C\r\n", 
//               main_temp, left_temp, right_temp);
//        
//        HAL_Delay(2000);  // 每2秒读取一次	
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
